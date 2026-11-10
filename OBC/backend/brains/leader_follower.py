# ==============================================================================
# AlitheiaHive_v3-Uprising: Leader-Follower Star Topology Brain (OBC Bubble)
# Pure I/O Boundary (TLK-01 Heartbeat 2):
#   - Zero imports from simulator or ground station; zero simulator handle.
#   - Leader & followers coordinate strictly via RadioPacket messages in inbox/outbox.
#   - Supports ECHELON_ESCORT, NATURAL_FLOCK (3D Boids), ORBIT_VORTEX,
#     PERIMETER_PATROL, PERIMETER_IN, and PERIMETER_OUT.
# ==============================================================================

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

from OBC.backend.brains.base import (
    BaseBrain,
    FlightIntent,
    HeardPeer,
    LocalTelemetry,
    RadioPacket,
)
from OBC.backend.brains.hungarian import hungarian_assignment

V_CRUISE = 12.0
V_LKP = 5.0
HEARTBEAT_INTERVAL = 0.10  # 10 Hz
LINK_TIMEOUT = 1.0         # 1.0s timeout

# Echelon Heading Smoothing Constants (TKT-05)
MAX_SLEW_RATE = math.pi / 4.0   # 45 deg/s maximum angular rotation
FREEZE_DIST = 8.0               # Freeze heading within 8.0m of target
FREEZE_SPEED = 1.5              # Freeze heading when leader speed < 1.5 m/s

# U-Turn Slot Inversion Constants (TKT-43)
INVERSION_ANGLE_THRESHOLD = (2.0 * math.pi) / 3.0  # 120 degrees


class LeaderFollowerBrain(BaseBrain):
    """Centralized Leader-Follower brain operating across the Pure I/O Boundary."""

    def __init__(self, drone_id: int = 0, **kwargs: Any) -> None:
        super().__init__(drone_id=drone_id)
        self.heartbeat_timer: float = 0.0
        self.formation_mode: str = "ECHELON_ESCORT"
        self.formation_heading: float = 0.0
        self.heading_initialized: bool = False
        self.assigned_slots: Dict[int, int] = {}
        self.inversion_cooldown: float = 0.0
        self.route_paused: bool = False
        self._last_clock_s: float = 0.0

    def reset(self) -> None:
        """Reset internal timers, formation heading state, and peer table."""
        super().reset()
        self.heartbeat_timer = 0.0
        self.formation_heading = 0.0
        self.heading_initialized = False
        self.assigned_slots.clear()
        self.inversion_cooldown = 0.0
        self.route_paused = False
        self._last_clock_s = 0.0

    def _finalize_flight_intent(
        self,
        telemetry: LocalTelemetry,
        nominal_acc: np.ndarray,
        target_yaw: Optional[float] = None,
        target_station: Optional[np.ndarray] = None,
        state_updates: Optional[Dict[str, Any]] = None,
    ) -> FlightIntent:
        """Combine nominal guidance with OBC Zone 2/3 collision avoidance reflexes (TKT-69)."""
        updates = state_updates if state_updates is not None else {}
        final_acc = self.apply_avoidance_reflex(telemetry, nominal_acc)
        return FlightIntent(
            target_acc=final_acc,
            target_yaw=target_yaw,
            target_station=target_station,
            state_updates=updates,
        )

    def step_drone(
        self,
        telemetry: LocalTelemetry,
        inbox: List[RadioPacket],
        dt: float,
    ) -> Tuple[FlightIntent, List[RadioPacket]]:
        """Execute one onboard guidance cycle from local sensors and received radio packets."""
        self._last_clock_s = float(telemetry.clock_s)
        updates: Dict[str, Any] = {}
        outbox: List[RadioPacket] = []

        # 0. Ingest incoming RF packets into local heard_peers table
        self._ingest_packets(telemetry, inbox, updates)

        # Update own entry in heard_peers so slot indexing and boids include self
        own_pos = (
            np.copy(telemetry.pos)
            if not np.isnan(telemetry.pos).any()
            else np.copy(telemetry.lkp)
        )
        is_crashed = telemetry.flight_state == "CRASHED" or telemetry.status == "CRASHED"
        self.heard_peers[telemetry.drone_id] = HeardPeer(
            drone_id=telemetry.drone_id,
            pos=own_pos,
            vel=np.copy(telemetry.vel),
            yaw=float(telemetry.yaw),
            is_leader=bool(telemetry.is_leader),
            tier=int(telemetry.tier),
            crashed=bool(is_crashed),
            enrolled=bool(telemetry.enrolled),
            flight_state=str(telemetry.flight_state),
            acknowledged_target=np.copy(updates.get("acknowledged_target", telemetry.acknowledged_target)),
            last_heard_clock=float(telemetry.clock_s),
        )

        # Prune stale peers not heard within LINK_TIMEOUT
        stale_ids = [
            pid for pid, peer in self.heard_peers.items()
            if pid != telemetry.drone_id and (telemetry.clock_s - peer.last_heard_clock) > LINK_TIMEOUT
        ]
        for pid in stale_ids:
            self.heard_peers.pop(pid, None)

        if is_crashed or telemetry.flight_state in ("LANDED", "GROUND_IDLE", "RETRIEVED"):
            return FlightIntent(
                target_acc=np.zeros(3, dtype=float),
                target_yaw=telemetry.yaw,
                state_updates=updates,
            ), outbox

        # Maintain slot assignments if leader
        if telemetry.is_leader:
            self._update_slot_assignments_from_peers()

        # 1. Tactical GNSS Denial Dead-Reckoning & RSSI Homing Override (TKT-52)
        if telemetry.gnss_denied:
            intent = self._compute_gnss_denied_intent(telemetry, dt)
            intent.state_updates.update(updates)
            self._emit_periodic_packets(telemetry, updates, outbox, dt)
            return intent, outbox

        # 2. Universal Onboard Failsafe Hierarchy (Corridor ERTB, Backtrack, Horizon Hold, Lost-Link)
        if telemetry.flight_state == "CORRIDOR_ERTB":
            base_pos = np.array([0.0, 0.0, 25.0], dtype=float)
            stack = telemetry.corridor_stack
            stack_len = len(stack)

            if stack_len > 0:
                curr_target = np.array(stack[-1], dtype=float)
            elif telemetry.corridor_target is not None:
                curr_target = np.array(telemetry.corridor_target, dtype=float)
            else:
                curr_target = base_pos

            dist_curr = float(np.linalg.norm(telemetry.pos - curr_target))

            if dist_curr <= 4.5 and stack_len > 1:
                next_target = np.array(stack[-2], dtype=float)
                updates["corridor_pop"] = True
                updates["corridor_target"] = np.copy(next_target)
                updates["acknowledged_target"] = np.copy(next_target)
                active_target = next_target
            else:
                updates["corridor_target"] = np.copy(curr_target)
                updates["acknowledged_target"] = np.copy(curr_target)
                active_target = curr_target

            dist_base = float(np.linalg.norm(telemetry.pos - base_pos))
            if (stack_len <= 1 or dist_curr <= 4.5) and dist_base <= 4.5:
                updates["flight_state"] = "OVERHEAD_HOLD"
                updates["status"] = "OVERHEAD_HOLD"
                active_target = base_pos
                updates["acknowledged_target"] = np.copy(base_pos)

            acc = self._steer_toward(telemetry.pos, telemetry.vel, active_target, max_speed=8.0)
            yaw = self._velocity_yaw(telemetry.vel)
            self._emit_periodic_packets(telemetry, updates, outbox, dt)
            return self._finalize_flight_intent(telemetry, acc, yaw, np.copy(active_target), updates), outbox

        if telemetry.flight_state == "BACKTRACK_HOLD":
            bt = telemetry.backtrack_target if telemetry.backtrack_target is not None else telemetry.lkp
            acc = self._steer_toward(telemetry.pos, telemetry.vel, bt, max_speed=8.0)
            yaw = self._velocity_yaw(telemetry.vel)
            self._emit_periodic_packets(telemetry, updates, outbox, dt)
            return self._finalize_flight_intent(telemetry, acc, yaw, np.copy(bt), updates), outbox

        if telemetry.flight_state == "PERIMETER_HOLD" or telemetry.holding_at_horizon:
            ack_t = np.copy(updates.get("acknowledged_target", telemetry.acknowledged_target))
            acc = self._steer_toward(telemetry.pos, telemetry.vel, ack_t, max_speed=2.0)
            yaw = self._velocity_yaw(telemetry.vel)
            self._emit_periodic_packets(telemetry, updates, outbox, dt)
            return self._finalize_flight_intent(telemetry, acc, yaw, ack_t, updates), outbox

        # Tandem Escort Rejoin (TKT-51 / BUG-11: Calculated in OBC from heard courier packets)
        if telemetry.flight_state == "ESCORT_REJOIN":
            escort_id = telemetry.escort_leader_id
            courier = self.heard_peers.get(escort_id) if escort_id is not None else None
            if courier is not None and not courier.crashed:
                lead_speed = float(np.linalg.norm(courier.vel[:2]))
                if lead_speed > 0.5:
                    tangent = courier.vel / lead_speed
                    trail_pos = courier.pos - tangent * 4.0
                    trail_pos[2] = courier.pos[2]
                else:
                    trail_pos = courier.pos + np.array([-4.0, 0.0, 0.0], dtype=float)
                target_dest = trail_pos
            else:
                target_dest = np.copy(telemetry.lkp)
            updates["acknowledged_target"] = np.copy(target_dest)
            acc = self._steer_toward(telemetry.pos, telemetry.vel, target_dest, max_speed=10.0)
            yaw = self._velocity_yaw(telemetry.vel)
            self._emit_periodic_packets(telemetry, updates, outbox, dt)
            return self._finalize_flight_intent(telemetry, acc, yaw, target_dest, updates), outbox

        # Courier S&R Escort & Payload Drop States (TKT-51 / TKT-59)
        if telemetry.flight_state in ("COURIER_TRANSIT", "RENDEZVOUS_DWELL", "ESCORT_LEAD", "PAYLOAD_TRANSIT", "PRECISION_DESCENT", "CARRIER_EGRESS"):
            ack_t = np.copy(updates.get("acknowledged_target", telemetry.acknowledged_target))
            spd = 0.0 if telemetry.flight_state == "RENDEZVOUS_DWELL" else (2.0 if telemetry.flight_state == "PRECISION_DESCENT" else 10.0)
            acc = self._steer_toward(telemetry.pos, telemetry.vel, ack_t, max_speed=spd)
            yaw = self._velocity_yaw(telemetry.vel)
            self._emit_periodic_packets(telemetry, updates, outbox, dt)
            return self._finalize_flight_intent(telemetry, acc, yaw, ack_t, updates), outbox

        if telemetry.lost_link and not telemetry.is_leader:
            lkp_t = np.copy(updates.get("lkp", telemetry.lkp))
            acc = self._steer_toward(telemetry.pos, telemetry.vel, lkp_t, max_speed=V_LKP)
            yaw = self._velocity_yaw(telemetry.vel)
            self._emit_periodic_packets(telemetry, updates, outbox, dt)
            return self._finalize_flight_intent(telemetry, acc, yaw, lkp_t, updates), outbox

        # 3. Breakaway Dive & Cable Jettison Guidance (TKT-16 / TKT-27)
        if telemetry.status == "BREAKAWAY_JETTISON" and not telemetry.tether_jettisoned:
            lead_z = self._get_heard_leader_z(telemetry.pos[2])
            z_floor = max(3.0, lead_z - 4.5)
            target_z = max(z_floor, lead_z - 4.0)
            dist_to_floor = max(0.0, telemetry.pos[2] - (z_floor + 0.1))
            max_sink = min(6.5, dist_to_floor * 2.5)
            desired_vz = float(np.clip(3.5 * (target_z - telemetry.pos[2]), -max_sink, 2.0))
            a_dive_z = float(np.clip(5.0 * (desired_vz - telemetry.vel[2]), -18.0, 15.0))

            heading = np.array([math.cos(self.formation_heading), math.sin(self.formation_heading), 0.0], dtype=float)
            peel_dir = telemetry.peel_off_dir
            if float(np.linalg.norm(peel_dir)) < 0.01:
                peel_dir = np.array([-heading[1], heading[0], 0.0], dtype=float)
            v_peel_target = peel_dir * 6.0
            a_peel_lat = (v_peel_target - telemetry.vel) * 2.5
            v_long = float(np.dot(telemetry.vel, heading))
            a_brake_long = -v_long * 2.5 * heading

            acc = np.array([
                a_peel_lat[0] + a_brake_long[0],
                a_peel_lat[1] + a_brake_long[1],
                a_dive_z,
            ], dtype=float)
            self._emit_periodic_packets(telemetry, updates, outbox, dt)
            return self._finalize_flight_intent(telemetry, acc, state_updates=updates), outbox

        if telemetry.tether_failsafe_policy == "RTB" and telemetry.tether_lost:
            rtb_pos = np.array([0.0, 0.0, 25.0], dtype=float)
            acc = self._steer_toward(telemetry.pos, telemetry.vel, rtb_pos, max_speed=V_CRUISE)
            self._emit_periodic_packets(telemetry, updates, outbox, dt)
            return self._finalize_flight_intent(telemetry, acc, target_station=rtb_pos, state_updates=updates), outbox

        # 4. Unenrolled Solo Scout Guidance (TKT-50)
        if not telemetry.enrolled:
            if telemetry.rejoin_target is not None:
                acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.rejoin_target, max_speed=V_CRUISE)
                station = np.copy(telemetry.rejoin_target)
            else:
                acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.pos, max_speed=0.0)
                station = np.copy(telemetry.pos)
            self._emit_periodic_packets(telemetry, updates, outbox, dt)
            return self._finalize_flight_intent(
                telemetry, acc, self._velocity_yaw(telemetry.vel), station, updates
            ), outbox

        # 5. Leader Navigation toward Onboard Acknowledged Target & 4D Waypoints
        target_dest = np.copy(updates.get("acknowledged_target", telemetry.acknowledged_target))
        leader_speed = V_CRUISE
        if telemetry.promotion_timer > 0.0:
            new_promo = max(0.0, telemetry.promotion_timer - dt)
            updates["promotion_timer"] = new_promo
            ramp = 1.0 - (new_promo / 1.5)
            leader_speed = 7.0 + (V_CRUISE - 7.0) * ramp

        if telemetry.is_leader and telemetry.mission_active and telemetry.mission_waypoints:
            is_paused = telemetry.mission_paused or self.route_paused
            wps = telemetry.mission_waypoints
            idx = telemetry.current_wp_idx
            if 0 <= idx < len(wps):
                raw_wp = wps[idx]
                target_wp = np.array(raw_wp[:3], dtype=float)
                loiter_duration = float(raw_wp[3]) if len(raw_wp) > 3 else 0.0
                target_dest = target_wp
                updates["acknowledged_target"] = np.copy(target_wp)
                dist = float(np.linalg.norm(telemetry.pos - target_wp))

                if is_paused:
                    leader_speed = 2.0
                elif dist <= telemetry.route_acceptance_radius:
                    if not telemetry.wp_loitering and loiter_duration > 0.0:
                        updates["wp_loitering"] = True
                        updates["wp_loiter_timer"] = max(0.0, loiter_duration - dt)
                        updates["wp_total_loiter"] = loiter_duration
                        leader_speed = 2.0
                    elif telemetry.wp_loitering:
                        new_loiter = max(0.0, telemetry.wp_loiter_timer - dt)
                        updates["wp_loiter_timer"] = new_loiter
                        if new_loiter <= 0.0:
                            if self._are_escorts_settled(telemetry, target_dest):
                                updates["wp_loitering"] = False
                                updates["wp_loiter_timer"] = 0.0
                                if idx + 1 < len(wps):
                                    next_idx = idx + 1
                                    next_wp = wps[next_idx]
                                    updates["current_wp_idx"] = next_idx
                                    target_dest = np.array(next_wp[:3], dtype=float)
                                    updates["acknowledged_target"] = np.copy(target_dest)
                                    updates["wp_total_loiter"] = float(next_wp[3]) if len(next_wp) > 3 else 0.0
                                elif telemetry.mission_loop:
                                    next_wp = wps[0]
                                    updates["current_wp_idx"] = 0
                                    target_dest = np.array(next_wp[:3], dtype=float)
                                    updates["acknowledged_target"] = np.copy(target_dest)
                                    updates["wp_total_loiter"] = float(next_wp[3]) if len(next_wp) > 3 else 0.0
                                else:
                                    updates["mission_active"] = False
                            else:
                                leader_speed = 2.0
                        else:
                            leader_speed = 2.0
                    else:
                        updates["wp_loitering"] = False
                        updates["wp_loiter_timer"] = 0.0
                        if idx + 1 < len(wps):
                            next_idx = idx + 1
                            next_wp = wps[next_idx]
                            updates["current_wp_idx"] = next_idx
                            target_dest = np.array(next_wp[:3], dtype=float)
                            updates["acknowledged_target"] = np.copy(target_dest)
                            updates["wp_total_loiter"] = float(next_wp[3]) if len(next_wp) > 3 else 0.0
                        elif telemetry.mission_loop:
                            next_wp = wps[0]
                            updates["current_wp_idx"] = 0
                            target_dest = np.array(next_wp[:3], dtype=float)
                            updates["acknowledged_target"] = np.copy(target_dest)
                            updates["wp_total_loiter"] = float(next_wp[3]) if len(next_wp) > 3 else 0.0
                        else:
                            updates["mission_active"] = False
                elif telemetry.wp_loitering:
                    leader_speed = 2.0

        if telemetry.is_leader:
            self._update_formation_heading_local(telemetry, target_dest, dt)
            acc = self._steer_toward(telemetry.pos, telemetry.vel, target_dest, max_speed=leader_speed)
            target_yaw = self._compute_target_yaw_local(telemetry, target_dest, target_dest)
            self._emit_periodic_packets(telemetry, updates, outbox, dt)
            return self._finalize_flight_intent(telemetry, acc, target_yaw, np.copy(target_dest), updates), outbox

        # 6. Follower Formation Guidance via Heard Leader Telemetry (Heartbeat 2)
        heard_leader = self._get_heard_leader()
        if heard_leader is not None:
            if self.formation_mode == "NATURAL_FLOCK" and telemetry.peel_off_timer <= 0.0 and not telemetry.overhead_marshal_active:
                acc = self._compute_boid_acceleration_from_peers(telemetry, heard_leader)
                slot_pos = self._compute_slot_from_heard_leader(telemetry, heard_leader)
            else:
                slot_pos = self._compute_slot_from_heard_leader(telemetry, heard_leader)
                if telemetry.peel_off_timer > 0.0:
                    new_peel = max(0.0, telemetry.peel_off_timer - dt)
                    updates["peel_off_timer"] = new_peel
                    peel_waypoint = slot_pos + (7.0 * telemetry.peel_off_dir)
                    acc = self._steer_toward(telemetry.pos, telemetry.vel, peel_waypoint, max_speed=V_CRUISE)
                elif telemetry.enrollment_timer > 0.0:
                    new_enroll = max(0.0, telemetry.enrollment_timer - dt)
                    updates["enrollment_timer"] = new_enroll
                    ramp = 1.0 - (new_enroll / 2.5)
                    settle_speed = 6.0 + (V_CRUISE - 6.0) * ramp
                    acc = self._steer_toward(telemetry.pos, telemetry.vel, slot_pos, max_speed=settle_speed)
                else:
                    acc = self._steer_toward(telemetry.pos, telemetry.vel, slot_pos, max_speed=V_CRUISE)

            center_pos = heard_leader.acknowledged_target
            target_yaw = self._compute_target_yaw_follower(telemetry, slot_pos, center_pos)
            self._emit_periodic_packets(telemetry, updates, outbox, dt)
            return self._finalize_flight_intent(telemetry, acc, target_yaw, np.copy(slot_pos), updates), outbox

        # No surviving/heard leader: loiter in place with velocity damping
        acc = -telemetry.vel * 2.0
        self._emit_periodic_packets(telemetry, updates, outbox, dt)
        return self._finalize_flight_intent(telemetry, acc, telemetry.yaw, np.copy(telemetry.pos), updates), outbox

    # ==========================================================================
    # Packet Ingestion & Emission (Pure I/O Airgap)
    # ==========================================================================
    def _ingest_packets(
        self,
        telemetry: LocalTelemetry,
        inbox: List[RadioPacket],
        updates: Dict[str, Any],
    ) -> None:
        """Process incoming RadioPackets from the transceiver inbox."""
        for pkt in inbox:
            p = pkt.payload
            if pkt.msg_type in ("LEADER_HEARTBEAT", "ESCORT_YIP"):
                pos_raw = p.get("pos")
                if pos_raw is not None:
                    pos_arr = np.array(pos_raw, dtype=float)
                    if not np.isnan(pos_arr).any():
                        ack_raw = p.get("acknowledged_target", [60.0, 40.0, 30.0])
                        self.heard_peers[pkt.sender_id] = HeardPeer(
                            drone_id=int(pkt.sender_id),
                            pos=pos_arr,
                            vel=np.array(p.get("vel", [0.0, 0.0, 0.0]), dtype=float),
                            yaw=float(p.get("yaw", 0.0)),
                            is_leader=bool(p.get("is_leader", False)),
                            tier=int(p.get("tier", 1)),
                            crashed=bool(p.get("crashed", False)),
                            enrolled=bool(p.get("enrolled", True)),
                            flight_state=str(p.get("flight_state", "CRUISE")),
                            acknowledged_target=np.array(ack_raw, dtype=float),
                            last_heard_clock=float(telemetry.clock_s),
                            gcs_contact=bool(p.get("gcs_contact", False)),
                            gateway_id=p.get("gateway_id"),
                        )

            if pkt.msg_type == "LEADER_HEARTBEAT" and not telemetry.is_leader:
                if "formation_mode" in p:
                    self.formation_mode = str(p["formation_mode"])
                if "formation_heading" in p:
                    self.formation_heading = float(p["formation_heading"])
                    self.heading_initialized = True
                if "assigned_slots" in p and isinstance(p["assigned_slots"], dict) and p["assigned_slots"]:
                    self.assigned_slots = {int(k): int(v) for k, v in p["assigned_slots"].items()}
                is_aux_mission = telemetry.flight_state in (
                    "COURIER_TRANSIT", "RENDEZVOUS_DWELL", "ESCORT_LEAD", "ESCORT_REJOIN",
                    "PAYLOAD_TRANSIT", "PRECISION_DESCENT", "CARRIER_EGRESS",
                )
                if "acknowledged_target" in p and telemetry.enrolled and not is_aux_mission:
                    ack_arr = np.array(p["acknowledged_target"], dtype=float)
                    updates["acknowledged_target"] = ack_arr
                    if telemetry.flight_state == "PERIMETER_HOLD":
                        updates["flight_state"] = "CRUISE"
                        updates["status"] = "ACTIVE"
                        updates["holding_at_horizon"] = False
                if "pos" in p and p["pos"] is not None:
                    l_pos = np.array(p["pos"], dtype=float)
                    if not np.isnan(l_pos).any():
                        if telemetry.enrolled:
                            updates["lkp"] = np.copy(telemetry.pos)
                            updates["lost_link"] = False
                            updates["last_heartbeat_time"] = float(telemetry.clock_s)
                        elif telemetry.rejoin_target is None:
                            updates["lkp"] = np.copy(telemetry.pos)
                            updates["lost_link"] = False
                            updates["last_heartbeat_time"] = float(telemetry.clock_s)

            elif pkt.msg_type in ("GCS_UPLINK", "C2_COMMAND"):
                if "target" in p:
                    updates["acknowledged_target"] = np.array(p["target"], dtype=float)
                    updates["holding_at_horizon"] = False
                    if telemetry.flight_state == "PERIMETER_HOLD":
                        updates["flight_state"] = "CRUISE"
                        updates["status"] = "FLEET_LEADER" if telemetry.is_leader else "ACTIVE"
                cmd = str(p.get("cmd", "")).upper()
                if cmd == "TAKEOFF" and telemetry.flight_state in ("GROUND_IDLE", "LANDED"):
                    updates["flight_state"] = "TAKEOFF"
                    updates["status"] = "ACTIVE"
                elif cmd == "E_LAND" and telemetry.flight_state not in ("LANDED", "GROUND_IDLE", "FIELD_LANDED", "E_LANDED"):
                    updates["flight_state"] = "EMERGENCY_LANDING"
                    updates["status"] = "E_LANDING"
                self.handle_command(dict(p))

        # Flagship Gateway Relay Discovery: if leader lacks direct GCS contact, bind heard peer relay
        if telemetry.is_leader:
            if not telemetry.uplink_active:
                active_relays = [
                    peer for peer in self.heard_peers.values()
                    if peer.drone_id != telemetry.drone_id and peer.gcs_contact and not peer.crashed
                ]
                if active_relays:
                    active_relays.sort(key=lambda r: (r.tier * 1000) - r.drone_id, reverse=True)
                    updates["gateway_id"] = active_relays[0].drone_id
                    updates["uplink_active"] = True
                else:
                    updates["gateway_id"] = None
            else:
                updates["gateway_id"] = None

    def _emit_periodic_packets(
        self,
        telemetry: LocalTelemetry,
        updates: Dict[str, Any],
        outbox: List[RadioPacket],
        dt: float,
    ) -> None:
        """Emit 10 Hz leader heartbeats and escort telemetry packets into outbox."""
        self.heartbeat_timer += dt
        is_10hz = False
        if self.heartbeat_timer >= HEARTBEAT_INTERVAL:
            self.heartbeat_timer -= HEARTBEAT_INTERVAL
            is_10hz = True

        is_crashed = telemetry.flight_state == "CRASHED" or telemetry.status == "CRASHED"
        ack_t = updates.get("acknowledged_target", telemetry.acknowledged_target)
        pos_list = None if np.isnan(telemetry.pos).any() else [float(v) for v in telemetry.pos]
        if telemetry.is_leader:
            outbox.append(
                RadioPacket(
                    sender_id=telemetry.drone_id,
                    msg_type="LEADER_HEARTBEAT",
                    timestamp=telemetry.clock_s,
                    payload={
                        "pos": pos_list,
                        "vel": [float(v) for v in telemetry.vel],
                        "yaw": float(telemetry.yaw),
                        "is_leader": True,
                        "tier": int(telemetry.tier),
                        "crashed": bool(is_crashed),
                        "enrolled": bool(telemetry.enrolled),
                        "flight_state": str(telemetry.flight_state),
                        "acknowledged_target": [float(v) for v in ack_t],
                        "formation_mode": self.formation_mode,
                        "formation_heading": float(self.formation_heading),
                        "assigned_slots": dict(self.assigned_slots),
                        "is_10hz": is_10hz,
                        "gcs_contact": bool(telemetry.uplink_active),
                        "gateway_id": updates.get("gateway_id"),
                    },
                )
            )
        else:
            outbox.append(
                RadioPacket(
                    sender_id=telemetry.drone_id,
                    msg_type="ESCORT_YIP",
                    timestamp=telemetry.clock_s,
                    payload={
                        "pos": pos_list,
                        "vel": [float(v) for v in telemetry.vel],
                        "yaw": float(telemetry.yaw),
                        "is_leader": False,
                        "tier": int(telemetry.tier),
                        "crashed": bool(is_crashed),
                        "enrolled": bool(telemetry.enrolled),
                        "flight_state": str(telemetry.flight_state),
                        "acknowledged_target": [float(v) for v in ack_t],
                        "is_10hz": is_10hz,
                        "gcs_contact": bool(telemetry.uplink_active),
                        "gateway_id": updates.get("gateway_id"),
                    },
                )
            )

    def _get_heard_leader(self) -> Optional[HeardPeer]:
        """Return the active leader from the local RF peer table."""
        leaders = [
            p for p in self.heard_peers.values()
            if p.is_leader and not p.crashed and p.flight_state not in ("LANDED", "DESCENDING", "RETRIEVED", "GROUND_IDLE")
        ]
        if not leaders:
            leaders = [p for p in self.heard_peers.values() if p.is_leader and not p.crashed]
        if not leaders:
            return None
        leaders.sort(key=lambda p: (p.tier * 1000) - p.drone_id, reverse=True)
        return leaders[0]

    def _get_heard_leader_z(self, fallback_z: float) -> float:
        """Return the last heard leader altitude from RF peer table, or fallback_z."""
        lead = self._get_heard_leader()
        if lead is not None:
            return float(lead.pos[2])
        return float(fallback_z)

    # ==========================================================================
    # Formation Slot Allocation, Geometry & Boids
    # ==========================================================================
    def _active_followers_from_peers(self) -> List[HeardPeer]:
        """Return sorted list of active enrolled follower peers heard over RF."""
        followers = [
            p for p in self.heard_peers.values()
            if not p.crashed and not p.is_leader and p.enrolled
        ]
        followers.sort(key=lambda p: p.drone_id)
        return followers

    def _update_slot_assignments_from_peers(self) -> None:
        """Maintain persistent 1-based formation slot indices (1..N) across heard followers."""
        active_followers = self._active_followers_from_peers()
        active_ids = {p.drone_id for p in active_followers}
        n = len(active_followers)

        self.assigned_slots = {
            did: slot for did, slot in self.assigned_slots.items()
            if did in active_ids
        }

        used_slots = set()
        kept_slots: Dict[int, int] = {}
        for did, slot in self.assigned_slots.items():
            if 1 <= slot <= n and slot not in used_slots:
                kept_slots[did] = slot
                used_slots.add(slot)
        self.assigned_slots = kept_slots

        available_slots = [s for s in range(1, n + 1) if s not in used_slots]
        avail_idx = 0
        for peer in active_followers:
            if peer.drone_id not in self.assigned_slots:
                if avail_idx < len(available_slots):
                    self.assigned_slots[peer.drone_id] = available_slots[avail_idx]
                    avail_idx += 1
                else:
                    self.assigned_slots[peer.drone_id] = n

    def _are_escorts_settled(self, telemetry: LocalTelemetry, leader_target: np.ndarray) -> bool:
        """Verify all heard enrolled followers are in CRUISE within 4.5m of their formation slots."""
        followers = self._active_followers_from_peers()
        if not followers:
            return True
        own_leader = self.heard_peers.get(telemetry.drone_id)
        if own_leader is None:
            return True
        n_followers = len(followers)
        for idx, f_peer in enumerate(followers):
            if f_peer.flight_state != "CRUISE":
                return False
            k = self.assigned_slots.get(f_peer.drone_id, idx + 1)
            slot_pos = self._compute_slot_geometry(
                k=k,
                n_followers=n_followers,
                leader_pos=telemetry.pos,
                leader_vel=telemetry.vel,
                leader_yaw=telemetry.yaw,
                center_target=leader_target,
                overhead_marshal=telemetry.overhead_marshal_active,
                clock_s=telemetry.clock_s,
            )
            if float(np.linalg.norm(f_peer.pos - slot_pos)) > 4.5:
                return False
        return True

    def _compute_slot_from_heard_leader(
        self, telemetry: LocalTelemetry, leader: HeardPeer
    ) -> np.ndarray:
        """Compute desired 3D formation slot for this follower relative to heard leader."""
        followers = self._active_followers_from_peers()
        n_followers = max(1, len(followers))
        self._update_slot_assignments_from_peers()
        k = self.assigned_slots.get(telemetry.drone_id)
        if k is None:
            for idx, p in enumerate(followers):
                if p.drone_id == telemetry.drone_id:
                    k = idx + 1
                    break
            if k is None:
                k = max(1, telemetry.drone_id)

        return self._compute_slot_geometry(
            k=k,
            n_followers=n_followers,
            leader_pos=leader.pos,
            leader_vel=leader.vel,
            leader_yaw=leader.yaw,
            center_target=leader.acknowledged_target,
            overhead_marshal=telemetry.overhead_marshal_active,
            clock_s=telemetry.clock_s,
        )

    def _get_formation_slot(
        self,
        drone: Any,
        leader: Any,
        n_followers: Optional[int] = None,
        overhead_marshal: bool = False,
        clock_s: Optional[float] = None,
    ) -> np.ndarray:
        """Pure geometric slot calculation helper for a given follower and leader state."""
        followers = self._active_followers_from_peers()
        if n_followers is None:
            n_followers = max(1, len(followers)) if followers else max(1, int(getattr(drone, "id", 1)))
        k = self.assigned_slots.get(int(getattr(drone, "id", 1)))
        if k is None:
            k = max(1, int(getattr(drone, "id", 1)))

        l_pos = np.array(getattr(leader, "pos", [0.0, 0.0, 20.0]), dtype=float)
        l_vel = np.array(getattr(leader, "vel", [0.0, 0.0, 0.0]), dtype=float)
        l_yaw = float(getattr(leader, "yaw", 0.0))
        c_target = np.array(
            getattr(leader, "acknowledged_target", l_pos),
            dtype=float,
        )
        t_clock = self._last_clock_s if clock_s is None else float(clock_s)
        return self._compute_slot_geometry(
            k=k,
            n_followers=n_followers,
            leader_pos=l_pos,
            leader_vel=l_vel,
            leader_yaw=l_yaw,
            center_target=c_target,
            overhead_marshal=overhead_marshal,
            clock_s=t_clock,
        )

    def _compute_slot_geometry(
        self,
        k: int,
        n_followers: int,
        leader_pos: np.ndarray,
        leader_vel: np.ndarray,
        leader_yaw: float,
        center_target: np.ndarray,
        overhead_marshal: bool = False,
        clock_s: float = 0.0,
    ) -> np.ndarray:
        """Pure mathematical 3D slot coordinate formula for slot index k of n_followers."""
        if overhead_marshal:
            radius = 20.0
            angle = (2.0 * math.pi * (k - 1)) / max(1, n_followers)
            return np.array(
                [radius * math.cos(angle), radius * math.sin(angle), 23.0],
                dtype=float,
            )

        if self.formation_mode == "ECHELON_ESCORT":
            if not self.heading_initialized:
                speed_xy = math.hypot(leader_vel[0], leader_vel[1])
                if speed_xy >= FREEZE_SPEED:
                    h_angle = math.atan2(leader_vel[1], leader_vel[0])
                else:
                    h_angle = leader_yaw
            else:
                h_angle = self.formation_heading

            heading = np.array([math.cos(h_angle), math.sin(h_angle), 0.0], dtype=float)
            lateral = np.array([-heading[1], heading[0], 0.0], dtype=float)

            row = (k + 1) // 2
            side = 1.0 if (k % 2 == 1) else -1.0
            return (
                leader_pos
                - (row * 6.0 * heading)
                + (side * row * 6.0 * lateral)
                + np.array([0.0, 0.0, row * 0.5], dtype=float)
            )

        elif self.formation_mode == "ORBIT_VORTEX":
            radius = 14.0
            omega = 0.8
            angle = omega * clock_s + (2.0 * math.pi * (k - 1) / max(1, n_followers))
            z_offset = 3.0 * math.sin(2.0 * angle)
            return (
                leader_pos
                + np.array([radius * math.cos(angle), radius * math.sin(angle), z_offset], dtype=float)
            )

        elif self.formation_mode in ("PERIMETER_PATROL", "PERIMETER_IN", "PERIMETER_OUT"):
            radius = 25.0
            angle = (2.0 * math.pi * (k - 1)) / max(1, n_followers)
            return (
                center_target
                + np.array([radius * math.cos(angle), radius * math.sin(angle), 0.0], dtype=float)
            )

        else:
            row = (k + 1) // 2
            side = 1.0 if (k % 2 == 1) else -1.0
            z_layer = ((k % 3) - 1) * 2.5
            return leader_pos + np.array([-row * 5.0, side * 5.0, z_layer], dtype=float)

    def _compute_boid_acceleration_from_peers(
        self, telemetry: LocalTelemetry, leader: HeardPeer
    ) -> np.ndarray:
        """Compute 3D Reynolds Boids flocking acceleration strictly from heard RF peers."""
        return self._compute_boid_math(
            drone_id=telemetry.drone_id,
            drone_pos=telemetry.pos,
            drone_vel=telemetry.vel,
            leader_pos=leader.pos,
            leader_vel=leader.vel,
            leader_yaw=leader.yaw,
            peers=list(self.heard_peers.values()),
            is_tethered=bool(telemetry.tether_active),
            tether_anchor=telemetry.tether_anchor_pos,
        )

    def _compute_boid_acceleration(
        self,
        drone: Any,
        leader: Any,
        peers: Optional[List[Any]] = None,
        is_tethered: bool = True,
        tether_anchor: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Compatibility helper for direct boid acceleration testing."""
        peer_list = peers if peers is not None else list(self.heard_peers.values())
        anchor = tether_anchor if tether_anchor is not None else np.zeros(3, dtype=float)
        return self._compute_boid_math(
            drone_id=int(getattr(drone, "id", 1)),
            drone_pos=np.array(getattr(drone, "pos", [0.0, 0.0, 20.0]), dtype=float),
            drone_vel=np.array(getattr(drone, "vel", [0.0, 0.0, 0.0]), dtype=float),
            leader_pos=np.array(getattr(leader, "pos", [0.0, 0.0, 20.0]), dtype=float),
            leader_vel=np.array(getattr(leader, "vel", [0.0, 0.0, 0.0]), dtype=float),
            leader_yaw=float(getattr(leader, "yaw", 0.0)),
            peers=peer_list,
            is_tethered=is_tethered,
            tether_anchor=anchor,
        )

    def _compute_boid_math(
        self,
        drone_id: int,
        drone_pos: np.ndarray,
        drone_vel: np.ndarray,
        leader_pos: np.ndarray,
        leader_vel: np.ndarray,
        leader_yaw: float,
        peers: List[Any],
        is_tethered: bool,
        tether_anchor: np.ndarray,
    ) -> np.ndarray:
        """Pure 3D Reynolds Boids math (Separation, Alignment, Cohesion, Floor & Tether Guard)."""
        r_sep = 8.5
        r_align = 22.0
        w_sep = 16.0
        w_align = 1.8
        w_coh = 1.6

        a_sep = np.zeros(3, dtype=float)
        v_neighbors = np.zeros(3, dtype=float)
        p_neighbors = np.zeros(3, dtype=float)
        align_count = 0

        for other in peers:
            oid = int(getattr(other, "drone_id", getattr(other, "id", -1)))
            if oid == drone_id or getattr(other, "crashed", False) or not getattr(other, "enrolled", True):
                continue
            o_pos = np.array(getattr(other, "pos"), dtype=float)
            o_vel = np.array(getattr(other, "vel"), dtype=float)
            diff = drone_pos - o_pos
            dist = float(np.linalg.norm(diff))
            if dist < 1e-3:
                diff = np.array([1.0, 0.0, 0.0], dtype=float)
                dist = 1.0

            if dist < r_sep:
                scale = ((r_sep - dist) / r_sep) / max(dist, 0.5)
                a_sep += (diff / dist) * (w_sep * scale * r_sep)

            if dist < r_align:
                v_neighbors += o_vel
                p_neighbors += o_pos
                align_count += 1

        if align_count > 0:
            v_avg = v_neighbors / align_count
            a_align = (v_avg - drone_vel) * w_align
            p_centroid = p_neighbors / align_count
        else:
            a_align = (leader_vel - drone_vel) * w_align
            p_centroid = np.copy(leader_pos)

        speed_xy = math.hypot(leader_vel[0], leader_vel[1])
        if speed_xy > 0.5:
            heading = np.array([leader_vel[0] / speed_xy, leader_vel[1] / speed_xy, 0.0], dtype=float)
        else:
            heading = np.array([math.cos(leader_yaw), math.sin(leader_yaw), 0.0], dtype=float)

        leader_anchor = leader_pos - (4.0 * heading)
        target_flock_center = 0.65 * leader_anchor + 0.35 * p_centroid

        k = self.assigned_slots.get(drone_id, drone_id)
        z_layer_offset = ((k % 3) - 1) * 2.5 + math.sin(self._last_clock_s * 1.2 + drone_id) * 1.2
        target_z = max(14.0, leader_pos[2] + z_layer_offset)
        target_flock_center[2] = target_z

        to_center = target_flock_center - drone_pos
        dist_center = float(np.linalg.norm(to_center))
        if dist_center > 0.25:
            desired_speed = min(V_CRUISE, dist_center * 1.5)
            desired_vel = (to_center / dist_center) * desired_speed + (0.5 * leader_vel)
            sp = float(np.linalg.norm(desired_vel))
            if sp > V_CRUISE:
                desired_vel = (desired_vel / sp) * V_CRUISE
            a_coh = (desired_vel - drone_vel) * w_coh
        else:
            a_coh = -drone_vel * 2.0

        a_floor = np.zeros(3, dtype=float)
        if drone_pos[2] < 14.0:
            a_floor[2] = 14.0 * ((14.0 - drone_pos[2]) / 4.0)

        a_tether = np.zeros(3, dtype=float)
        if is_tethered:
            d_perp, r_perp = self._dist_to_segment(drone_pos, tether_anchor, leader_pos)
            tether_buffer = 6.0
            if d_perp < tether_buffer:
                if d_perp < 0.01:
                    push_dir = np.array([1.0, 0.0, 0.0], dtype=float)
                else:
                    push_dir = r_perp / d_perp
                mag = 22.0 * ((tether_buffer - d_perp) / tether_buffer)
                a_tether = push_dir * mag

        return a_sep + a_align + a_coh + a_floor + a_tether

    @staticmethod
    def _dist_to_segment(
        point: np.ndarray, seg_a: np.ndarray, seg_b: np.ndarray
    ) -> Tuple[float, np.ndarray]:
        """Calculate Euclidean distance and perpendicular offset vector from a point to [seg_a, seg_b]."""
        ab = seg_b - seg_a
        ab_len_sq = float(np.dot(ab, ab))
        if ab_len_sq < 1e-6:
            r = point - seg_a
            return float(np.linalg.norm(r)), r
        t = float(np.clip(np.dot(point - seg_a, ab) / ab_len_sq, 0.0, 1.0))
        closest = seg_a + (t * ab)
        r_perp = point - closest
        return float(np.linalg.norm(r_perp)), r_perp

    # ==========================================================================
    # Heading Smoothing & Hungarian U-Turn Slot Inversion (TKT-05 / TKT-43)
    # ==========================================================================
    def _update_formation_heading_local(
        self, telemetry: LocalTelemetry, leader_target: np.ndarray, dt: float
    ) -> None:
        """Update and smooth echelon formation heading from local leader telemetry."""
        speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
        to_target = leader_target - telemetry.pos
        dist_to_target = float(np.linalg.norm(to_target))

        if not self.heading_initialized:
            if speed_xy >= FREEZE_SPEED:
                self.formation_heading = math.atan2(telemetry.vel[1], telemetry.vel[0])
            else:
                dist_xy = math.hypot(to_target[0], to_target[1])
                if dist_xy > 1.0:
                    self.formation_heading = math.atan2(to_target[1], to_target[0])
                else:
                    self.formation_heading = float(telemetry.yaw)
            self.heading_initialized = True
            return

        self.inversion_cooldown = max(0.0, self.inversion_cooldown - dt)

        if not telemetry.patrol_active and (speed_xy < FREEZE_SPEED or dist_to_target < FREEZE_DIST):
            return

        if speed_xy >= FREEZE_SPEED:
            target_heading = math.atan2(telemetry.vel[1], telemetry.vel[0])
        elif dist_to_target > 1.0:
            target_heading = math.atan2(to_target[1], to_target[0])
        else:
            return

        # U-Turn Heading Reversal Detection (> 120 deg) (TKT-43)
        if not telemetry.patrol_active and dist_to_target > FREEZE_DIST and self.inversion_cooldown <= 0.0:
            target_bearing = math.atan2(to_target[1], to_target[0])
            reversal_diff = (target_bearing - self.formation_heading + math.pi) % (2.0 * math.pi) - math.pi
            if abs(reversal_diff) >= INVERSION_ANGLE_THRESHOLD:
                self.formation_heading = target_bearing
                self.inversion_cooldown = 2.0
                if self.formation_mode == "ECHELON_ESCORT":
                    self._invert_formation_slots(telemetry.pos)
                return

        effective_slew_rate = math.radians(60.0) if telemetry.patrol_active else MAX_SLEW_RATE
        yaw_diff = (target_heading - self.formation_heading + math.pi) % (2.0 * math.pi) - math.pi
        max_step = effective_slew_rate * dt
        slew = float(np.clip(yaw_diff, -max_step, max_step))
        self.formation_heading = (self.formation_heading + slew + math.pi) % (2.0 * math.pi) - math.pi

    def _invert_formation_slots(self, leader_pos: np.ndarray) -> None:
        """Remap follower formation slots via Hungarian matching on U-turn reversal (TKT-43)."""
        followers = self._active_followers_from_peers()
        n = len(followers)
        if n <= 1:
            return

        h_angle = self.formation_heading
        heading = np.array([math.cos(h_angle), math.sin(h_angle), 0.0], dtype=float)
        lateral = np.array([-heading[1], heading[0], 0.0], dtype=float)

        slot_coords: List[np.ndarray] = []
        for k in range(1, n + 1):
            row = (k + 1) // 2
            side = 1.0 if (k % 2 == 1) else -1.0
            s_pos = (
                leader_pos
                - (row * 6.0 * heading)
                + (side * row * 6.0 * lateral)
                + np.array([0.0, 0.0, row * 0.5], dtype=float)
            )
            slot_coords.append(s_pos)

        cost_matrix = np.zeros((n, n), dtype=float)
        for i, peer in enumerate(followers):
            for j, s_pos in enumerate(slot_coords):
                diff = peer.pos - s_pos
                cost_matrix[i, j] = float(np.dot(diff, diff))

        row_ind, col_ind = hungarian_assignment(cost_matrix)
        new_assignments: Dict[int, int] = {}
        for r, c in zip(row_ind, col_ind):
            new_assignments[followers[r].drone_id] = c + 1
        self.assigned_slots = new_assignments

    # ==========================================================================
    # Yaw & GNSS-Denied Helpers
    # ==========================================================================
    def _compute_target_yaw_local(
        self, telemetry: LocalTelemetry, target_dest: np.ndarray, center_pos: np.ndarray
    ) -> float:
        """Compute desired yaw heading (radians) for leader or solo drone."""
        if self.formation_mode in ("PERIMETER_PATROL", "PERIMETER_IN"):
            to_center = center_pos[:2] - telemetry.pos[:2]
            if math.hypot(to_center[0], to_center[1]) > 0.5:
                return math.atan2(to_center[1], to_center[0])
        elif self.formation_mode == "PERIMETER_OUT":
            from_center = telemetry.pos[:2] - center_pos[:2]
            if math.hypot(from_center[0], from_center[1]) > 0.5:
                return math.atan2(from_center[1], from_center[0])

        speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
        if speed_xy > 0.5:
            return math.atan2(telemetry.vel[1], telemetry.vel[0])
        to_dest = target_dest[:2] - telemetry.pos[:2]
        if math.hypot(to_dest[0], to_dest[1]) > 0.5:
            return math.atan2(to_dest[1], to_dest[0])
        return telemetry.yaw

    def _compute_target_yaw_follower(
        self, telemetry: LocalTelemetry, slot_pos: np.ndarray, center_pos: np.ndarray
    ) -> float:
        """Compute desired yaw heading (radians) for a formation follower (TKT-55)."""
        if self.formation_mode in ("PERIMETER_PATROL", "PERIMETER_IN"):
            to_center = center_pos[:2] - telemetry.pos[:2]
            if math.hypot(to_center[0], to_center[1]) > 0.5:
                return math.atan2(to_center[1], to_center[0])
            return telemetry.yaw

        if self.formation_mode == "PERIMETER_OUT":
            from_center = telemetry.pos[:2] - center_pos[:2]
            if math.hypot(from_center[0], from_center[1]) > 0.5:
                return math.atan2(from_center[1], from_center[0])
            return telemetry.yaw

        if self.formation_mode == "ECHELON_ESCORT":
            return float(self.formation_heading)

        speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
        if speed_xy > 0.5:
            return math.atan2(telemetry.vel[1], telemetry.vel[0])
        return float(self.formation_heading)

    @staticmethod
    def _velocity_yaw(vel: np.ndarray) -> Optional[float]:
        speed_xy = math.hypot(vel[0], vel[1])
        if speed_xy > 0.5:
            return math.atan2(vel[1], vel[0])
        return None

    def on_succession(self, old_leader_id: Optional[int], new_leader_id: int) -> None:
        """Transfer vacated follower slot from newly promoted leader to demoted leader."""
        vacated_slot = self.assigned_slots.pop(new_leader_id, None)
        if old_leader_id is not None and vacated_slot is not None:
            self.assigned_slots[old_leader_id] = vacated_slot

    def handle_command(self, cmd_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Process brain-specific C2 directives."""
        cmd = str(cmd_dict.get("cmd", "")).upper()
        if cmd == "SET_FORMATION":
            mode = str(cmd_dict.get("mode", "")).upper()
            if mode in (
                "ECHELON_ESCORT",
                "NATURAL_FLOCK",
                "ORBIT_VORTEX",
                "PERIMETER_PATROL",
                "PERIMETER_IN",
                "PERIMETER_OUT",
            ):
                self.formation_mode = mode
                return {"status": "OK", "formation": self.formation_mode}
        elif cmd == "PAUSE_ROUTE":
            self.route_paused = True
            return {"status": "OK", "paused": True}
        elif cmd == "RESUME_ROUTE":
            self.route_paused = False
            return {"status": "OK", "paused": False}
        elif cmd in ("E_LAND", "HOLD_POSITION", "TAKEOFF", "SCRAMBLE"):
            return {"status": "OK", "cmd": cmd}
        return None

    def get_telemetry_extra(self) -> Dict[str, Any]:
        return {
            "brain_mode": "LEADER_FOLLOWER",
            "formation": self.formation_mode,
            "formation_heading_deg": round(math.degrees(self.formation_heading) % 360.0, 1),
        }

    @staticmethod
    def _steer_toward(
        current_pos: np.ndarray,
        current_vel: np.ndarray,
        target_pos: np.ndarray,
        max_speed: float,
        arrival_radius: float = 6.0,
    ) -> np.ndarray:
        """Reynolds Arrival Steering behavior: decelerates smoothly as target is approached."""
        offset = target_pos - current_pos
        dist = float(np.linalg.norm(offset))
        if dist < 0.25:
            return -current_vel * 3.0
        if dist < arrival_radius:
            desired_speed = max_speed * (dist / arrival_radius)
        else:
            desired_speed = max_speed
        desired_vel = (offset / dist) * desired_speed
        return (desired_vel - current_vel) * 2.5
