"""
SwarmTech3_UpRising: Leader-Follower Guidance Brain (leader_follower.py)
Per-drone autonomous OBC guidance plugin (Phase 4 Pure I/O Boundary).
Heartbeat 1: Solo Navigation, Onboard 4D Waypoint Queue, Breadcrumb Backtrack,
GNSS-Denied Dead-Reckoning, and Breakaway Jettison Failsafe.
"""

from __future__ import annotations
import math
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

from .base import (
    BaseBrain,
    RadioPacket,
    HeardPeer,
    LocalTelemetry,
    FlightIntent,
)
from .hungarian import hungarian_assignment

RADIO_RANGE: float = 120.0
HEARTBEAT_INTERVAL: float = 0.1
LINK_TIMEOUT: float = 1.0
V_CRUISE: float = 12.0
V_LKP: float = 8.0
TIER_LEADER: int = 3
TIER_ESCORT: int = 2

# Echelon Anti-Jitter & Heading Slew Parameters
FREEZE_SPEED: float = 1.5        # m/s: leader speeds below this freeze echelon heading
FREEZE_DIST: float = 8.0         # meters: leader distance to target below this freezes echelon heading
DEADBAND_RADIUS: float = 0.25    # meters: arrival deadband stops micro-hunting/jitter
MAX_SLEW_RATE: float = math.radians(45.0)  # 45 deg/s = ~0.7854 rad/s slew limit


class LeaderFollowerBrain(BaseBrain):
    """Per-drone autonomous OBC brain for Leader-Follower operations.
    Operates strictly on LocalTelemetry and received RadioPackets with zero simulator references."""

    brain_name: str = "LEADER_FOLLOWER"

    def __init__(self, drone_id: int = 0) -> None:
        super().__init__(drone_id=drone_id)
        self.last_heartbeat_broadcast: float = -1.0
        self.formation_mode: str = "ECHELON_ESCORT"
        self.formation_heading: float = 0.0
        self.heading_initialized: bool = False
        self.assigned_slots: Dict[int, int] = {}
        self.inversion_cooldown: float = 0.0
        self.route_paused: bool = False

    def reset(self) -> None:
        self.last_heartbeat_broadcast = -1.0
        self.formation_heading = 0.0
        self.heading_initialized = False
        self.assigned_slots.clear()
        self.heard_peers.clear()
        self.inversion_cooldown = 0.0
        self.route_paused = False

    def on_succession(self, old_leader_id: Optional[int], new_leader_id: int) -> None:
        """Deconflict slot assignments during leadership transitions (TKT-15)."""
        if new_leader_id in self.assigned_slots:
            vacated_slot = self.assigned_slots.pop(new_leader_id)
            if old_leader_id is not None:
                self.assigned_slots[old_leader_id] = vacated_slot

    def step_drone(
        self,
        telemetry: LocalTelemetry,
        inbox: List[RadioPacket],
        dt: float,
    ) -> Tuple[FlightIntent, List[RadioPacket]]:
        """Heartbeat 1: Pure per-drone onboard guidance step (Solo Navigation & Failsafes)."""
        outbox: List[RadioPacket] = []
        updates: Dict[str, Any] = {}

        # 0. Ingest any C2 command packets delivered in inbox
        for pkt in inbox:
            if pkt.msg_type == "GCS_UPLINK":
                cmd = str(pkt.payload.get("cmd", "")).upper()
                if cmd == "SET_FORMATION":
                    self.handle_command(pkt.payload)
                elif cmd in ("SET_TARGET", "DISPATCH_TARGET") and "target" in pkt.payload:
                    updates["acknowledged_target"] = np.array(pkt.payload["target"], dtype=float)
                elif cmd == "PAUSE_ROUTE":
                    self.route_paused = True
                    updates["mission_paused"] = True
                elif cmd == "RESUME_ROUTE":
                    self.route_paused = False
                    updates["mission_paused"] = False

        if telemetry.flight_state in ("LANDED", "DESCENDING", "GROUND_IDLE", "CRASHED"):
            return FlightIntent(target_acc=np.zeros(3, dtype=float)), outbox

        # 1. Tactical GNSS Denial & Dead-Reckoning RTB Override (TKT-52)
        if telemetry.gnss_denied:
            intent = self._compute_gnss_denied_intent(telemetry, dt)
            intent.state_updates.update(updates)
            return intent, outbox

        # 2. Universal Onboard Failsafe Hierarchy (Backtrack, Horizon Hold, Lost-Link)
        if telemetry.flight_state == "BACKTRACK_HOLD" and telemetry.backtrack_target is not None:
            acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.backtrack_target, max_speed=8.0)
            yaw = self._velocity_yaw(telemetry.vel)
            return FlightIntent(
                target_acc=acc,
                target_yaw=yaw,
                target_station=np.copy(telemetry.backtrack_target),
                state_updates=updates,
            ), outbox

        if telemetry.flight_state == "PERIMETER_HOLD" or telemetry.holding_at_horizon:
            acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.acknowledged_target, max_speed=2.0)
            yaw = self._velocity_yaw(telemetry.vel)
            return FlightIntent(
                target_acc=acc,
                target_yaw=yaw,
                target_station=np.copy(telemetry.acknowledged_target),
                state_updates=updates,
            ), outbox

        if telemetry.lost_link:
            acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.lkp, max_speed=V_LKP)
            yaw = self._velocity_yaw(telemetry.vel)
            return FlightIntent(
                target_acc=acc,
                target_yaw=yaw,
                target_station=np.copy(telemetry.lkp),
                state_updates=updates,
            ), outbox

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
            return FlightIntent(target_acc=acc, state_updates=updates), outbox

        if telemetry.tether_failsafe_policy == "RTB" and telemetry.tether_lost:
            rtb_pos = np.array([0.0, 0.0, 25.0], dtype=float)
            acc = self._steer_toward(telemetry.pos, telemetry.vel, rtb_pos, max_speed=V_CRUISE)
            return FlightIntent(target_acc=acc, target_station=rtb_pos, state_updates=updates), outbox

        # 4. Unenrolled Solo Scout Guidance (TKT-50)
        if not telemetry.enrolled:
            if telemetry.rejoin_target is not None:
                acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.rejoin_target, max_speed=V_CRUISE)
                station = np.copy(telemetry.rejoin_target)
            else:
                acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.pos, max_speed=0.0)
                station = np.copy(telemetry.pos)
            return FlightIntent(
                target_acc=acc,
                target_yaw=self._velocity_yaw(telemetry.vel),
                target_station=station,
                state_updates=updates,
            ), outbox

        # 5. Solo / Leader Navigation toward Onboard Acknowledged Target & 4D Waypoints
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
            return FlightIntent(
                target_acc=acc,
                target_yaw=target_yaw,
                target_station=np.copy(target_dest),
                state_updates=updates,
            ), outbox

        # Heartbeat 1 Follower Coma State: without Heartbeat 2 RF packet slot computation,
        # follower steers solo toward its own local acknowledged_target register.
        acc = self._steer_toward(telemetry.pos, telemetry.vel, target_dest, max_speed=V_CRUISE)
        target_yaw = self._compute_target_yaw_local(telemetry, target_dest, target_dest)
        return FlightIntent(
            target_acc=acc,
            target_yaw=target_yaw,
            target_station=np.copy(target_dest),
            state_updates=updates,
        ), outbox

    def _get_heard_leader_z(self, fallback_z: float) -> float:
        """Return the last heard leader altitude from RF peer table, or fallback_z."""
        for p in self.heard_peers.values():
            if p.is_leader and not p.crashed:
                return float(p.pos[2])
        return float(fallback_z)

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

        effective_slew_rate = math.radians(60.0) if telemetry.patrol_active else MAX_SLEW_RATE
        yaw_diff = (target_heading - self.formation_heading + math.pi) % (2.0 * math.pi) - math.pi
        max_step = effective_slew_rate * dt
        slew = float(np.clip(yaw_diff, -max_step, max_step))
        self.formation_heading = (self.formation_heading + slew + math.pi) % (2.0 * math.pi) - math.pi

    def _compute_target_yaw_local(
        self, telemetry: LocalTelemetry, center: np.ndarray, station: np.ndarray
    ) -> Optional[float]:
        speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
        if self.formation_mode in ("PERIMETER_PATROL", "PERIMETER_IN", "PERIMETER_OUT"):
            r_xy = math.hypot(telemetry.pos[0] - center[0], telemetry.pos[1] - center[1])
            if self.formation_mode == "PERIMETER_OUT":
                psi_target = math.atan2(telemetry.pos[1] - center[1], telemetry.pos[0] - center[0]) if r_xy > 0.5 else self.formation_heading
            else:
                psi_target = math.atan2(center[1] - telemetry.pos[1], center[0] - telemetry.pos[0]) if r_xy > 0.5 else self.formation_heading
            dist_to_slot = float(np.linalg.norm(telemetry.pos - station))
            if dist_to_slot > 6.0 and speed_xy > 2.0:
                return math.atan2(telemetry.vel[1], telemetry.vel[0])
            return psi_target
        elif self.formation_mode == "ECHELON_ESCORT":
            return self.formation_heading
        elif self.formation_mode == "ORBIT_VORTEX":
            return math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.5 else self.formation_heading
        elif self.formation_mode == "NATURAL_FLOCK":
            return math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.2 else float(telemetry.yaw)
        return None

    @staticmethod
    def _velocity_yaw(vel: np.ndarray) -> Optional[float]:
        speed_xy = math.hypot(vel[0], vel[1])
        return math.atan2(vel[1], vel[0]) if speed_xy > 0.5 else None

    @staticmethod
    def _steer_toward(
        current_pos: np.ndarray,
        current_vel: np.ndarray,
        target_pos: np.ndarray,
        max_speed: float,
        arrival_radius: float = 6.0,
        deadband: float = DEADBAND_RADIUS,
    ) -> np.ndarray:
        offset = target_pos - current_pos
        dist = float(np.linalg.norm(offset))
        if dist < deadband:
            return -current_vel * 3.0

        if dist < arrival_radius:
            desired_speed = max_speed * (dist / arrival_radius)
        else:
            desired_speed = max_speed

        desired_vel = (offset / dist) * desired_speed
        return (desired_vel - current_vel) * 2.5

    def handle_command(self, cmd_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        cmd = cmd_dict.get("cmd", "").upper()
        if cmd == "SET_FORMATION":
            mode = cmd_dict.get("mode", "ECHELON_ESCORT")
            mode_upper = mode.upper().replace("-", "_").strip()
            if mode_upper in ("BOIDS", "BOID", "FLOCK"):
                mode_upper = "NATURAL_FLOCK"
            elif mode_upper in ("PERIMETER_PATROL_IN", "PERIMETER_IN"):
                mode_upper = "PERIMETER_IN"
            elif mode_upper in ("PERIMETER_PATROL_OUT", "PERIMETER_OUT"):
                mode_upper = "PERIMETER_OUT"
            elif mode_upper == "PERIMETER":
                mode_upper = "PERIMETER_PATROL"

            valid = ["ECHELON_ESCORT", "NATURAL_FLOCK", "ORBIT_VORTEX", "PERIMETER_PATROL", "PERIMETER_IN", "PERIMETER_OUT"]
            if mode_upper in valid:
                self.formation_mode = mode_upper
                return {"status": "OK", "formation": self.formation_mode}

        elif cmd == "PAUSE_ROUTE":
            self.route_paused = True
            return {"status": "OK", "action": "ROUTE_PAUSED"}

        elif cmd == "RESUME_ROUTE":
            self.route_paused = False
            return {"status": "OK", "action": "ROUTE_RESUMED"}

        return None

    def get_telemetry_extra(self) -> Dict[str, Any]:
        return {
            "brain_mode": "LEADER_FOLLOWER",
            "formation": self.formation_mode,
            "formation_heading_deg": round(math.degrees(self.formation_heading) % 360.0, 1),
        }
