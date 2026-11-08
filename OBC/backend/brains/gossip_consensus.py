# ==============================================================================
# AlitheiaHive_v3-Uprising: Decentralized P2P Gossip Consensus Brain (OBC Bubble)
# Pure I/O Boundary (TLK-01 Heartbeat 3):
#   - Zero imports from simulator or ground station; zero simulator handle.
#   - Multi-hop P2P gossip echo propagation, 75% quorum barrier synchronization,
#     4D mission route progression, 3D Reynolds Boids, and geometric formations
#     computed strictly from LocalTelemetry and received 60m RadioPackets.
# ==============================================================================

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Set, Tuple
import numpy as np

from OBC.backend.brains.base import (
    BaseBrain,
    FlightIntent,
    HeardPeer,
    LocalTelemetry,
    RadioPacket,
)
from OBC.backend.brains.hungarian import hungarian_assignment

RANGE_HOWL: float = 140.0
RANGE_BARK: float = 90.0
RANGE_YIP: float = 50.0
RANGE_P2P: float = 60.0
RADIO_RANGE: float = RANGE_HOWL
HEARTBEAT_INTERVAL: float = 0.1
LINK_TIMEOUT: float = 1.0
V_CRUISE: float = 12.0
V_LKP: float = 6.0
QUORUM_RATIO: float = 0.75

DEADBAND_RADIUS: float = 0.35
MAX_SLEW_RATE: float = math.radians(45.0)
FREEZE_SPEED: float = 0.5
FREEZE_DIST: float = 2.0


class DronePeerState:
    """Decentralized state tracked on an individual drone's onboard memory."""

    def __init__(self, drone_id: int, initial_target: np.ndarray) -> None:
        self.drone_id: int = int(drone_id)
        self.active_target: np.ndarray = np.array(initial_target, dtype=float)
        self.active_seq: int = 1
        self.pending_target: np.ndarray = np.array(initial_target, dtype=float)
        self.pending_seq: int = 1
        self.echo_ack_peers: Set[int] = {int(drone_id)}
        self.gossip_status: str = "COORDINATED"
        self.last_tx_time: float = 0.0
        self.target_timestamp: float = 0.0
        self.propagating_timer: float = 0.0


class GossipConsensusBrain(BaseBrain):
    """Per-drone Decentralized P2P Gossip Consensus OBC Brain."""

    brain_name: str = "GOSSIP_CONSENSUS"

    def __init__(
        self,
        drone_id: int = 0,
        initial_target: Optional[np.ndarray] = None,
        formation_mode: str = "PERIMETER_PATROL",
    ) -> None:
        super().__init__(drone_id=drone_id)
        self._default_target: np.ndarray = (
            np.array(initial_target, dtype=float)
            if initial_target is not None
            else np.array([60.0, 40.0, 30.0], dtype=float)
        )
        self.current_seq: int = 1
        self.peer_state: DronePeerState = DronePeerState(self.drone_id, self._default_target)
        self.peer_states: Dict[int, DronePeerState] = {self.drone_id: self.peer_state}
        self.last_broadcast_tick: float = -1.0
        self.formation_mode: str = formation_mode
        self.formation_heading: float = 0.0
        self.heading_initialized: bool = False
        self.route_waypoints: List[np.ndarray] = []
        self.current_route_wp_idx: int = 0
        self.route_active: bool = False
        self.route_paused: bool = False
        self.route_loop: bool = True
        self.wp_loiter_timer: float = 0.0
        self.wp_total_loiter: float = 0.0
        self.wp_loitering: bool = False
        self.assigned_slots: Dict[int, int] = {}
        self.inversion_cooldown: float = 0.0
        self._last_clock_s: float = 0.0
        self.gcs_lost_timer: float = 0.0

    def reset(self) -> None:
        super().reset()
        self.current_seq = 1
        self.peer_state = DronePeerState(self.drone_id, self._default_target)
        self.peer_states = {self.drone_id: self.peer_state}
        self.last_broadcast_tick = -1.0
        self.formation_heading = 0.0
        self.heading_initialized = False
        self.assigned_slots.clear()
        self.inversion_cooldown = 0.0
        self.route_waypoints.clear()
        self.current_route_wp_idx = 0
        self.route_active = False
        self.route_paused = False
        self.route_loop = True
        self.wp_loiter_timer = 0.0
        self.wp_total_loiter = 0.0
        self.wp_loitering = False
        self._last_clock_s = 0.0
        self.gcs_lost_timer = 0.0

    def _get_peer_state(self, drone_id: Optional[int] = None) -> DronePeerState:
        did = self.drone_id if drone_id is None else int(drone_id)
        if did not in self.peer_states:
            self.peer_states[did] = DronePeerState(did, self._default_target)
        return self.peer_states[did]

    # ==========================================================================
    # 1. P2P Mesh Packet Emission & Gossip Quorum Ingestion
    # ==========================================================================
    def emit_gossip_packet(self, telemetry: LocalTelemetry) -> RadioPacket:
        """Build a 10 Hz GOSSIP_ECHO packet from local telemetry and peer state."""
        pos_list = None if np.isnan(telemetry.pos).any() else [float(v) for v in telemetry.pos]
        mesh_summary: Dict[int, Dict[str, Any]] = {}
        for pid, hp in self.heard_peers.items():
            if not hp.crashed and hp.enrolled and not np.isnan(hp.pos).any():
                mesh_summary[int(pid)] = {
                    "pos": [float(v) for v in hp.pos],
                    "vel": [float(v) for v in hp.vel],
                    "yaw": float(hp.yaw),
                    "enrolled": bool(hp.enrolled),
                    "flight_state": str(hp.flight_state),
                    "last_heard": float(hp.last_heard_clock),
                    "gcs_contact": bool(hp.gcs_contact),
                }
        if pos_list is not None and telemetry.enrolled:
            mesh_summary[int(telemetry.drone_id)] = {
                "pos": pos_list,
                "vel": [float(v) for v in telemetry.vel],
                "yaw": float(telemetry.yaw),
                "enrolled": bool(telemetry.enrolled),
                "flight_state": str(telemetry.flight_state),
                "last_heard": float(telemetry.clock_s),
                "gcs_contact": bool(telemetry.uplink_active),
            }

        return RadioPacket(
            sender_id=telemetry.drone_id,
            msg_type="GOSSIP_ECHO",
            timestamp=float(telemetry.clock_s),
            seq=int(self.peer_state.pending_seq),
            payload={
                "id": int(telemetry.drone_id),
                "pos": pos_list,
                "vel": [float(v) for v in telemetry.vel],
                "yaw": float(telemetry.yaw),
                "enrolled": bool(telemetry.enrolled),
                "flight_state": str(telemetry.flight_state),
                "active_seq": int(self.peer_state.active_seq),
                "echo_seq": int(self.peer_state.pending_seq),
                "echo_target": [float(v) for v in self.peer_state.pending_target],
                "echo_ack_peers": list(self.peer_state.echo_ack_peers),
                "target_timestamp": float(getattr(self.peer_state, "target_timestamp", telemetry.clock_s)),
                "gcs_contact": bool(telemetry.uplink_active),
                "mesh_summary": mesh_summary,
            },
        )

    def ingest_gossip_packets(
        self,
        telemetry: LocalTelemetry,
        inbox: List[RadioPacket],
        living_peer_ids: Optional[Set[int]] = None,
        dt: float = 0.016,
    ) -> None:
        """Process incoming GOSSIP_ECHO and GCS_UPLINK packets and evaluate 75% quorum barrier."""
        self._last_clock_s = float(telemetry.clock_s)
        own_pos = (
            np.copy(telemetry.pos)
            if not np.isnan(telemetry.pos).any()
            else np.copy(telemetry.lkp)
        )
        is_crashed = telemetry.flight_state == "CRASHED" or telemetry.status == "CRASHED"
        if not telemetry.gnss_denied and telemetry.enrolled and not is_crashed:
            self.heard_peers[telemetry.drone_id] = HeardPeer(
                drone_id=telemetry.drone_id,
                pos=own_pos,
                vel=np.copy(telemetry.vel),
                yaw=float(telemetry.yaw),
                enrolled=bool(telemetry.enrolled),
                flight_state=str(telemetry.flight_state),
                last_heard_clock=float(telemetry.clock_s),
                active_seq=self.peer_state.active_seq,
                echo_seq=self.peer_state.pending_seq,
                echo_target=np.copy(self.peer_state.pending_target),
                gossip_status=self.peer_state.gossip_status,
                gcs_contact=bool(telemetry.uplink_active),
            )

        for pkt in inbox:
            p = pkt.payload
            if pkt.msg_type in ("GOSSIP_ECHO", "PEER_BEACON"):
                if p.get("crashed", False):
                    self.heard_peers.pop(int(pkt.sender_id), None)
                    self.assigned_slots.pop(int(pkt.sender_id), None)
                    continue
                pos_raw = p.get("pos")
                if pos_raw is not None:
                    pos_arr = np.array(pos_raw, dtype=float)
                    if not np.isnan(pos_arr).any():
                        self.heard_peers[pkt.sender_id] = HeardPeer(
                            drone_id=int(pkt.sender_id),
                            pos=pos_arr,
                            vel=np.array(p.get("vel", [0.0, 0.0, 0.0]), dtype=float),
                            yaw=float(p.get("yaw", 0.0)),
                            enrolled=bool(p.get("enrolled", True)),
                            flight_state=str(p.get("flight_state", "CRUISE")),
                            last_heard_clock=float(telemetry.clock_s),
                            active_seq=int(p.get("active_seq", 1)),
                            echo_seq=int(p.get("echo_seq", 1)),
                            echo_target=np.array(p.get("echo_target", self.peer_state.pending_target), dtype=float),
                            gcs_contact=bool(p.get("gcs_contact", False)),
                        )
                mesh_sum = p.get("mesh_summary")
                if isinstance(mesh_sum, dict):
                    for m_id_str, m_info in mesh_sum.items():
                        mid = int(m_id_str)
                        if mid == telemetry.drone_id:
                            continue
                        m_pos = m_info.get("pos")
                        if m_pos is not None:
                            m_last_heard = float(m_info.get("last_heard", telemetry.clock_s))
                            if mid in self.heard_peers:
                                effective_clock = max(self.heard_peers[mid].last_heard_clock, m_last_heard)
                            else:
                                effective_clock = m_last_heard
                            self.heard_peers[mid] = HeardPeer(
                                drone_id=mid,
                                pos=np.array(m_pos, dtype=float),
                                vel=np.array(m_info.get("vel", [0.0, 0.0, 0.0]), dtype=float),
                                yaw=float(m_info.get("yaw", 0.0)),
                                enrolled=bool(m_info.get("enrolled", True)),
                                flight_state=str(m_info.get("flight_state", "CRUISE")),
                                last_heard_clock=effective_clock,
                                gcs_contact=bool(m_info.get("gcs_contact", False)),
                            )

            if pkt.msg_type == "GOSSIP_ECHO":
                peer_echo_seq = int(p.get("echo_seq", 1))
                peer_echo_target = np.array(p.get("echo_target", self.peer_state.pending_target), dtype=float)
                peer_acks = {int(a) for a in p.get("echo_ack_peers", [])}
                peer_target_ts = float(p.get("target_timestamp", telemetry.clock_s))

                if peer_echo_seq > self.peer_state.pending_seq:
                    self.peer_state.pending_seq = peer_echo_seq
                    self.peer_state.pending_target = np.copy(peer_echo_target)
                    self.peer_state.target_timestamp = peer_target_ts
                    self.peer_state.propagating_timer = 0.0
                    self.peer_state.echo_ack_peers = {telemetry.drone_id, int(pkt.sender_id)} | peer_acks
                    self.peer_state.gossip_status = "PROPAGATING"
                elif peer_echo_seq == self.peer_state.pending_seq:
                    target_diff = float(np.linalg.norm(peer_echo_target - self.peer_state.pending_target))
                    if target_diff < 0.1:
                        self.peer_state.echo_ack_peers.add(int(pkt.sender_id))
                        self.peer_state.echo_ack_peers.update(peer_acks)
                    else:
                        # Tie-breaking for equal seq with divergent targets:
                        # Newer target_timestamp wins. If tied within 1e-4, lower drone/sender ID wins.
                        my_ts = float(getattr(self.peer_state, "target_timestamp", 0.0))
                        peer_wins = False
                        if peer_target_ts > my_ts + 1e-4:
                            peer_wins = True
                        elif abs(peer_target_ts - my_ts) <= 1e-4:
                            if int(pkt.sender_id) < telemetry.drone_id:
                                peer_wins = True

                        if peer_wins:
                            self.peer_state.pending_target = np.copy(peer_echo_target)
                            self.peer_state.target_timestamp = peer_target_ts
                            self.peer_state.propagating_timer = 0.0
                            self.peer_state.echo_ack_peers = {telemetry.drone_id, int(pkt.sender_id)} | peer_acks
                            self.peer_state.gossip_status = "PROPAGATING"

            elif pkt.msg_type in ("GCS_UPLINK", "C2_COMMAND"):
                if "target" in p:
                    t_arr = np.array(p["target"], dtype=float)
                    seq = int(p.get("seq", self.current_seq + 1))
                    self.current_seq = max(self.current_seq, seq)
                    self.peer_state.pending_seq = seq
                    self.peer_state.pending_target = np.copy(t_arr)
                    self.peer_state.target_timestamp = float(telemetry.clock_s)
                    self.peer_state.propagating_timer = 0.0
                    self.peer_state.active_seq = seq
                    self.peer_state.active_target = np.copy(t_arr)
                    self.peer_state.echo_ack_peers = {telemetry.drone_id}
                    self.peer_state.gossip_status = "COORDINATED"
                self.handle_command(dict(p))

        # Prune stale peers
        stale = [
            pid for pid, hp in self.heard_peers.items()
            if pid != telemetry.drone_id and (telemetry.clock_s - hp.last_heard_clock) > LINK_TIMEOUT
        ]
        for pid in stale:
            self.heard_peers.pop(pid, None)

        # Evaluate 75% Quorum Barrier Synchronization
        if living_peer_ids is None:
            living_peer_ids = {
                pid for pid, hp in self.heard_peers.items()
                if not hp.crashed and hp.enrolled and hp.flight_state not in ("LANDED", "DESCENDING")
            }
            living_peer_ids.add(telemetry.drone_id)

        living_count = max(1, len(living_peer_ids))
        acks_in_range = self.peer_state.echo_ack_peers.intersection(living_peer_ids)
        ack_ratio = len(acks_in_range) / float(living_count)

        if self.peer_state.pending_seq > self.peer_state.active_seq:
            self.peer_state.propagating_timer += dt
            can_commit = (ack_ratio >= QUORUM_RATIO) or (
                self.peer_state.propagating_timer >= 2.0 and len(acks_in_range) >= living_count
            )
            if can_commit:
                self.peer_state.active_seq = self.peer_state.pending_seq
                self.peer_state.active_target = np.copy(self.peer_state.pending_target)
                self.peer_state.gossip_status = "COORDINATED"
                self.peer_state.propagating_timer = 0.0
                self.current_seq = max(self.current_seq, self.peer_state.active_seq)

    # ==========================================================================
    # 2. Per-Drone Decentralized Guidance Step (Pure I/O Boundary)
    # ==========================================================================
    def step_drone(
        self,
        telemetry: LocalTelemetry,
        inbox: List[RadioPacket],
        dt: float,
    ) -> Tuple[FlightIntent, List[RadioPacket]]:
        """Execute one onboard guidance cycle from LocalTelemetry and received RadioPackets."""
        self._last_clock_s = float(telemetry.clock_s)
        outbox: List[RadioPacket] = []
        updates: Dict[str, Any] = {}

        self.ingest_gossip_packets(telemetry, inbox, dt=dt)

        if telemetry.flight_state in ("LANDED", "DESCENDING", "GROUND_IDLE", "CRASHED"):
            return FlightIntent(target_acc=np.zeros(3, dtype=float)), outbox

        # 1. Tactical GNSS Denial & Dead-Reckoning RTB Override (TKT-52)
        if telemetry.gnss_denied:
            intent = self._compute_gnss_denied_intent(telemetry, dt)
            return intent, outbox

        # 2. Unenrolled Solo Scout Guidance (TKT-50)
        if not telemetry.enrolled:
            if telemetry.lost_link:
                acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.lkp, max_speed=V_LKP)
                station = np.copy(telemetry.lkp)
            elif telemetry.rejoin_target is not None:
                acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.rejoin_target, max_speed=V_CRUISE)
                station = np.copy(telemetry.rejoin_target)
            else:
                acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.pos, max_speed=0.0)
                station = np.copy(telemetry.pos)
            speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
            yaw = math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.5 else None
            return FlightIntent(
                target_acc=acc,
                target_yaw=yaw,
                target_station=station,
                state_updates=updates,
            ), outbox

        # 3. Quorum GCS Contact Bitmask & Corridor ERTB (BUG-06)
        any_gcs = any(peer.gcs_contact for peer in self.heard_peers.values())
        if any_gcs:
            self.gcs_lost_timer = 0.0
            if telemetry.flight_state == "CORRIDOR_ERTB":
                updates["flight_state"] = "PERIMETER_HOLD"
                updates["status"] = "PERIMETER_HOLD"
        else:
            self.gcs_lost_timer += dt
            if self.gcs_lost_timer >= 2.0 and telemetry.flight_state not in ("CORRIDOR_ERTB", "BACKTRACK_HOLD", "PERIMETER_HOLD", "LANDED", "GROUND_IDLE", "CRASHED", "DESCENDING", "COURIER_TRANSIT", "RENDEZVOUS_DWELL", "ESCORT_LEAD", "ESCORT_REJOIN"):
                updates["flight_state"] = "CORRIDOR_ERTB"
                updates["status"] = "CORRIDOR_ERTB"

        if telemetry.flight_state == "CORRIDOR_ERTB" or updates.get("flight_state") == "CORRIDOR_ERTB":
            ct = telemetry.corridor_target if telemetry.corridor_target is not None else np.array([0.0, 0.0, 25.0], dtype=float)
            acc = self._steer_toward(telemetry.pos, telemetry.vel, ct, max_speed=8.0)
            speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
            yaw = math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.5 else None
            return FlightIntent(
                target_acc=acc,
                target_yaw=yaw,
                target_station=np.copy(ct),
                state_updates=updates,
            ), outbox

        # 4. Backtrack, Horizon Hold, and Lost-Link Failsafes (TKT-54)
        if telemetry.flight_state == "BACKTRACK_HOLD":
            bt = telemetry.backtrack_target if telemetry.backtrack_target is not None else telemetry.lkp
            acc = self._steer_toward(telemetry.pos, telemetry.vel, bt, max_speed=8.0)
            speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
            yaw = math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.5 else None
            return FlightIntent(
                target_acc=acc,
                target_yaw=yaw,
                target_station=np.copy(bt),
                state_updates=updates,
            ), outbox

        if telemetry.flight_state == "PERIMETER_HOLD" or telemetry.holding_at_horizon:
            acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.acknowledged_target, max_speed=2.0)
            speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
            yaw = math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.5 else None
            return FlightIntent(
                target_acc=acc,
                target_yaw=yaw,
                target_station=np.copy(telemetry.acknowledged_target),
                state_updates=updates,
            ), outbox

        # Courier S&R Escort States (TKT-51)
        if telemetry.flight_state in ("COURIER_TRANSIT", "RENDEZVOUS_DWELL", "ESCORT_LEAD", "ESCORT_REJOIN"):
            ack_t = np.copy(updates.get("acknowledged_target", telemetry.acknowledged_target))
            spd = 0.0 if telemetry.flight_state == "RENDEZVOUS_DWELL" else 10.0
            acc = self._steer_toward(telemetry.pos, telemetry.vel, ack_t, max_speed=spd)
            speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
            yaw = math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.5 else None
            return FlightIntent(
                target_acc=acc,
                target_yaw=yaw,
                target_station=ack_t,
                state_updates=updates,
            ), outbox

        if telemetry.lost_link:
            acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.lkp, max_speed=V_LKP)
            speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
            yaw = math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.5 else None
            return FlightIntent(
                target_acc=acc,
                target_yaw=yaw,
                target_station=np.copy(telemetry.lkp),
                state_updates=updates,
            ), outbox

        # Ensure self is up to date in heard_peers
        self.heard_peers[telemetry.drone_id] = HeardPeer(
            drone_id=telemetry.drone_id,
            pos=np.copy(telemetry.pos),
            vel=np.copy(telemetry.vel),
            yaw=float(telemetry.yaw),
            enrolled=bool(telemetry.enrolled),
            flight_state=str(telemetry.flight_state),
            last_heard_clock=float(telemetry.clock_s),
        )

        active_peers = sorted(
            [
                hp for hp in self.heard_peers.values()
                if not hp.crashed and hp.enrolled and hp.flight_state not in ("LANDED", "DESCENDING")
            ],
            key=lambda p: p.drone_id,
        )
        living_count = max(1, len(active_peers))
        sorted_ids = [p.drone_id for p in active_peers]
        active_ids = set(sorted_ids)
        for did in list(self.assigned_slots.keys()):
            if did not in active_ids:
                del self.assigned_slots[did]

        is_paused = bool(self.route_paused or telemetry.mission_paused)

        # 4. 4D Mission Route Waypoint Advancement for Gossip Quorum (TKT-28, TKT-30, TKT-56)
        if self.route_active and self.route_waypoints:
            wps = self.route_waypoints
            idx = self.current_route_wp_idx
            if 0 <= idx < len(wps):
                raw_wp = wps[idx]
                target_wp = np.array(raw_wp[:3], dtype=float)
                loiter_duration = float(raw_wp[3]) if len(raw_wp) > 3 else 0.0
                centroid = np.mean([p.pos for p in active_peers], axis=0)
                dist_centroid = float(np.linalg.norm(centroid - target_wp))
                min_peer_dist = min(float(np.linalg.norm(p.pos - target_wp)) for p in active_peers)
                arrival_dist = min(dist_centroid, min_peer_dist)

                updates["wp_total_loiter"] = loiter_duration
                updates["wp_loiter_timer"] = self.wp_loiter_timer
                updates["wp_loitering"] = self.wp_loitering
                updates["mission_paused"] = is_paused

                if not is_paused and arrival_dist <= telemetry.route_acceptance_radius:
                    peers_settled = (
                        all(
                            float(
                                np.linalg.norm(
                                    p.pos
                                    - self._get_formation_slot(
                                        p,
                                        self.assigned_slots.get(p.drone_id, sorted_ids.index(p.drone_id)),
                                        living_count,
                                        target_wp,
                                        overhead_marshal=telemetry.overhead_marshal_active,
                                        clock_s=telemetry.clock_s,
                                    )
                                )
                            )
                            <= 4.5
                            for p in active_peers
                        )
                        if living_count > 1
                        else True
                    )

                    if not self.wp_loitering and loiter_duration > 0.0:
                        self.wp_loitering = True
                        self.wp_loiter_timer = max(0.0, loiter_duration - dt)
                        self.wp_total_loiter = loiter_duration
                        updates["wp_loitering"] = True
                        updates["wp_loiter_timer"] = self.wp_loiter_timer
                        updates["wp_total_loiter"] = loiter_duration
                    elif self.wp_loitering:
                        self.wp_loiter_timer = max(0.0, self.wp_loiter_timer - dt)
                        updates["wp_loiter_timer"] = self.wp_loiter_timer
                        if self.wp_loiter_timer <= 0.0 and peers_settled:
                            self._advance_route_waypoint(target_wp, updates)
                    else:
                        self._advance_route_waypoint(target_wp, updates)

        # 5. Update Virtual Swarm Heading toward Consensus Target
        consensus_target = (
            np.copy(self.peer_state.pending_target)
            if not self.heading_initialized and self.peer_state.pending_seq > self.peer_state.active_seq
            else np.copy(self.peer_state.active_target)
        )
        self._update_formation_heading(
            active_peers=active_peers,
            target_pos=consensus_target,
            dt=dt,
            patrol_active=telemetry.patrol_active,
            overhead_marshal=telemetry.overhead_marshal_active,
        )

        # 6. Compute Formation Slot or 3D Boids Acceleration
        active_target = np.copy(self.peer_state.active_target)
        updates["acknowledged_target"] = np.copy(active_target)

        if self.formation_mode == "NATURAL_FLOCK":
            station = np.copy(active_target)
            acc = self._compute_boid_acceleration_from_peers(telemetry, active_peers, active_target)
        else:
            my_idx = sorted_ids.index(telemetry.drone_id) if telemetry.drone_id in sorted_ids else 0
            if telemetry.overhead_marshal_active:
                k = my_idx
            else:
                k = self.assigned_slots.get(telemetry.drone_id, my_idx)
            slot_pos = self._get_formation_slot(
                telemetry,
                k,
                living_count,
                active_target,
                overhead_marshal=telemetry.overhead_marshal_active,
                clock_s=telemetry.clock_s,
            )
            station = np.copy(slot_pos)

            target_speed = V_CRUISE
            if is_paused or self.wp_loitering:
                target_speed = 2.0
            elif telemetry.enrollment_timer > 0.0:
                new_enroll = max(0.0, telemetry.enrollment_timer - dt)
                updates["enrollment_timer"] = new_enroll
                ramp = max(0.0, min(1.0, 1.0 - (new_enroll / 2.5)))
                target_speed = 7.0 + (V_CRUISE - 7.0) * ramp

            target_acc = self._steer_toward(telemetry.pos, telemetry.vel, slot_pos, max_speed=target_speed)

            flock_acc = np.zeros(3, dtype=float)
            neighbor_count = 0
            for peer in active_peers:
                if peer.drone_id == telemetry.drone_id:
                    continue
                d = float(np.linalg.norm(telemetry.pos - peer.pos))
                if d <= 15.0:
                    flock_acc += (peer.vel - telemetry.vel) * 0.3
                    neighbor_count += 1
            if neighbor_count > 0:
                flock_acc /= float(neighbor_count)

            acc = target_acc + flock_acc

        # 7. Tactical Formation Orientations & Headings (TKT-55)
        target_yaw = self._compute_tactical_yaw(telemetry, station, active_target)

        return FlightIntent(
            target_acc=acc,
            target_yaw=target_yaw,
            target_station=station,
            state_updates=updates,
        ), outbox

    def _advance_route_waypoint(self, current_target_wp: np.ndarray, updates: Dict[str, Any]) -> None:
        """Advance local 4D waypoint index and consensus target."""
        wps = self.route_waypoints
        idx = self.current_route_wp_idx
        self.wp_loitering = False
        self.wp_loiter_timer = 0.0
        if idx + 1 < len(wps):
            self.current_route_wp_idx += 1
            next_wp = np.array(wps[self.current_route_wp_idx][:3], dtype=float)
            next_loiter = float(wps[self.current_route_wp_idx][3]) if len(wps[self.current_route_wp_idx]) > 3 else 0.0
        elif self.route_loop:
            self.current_route_wp_idx = 0
            next_wp = np.array(wps[0][:3], dtype=float)
            next_loiter = float(wps[0][3]) if len(wps[0]) > 3 else 0.0
        else:
            self.route_active = False
            next_wp = current_target_wp
            next_loiter = 0.0

        self.wp_total_loiter = next_loiter
        if self.route_active:
            self.current_seq += 1
            self.peer_state.active_seq = self.current_seq
            self.peer_state.pending_seq = self.current_seq
            self.peer_state.active_target = np.copy(next_wp)
            self.peer_state.pending_target = np.copy(next_wp)
            updates["current_wp_idx"] = self.current_route_wp_idx
            updates["acknowledged_target"] = np.copy(next_wp)
            updates["wp_loitering"] = False
            updates["wp_loiter_timer"] = 0.0
            updates["wp_total_loiter"] = next_loiter

    def _compute_tactical_yaw(
        self,
        telemetry: LocalTelemetry,
        station: np.ndarray,
        center: np.ndarray,
    ) -> float:
        """Compute desired airframe yaw orientation (TKT-55)."""
        speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
        if self.formation_mode in ("PERIMETER_PATROL", "PERIMETER_IN", "PERIMETER_OUT"):
            r_xy = math.hypot(telemetry.pos[0] - center[0], telemetry.pos[1] - center[1])
            if self.formation_mode == "PERIMETER_OUT":
                psi_target = (
                    math.atan2(telemetry.pos[1] - center[1], telemetry.pos[0] - center[0])
                    if r_xy > 0.5
                    else self.formation_heading
                )
            else:
                psi_target = (
                    math.atan2(center[1] - telemetry.pos[1], center[0] - telemetry.pos[0])
                    if r_xy > 0.5
                    else self.formation_heading
                )
            dist_to_slot = float(np.linalg.norm(telemetry.pos - station))
            if dist_to_slot > 6.0 and speed_xy > 2.0:
                return math.atan2(telemetry.vel[1], telemetry.vel[0])
            return psi_target

        if self.formation_mode == "ECHELON_ESCORT":
            return float(self.formation_heading)

        if self.formation_mode == "ORBIT_VORTEX":
            return math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.5 else float(self.formation_heading)

        return math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.2 else float(telemetry.yaw)

    # ==========================================================================
    # 3. Decentralized Formation Geometries, Heading & Boids
    # ==========================================================================
    def _update_formation_heading(
        self,
        active_peers: List[Any],
        target_pos: np.ndarray,
        dt: float,
        patrol_active: bool = False,
        overhead_marshal: bool = False,
    ) -> None:
        """Update virtual swarm heading vector toward consensus target with smooth slew."""
        if not active_peers:
            return

        mean_vel = np.mean([np.array(getattr(p, "vel"), dtype=float) for p in active_peers], axis=0)
        speed_xy = float(math.hypot(mean_vel[0], mean_vel[1]))

        sorted_peers = sorted(
            active_peers,
            key=lambda p: int(getattr(p, "drone_id", getattr(p, "id", 0))),
        )
        apex_drone = sorted_peers[0]
        apex_pos = np.array(getattr(apex_drone, "pos"), dtype=float)
        to_target = target_pos - apex_pos
        dist_to_target = float(math.hypot(to_target[0], to_target[1]))

        if not self.heading_initialized:
            if speed_xy >= FREEZE_SPEED:
                self.formation_heading = math.atan2(mean_vel[1], mean_vel[0])
            else:
                if dist_to_target > 1.0:
                    self.formation_heading = math.atan2(to_target[1], to_target[0])
                else:
                    self.formation_heading = float(getattr(apex_drone, "yaw", 0.0))
            self.heading_initialized = True
            return

        self.inversion_cooldown = max(0.0, self.inversion_cooldown - dt)

        if (
            dist_to_target > 5.0
            and self.inversion_cooldown <= 0.0
            and self.formation_mode == "ECHELON_ESCORT"
            and not overhead_marshal
        ):
            to_tgt_dir = np.array([to_target[0] / dist_to_target, to_target[1] / dist_to_target])
            h_dir = np.array([math.cos(self.formation_heading), math.sin(self.formation_heading)])
            if float(np.dot(to_tgt_dir, h_dir)) < -0.5:
                rev_heading = math.atan2(to_target[1], to_target[0])
                if self.invert_formation_slots(active_peers, target_pos, rev_heading):
                    self.formation_heading = rev_heading
                    return

        if not patrol_active and (speed_xy < FREEZE_SPEED or dist_to_target < FREEZE_DIST):
            return

        if speed_xy >= FREEZE_SPEED:
            desired_yaw = math.atan2(mean_vel[1], mean_vel[0])
        elif dist_to_target > 1.0:
            desired_yaw = math.atan2(to_target[1], to_target[0])
        else:
            return

        yaw_diff = (desired_yaw - self.formation_heading + math.pi) % (2.0 * math.pi) - math.pi

        if (
            abs(yaw_diff) > math.radians(120.0)
            and self.inversion_cooldown <= 0.0
            and self.formation_mode == "ECHELON_ESCORT"
        ):
            if self.invert_formation_slots(active_peers, target_pos, desired_yaw):
                self.formation_heading = desired_yaw
                return

        effective_slew_rate = math.radians(60.0) if patrol_active else MAX_SLEW_RATE
        max_step = effective_slew_rate * dt
        slew = float(np.clip(yaw_diff, -max_step, max_step))
        self.formation_heading = (self.formation_heading + slew + math.pi) % (2.0 * math.pi) - math.pi

    def invert_formation_slots(
        self, active_peers: List[Any], target_pos: np.ndarray, target_heading: float
    ) -> bool:
        """Perform Hungarian minimal-displacement slot re-assignment on heading reversal (TKT-43)."""
        n = len(active_peers)
        if n <= 1:
            return False

        heading_vec = np.array([math.cos(target_heading), math.sin(target_heading), 0.0], dtype=float)
        lateral_vec = np.array([-heading_vec[1], heading_vec[0], 0.0], dtype=float)

        candidate_slots = []
        for k in range(n):
            if k == 0:
                s_pos = np.copy(target_pos)
            else:
                row = (k + 1) // 2
                side = 1.0 if (k % 2 == 1) else -1.0
                s_pos = (
                    target_pos
                    - (row * 6.0 * heading_vec)
                    + (side * row * 6.0 * lateral_vec)
                    + np.array([0.0, 0.0, row * 0.5], dtype=float)
                )
            candidate_slots.append(s_pos)

        cost_matrix = np.zeros((n, n), dtype=float)
        for i, peer in enumerate(active_peers):
            p_pos = np.array(getattr(peer, "pos"), dtype=float)
            for j, s_pos in enumerate(candidate_slots):
                diff = p_pos - s_pos
                cost_matrix[i, j] = float(np.dot(diff, diff))

        row_ind, col_ind = hungarian_assignment(cost_matrix)
        for r, c in zip(row_ind, col_ind):
            pid = int(getattr(active_peers[r], "drone_id", getattr(active_peers[r], "id", r)))
            self.assigned_slots[pid] = c

        self.inversion_cooldown = 2.5
        return True

    def _get_formation_slot(
        self,
        drone: Any,
        k: int,
        N: int,
        target_pos: np.ndarray,
        overhead_marshal: bool = False,
        clock_s: Optional[float] = None,
    ) -> np.ndarray:
        """Compute 3D position of decentralized geometric slot k in [0, N-1] (TKT-11)."""
        target_arr = np.array(target_pos, dtype=float)
        if k == 0:
            azimuth = 0.0
        else:
            side = 1.0 if (k % 2 == 1) else -1.0
            pair = (k + 1) // 2
            azimuth = side * pair * (2.0 * math.pi / max(1, N))

        if overhead_marshal:
            radius = 20.0
            return np.array([radius * math.cos(azimuth), radius * math.sin(azimuth), 25.0], dtype=float)

        if self.formation_mode in ("PERIMETER_PATROL", "PERIMETER_IN", "PERIMETER_OUT"):
            radius = 25.0
            return (
                target_arr
                + np.array([radius * math.cos(azimuth), radius * math.sin(azimuth), 0.0], dtype=float)
            )

        elif self.formation_mode == "ORBIT_VORTEX":
            radius = 20.0
            omega = 0.6
            t_clock = self._last_clock_s if clock_s is None else float(clock_s)
            angle = omega * t_clock + azimuth
            z_offset = 3.0 * math.sin(2.0 * angle)
            return (
                target_arr
                + np.array([radius * math.cos(angle), radius * math.sin(angle), z_offset], dtype=float)
            )

        elif self.formation_mode == "ECHELON_ESCORT":
            heading = np.array(
                [math.cos(self.formation_heading), math.sin(self.formation_heading), 0.0], dtype=float
            )
            lateral = np.array([-heading[1], heading[0], 0.0], dtype=float)

            if k == 0:
                return target_arr

            apex_pos = target_arr
            if self.assigned_slots and self.heard_peers:
                for pid, hp in self.heard_peers.items():
                    if not hp.crashed and self.assigned_slots.get(pid) == 0:
                        apex_pos = np.copy(hp.pos)
                        break

            row = (k + 1) // 2
            side = 1.0 if (k % 2 == 1) else -1.0
            return (
                apex_pos
                - (row * 6.0 * heading)
                + (side * row * 6.0 * lateral)
                + np.array([0.0, 0.0, row * 0.5], dtype=float)
            )

        return target_arr

    def _compute_boid_acceleration_from_peers(
        self,
        telemetry: LocalTelemetry,
        active_peers: List[HeardPeer],
        consensus_target: np.ndarray,
    ) -> np.ndarray:
        """Compute decentralized 3D Reynolds boid flocking acceleration from heard peers (TKT-23)."""
        a_goal = self._steer_toward(
            telemetry.pos, telemetry.vel, consensus_target, max_speed=V_CRUISE, arrival_radius=8.0
        )

        a_sep = np.zeros(3, dtype=float)
        R_SEP = 5.0
        for peer in active_peers:
            if peer.drone_id == telemetry.drone_id:
                continue
            diff = telemetry.pos - peer.pos
            d = float(np.linalg.norm(diff))
            if 0.001 < d < R_SEP:
                rep_strength = (R_SEP - d) / (d + 0.1)
                a_sep += (diff / d) * rep_strength * 4.0

                d_xy = float(math.hypot(diff[0], diff[1]))
                if d_xy < 3.5:
                    z_sign = 1.0 if diff[2] >= 0.0 else -1.0
                    a_sep[2] += z_sign * (3.5 - d_xy) * 3.0

                if d < 3.5:
                    v_rel = telemetry.vel - peer.vel
                    closing = float(np.dot(v_rel, diff / d))
                    if closing < 0:
                        a_sep += -(diff / d) * closing * 2.5

        a_align = np.zeros(3, dtype=float)
        R_ALIGN = 14.0
        align_weight_sum = 0.0
        vel_sum = np.zeros(3, dtype=float)
        for peer in active_peers:
            if peer.drone_id == telemetry.drone_id:
                continue
            d = float(np.linalg.norm(telemetry.pos - peer.pos))
            if d < R_ALIGN:
                w = 1.0 - (d / R_ALIGN)
                vel_sum += peer.vel * w
                align_weight_sum += w

        if align_weight_sum > 0:
            mean_vel = vel_sum / align_weight_sum
            a_align = (mean_vel - telemetry.vel) * 1.5

        a_coh = np.zeros(3, dtype=float)
        R_COH = 20.0
        pos_sum = np.zeros(3, dtype=float)
        coh_count = 0
        for peer in active_peers:
            if peer.drone_id == telemetry.drone_id:
                continue
            d = float(np.linalg.norm(telemetry.pos - peer.pos))
            if d < R_COH:
                pos_sum += peer.pos
                coh_count += 1

        if coh_count > 0:
            centroid = pos_sum / float(coh_count)
            offset_cent = centroid - telemetry.pos
            d_cent = float(np.linalg.norm(offset_cent))
            if d_cent > 3.0:
                a_coh = (offset_cent / d_cent) * min(V_CRUISE, d_cent * 1.2) - telemetry.vel
                a_coh *= 0.8

        a_z_guard = 0.0
        if telemetry.pos[2] < 12.0:
            a_z_guard += max(0.0, (14.0 - telemetry.pos[2]) * 4.0 - telemetry.vel[2] * 2.0)
        elif telemetry.pos[2] > 40.0:
            a_z_guard -= max(0.0, (telemetry.pos[2] - 38.0) * 4.0 + telemetry.vel[2] * 2.0)

        total_acc = (
            2.0 * a_goal
            + 3.5 * a_sep
            + 1.5 * a_align
            + 1.0 * a_coh
        )
        total_acc[2] += a_z_guard

        norm_a = float(np.linalg.norm(total_acc))
        A_MAX = 15.0
        if norm_a > A_MAX:
            total_acc = (total_acc / norm_a) * A_MAX

        return total_acc

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
            mode = cmd_dict.get("mode", "PERIMETER_PATROL")
            mode_upper = mode.upper().replace("-", "_").strip()
            if mode_upper in ("BOIDS", "BOID", "FLOCK"):
                mode_upper = "NATURAL_FLOCK"
            elif mode_upper in ("PERIMETER_PATROL_IN", "PERIMETER_IN"):
                mode_upper = "PERIMETER_IN"
            elif mode_upper in ("PERIMETER_PATROL_OUT", "PERIMETER_OUT"):
                mode_upper = "PERIMETER_OUT"
            elif mode_upper == "PERIMETER":
                mode_upper = "PERIMETER_PATROL"

            valid = ("PERIMETER_PATROL", "PERIMETER_IN", "PERIMETER_OUT", "ECHELON_ESCORT", "ORBIT_VORTEX", "NATURAL_FLOCK")
            if mode_upper in valid:
                self.formation_mode = mode_upper
                return {"status": "OK", "cmd": cmd, "formation": self.formation_mode}
        elif cmd == "PAUSE_ROUTE":
            self.route_paused = True
            return {"status": "OK", "action": "ROUTE_PAUSED"}
        elif cmd == "RESUME_ROUTE":
            self.route_paused = False
            return {"status": "OK", "action": "ROUTE_RESUMED"}
        elif cmd in ("ABORT_ROUTE", "STOP_ROUTE"):
            self.route_active = False
            self.route_paused = False
            self.wp_loitering = False
            self.wp_loiter_timer = 0.0
            return {"status": "OK", "action": "ROUTE_ABORTED"}
        return None

    def get_telemetry_extra(self) -> Dict[str, Any]:
        return {
            "brain_mode": "GOSSIP_CONSENSUS",
            "formation": self.formation_mode,
            "formation_heading_deg": round(math.degrees(self.formation_heading) % 360.0, 1),
            "target_seq": self.current_seq,
            "propagating_drones": 1 if self.peer_state.gossip_status == "PROPAGATING" else 0,
            "consensus_reached": (self.peer_state.gossip_status != "PROPAGATING"),
        }
