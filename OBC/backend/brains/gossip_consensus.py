"""
SwarmTech3_UpRising: P2P Gossip Consensus Brain (gossip_consensus.py)
Per-drone autonomous OBC guidance plugin (Phase 4 Pure I/O Boundary).
Heartbeat 1: Solo Navigation, Onboard Failsafes, and Isolated Peer State Register.
"""

from __future__ import annotations
import math
from typing import Dict, List, Optional, Set, Tuple, Any
import numpy as np

from .base import (
    BaseBrain,
    RadioPacket,
    HeardPeer,
    LocalTelemetry,
    FlightIntent,
)
from .hungarian import hungarian_assignment

# Radio Comms & Asymmetric Link Budget (Canine Acoustic Taxonomy - TKT-49)
RANGE_HOWL: float = 140.0
RANGE_BARK: float = 90.0
RANGE_YIP: float = 50.0
RANGE_P2P: float = 60.0
RADIO_RANGE: float = RANGE_HOWL
HEARTBEAT_INTERVAL: float = 0.1
V_CRUISE: float = 12.0
V_LKP: float = 6.0
QUORUM_RATIO: float = 0.75

DEADBAND_RADIUS: float = 0.35
MAX_SLEW_RATE: float = math.radians(45.0)
FREEZE_SPEED: float = 0.5
FREEZE_DIST: float = 2.0


class DronePeerState:
    """Decentralized state tracked on an individual drone's onboard memory."""

    def __init__(self, drone_id: int, initial_target: np.ndarray):
        self.drone_id: int = drone_id
        self.active_target: np.ndarray = np.array(initial_target, dtype=float)
        self.active_seq: int = 1
        self.pending_target: np.ndarray = np.array(initial_target, dtype=float)
        self.pending_seq: int = 1
        self.echo_ack_peers: Set[int] = {drone_id}
        self.gossip_status: str = "COORDINATED"
        self.last_tx_time: float = 0.0


class GossipConsensusBrain(BaseBrain):
    """Per-drone Decentralized P2P Gossip Consensus OBC Brain.
    Operates strictly on LocalTelemetry and received RadioPackets with zero simulator references."""

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

    def reset(self) -> None:
        self.current_seq = 1
        self.peer_state = DronePeerState(self.drone_id, self._default_target)
        self.peer_states = {self.drone_id: self.peer_state}
        self.heard_peers.clear()
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

    def _get_peer_state(self, drone_id: Optional[int] = None) -> DronePeerState:
        did = self.drone_id if drone_id is None else int(drone_id)
        if did not in self.peer_states:
            self.peer_states[did] = DronePeerState(did, self._default_target)
        return self.peer_states[did]

    def step_drone(
        self,
        telemetry: LocalTelemetry,
        inbox: List[RadioPacket],
        dt: float,
    ) -> Tuple[FlightIntent, List[RadioPacket]]:
        """Heartbeat 1: Pure per-drone onboard guidance step (Solo Navigation & Failsafes)."""
        outbox: List[RadioPacket] = []
        updates: Dict[str, Any] = {}

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

        # 3. Backtrack, Horizon Hold, and Lost-Link Failsafes (TKT-54)
        if telemetry.flight_state == "BACKTRACK_HOLD" and telemetry.backtrack_target is not None:
            acc = self._steer_toward(telemetry.pos, telemetry.vel, telemetry.backtrack_target, max_speed=8.0)
            speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
            yaw = math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.5 else None
            return FlightIntent(
                target_acc=acc,
                target_yaw=yaw,
                target_station=np.copy(telemetry.backtrack_target),
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

        # 4. Solo Navigation toward Local Active Target (Heartbeat 1 Coma State)
        target_dest = np.copy(self.peer_state.active_target)
        updates["acknowledged_target"] = np.copy(target_dest)
        acc = self._steer_toward(telemetry.pos, telemetry.vel, target_dest, max_speed=V_CRUISE)
        speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
        yaw = math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.5 else float(telemetry.yaw)

        return FlightIntent(
            target_acc=acc,
            target_yaw=yaw,
            target_station=target_dest,
            state_updates=updates,
        ), outbox

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
