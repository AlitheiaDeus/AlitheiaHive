"""
SwarmTech3_UpRising: Base Guidance Brain Protocol (base.py)
Abstract base interface for swappable per-drone onboard guidance and comms plugins.
Enforces the Phase 4 Brain Airgap: zero simulator references or ground-truth handles.
"""

from __future__ import annotations
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
import numpy as np


@dataclass(frozen=True)
class RadioPacket:
    """Serialized over-the-air or hardline packet crossing the Canyon / Brain Airgap."""
    sender_id: int                  # -1 for GCS C2 Mast, 0..N for peer airframe ID
    msg_type: str                   # "GCS_UPLINK", "LEADER_HEARTBEAT", "ESCORT_YIP",
                                    # "GOSSIP_ECHO", "SUCCESSION_NOTICE", "ENROLL_HANDSHAKE"
    timestamp: float                # Transmission clock (s)
    seq: int = 0                    # Monotonic sequence number
    payload: Dict[str, Any] = field(default_factory=dict)


@dataclass
class HeardPeer:
    """Onboard cached state of a radio peer heard over the RF channel within LINK_TIMEOUT."""
    drone_id: int
    pos: np.ndarray
    vel: np.ndarray
    yaw: float = 0.0
    is_leader: bool = False
    tier: int = 2
    enrolled: bool = True
    last_heard_s: float = 0.0
    acknowledged_target: Optional[np.ndarray] = None
    formation_mode: Optional[str] = None
    formation_heading: Optional[float] = None
    heading_initialized: bool = False
    assigned_slots: Optional[Dict[int, int]] = None
    active_seq: int = 1
    echo_seq: int = 1
    echo_target: Optional[np.ndarray] = None
    gossip_status: str = "COORDINATED"


@dataclass(frozen=True)
class LocalTelemetry:
    """Immutable onboard sensor & memory snapshot for a SINGLE airframe.
    Contains ZERO references to any simulator or peer airframe objects."""
    # 1. Identity & Lifecycle
    drone_id: int
    tier: int
    is_leader: bool
    enrolled: bool
    flight_state: str
    status: str
    clock_s: float

    # 2. Local Navigation Sensors (pos[:2] is [NaN, NaN] when gnss_denied=True)
    pos: np.ndarray
    vel: np.ndarray
    yaw: float
    reciprocal_heading: float

    # 3. Local RF Receiver & GNSS Hardware State
    rssi: float
    rssi_prev: float
    link_quality: float
    lost_link: bool
    uplink_active: bool
    downlink_active: bool
    gnss_denied: bool
    gnss_denied_timer: float
    drift_radius: float
    gnss_settled: bool
    battery: float

    # 4. Local Onboard Target & Waypoint Registers
    acknowledged_target: np.ndarray
    lkp: np.ndarray
    backtrack_target: Optional[np.ndarray]
    holding_at_horizon: bool
    rejoin_target: Optional[np.ndarray]
    mission_active: bool
    mission_paused: bool
    mission_loop: bool
    mission_waypoints: Tuple[np.ndarray, ...]
    current_wp_idx: int
    wp_loitering: bool
    wp_loiter_timer: float
    wp_total_loiter: float
    route_acceptance_radius: float

    # 5. Local Deconfliction & Tether Registers
    peel_off_timer: float
    peel_off_dir: np.ndarray
    promotion_timer: float
    enrollment_timer: float
    tether_active: bool
    tether_lost: bool
    tether_jettisoned: bool
    tether_failsafe_policy: str
    tether_anchor_pos: np.ndarray
    overhead_marshal_active: bool
    patrol_active: bool


@dataclass
class FlightIntent:
    """Pure control output and onboard register updates returned by the OBC."""
    target_acc: np.ndarray
    target_yaw: Optional[float] = None
    target_station: Optional[np.ndarray] = None
    state_updates: Dict[str, Any] = field(default_factory=dict)


class BaseBrain(ABC):
    """Abstract interface for per-drone autonomous swarm guidance brains.
    Operates strictly as a Phase 4 pure I/O black box:
    (LocalTelemetry, [InboxPackets], dt) -> (FlightIntent, [OutboxPackets])."""

    brain_name: str = "BASE"

    def __init__(self, drone_id: int = 0) -> None:
        self.drone_id: int = int(drone_id)
        self.heard_peers: Dict[int, HeardPeer] = {}

    @abstractmethod
    def reset(self) -> None:
        """Reset internal onboard brain registers upon reboot or re-initialization."""
        pass

    @abstractmethod
    def step_drone(
        self,
        telemetry: LocalTelemetry,
        inbox: List[RadioPacket],
        dt: float,
    ) -> Tuple[FlightIntent, List[RadioPacket]]:
        """Execute one onboard guidance and comms cycle for this airframe."""
        pass

    def handle_command(self, cmd_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Optional hook to handle brain-specific commands received via C2 packet."""
        return None

    def on_succession(self, old_leader_id: Optional[int], new_leader_id: int) -> None:
        """Optional hook invoked when autonomic succession elects a new leader."""
        pass

    def get_telemetry_extra(self) -> Dict[str, Any]:
        """Optional dictionary of brain-specific state variables to include
        in the downlinked telemetry stream."""
        return {}

    def _compute_gnss_denied_intent(
        self, telemetry: LocalTelemetry, dt: float
    ) -> FlightIntent:
        """Compute pure FlightIntent for GNSS-denied dead-reckoning RTB (TKT-52).
        Relies exclusively on compass reciprocal heading, barometric altitude (pos[2]),
        inertial velocity damping (vel), and Howl RF beacon RSSI gradient."""
        new_timer = telemetry.gnss_denied_timer + dt
        new_drift = min(15.0, 0.5 + 0.4 * new_timer)
        settled = bool(telemetry.gnss_settled or telemetry.rssi >= -45.0 or new_timer >= 25.0)
        new_heading = telemetry.reciprocal_heading

        if settled:
            ax = float(np.clip(-4.0 * telemetry.vel[0], -6.0, 6.0))
            ay = float(np.clip(-4.0 * telemetry.vel[1], -6.0, 6.0))
            target_vz = -0.8
            az = float(np.clip(3.5 * (target_vz - telemetry.vel[2]), -5.0, 5.0))
            acc = np.array([ax, ay, az], dtype=float)
        else:
            target_z = max(6.0, 25.0 - (telemetry.drone_id * 2.0))
            az = float(np.clip(3.0 * (target_z - telemetry.pos[2]) - 2.5 * telemetry.vel[2], -6.0, 6.0))

            if telemetry.rssi_prev > -99.0:
                delta_rssi = telemetry.rssi - telemetry.rssi_prev
                if delta_rssi < -0.05:
                    trim = 0.05 if (telemetry.drone_id % 2 == 0) else -0.05
                    new_heading = float((new_heading + trim) % (2.0 * math.pi))

            psi = new_heading
            u_parallel = np.array([math.cos(psi), math.sin(psi)], dtype=float)
            u_perp = np.array([-math.sin(psi), math.cos(psi)], dtype=float)

            side = 1.0 if (telemetry.drone_id % 2 == 0) else -1.0
            v_lateral = side * 2.0 * min(1.0, new_drift / 5.0)
            v_cruise = 8.0

            v_target_xy = v_cruise * u_parallel + v_lateral * u_perp
            ax = float(np.clip(3.0 * (v_target_xy[0] - telemetry.vel[0]), -6.0, 6.0))
            ay = float(np.clip(3.0 * (v_target_xy[1] - telemetry.vel[1]), -6.0, 6.0))
            acc = np.array([ax, ay, az], dtype=float)

        speed_xy = math.hypot(telemetry.vel[0], telemetry.vel[1])
        target_yaw = math.atan2(telemetry.vel[1], telemetry.vel[0]) if speed_xy > 0.5 else None

        return FlightIntent(
            target_acc=acc,
            target_yaw=target_yaw,
            state_updates={
                "gnss_denied_timer": new_timer,
                "drift_radius": new_drift,
                "gnss_settled": settled,
                "reciprocal_heading": new_heading,
                "rssi_prev": telemetry.rssi,
            },
        )

    def compute_gnss_denied_guidance(self, drone: Any, dt: float) -> np.ndarray:
        """Compute autonomous dead-reckoning RTB guidance for GNSS-denied airframe (TKT-52).
        STRICT AIRGAP & NON-CHEATING INVARIANT:
        The OBC is strictly forbidden from reading true horizontal spatial coordinates (pos[:2])
        or ground-truth target coordinates. Guidance relies exclusively on:
        1. Compass Heading (reciprocal_heading)
        2. Barometric Altitude (pos[2])
        3. Inertial Velocity Damping (vel)
        4. GCS Mast Howl RF Beacon Gradient (rssi vs rssi_prev)
        """
        drone.gnss_denied_timer += dt
        drone.drift_radius = min(15.0, 0.5 + 0.4 * drone.gnss_denied_timer)

        if drone.rssi >= -45.0 or drone.gnss_denied_timer >= 25.0:
            drone.gnss_settled = True

        if getattr(drone, "gnss_settled", False):
            ax = float(np.clip(-4.0 * drone.vel[0], -6.0, 6.0))
            ay = float(np.clip(-4.0 * drone.vel[1], -6.0, 6.0))
            target_vz = -0.8
            az = float(np.clip(3.5 * (target_vz - drone.vel[2]), -5.0, 5.0))
            return np.array([ax, ay, az], dtype=float)

        target_z = max(6.0, 25.0 - (drone.id * 2.0))
        az = float(np.clip(3.0 * (target_z - drone.pos[2]) - 2.5 * drone.vel[2], -6.0, 6.0))

        if getattr(drone, "rssi_prev", -100.0) > -99.0:
            delta_rssi = drone.rssi - drone.rssi_prev
            if delta_rssi < -0.05:
                trim = 0.05 if (drone.id % 2 == 0) else -0.05
                drone.reciprocal_heading = float((drone.reciprocal_heading + trim) % (2.0 * math.pi))
        drone.rssi_prev = drone.rssi

        psi = drone.reciprocal_heading
        u_parallel = np.array([math.cos(psi), math.sin(psi)], dtype=float)
        u_perp = np.array([-math.sin(psi), math.cos(psi)], dtype=float)

        side = 1.0 if (drone.id % 2 == 0) else -1.0
        v_lateral = side * 2.0 * min(1.0, drone.drift_radius / 5.0)
        v_cruise = 8.0

        v_target_xy = v_cruise * u_parallel + v_lateral * u_perp
        ax = float(np.clip(3.0 * (v_target_xy[0] - drone.vel[0]), -6.0, 6.0))
        ay = float(np.clip(3.0 * (v_target_xy[1] - drone.vel[1]), -6.0, 6.0))

        return np.array([ax, ay, az], dtype=float)
