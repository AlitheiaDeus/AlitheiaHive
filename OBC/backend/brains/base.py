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
    crashed: bool = False
    enrolled: bool = True
    flight_state: str = "CRUISE"
    last_heard_s: float = 0.0
    last_heard_clock: float = 0.0
    acknowledged_target: Optional[np.ndarray] = None
    formation_mode: Optional[str] = None
    formation_heading: Optional[float] = None
    heading_initialized: bool = False
    assigned_slots: Optional[Dict[int, int]] = None
    active_seq: int = 1
    echo_seq: int = 1
    echo_target: Optional[np.ndarray] = None
    gossip_status: str = "COORDINATED"
    gcs_contact: bool = False
    gateway_id: Optional[int] = None


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
    tether_heartbeat_watchdog: float = 0.5
    corridor_target: Optional[np.ndarray] = None
    nearby_peers: Tuple[Any, ...] = ()
    urban_obstacles: Tuple[Any, ...] = ()


@dataclass
class FlightIntent:
    """Pure control output and onboard register updates returned by the OBC."""
    target_acc: np.ndarray
    target_yaw: Optional[float] = None
    target_station: Optional[np.ndarray] = None
    state_updates: Dict[str, Any] = field(default_factory=dict)


def _is_on_obstacle_top(pos: np.ndarray, obs: Any, vertical_margin: float = 0.25, horizontal_inset: float = -0.1) -> bool:
    if not getattr(obs, "is_landable", False):
        return False
    p = np.asarray(pos, dtype=float)
    x, y, z = p[0], p[1], p[2]
    x_min = getattr(obs, "x_min", getattr(obs, "bounds", [0, 0, 0, 0, 0, 0])[0])
    x_max = getattr(obs, "x_max", getattr(obs, "bounds", [0, 0, 0, 0, 0, 0])[1])
    y_min = getattr(obs, "y_min", getattr(obs, "bounds", [0, 0, 0, 0, 0, 0])[2])
    y_max = getattr(obs, "y_max", getattr(obs, "bounds", [0, 0, 0, 0, 0, 0])[3])
    z_ceiling = getattr(obs, "z_ceiling", getattr(obs, "bounds", [0, 0, 0, 0, 0, 0])[5])
    in_xy = (x_min + horizontal_inset <= x <= x_max - horizontal_inset) and \
            (y_min + horizontal_inset <= y <= y_max - horizontal_inset)
    at_top = (z >= z_ceiling - vertical_margin)
    return bool(in_xy and at_top)


def _point_to_aabb_repulsion(
    pos: np.ndarray,
    vel: np.ndarray,
    bounds: Tuple[float, float, float, float, float, float],
    zone3_outer: float = 3.5,
    zone4_clash: float = 1.5,
) -> Tuple[float, np.ndarray]:
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    p = np.asarray(pos, dtype=float)
    v = np.asarray(vel, dtype=float)

    cx = np.clip(p[0], x_min, x_max)
    cy = np.clip(p[1], y_min, y_max)
    cz = np.clip(p[2], z_min, z_max)

    c = np.array([cx, cy, cz], dtype=float)
    diff = p - c
    dist = float(np.linalg.norm(diff))

    if dist > zone3_outer:
        return dist, np.zeros(3, dtype=float)

    if dist > 1e-4:
        normal = diff / dist
    else:
        normal = np.array([0.0, 0.0, 1.0], dtype=float)

    if dist <= zone4_clash:
        return dist, np.zeros(3, dtype=float)

    norm_interp = (zone3_outer - dist) / (zone3_outer - zone4_clash)
    push_mag = 16.0 + 12.0 * norm_interp
    a_push = push_mag * normal

    v_into_wall = float(np.dot(v, -normal))
    if v_into_wall > 0.0:
        a_damp = 1.5 * v_into_wall * normal
    else:
        a_damp = np.zeros(3, dtype=float)

    return dist, a_push + a_damp


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

    # ==========================================================================
    # Autonomous OBC Zone 2 & Zone 3 Avoidance Reflexes (TKT-69 / Brain Airgap)
    # ==========================================================================
    def compute_inter_drone_avoidance(
        self,
        telemetry: LocalTelemetry,
        peers: Optional[List[Any]] = None,
    ) -> np.ndarray:
        """Compute Zone 2 caution separation and Zone 3 emergency thrust + relative velocity damping
        against heard peer drones from V2V radio telemetry (Brain Airgap)."""
        acc_avoid = np.zeros(3, dtype=float)
        own_id = telemetry.drone_id
        own_pos = telemetry.pos
        own_vel = telemetry.vel
        if own_pos is None or np.any(np.isnan(own_pos)):
            return np.zeros(3, dtype=float)
        if telemetry.flight_state in ("LANDED", "GROUND_IDLE", "FIELD_LANDED", "E_LANDED"):
            return np.zeros(3, dtype=float)

        # Combine heard RF peers with any proximity telemetry sensors
        known_peers: Dict[int, Any] = dict(self.heard_peers)
        if hasattr(telemetry, "nearby_peers") and telemetry.nearby_peers:
            for np_peer in telemetry.nearby_peers:
                np_id = int(getattr(np_peer, "id", getattr(np_peer, "drone_id", -1)))
                if np_id not in known_peers and np_id != own_id:
                    known_peers[np_id] = np_peer

        peer_list = peers if peers is not None else list(known_peers.values())

        for other in peer_list:
            oid = int(getattr(other, "drone_id", getattr(other, "id", -1)))
            if oid == own_id:
                continue
            if getattr(other, "crashed", False):
                continue
            if not getattr(other, "enrolled", True):
                continue
            other_state = str(getattr(other, "flight_state", ""))
            if other_state in ("LANDED", "GROUND_IDLE", "FIELD_LANDED", "E_LANDED"):
                continue

            other_pos = np.array(getattr(other, "pos"), dtype=float)
            if np.any(np.isnan(other_pos)):
                continue
            other_vel = np.array(getattr(other, "vel", [0.0, 0.0, 0.0]), dtype=float)

            diff = own_pos - other_pos
            dist = float(np.linalg.norm(diff))

            if dist <= 0.01:
                n_ij = np.array([1.0, 0.0, 0.0], dtype=float)
            else:
                n_ij = diff / dist

            # Zone 3: Critical Danger (1.5m < dist <= 3.5m)
            if dist <= 3.5:
                push_mag = 16.0 + 12.0 * ((3.5 - dist) / (3.5 - 1.5))
                a_push = push_mag * n_ij

                # Horizontal lateral breakout
                d_xy = float(math.hypot(diff[0], diff[1]))
                if d_xy < 2.8:
                    if d_xy > 0.05:
                        n_xy = np.array([diff[0] / d_xy, diff[1] / d_xy, 0.0], dtype=float)
                    else:
                        n_xy = np.array([1.0, 0.0, 0.0], dtype=float)
                    lat_push = 14.0 * ((2.8 - d_xy) / 2.8)
                    a_push += lat_push * n_xy

                    # Vertical corridor expansion
                    z_diff = diff[2]
                    if abs(z_diff) < 2.0:
                        z_sign = 1.0 if z_diff >= 0.0 else -1.0
                        z_push = 16.0 * ((2.0 - abs(z_diff)) / 2.0)
                        a_push[2] += z_sign * z_push

                # Relative-velocity damping (brakes incoming momentum)
                v_rel = own_vel - other_vel
                v_closing = float(np.dot(v_rel, n_ij))
                if v_closing < 0.0:
                    a_damp = -3.5 * (v_closing * n_ij)
                else:
                    a_damp = np.zeros(3, dtype=float)

                acc_avoid += a_push + a_damp

                # P2P Yip RSSI Emergency Cushion
                if dist <= 2.2:
                    cushion_mag = 22.0 * (2.2 - dist)
                    acc_avoid += cushion_mag * n_ij

            # Zone 2: Caution Buffer (3.5m < dist <= 8.0m)
            elif dist <= 8.0:
                sep_mag = 5.0 * ((8.0 - dist) / (8.0 - 3.5))
                acc_avoid += sep_mag * n_ij

        # Suppress horizontal avoidance during pad climb to ascend cleanly along vertical corridor
        if telemetry.flight_state == "TAKEOFF" and telemetry.pos[2] < 5.0:
            acc_avoid[0] = 0.0
            acc_avoid[1] = 0.0
        elif telemetry.flight_state in ("DESCENDING", "FIELD_DESCENDING", "EMERGENCY_LANDING") and telemetry.pos[2] < 3.0:
            acc_avoid[0] *= 0.1
            acc_avoid[1] *= 0.1

        return acc_avoid

    def compute_obstacle_avoidance(
        self,
        telemetry: LocalTelemetry,
        obstacles: Optional[Union[Tuple[Any, ...], List[Any]]] = None,
    ) -> np.ndarray:
        """Compute Zone 3 reflexive wall evasion and momentum damping against static obstacles (Brain Airgap)."""
        obs_list = obstacles if obstacles is not None else getattr(telemetry, "urban_obstacles", ())
        if not obs_list:
            return np.zeros(3, dtype=float)

        acc_obs = np.zeros(3, dtype=float)
        pos = telemetry.pos
        vel = telemetry.vel
        if pos is None or np.any(np.isnan(pos)):
            return np.zeros(3, dtype=float)
        is_landing_or_takeoff = telemetry.flight_state in (
            "TAKEOFF", "FIELD_DESCENDING", "EMERGENCY_LANDING", "DESCENDING"
        )

        for obs in obs_list:
            if getattr(obs, "is_landable", False) and _is_on_obstacle_top(pos, obs):
                if is_landing_or_takeoff or pos[2] >= getattr(obs, "z_ceiling", 0.0) - 0.05:
                    continue
            bounds = getattr(obs, "bounds", None)
            if bounds is None:
                continue
            dist, a_evade = _point_to_aabb_repulsion(
                pos, vel, bounds, zone3_outer=3.5, zone4_clash=1.5
            )
            acc_obs += a_evade

        return acc_obs

    def apply_avoidance_reflex(
        self,
        telemetry: LocalTelemetry,
        a_nominal: np.ndarray,
        peers: Optional[List[Any]] = None,
    ) -> np.ndarray:
        """Combine nominal guidance intent with OBC Zone 2 & 3 collision avoidance reflexes."""
        if telemetry.flight_state in ("LANDED", "GROUND_IDLE", "FIELD_LANDED", "E_LANDED", "CRASHED") or telemetry.status == "CRASHED":
            return np.zeros(3, dtype=float)

        a_peers = self.compute_inter_drone_avoidance(telemetry, peers)
        a_obs = self.compute_obstacle_avoidance(telemetry)
        a_avoid = a_peers + a_obs

        avoid_mag = float(np.linalg.norm(a_avoid))
        res_nominal = np.copy(a_nominal)
        if avoid_mag > 8.0:
            # Attenuate forward guidance during emergency Zone 3 avoidance so drone brakes instead of plowing forward
            attenuation = max(0.2, 1.0 - (avoid_mag - 8.0) / 25.0)
            res_nominal = res_nominal * attenuation

        total_acc = res_nominal + a_avoid
        # Bounded to airframe kinematic acceleration limit A_MAX = 15.0 m/s^2
        acc_mag = float(np.linalg.norm(total_acc))
        if acc_mag > 15.0:
            total_acc = (total_acc / acc_mag) * 15.0
        return total_acc
