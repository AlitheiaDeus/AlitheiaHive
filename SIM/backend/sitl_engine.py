"""
SwarmTech3_UpRising: Core Simulation Engine (sim_core.py)
Vectorized 60 FPS Boid Kinematics, Sane 4-Zone Spatial Safety,
Tether Collision & Snap Dynamics, 10 Hz Baud Radio Comms & LKP Recovery,
and Autonomic Leadership Succession.
"""

from __future__ import annotations
import math
import time
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any, Set

# Ensure repository root is on sys.path
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# Self-healing Conda Windows DLL path bootstrap:
if sys.platform == "win32":
    _prefix = Path(sys.prefix)
    _dll_candidates = [
        _prefix / "Library" / "bin",
        _prefix / "Scripts",
        _prefix.parent.parent / "Library" / "bin",
    ]
    for _d in _dll_candidates:
        if _d.exists():
            _p_str = str(_d)
            if _p_str not in os.environ.get("PATH", ""):
                os.environ["PATH"] = _p_str + os.pathsep + os.environ.get("PATH", "")
            try:
                os.add_dll_directory(_p_str)
            except (AttributeError, OSError):
                pass

import numpy as np

from OBC.backend.drone_firmware import Drone, TIER_LEADER, TIER_ESCORT, TIER_SCOUT
from OBC.backend.brains import BaseBrain, BRAIN_REGISTRY, LeaderFollowerBrain, GossipConsensusBrain
from GCS.backend.c2_gateway import C2Gateway, ROUTE_PRESETS
from SIM.backend.fleet_registry import FleetRegistry


# ==============================================================================
# Physical Constants & Tuning Parameters
# ==============================================================================
DT: float = 1.0 / 60.0                 # 60 FPS simulation time step (s)
BODY_RADIUS: float = 0.75              # Physical drone airframe radius (m) -> 1.5m wingspan
ZONE4_CLASH: float = 1.5               # Prop clash collision threshold (m)
ZONE3_OUTER: float = 3.5               # Critical evasion & velocity-damping zone (m)
ZONE2_OUTER: float = 8.0               # Smooth separation caution zone (m)

V_CRUISE: float = 12.0                 # Standard cruise speed (m/s)
V_MAX: float = 20.0                    # Hard velocity clamp (m/s)
V_LKP: float = 8.0                     # Conservative speed when navigating to LKP (m/s)
A_MAX: float = 15.0                    # Maximum acceleration (m/s^2)
GRAVITY: float = -9.81                 # Ballistic drop acceleration (m/s^2)

# Radio Comms & Asymmetric Link Budget (Canine Acoustic Taxonomy - TKT-49)
RANGE_HOWL: float = 140.0              # High-power GCS mast TX (blasting uplink) (m)
RANGE_BARK: float = 90.0               # Tier-1 Flagship onboard TX (leader downlink) (m)
RANGE_YIP: float = 50.0                # Tier-2 Escort micro-transceiver (escort downlink) (m)
RANGE_P2P: float = 60.0                # Inter-drone P2P mesh gossip radio propagation limit (m)
RADIO_RANGE: float = RANGE_HOWL        # Backward-compatible generic radio horizon alias (m)
HEARTBEAT_INTERVAL: float = 0.1        # 10 Hz broadcast interval (s)
LINK_TIMEOUT: float = 1.0              # Lost-link timeout threshold (s)

# Tether Dynamics
TETHER_ANCHOR = np.array([0.0, 0.0, 1.5], dtype=float)  # GCS mast anchor (m)
TETHER_AVOID_DIST: float = 3.0         # Follower clearance envelope from cable (m)
TETHER_SNAP_DIST: float = 1.0          # Follower collision severance threshold (m)
TETHER_MAX_LEN: float = 300.0          # Max cable length before tensile snap (m)

# Hardware Tiers for Succession Scoring
TIER_LEADER: int = 3
TIER_ESCORT: int = 2
TIER_SCOUT: int = 1

# Discrete Waypoint Mission Route Presets (TKT-28)
ROUTE_PRESETS: Dict[str, List[List[float]]] = {
    "BOX_PATROL": [
        [40.0, -20.0, 25.0],
        [40.0, 40.0, 28.0],
        [-20.0, 40.0, 25.0],
        [-20.0, -20.0, 22.0],
    ],
    "DIAMOND_SURVEY": [
        [50.0, 10.0, 28.0],
        [15.0, 45.0, 25.0],
        [-20.0, 10.0, 24.0],
        [15.0, -25.0, 26.0],
    ],
    "HEXAGON_SWEEP": [
        [45.0, 25.0, 26.0],
        [20.0, 45.0, 28.0],
        [-15.0, 30.0, 25.0],
        [-25.0, -5.0, 24.0],
        [-5.0, -30.0, 26.0],
        [30.0, -20.0, 27.0],
    ],
}


# ==============================================================================
# Simulation World & Physics Engine (Bubble 3: SITL World Simulator)
# ==============================================================================
class SITLWorldEngine:
    """Core physics and guidance simulator running at 60 FPS.
    Handles vectorized boid flocking, 4-zone safety, tether dynamics,
    10 Hz comms with LKP failsafe, and autonomic succession."""
    """Core physics and guidance simulator running at 60 FPS.
    Handles vectorized boid flocking, 4-zone safety, tether dynamics,
    10 Hz comms with LKP failsafe, and autonomic succession."""

    def __init__(
        self,
        drone_count: int = 8,
        enable_packet_loss: bool = False,
        seed: Optional[int] = None,
        home_lat: Optional[float] = None,
        home_lon: Optional[float] = None,
        home_alt: Optional[float] = None,
        cold_start: bool = False,
        start_in_air: Optional[bool] = None,
        count: Optional[int] = None,
    ):
        if count is not None:
            drone_count = count
        if start_in_air is not None:
            cold_start = not start_in_air
        if seed is not None:
            np.random.seed(seed)

        self.sim_time: float = 0.0
        self.last_heartbeat_broadcast: float = -1.0
        self.enable_packet_loss: bool = enable_packet_loss
        self.comms_mode: str = "TETHERED_RELAY"  # "TETHERED_RELAY", "LEADER_FOLLOWER", "GOSSIP_CONSENSUS"
        self.rf_jamming_rate: float = 0.0        # Electronic warfare RF jamming rate [0.0, 0.5]

        # Cold-Start Pad Scramble & Sequenced Climb-Out Corridor (TKT-41)
        self.cold_start: bool = cold_start
        self.scramble_active: bool = False
        self.scramble_timer: float = 0.0
        self.scramble_stagger: float = 1.2

        # Reference Geographic Home Anchor (Bubble 3: World Ground Truth)
        # Default: Tokyo Bay / Shinagawa Canal Waterfront (Tennozu Isle)
        self.home_lat: float = float(home_lat if home_lat is not None else 35.625971)
        self.home_lon: float = float(home_lon if home_lon is not None else 139.734759)
        self.home_alt: float = float(home_alt if home_alt is not None else 20.0)  # Meters AMSL

        # Global Mission State
        self.formation_mode: str = "ECHELON_ESCORT"  # ECHELON_ESCORT, NATURAL_FLOCK, ORBIT_VORTEX, PERIMETER_PATROL
        self.tether_anchor: np.ndarray = np.copy(TETHER_ANCHOR)
        self.tether_severed: bool = False

        # Bubble 1: Tactical C2 Gateway (GCS Intent & Mission Layer)
        self.c2: C2Gateway = C2Gateway(self)

        # Fleet Doctrine A (Lost-Link Failsafe Loiter & Automated RTB) (TKT-16)
        self.lost_link_loiter_timer: float = 0.0
        self.lost_link_policy: str = "DOCTRINE_A"
        self.swarm_rtb_active: bool = False

        # Autonomous Swarm RTB & Coordinated Soft-Landing (TKT-19)
        self.overhead_marshal_active: bool = False
        self.recovery_stage: str = "IDLE"  # "IDLE", "OVERHEAD_MARSHAL", "LANDING"
        self.landing_pads: Dict[int, int] = {}  # pad_idx -> drone_id

        # Batched ATC Pad Recovery & Downwash Clearance Cycles (TKT-42)
        self.atc_capacity: int = 8                # Bounded to 8 numbered recovery pads (0..7)
        self.atc_current_wave: int = 1            # Current landing batch wave (1, 2, ...)
        self.atc_waiting_queue: List[int] = []    # Drone IDs holding in overhead marshal shelves
        self.turnaround_timer: float = 0.0        # Turnaround countdown timer (12.0s)
        self.auto_turnaround_enabled: bool = True # Automatically release next wave on turnaround timeout

        # Tactical GNSS Denial & Dead-Reckoning RTB (TKT-52)
        self.gnss_denied_all: bool = False

        # Build Drone Fleet
        self.drones: List[Drone] = []
        self._highest_drone_id: int = drone_count - 1
        self._init_fleet(drone_count)

        # Pluggable Guidance Brain Plugin (Default: LeaderFollowerBrain)
        self.active_brain_name: str = "LEADER_FOLLOWER"
        self.brain: BaseBrain = LeaderFollowerBrain(self)

    def set_brain(self, brain_name: str, force: bool = False) -> bool:
        """Hot-swap the guidance brain plugin at runtime. Gated by comms range unless forced."""
        if not force and not self.is_in_comms_range():
            return False
        key = brain_name.upper()
        if key in BRAIN_REGISTRY:
            self.active_brain_name = key
            old_heading = getattr(self.brain, "formation_heading", 0.0)
            old_init = getattr(self.brain, "heading_initialized", False)
            old_formation = getattr(self.brain, "formation_mode", "ECHELON_ESCORT")
            old_slots = dict(getattr(self.brain, "assigned_slots", {}))

            self.brain = BRAIN_REGISTRY[key](self)
            self.brain.reset()
            self.brain.formation_mode = old_formation
            if old_init:
                self.brain.formation_heading = old_heading
                self.brain.heading_initialized = True

            # Preserve slot assignments across brain hot-swaps to avoid cross-path scramble
            if old_slots:
                if key == "GOSSIP_CONSENSUS" and self.leader is not None:
                    if self.leader.id not in old_slots:
                        old_slots[self.leader.id] = 0
                self.brain.assigned_slots = old_slots

            if key == "GOSSIP_CONSENSUS":
                self.comms_mode = "GOSSIP_CONSENSUS"
            elif key == "LEADER_FOLLOWER" and self.comms_mode == "GOSSIP_CONSENSUS":
                self.comms_mode = "LEADER_FOLLOWER"
            return True
        return False

    def set_comms_mode(self, mode_name: str, enforce_grounded: bool = False) -> bool:
        """Switch comms mode: TETHERED_RELAY, LEADER_FOLLOWER, or GOSSIP_CONSENSUS."""
        mode = mode_name.upper().replace("-", "_").replace(" ", "_")
        if mode in ("TETHERED_RELAY", "TETHER_RELAY", "TETHERED"):
            # Check if Flagship Drone #0 is alive to anchor the physical umbilical
            drone0 = next((d for d in self.drones if d.id == 0), None)
            if drone0 is None or drone0.crashed:
                # Cannot attach physical tether if primary flagship is destroyed
                return False

            # Invariant: Physical tether umbilical cannot be reattached mid-flight
            # if currently operating in untethered wireless modes
            if enforce_grounded and self.comms_mode != "TETHERED_RELAY":
                airborne = any(
                    not d.crashed and d.flight_state not in ("LANDED", "GROUND_IDLE")
                    for d in self.drones
                )
                if airborne:
                    return False

            self.comms_mode = "TETHERED_RELAY"
            self.tether_severed = False
            drone0.tether_lost = False
            drone0.tether_jettisoned = False
            drone0.is_leader = True
            self.lost_link_loiter_timer = 0.0
            if self.active_brain_name != "LEADER_FOLLOWER":
                self.set_brain("LEADER_FOLLOWER")
            return True
        elif mode in ("LEADER_FOLLOWER", "WIRELESS_STAR", "STAR"):
            self.comms_mode = "LEADER_FOLLOWER"
            self.lost_link_loiter_timer = 0.0  # Clear tether severance timer in wireless mode
            if self.active_brain_name != "LEADER_FOLLOWER":
                self.set_brain("LEADER_FOLLOWER")
            return True
        elif mode in ("GOSSIP_CONSENSUS", "P2P_MESH", "MESH", "GOSSIP"):
            self.comms_mode = "GOSSIP_CONSENSUS"
            self.lost_link_loiter_timer = 0.0  # Clear tether severance timer in mesh mode
            if self.active_brain_name != "GOSSIP_CONSENSUS":
                self.set_brain("GOSSIP_CONSENSUS")
            return True
        return False

    def set_rf_jamming(self, rate: float) -> float:
        """Set electronic warfare RF jamming rate [0.0, 0.5]."""
        self.rf_jamming_rate = float(np.clip(rate, 0.0, 0.5))
        if self.rf_jamming_rate > 0.0:
            self.enable_packet_loss = True
        return self.rf_jamming_rate

    def calculate_packet_drop_prob(
        self, distance: float, is_hardline: bool = False, max_range: Optional[float] = None
    ) -> float:
        """Calculate packet loss probability using distance-dependent path loss and jamming.
        P_loss(d) = gamma + (1 - gamma) * 0.25 * (d / R_max)^2"""
        if is_hardline:
            return 0.0
        r_max = max_range if max_range is not None else RADIO_RANGE
        if distance > r_max:
            return 1.0
        if not self.enable_packet_loss and self.rf_jamming_rate <= 0.0:
            return 0.0
        gamma = self.rf_jamming_rate
        norm_dist = distance / r_max
        prob = gamma + (1.0 - gamma) * 0.25 * (norm_dist ** 2)
        return float(np.clip(prob, 0.0, 1.0))

    def is_packet_dropped(
        self, distance: float, is_hardline: bool = False, max_range: Optional[float] = None
    ) -> bool:
        """Stochastically determine if packet is lost given distance and jamming rate."""
        p = self.calculate_packet_drop_prob(distance, is_hardline, max_range=max_range)
        if p <= 0.0:
            return False
        if p >= 1.0:
            return True
        return bool(np.random.random() < p)

    def calculate_rssi(self, distance: float, is_hardline: bool = False) -> Tuple[float, float]:
        """Calculate RSSI (dBm) and normalized link quality [0.0, 1.0].
        Hardline fiber: -35.0 dBm, 1.0 quality.
        Wireless: Free-space loss from -40.0 dBm at 1m with -40*gamma jamming penalty."""
        if is_hardline:
            return -35.0, 1.0
        d = max(1.0, float(distance))
        loss = 20.0 * math.log10(d)
        jamming_penalty = 40.0 * self.rf_jamming_rate
        rssi = -40.0 - loss - jamming_penalty
        rssi = float(np.clip(rssi, -115.0, -35.0))
        lq = max(0.0, min(1.0, (rssi - (-100.0)) / 60.0)) * (1.0 - self.rf_jamming_rate)
        return round(rssi, 1), round(float(lq), 2)

    def is_within_base_rf_dome(self, pos_or_dist: Union[float, List[float], np.ndarray]) -> bool:
        """Check whether position or scalar distance is within 50m High-Gain RF Dome (TKT-42)."""
        if isinstance(pos_or_dist, (int, float)):
            return float(pos_or_dist) <= 50.0
        pos_arr = np.array(pos_or_dist, dtype=float)
        return float(np.linalg.norm(pos_arr - self.tether_anchor)) <= 50.0

    def calculate_base_rf_link(self, pos_or_dist: Union[float, List[float], np.ndarray]) -> Tuple[float, float, float]:
        """Compute (drop_prob, rssi, link_quality) incorporating 50m Base Horizon High-Gain RF Dome (TKT-42).
        Within 50m radius of base anchor, high-gain base station radio guarantees 100% link quality."""
        d = float(pos_or_dist) if isinstance(pos_or_dist, (int, float)) else float(np.linalg.norm(np.array(pos_or_dist, dtype=float) - self.tether_anchor))
        if d <= 50.0:
            return 0.0, -35.0, 1.0
        rssi, lq = self.calculate_rssi(d)
        drop_p = self.calculate_packet_drop_prob(d)
        return drop_p, rssi, lq
        loss = 20.0 * math.log10(d)
        jamming_penalty = 40.0 * self.rf_jamming_rate
        rssi = -40.0 - loss - jamming_penalty
        rssi = float(np.clip(rssi, -115.0, -35.0))
        lq = max(0.0, min(1.0, (rssi - (-100.0)) / 60.0)) * (1.0 - self.rf_jamming_rate)
        return round(rssi, 1), round(float(lq), 2)

    def _init_fleet(self, count: int, cold_start: Optional[bool] = None) -> None:
        """Initialize leader and followers in safe starting formation or cold pads."""
        if cold_start is None:
            cold_start = getattr(self, "cold_start", False)
        self.drones.clear()
        self.tether_severed = False
        self.lost_link_loiter_timer = 0.0
        self.swarm_rtb_active = False
        self.overhead_marshal_active = False
        self.recovery_stage = "IDLE"
        self.landing_pads.clear()
        self.atc_current_wave = 1
        self.atc_waiting_queue.clear()
        self.turnaround_timer = 0.0
        self.scramble_active = False
        self.scramble_timer = 0.0
        self.gnss_denied_all = False
        self._highest_drone_id = count - 1

        # Fleet Hardware Identity Registry (TKT-48)
        self.fleet_registry = FleetRegistry(initial_count=count)

        if cold_start:
            # Cold-start on numbered ground launch pads (TKT-41)
            # Leader #0 positioned on Pad 0 (Z = 0.05m)
            rec0 = self.fleet_registry.get_by_drone_id(0)
            pad0_pos = self.get_pad_position(0)
            pad0_pos[2] = 0.05
            leader = Drone(
                drone_id=0,
                tier=TIER_LEADER,
                pos=pad0_pos,
                is_leader=True,
                hardware_uid=rec0.hardware_uid if rec0 else "3842A10F9B200001",
                tail_number=rec0.tail_number if rec0 else "ALITHEIA-V01",
            )
            leader.flight_state = "GROUND_IDLE"
            leader.status = "GROUND_IDLE"
            leader.rotor_rpm = 0.0
            leader.landing_pad_idx = 0
            leader.landing_pad = np.copy(pad0_pos)
            self.landing_pads[0] = leader.id
            self.drones.append(leader)

            # Followers initialized cold on remaining pads
            for i in range(1, count):
                pad_idx = i % 8
                pad_pos = self.get_pad_position(pad_idx)
                pad_pos[2] = 0.05
                rec_i = self.fleet_registry.get_by_drone_id(i)
                drone = Drone(
                    drone_id=i,
                    tier=TIER_ESCORT,
                    pos=pad_pos,
                    is_leader=False,
                    hardware_uid=rec_i.hardware_uid if rec_i else f"3842A10F9B20{i + 1:04X}",
                    tail_number=rec_i.tail_number if rec_i else f"ALITHEIA-V{i + 1:02d}",
                )
                drone.flight_state = "GROUND_IDLE"
                drone.status = "GROUND_IDLE"
                drone.rotor_rpm = 0.0
                drone.landing_pad_idx = pad_idx
                drone.landing_pad = np.copy(pad_pos)
                self.landing_pads[pad_idx] = drone.id
                self.drones.append(drone)
        else:
            # Airborne initialization in staggered grid behind leader (default cruise)
            rec0 = self.fleet_registry.get_by_drone_id(0)
            leader = Drone(
                drone_id=0,
                tier=TIER_LEADER,
                pos=np.array([0.0, 0.0, 20.0], dtype=float),
                is_leader=True,
                hardware_uid=rec0.hardware_uid if rec0 else "3842A10F9B200001",
                tail_number=rec0.tail_number if rec0 else "ALITHEIA-V01",
            )
            self.drones.append(leader)

            # Followers initialized in staggered grid behind leader
            for i in range(1, count):
                row = (i + 1) // 2
                side = 1.0 if (i % 2 == 1) else -1.0
                x = -row * 6.0
                y = side * row * 6.0
                z = 20.0 + (row * 0.5)
                rec_i = self.fleet_registry.get_by_drone_id(i)
                drone = Drone(
                    drone_id=i,
                    tier=TIER_ESCORT,
                    pos=np.array([x, y, z], dtype=float),
                    is_leader=False,
                    hardware_uid=rec_i.hardware_uid if rec_i else f"3842A10F9B20{i + 1:04X}",
                    tail_number=rec_i.tail_number if rec_i else f"ALITHEIA-V{i + 1:02d}",
                )
                self.drones.append(drone)

        for d in self.drones:
            d.acknowledged_target = np.copy(self.target_pos)

    # ==========================================================================
    # C2 Mission Gateway Delegated Properties (GCS Bubble 1)
    # ==========================================================================
    @property
    def target_pos(self) -> np.ndarray:
        """Commanded target waypoint at GCS C2 layer."""
        return self.c2.target_pos

    @target_pos.setter
    def target_pos(self, value: Union[List[float], np.ndarray]) -> None:
        self.c2.target_pos = value

    @property
    def patrol_active(self) -> bool:
        return self.c2.patrol_active

    @patrol_active.setter
    def patrol_active(self, val: bool) -> None:
        self.c2.patrol_active = val

    @property
    def patrol_speed(self) -> float:
        return self.c2.patrol_speed

    @patrol_speed.setter
    def patrol_speed(self, val: float) -> None:
        self.c2.patrol_speed = val

    @property
    def patrol_time(self) -> float:
        return self.c2.patrol_time

    @patrol_time.setter
    def patrol_time(self, val: float) -> None:
        self.c2.patrol_time = val

    @property
    def patrol_center(self) -> np.ndarray:
        return self.c2.patrol_center

    @patrol_center.setter
    def patrol_center(self, val: np.ndarray) -> None:
        self.c2.patrol_center = val

    @property
    def patrol_scale(self) -> np.ndarray:
        return self.c2.patrol_scale

    @patrol_scale.setter
    def patrol_scale(self, val: np.ndarray) -> None:
        self.c2.patrol_scale = val

    @property
    def last_gossip_inject_time(self) -> float:
        return self.c2.last_gossip_inject_time

    @last_gossip_inject_time.setter
    def last_gossip_inject_time(self, val: float) -> None:
        self.c2.last_gossip_inject_time = val

    @property
    def route_active(self) -> bool:
        return self.c2.route_active

    @route_active.setter
    def route_active(self, val: bool) -> None:
        self.c2.route_active = val

    @property
    def route_name(self) -> str:
        return self.c2.route_name

    @route_name.setter
    def route_name(self, val: str) -> None:
        self.c2.route_name = val

    @property
    def route_waypoints(self) -> List[np.ndarray]:
        return self.c2.route_waypoints

    @route_waypoints.setter
    def route_waypoints(self, val: List[np.ndarray]) -> None:
        self.c2.route_waypoints = val

    @property
    def route_loop(self) -> bool:
        return self.c2.route_loop

    @route_loop.setter
    def route_loop(self, val: bool) -> None:
        self.c2.route_loop = val

    @property
    def route_acceptance_radius(self) -> float:
        return self.c2.route_acceptance_radius

    @route_acceptance_radius.setter
    def route_acceptance_radius(self, val: float) -> None:
        self.c2.route_acceptance_radius = val

    @property
    def rng_recon_active(self) -> bool:
        return self.c2.rng_recon_active

    @rng_recon_active.setter
    def rng_recon_active(self, val: bool) -> None:
        self.c2.rng_recon_active = val

    @property
    def rng_dwelling(self) -> bool:
        return self.c2.rng_dwelling

    @rng_dwelling.setter
    def rng_dwelling(self, val: bool) -> None:
        self.c2.rng_dwelling = val

    @property
    def rng_dwell_time(self) -> float:
        return self.c2.rng_dwell_time

    @rng_dwell_time.setter
    def rng_dwell_time(self, val: float) -> None:
        self.c2.rng_dwell_time = val

    @property
    def rng_dwell_timer(self) -> float:
        return self.c2.rng_dwell_timer

    @rng_dwell_timer.setter
    def rng_dwell_timer(self, val: float) -> None:
        self.c2.rng_dwell_timer = val

    @property
    def rng_station_count(self) -> int:
        return self.c2.rng_station_count

    @rng_station_count.setter
    def rng_station_count(self, val: int) -> None:
        self.c2.rng_station_count = val

    @property
    def rng_geofence_min(self) -> np.ndarray:
        return self.c2.rng_geofence_min

    @rng_geofence_min.setter
    def rng_geofence_min(self, val: np.ndarray) -> None:
        self.c2.rng_geofence_min = val

    @property
    def rng_geofence_max(self) -> np.ndarray:
        return self.c2.rng_geofence_max

    @rng_geofence_max.setter
    def rng_geofence_max(self, val: np.ndarray) -> None:
        self.c2.rng_geofence_max = val

    @property
    def rng_min_variance(self) -> float:
        return self.c2.rng_min_variance

    @rng_min_variance.setter
    def rng_min_variance(self, val: float) -> None:
        self.c2.rng_min_variance = val

    @property
    def rng_tether_clearance(self) -> float:
        return self.c2.rng_tether_clearance

    @rng_tether_clearance.setter
    def rng_tether_clearance(self, val: float) -> None:
        self.c2.rng_tether_clearance = val

    @property
    def rng_status(self) -> str:
        return self.c2.rng_status

    @property
    def rng_paused(self) -> bool:
        return self.c2.rng_paused

    def is_link_active(self) -> bool:
        return self.c2.is_link_active()

    def get_acknowledged_target(self) -> np.ndarray:
        """Return the current onboard target acknowledged by the active swarm guidance."""
        if self.comms_mode in ("TETHERED_RELAY", "LEADER_FOLLOWER"):
            current_leader = self.leader
            if current_leader is not None:
                return np.copy(current_leader.acknowledged_target)
            return np.copy(self.target_pos)
        elif self.comms_mode == "GOSSIP_CONSENSUS":
            if hasattr(self.brain, "peer_states") and self.brain.peer_states:
                for d in self.drones:
                    if not d.crashed and d.id in self.brain.peer_states:
                        return np.copy(self.brain.peer_states[d.id].active_target)
            return np.copy(self.target_pos)
        return np.copy(self.target_pos)

    @property
    def leader(self) -> Optional[Drone]:
        """Get the active fleet leader. Prefers an airborne leader over a landed/descending drone
        when other airborne drones exist in the fleet."""
        designated_leader = None
        for drone in self.drones:
            if drone.is_leader and not drone.crashed:
                if drone.flight_state not in ("LANDED", "DESCENDING", "RETRIEVED", "GROUND_IDLE"):
                    return drone
                designated_leader = drone

        # If designated leader is landed or descending, check if airborne followers exist
        airborne_candidates = [
            d for d in self.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING", "RETRIEVED", "GROUND_IDLE")
        ]
        if airborne_candidates:
            successor = max(airborne_candidates, key=lambda d: d.succession_score())
            if designated_leader is not None:
                designated_leader.is_leader = False
                if designated_leader.status == "FLEET_LEADER":
                    designated_leader.status = "ACTIVE"
            successor.is_leader = True
            successor.status = "FLEET_LEADER"
            if hasattr(self, "brain") and hasattr(self.brain, "on_succession"):
                self.brain.on_succession(designated_leader, successor)
            return successor

        return designated_leader

    def get_drone_by_id(self, drone_id: int) -> Optional[Drone]:
        """Retrieve drone instance by vehicle ID."""
        for d in self.drones:
            if d.id == drone_id:
                return d
        return None

    def get_2ic(self) -> Optional[Drone]:
        """Dynamically determine the Second-in-Command (2IC) drone (TKT-16).
        Highest succession score among surviving, non-leader escorts:
        score = (Tier * 1000) - ID."""
        candidates = [
            d for d in self.drones
            if not d.crashed and not d.is_leader and not d.tether_lost
        ]
        if not candidates:
            candidates = [d for d in self.drones if not d.crashed and not d.is_leader]
        if not candidates:
            return None
        candidates.sort(key=lambda d: d.succession_score(), reverse=True)
        return candidates[0]

    # ==========================================================================
    # Step Integration Loop (60 FPS) & GCS C2 Dispatcher (TKT-25)
    # ==========================================================================
    def step_c2(self, dt: float) -> None:
        """Advance GCS C2 layer: auto-patrol trajectory generation, RNG recon touch-and-go, and periodic uplink transmission."""
        self.c2.step(dt)

    def step(self, dt: float = DT, advance_c2: bool = True) -> None:
        """Advance the physical simulation world by dt seconds."""
        if advance_c2:
            self.step_c2(dt)

        self.sim_time += dt

        # 0b. Advance Universal Lost-Link Loiter & Automated RTB Timer (TKT-16 / TKT-19 / FIX-05)
        fleet_link_active = self.is_in_comms_range() and not (self.comms_mode == "TETHERED_RELAY" and self.tether_severed)
        if not fleet_link_active and not self.swarm_rtb_active:
            if self.lost_link_loiter_timer <= 0.0:
                self.lost_link_loiter_timer = 30.0 if (self.comms_mode == "TETHERED_RELAY" and self.tether_severed) else 15.0
            else:
                self.lost_link_loiter_timer = max(0.0, self.lost_link_loiter_timer - dt)
                if self.lost_link_loiter_timer <= 0.0:
                    # Loiter grace period expired: automated return to base (Stage 1 Overhead Marshal)
                    self.command_rtb_hold(force_autonomous=True)
        elif fleet_link_active and not self.swarm_rtb_active:
            if self.lost_link_loiter_timer > 0.0:
                self.lost_link_loiter_timer = 0.0

        # 0c. Advance Drone Breakaway Jettison State Machine (TKT-16 / TKT-27)
        for drone in self.drones:
            if not drone.crashed and drone.status == "BREAKAWAY_JETTISON" and not drone.tether_jettisoned:
                drone.jettison_timer += dt
                drone.jettison_watchdog = max(0.0, drone.jettison_watchdog - dt)

                # Physical Clearance Envelope Gates:
                # 1. Altitude gate: Delta Z >= 4.0m below formation or reached safety floor (max(3.0m, lead_z - 4.5m))
                current_leader = self.leader
                lead_z = current_leader.pos[2] if current_leader is not None else 20.0
                z_floor = max(3.0, lead_z - 4.5)
                alt_cleared = (drone.pos[2] <= lead_z - 4.0) or (drone.pos[2] <= z_floor + 0.25)

                # 2. Lateral gate: Delta R_perp >= 3.0m displacement from leader/formation corridor
                if current_leader is not None:
                    peel_dir = getattr(drone, "peel_off_dir", np.zeros(3, dtype=float))
                    if float(np.linalg.norm(peel_dir[:2])) > 0.01:
                        lat_offset = abs(float(np.dot(drone.pos[:2] - current_leader.pos[:2], peel_dir[:2])))
                    else:
                        lat_offset = float(np.linalg.norm(drone.pos[:2] - current_leader.pos[:2]))
                    lat_cleared = lat_offset >= 3.0
                else:
                    lat_cleared = True

                # 3. Watchdog fallback (6.0s timeout)
                watchdog_tripped = drone.jettison_watchdog <= 0.0

                if (alt_cleared and lat_cleared) or watchdog_tripped:
                    drone.tether_jettisoned = True
                    drone.peel_off_timer = 0.0
                    drone.status = "ACTIVE"

        # 0d. Advance Drone Landing Stagger Clearance & Downwash Anti-Vortex Gate (TKT-42)
        for drone in self.drones:
            if not drone.crashed and drone.flight_state == "DESCENDING":
                if drone.landing_wait_timer > 0.0:
                    drone.landing_wait_timer = max(0.0, drone.landing_wait_timer - dt)

                if drone.landing_stage == "WAITING_CLEARANCE":
                    # Check Downwash Anti-Vortex Gate:
                    # Gated until preceding drone confirms touchdown (or crashed / dead)
                    gate_cleared = True
                    if getattr(drone, "gated_by_drone_id", None) is not None:
                        g_drone = self.get_drone_by_id(drone.gated_by_drone_id)
                        if g_drone and not g_drone.crashed:
                            is_down = (
                                g_drone.flight_state in ("LANDED", "GROUND_IDLE")
                                and getattr(g_drone, "touchdown_confirmed", False)
                            )
                            if not is_down:
                                gate_cleared = False

                    # Only advance to APPROACH if both initial timer and downwash gate are clear
                    if drone.landing_wait_timer <= 0.0 and gate_cleared:
                        drone.landing_stage = "APPROACH"

        # 0e. Batched ATC Recovery Turnaround Countdown (TKT-42)
        if getattr(self, "auto_turnaround_enabled", True) and self.recovery_stage == "LANDING":
            active_landed = True
            if self.landing_pads:
                for pad_idx, did in self.landing_pads.items():
                    d = self.get_drone_by_id(did)
                    if d and not d.crashed and d.flight_state != "LANDED":
                        active_landed = False
                        break
            else:
                active_landed = False

            waiting_drones = [
                d for d in self.drones
                if not d.crashed and d.flight_state == "OVERHEAD_HOLD"
            ]

            if active_landed and waiting_drones:
                if self.turnaround_timer <= 0.0:
                    self.turnaround_timer = 12.0
                else:
                    self.turnaround_timer = max(0.0, self.turnaround_timer - dt)
                    if self.turnaround_timer <= 0.0:
                        self.retrieve_all_landed_drones()
                        self.clear_pads()
            else:
                self.turnaround_timer = 0.0

        # 1. Update Physical RF Channel, Asymmetric Link Budget & Mesh Gateway (TKT-49)
        self._update_radio_comms()

        # 1b. Update In-Flight Dynamic Swarm Enrollment & P2P Handshakes (TKT-50)
        self._update_in_flight_enrollment(dt)

        # 2. Comms & Packet Protocol from active Brain Plugin
        self.brain.update_comms(dt)

        # 2. Check Tether Dynamics & Severance (Tethered Relay Mode)
        if self.comms_mode == "TETHERED_RELAY":
            self._update_tether_dynamics()

        # 3. Compute Guidance Targets from active Brain Plugin
        guidance_accs = self.brain.compute_guidance(dt)

        # 4. Compute 4-Zone Separation & Relative-Velocity Evasion
        # Bounded flight controller steering effort (not god-mode physics)
        avoidance_accs = self._compute_4zone_avoidance()

        # 5. Compute Tether Clearance Repulsion (if tethered)
        tether_accs = (
            self._compute_tether_avoidance()
            if self.comms_mode == "TETHERED_RELAY"
            else {d.id: np.zeros(3, dtype=float) for d in self.drones}
        )

        # 5a-2. Sequenced Pad Scramble Progression (TKT-41)
        if getattr(self, "scramble_active", False):
            self.scramble_timer += dt
            for d in self.drones:
                if not d.crashed and d.flight_state in ("GROUND_IDLE", "LANDED"):
                    if self.scramble_timer >= getattr(d, "scramble_time", 0.0):
                        d.flight_state = "TAKEOFF"
                        d.status = "ACTIVE"
                        d.landing_stage = "NONE"
                        d.rotor_rpm = 5200.0
                        d.vel = np.zeros(3, dtype=float)
                        d.acc = np.zeros(3, dtype=float)
                        if d.landing_pad_idx is not None and d.landing_pad_idx in self.landing_pads:
                            del self.landing_pads[d.landing_pad_idx]

            living = [d for d in self.drones if not d.crashed]
            if living and all(d.flight_state in ("EN_ROUTE", "CRUISE") for d in living):
                self.scramble_active = False

        # 5b. Flight State Machine Overrides (TAKEOFF, EN_ROUTE, DESCENDING, LANDED, GROUND_IDLE)
        for drone in self.drones:
            if drone.crashed:
                continue

            if drone.flight_state == "TAKEOFF":
                # Phase 1: Pure vertical climb along designated pad corridor to Z >= 20.0m
                pad_pos = drone.landing_pad if drone.landing_pad is not None else drone.pos
                to_pad_xy = pad_pos[:2] - drone.pos[:2]
                hold_ax = float(np.clip(3.0 * to_pad_xy[0] - 4.0 * drone.vel[0], -6.0, 6.0))
                hold_ay = float(np.clip(3.0 * to_pad_xy[1] - 4.0 * drone.vel[1], -6.0, 6.0))

                target_vz = 5.0
                climb_az = float(np.clip(4.0 * (target_vz - drone.vel[2]), 0.0, 8.0))
                guidance_accs[drone.id] = np.array([hold_ax, hold_ay, climb_az], dtype=float)

                # Near launch pad, suppress horizontal avoidance so drone ascends cleanly along corridor
                if drone.pos[2] < 5.0:
                    avoidance_accs[drone.id][0] = 0.0
                    avoidance_accs[drone.id][1] = 0.0

            elif drone.flight_state == "EN_ROUTE":
                # Phase 2: Autonomous transit toward swarm formation slot or active target
                # Check arrival proximity to transition to Phase 3: CRUISE (only once enrolled)
                if getattr(drone, "enrolled", True):
                    if self.active_brain_name == "LEADER_FOLLOWER":
                        current_leader = self.leader
                        if current_leader is not None:
                            if hasattr(self.brain, "_get_formation_slot"):
                                slot_pos = self.brain._get_formation_slot(drone, current_leader)
                                if float(np.linalg.norm(drone.pos - slot_pos)) <= 5.0:
                                    drone.flight_state = "CRUISE"
                        else:
                            drone.flight_state = "CRUISE"
                    elif self.active_brain_name == "GOSSIP_CONSENSUS":
                        target_check = getattr(drone, "target_station", None)
                        if target_check is None and hasattr(self.brain, "peer_states") and drone.id in self.brain.peer_states:
                            target_check = self.brain.peer_states[drone.id].active_target
                        if target_check is not None and float(np.linalg.norm(drone.pos - target_check)) <= 5.0:
                            drone.flight_state = "CRUISE"
                    else:
                        drone.flight_state = "CRUISE"

            elif drone.flight_state in ("LANDED", "GROUND_IDLE"):
                guidance_accs[drone.id] = np.zeros(3, dtype=float)
                avoidance_accs[drone.id] = np.zeros(3, dtype=float)
                tether_accs[drone.id] = np.zeros(3, dtype=float)
                drone.vel = np.zeros(3, dtype=float)
                drone.acc = np.zeros(3, dtype=float)
                drone.rotor_rpm = 0.0
                drone.vel = np.zeros(3, dtype=float)
                drone.acc = np.zeros(3, dtype=float)
                drone.rotor_rpm = 0.0

            elif drone.flight_state == "DESCENDING":
                pad_pos = (
                    drone.landing_pad
                    if drone.landing_pad is not None
                    else (self.get_pad_position(drone.landing_pad_idx) if drone.landing_pad_idx is not None else None)
                )
                if pad_pos is None:
                    # Safe overhead hover: do not dive to (0,0,0) if pad is unassigned
                    safe_hold = np.array([drone.pos[0], drone.pos[1], 25.0], dtype=float)
                    drone.target_station = safe_hold
                    accs_z = float(np.clip(3.0 * (25.0 - drone.pos[2]) - 2.0 * drone.vel[2], -4.0, 4.0))
                    guidance_accs[drone.id] = np.array([-3.0 * drone.vel[0], -3.0 * drone.vel[1], accs_z], dtype=float)
                    continue

                drone.target_station = pad_pos

                if drone.landing_wait_timer > 0.0 or drone.landing_stage == "WAITING_CLEARANCE":
                    drone.landing_stage = "WAITING_CLEARANCE"
                    # Active horizontal column hold above assigned pad to prevent radial drift/scatter (FIX-08)
                    to_pad_xy = pad_pos[:2] - drone.pos[:2]
                    ax_hold = float(np.clip(2.5 * to_pad_xy[0] - 3.0 * drone.vel[0], -6.0, 6.0))
                    ay_hold = float(np.clip(2.5 * to_pad_xy[1] - 3.0 * drone.vel[1], -6.0, 6.0))

                    # Staggered vertical holding shelves (2.0m spacing per pad) outside tether cylinder (Z >= 18m)
                    target_hold_z = 22.0 + (drone.landing_pad_idx or 0) * 2.0
                    az_hold = float(np.clip(3.0 * (target_hold_z - drone.pos[2]) - 2.0 * drone.vel[2], -4.0, 4.0))
                    guidance_accs[drone.id] = np.array([ax_hold, ay_hold, az_hold], dtype=float)
                else:
                    to_pad_xy = pad_pos[:2] - drone.pos[:2]
                    dist_xy = float(np.linalg.norm(to_pad_xy))

                    # 1. Horizontal Guidance toward Assigned Recovery Pad
                    if dist_xy > 0.15:
                        dir_xy = to_pad_xy / dist_xy
                        v_target_xy = min(4.0, max(0.8, dist_xy * 1.5)) * dir_xy
                        a_target_x = float(np.clip(3.5 * (v_target_xy[0] - drone.vel[0]), -6.0, 6.0))
                        a_target_y = float(np.clip(3.5 * (v_target_xy[1] - drone.vel[1]), -6.0, 6.0))
                    else:
                        a_target_x = float(-5.0 * drone.vel[0])
                        a_target_y = float(-5.0 * drone.vel[1])

                    # 2. Vertical Guidance: 2-Stage Deceleration Sink Rate
                    # Stage 1 Sink: vz = -1.5 m/s down to Z = 1.0m
                    # Stage 2 Cushion: vz = -0.5 m/s from Z = 1.0m down to Z = 0.5m
                    current_z = drone.pos[2]
                    if dist_xy > 2.5:
                        # Maintain safe approach altitude while closing horizontal distance
                        target_vz = float(np.clip(2.0 * (15.0 - current_z), -1.5, 2.0))
                        drone.landing_stage = "APPROACH"
                    elif current_z > 1.0:
                        target_vz = -1.5
                        drone.landing_stage = "APPROACH"
                    else:
                        target_vz = -0.5
                        drone.landing_stage = "FINAL_DESCENT"

                    a_target_z = float(np.clip(4.0 * (target_vz - drone.vel[2]), -6.0, 6.0))
                    guidance_accs[drone.id] = np.array([a_target_x, a_target_y, a_target_z], dtype=float)

                    # Suppress horizontal avoidance near ground (Z < 3.0m) to allow clean pad docking
                    if current_z < 3.0:
                        avoidance_accs[drone.id][0] *= 0.1
                        avoidance_accs[drone.id][1] *= 0.1

                    # Touchdown Check: Z <= 0.52m and dist_xy <= 0.35m
                    if current_z <= 0.52 and dist_xy <= 0.35:
                        # Ground Truth Pad Occupancy Check (Operator Incompetence Failsafe):
                        # Physical reality in WE: If another airframe is sitting on this pad, Zone 4 clash!
                        occupant = None
                        for other in self.drones:
                            if other.id == drone.id or getattr(other, "retrieved", False):
                                continue
                            d_other_xy = float(np.linalg.norm(other.pos[:2] - pad_pos[:2]))
                            if d_other_xy <= 1.5 and other.pos[2] <= 1.0:
                                occupant = other
                                break

                        if occupant is not None:
                            self._trigger_crash(drone, reason=f"UNRETRIEVED_PAD_COLLISION_WITH_DRONE_{occupant.id}")
                            if not occupant.crashed:
                                self._trigger_crash(occupant, reason=f"UNRETRIEVED_PAD_COLLISION_WITH_DRONE_{drone.id}")
                            continue

                        drone.pos = np.copy(pad_pos)
                        drone.vel = np.zeros(3, dtype=float)
                        drone.acc = np.zeros(3, dtype=float)
                        drone.flight_state = "LANDED"
                        drone.status = "LANDED"
                        drone.landing_stage = "TOUCHDOWN_CONFIRMED"
                        drone.touchdown_confirmed = True
                        drone.rotor_rpm = 0.0

                        # Succession if landed drone was fleet leader
                        if drone.is_leader:
                            drone.is_leader = False
                            living_airborne = [
                                d for d in self.drones
                                if not d.crashed and d.flight_state != "LANDED" and d.id != drone.id
                            ]
                            if living_airborne:
                                successor = max(living_airborne, key=lambda d: d.succession_score())
                                successor.is_leader = True
                                successor.tier = TIER_LEADER
                                successor.status = "FLEET_LEADER"

        # 6. Integrate Kinematics (Euler) & Boundary Clamps
        for drone in self.drones:
            if drone.crashed:
                # Motor disengaged: ballistic drop
                drone.acc = np.array([0.0, 0.0, GRAVITY], dtype=float)
                drone.vel[0] *= 0.98
                drone.vel[1] *= 0.98
                drone.vel[2] += GRAVITY * dt
                drone.pos += drone.vel * dt
                drone.rotor_rpm = max(0.0, drone.rotor_rpm - 300.0)

                # Ground contact clamp
                if drone.pos[2] <= 0.0:
                    drone.pos[2] = 0.0
                    drone.vel = np.zeros(3, dtype=float)
                    drone.acc = np.zeros(3, dtype=float)
                    drone.rotor_rpm = 0.0
                continue

            if drone.flight_state in ("LANDED", "GROUND_IDLE"):
                # Grounded on pad: motors disarmed, zero kinematics
                drone.acc = np.zeros(3, dtype=float)
                drone.vel = np.zeros(3, dtype=float)
                if drone.landing_pad is not None:
                    drone.pos = np.copy(drone.landing_pad)
                drone.rotor_rpm = 0.0
                continue

            # Priority Safety Overrides: attenuate forward guidance during emergency Zone 3 avoidance
            a_avoid = avoidance_accs[drone.id]
            avoid_mag = float(np.linalg.norm(a_avoid))
            a_guide = guidance_accs[drone.id]
            if avoid_mag > 8.0:
                attenuation = max(0.2, 1.0 - (avoid_mag - 8.0) / 25.0)
                a_guide = a_guide * attenuation

            # Sum and clamp accelerations
            total_acc = (
                a_guide
                + a_avoid
                + tether_accs[drone.id]
            )
            acc_mag = float(np.linalg.norm(total_acc))
            if acc_mag > A_MAX:
                total_acc = (total_acc / acc_mag) * A_MAX
            drone.acc = total_acc

            # Update velocity with clamp
            drone.vel += drone.acc * dt
            vel_mag = float(np.linalg.norm(drone.vel))
            if vel_mag > V_MAX:
                drone.vel = (drone.vel / vel_mag) * V_MAX

            # If TAKEOFF and near ground, damp horizontal velocity
            if drone.flight_state == "TAKEOFF" and drone.pos[2] < 5.0:
                drone.vel[0] *= 0.85
                drone.vel[1] *= 0.85

            # Update position
            drone.pos += drone.vel * dt

            # Transition from TAKEOFF to EN_ROUTE once Z >= 20.0m
            if drone.flight_state == "TAKEOFF" and drone.pos[2] >= 20.0:
                drone.flight_state = "EN_ROUTE"
                drone.landing_pad = None
                drone.landing_pad_idx = None
                # If no living leader exists in the fleet, promote this drone to leader
                has_living_leader = any(
                    d.is_leader and not d.crashed for d in self.drones if d.id != drone.id
                )
                if not has_living_leader:
                    drone.is_leader = True
                    drone.tier = TIER_LEADER
                    drone.status = "FLEET_LEADER"
                    drone.flight_state = "CRUISE"
                    drone.enrolled = True
                    drone.enrollment_status = "ENROLLED"

            # Strict safety floor clamp during breakaway dive (TKT-27)
            if drone.status == "BREAKAWAY_JETTISON" and not drone.tether_jettisoned:
                lead_z = self.leader.pos[2] if self.leader is not None else 20.0
                z_floor = max(3.0, lead_z - 4.5)
                if drone.pos[2] < z_floor:
                    drone.pos[2] = z_floor
                    if drone.vel[2] < 0.0:
                        drone.vel[2] = 0.0

            # Floor safety clamp (keep airborne above 0.5m, unless taking off from pad)
            floor_limit = 0.05 if drone.flight_state == "TAKEOFF" else 0.5
            if drone.pos[2] < floor_limit:
                drone.pos[2] = floor_limit
                if drone.vel[2] < 0.0:
                    drone.vel[2] = 0.0

            # Emergency Blind Settle Touchdown (TKT-52)
            if getattr(drone, "gnss_settled", False) and drone.pos[2] <= 0.52:
                drone.pos[2] = 0.5
                drone.vel = np.zeros(3, dtype=float)
                drone.acc = np.zeros(3, dtype=float)
                drone.flight_state = "LANDED"
                drone.status = "LANDED"
                drone.rotor_rpm = 0.0

            # Orientation update
            speed_xy = math.hypot(drone.vel[0], drone.vel[1])
            if speed_xy > 0.2:
                target_yaw = math.atan2(drone.vel[1], drone.vel[0])
                # Smooth yaw slew
                yaw_diff = (target_yaw - drone.yaw + math.pi) % (2.0 * math.pi) - math.pi
                drone.yaw += yaw_diff * min(1.0, 8.0 * dt)

            # Bank tilt from acceleration
            drone.pitch = float(np.clip(-drone.acc[0] * 0.04, -0.4, 0.4))
            drone.roll = float(np.clip(-drone.acc[1] * 0.04, -0.4, 0.4))

            # Battery drain
            drone.battery = max(0.0, drone.battery - 0.005 * dt)

    # ==========================================================================
    # 1. Radio Comms (10 Hz Baud), Asymmetric Link Budget & Mesh Gateway (TKT-49)
    # ==========================================================================
    def _update_radio_comms(self) -> None:
        """Simulate realistic 10 Hz radio broadcast packets, asymmetric link budget (Canine Taxonomy),
        Mesh Telemetry Gateway relay, and timeout-based Lost-Link state transitions (TKT-49)."""
        current_leader = self.leader

        # Check if 10 Hz broadcast timer fired
        broadcast_tick = False
        if self.sim_time - self.last_heartbeat_broadcast >= HEARTBEAT_INTERVAL:
            self.last_heartbeat_broadcast = self.sim_time
            broadcast_tick = True

        living = [d for d in self.drones if not d.crashed]

        # Reset crashed drones
        for d in self.drones:
            if d.crashed:
                d.uplink_active = False
                d.downlink_active = False
                d.yip_lost = False
                d.gateway_id = None

        if self.comms_mode == "GOSSIP_CONSENSUS":
            # 1. Asymmetric Link Budget: Uplink & Direct Downlink
            # High-power GCS Howl (140m) reaches all drones within 140m (unless jammed/dropped)
            # Direct Downlink: 90m for Tier-1 Leader (Bark), 50m for Tier-2 Escorts (Yip)
            direct_gateways: Set[int] = set()

            for d in living:
                d_gcs = float(np.linalg.norm(d.pos - self.tether_anchor))

                # Base Horizon High-Gain RF Dome (TKT-42):
                # Within 50m radius of base anchor, high-gain base station radio guarantees 100% link quality
                if d_gcs <= 50.0:
                    d.uplink_active = True
                    d.downlink_active = True
                    d.yip_lost = False
                    d.gateway_id = d.id
                    direct_gateways.add(d.id)
                    continue

                dropped_up = self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_HOWL)
                d.uplink_active = (d_gcs <= RANGE_HOWL and not dropped_up)

                r_down = RANGE_BARK if (d.tier == TIER_LEADER or d.is_leader) else RANGE_YIP
                dropped_down = self.is_packet_dropped(d_gcs, is_hardline=False, max_range=r_down)
                has_direct = (d_gcs <= r_down and not dropped_down)

                if has_direct:
                    d.downlink_active = True
                    d.gateway_id = d.id  # Acting as its own direct gateway
                    direct_gateways.add(d.id)
                else:
                    d.downlink_active = False
                    d.gateway_id = None

            # 2. Mesh Telemetry Gateway (The Relay Proxy)
            # Flood-fill from direct gateways over RANGE_P2P (60.0m) inter-drone mesh hops
            queue = [(gw_id, gw_id) for gw_id in direct_gateways]
            visited = set(direct_gateways)

            while queue:
                curr_id, root_gw = queue.pop(0)
                curr_drone = next((d for d in living if d.id == curr_id), None)
                if curr_drone is None:
                    continue

                for peer in living:
                    if peer.id not in visited:
                        d_peer = float(np.linalg.norm(peer.pos - curr_drone.pos))
                        dropped_peer = self.is_packet_dropped(d_peer, is_hardline=False, max_range=RANGE_P2P)
                        if d_peer <= RANGE_P2P and not dropped_peer:
                            peer.downlink_active = True
                            peer.gateway_id = root_gw
                            visited.add(peer.id)
                            queue.append((peer.id, root_gw))

            # 3. Yip-Lost State & Heartbeat / RSSI
            for d in living:
                # "Yip-Back" State: hears GCS commands (Howl) but cannot transmit telemetry back
                d.yip_lost = bool(d.uplink_active and not d.downlink_active)

                # RSSI: calculated against nearest active peer or GCS anchor
                d_gcs = float(np.linalg.norm(d.pos - self.tether_anchor))
                if d_gcs <= 50.0:
                    d.rssi, d.link_quality = -35.0, 1.0
                else:
                    min_dist = d_gcs
                    for peer in living:
                        if peer.id != d.id:
                            min_dist = min(min_dist, float(np.linalg.norm(d.pos - peer.pos)))
                    d.rssi, d.link_quality = self.calculate_rssi(min_dist, is_hardline=False)

                if broadcast_tick:
                    if d.downlink_active or d.uplink_active:
                        d.last_heartbeat_time = self.sim_time
                        d.lost_link = False
                        d.lkp = np.copy(d.pos)
                        if d.status == "LOST_LINK":
                            d.status = "ACTIVE"
                if self.sim_time - d.last_heartbeat_time > LINK_TIMEOUT:
                    d.lost_link = True
                    d.status = "LOST_LINK"

        elif self.comms_mode == "TETHERED_RELAY":
            # Primary leader on hardline umbilical
            for drone in living:
                if drone.id == 0 or drone.is_leader:
                    if not self.tether_severed:
                        drone.rssi, drone.link_quality = self.calculate_rssi(0.0, is_hardline=True)
                        drone.uplink_active = True
                        drone.downlink_active = True
                        drone.yip_lost = False
                        drone.gateway_id = None
                        drone.lost_link = False
                        drone.last_heartbeat_time = self.sim_time
                        if drone.status == "LOST_LINK":
                            drone.status = "FLEET_LEADER"
                    else:
                        d_gcs = float(np.linalg.norm(drone.pos - self.tether_anchor))
                        if d_gcs <= 50.0:
                            drone.rssi, drone.link_quality = -35.0, 1.0
                            drone.uplink_active = True
                            drone.downlink_active = True
                        else:
                            drone.rssi, drone.link_quality = self.calculate_rssi(d_gcs, is_hardline=False)
                            dropped_up = self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_HOWL)
                            dropped_down = self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_BARK)
                            drone.uplink_active = (d_gcs <= RANGE_HOWL and not dropped_up)
                            drone.downlink_active = (d_gcs <= RANGE_BARK and not dropped_down)
                        drone.yip_lost = bool(drone.uplink_active and not drone.downlink_active)
                        drone.gateway_id = None
                        if broadcast_tick and drone.uplink_active:
                            drone.last_heartbeat_time = self.sim_time
                            drone.lost_link = False
                            drone.lkp = np.copy(drone.pos)
                            if drone.status == "LOST_LINK":
                                drone.status = "FLEET_LEADER"
                        if self.sim_time - drone.last_heartbeat_time > LINK_TIMEOUT:
                            drone.lost_link = True
                            drone.status = "LOST_LINK"
                else:
                    # Escort in Tethered Relay
                    d_gcs = float(np.linalg.norm(drone.pos - self.tether_anchor))
                    dist_to_leader = float(np.linalg.norm(drone.pos - current_leader.pos)) if current_leader else 999.0
                    if d_gcs <= 50.0:
                        drone.rssi, drone.link_quality = -35.0, 1.0
                        drone.uplink_active = True
                        drone.downlink_active = True
                        drone.gateway_id = None
                    else:
                        dropped_up = self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_HOWL)
                        drone.uplink_active = (d_gcs <= RANGE_HOWL and not dropped_up)
                        drone.rssi, drone.link_quality = self.calculate_rssi(min(d_gcs, dist_to_leader), is_hardline=False)

                        dropped_lead = self.is_packet_dropped(dist_to_leader, is_hardline=False, max_range=RANGE_YIP)
                        if d_gcs <= RANGE_YIP and not self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_YIP):
                            drone.downlink_active = True
                            drone.gateway_id = None
                        elif current_leader and current_leader.downlink_active and dist_to_leader <= RANGE_YIP and not dropped_lead:
                            drone.downlink_active = True
                            drone.gateway_id = current_leader.id
                        else:
                            drone.downlink_active = False
                            drone.gateway_id = None

                    drone.yip_lost = bool(drone.uplink_active and not drone.downlink_active)

                    if broadcast_tick:
                        in_range_lead = (dist_to_leader <= RANGE_BARK and not self.is_packet_dropped(dist_to_leader, is_hardline=False, max_range=RANGE_BARK))
                        if current_leader and not current_leader.lost_link and in_range_lead:
                            drone.lkp = np.copy(current_leader.pos)
                            drone.last_heartbeat_time = self.sim_time
                            drone.lost_link = False
                            if drone.status == "LOST_LINK":
                                drone.status = "ACTIVE"
                    if self.sim_time - drone.last_heartbeat_time > LINK_TIMEOUT:
                        drone.lost_link = True
                        drone.status = "LOST_LINK"

        elif self.comms_mode == "LEADER_FOLLOWER":
            # Wireless Star
            for drone in living:
                if drone.is_leader:
                    d_gcs = float(np.linalg.norm(drone.pos - self.tether_anchor))
                    drone.rssi, drone.link_quality = self.calculate_rssi(d_gcs, is_hardline=False)
                    dropped_up = self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_HOWL)
                    dropped_down = self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_BARK)
                    drone.uplink_active = (d_gcs <= RANGE_HOWL and not dropped_up)
                    drone.downlink_active = (d_gcs <= RANGE_BARK and not dropped_down)
                    drone.yip_lost = bool(drone.uplink_active and not drone.downlink_active)
                    drone.gateway_id = None

                    if broadcast_tick and drone.uplink_active:
                        drone.last_heartbeat_time = self.sim_time
                        drone.lost_link = False
                        drone.lkp = np.copy(drone.pos)
                        if drone.status == "LOST_LINK":
                            drone.status = "FLEET_LEADER"
                    if self.sim_time - drone.last_heartbeat_time > LINK_TIMEOUT:
                        drone.lost_link = True
                        drone.status = "LOST_LINK"
                else:
                    d_gcs = float(np.linalg.norm(drone.pos - self.tether_anchor))
                    dropped_up = self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_HOWL)
                    drone.uplink_active = (d_gcs <= RANGE_HOWL and not dropped_up)

                    dist_to_leader = float(np.linalg.norm(drone.pos - current_leader.pos)) if current_leader else 999.0
                    drone.rssi, drone.link_quality = self.calculate_rssi(min(d_gcs, dist_to_leader), is_hardline=False)

                    dropped_lead = self.is_packet_dropped(dist_to_leader, is_hardline=False, max_range=RANGE_YIP)
                    if d_gcs <= RANGE_YIP and not self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_YIP):
                        drone.downlink_active = True
                        drone.gateway_id = None
                    elif current_leader and current_leader.downlink_active and dist_to_leader <= RANGE_YIP and not dropped_lead:
                        drone.downlink_active = True
                        drone.gateway_id = current_leader.id
                    else:
                        drone.downlink_active = False
                        drone.gateway_id = None

                    drone.yip_lost = bool(drone.uplink_active and not drone.downlink_active)

                    if broadcast_tick:
                        in_range_lead = (dist_to_leader <= RANGE_BARK and not self.is_packet_dropped(dist_to_leader, is_hardline=False, max_range=RANGE_BARK))
                        if current_leader and not current_leader.lost_link and in_range_lead:
                            drone.lkp = np.copy(current_leader.pos)
                            drone.last_heartbeat_time = self.sim_time
                            drone.lost_link = False
                            if drone.status == "LOST_LINK":
                                drone.status = "ACTIVE"
                    if self.sim_time - drone.last_heartbeat_time > LINK_TIMEOUT:
                        drone.lost_link = True
                        drone.status = "LOST_LINK"

        # Update Howl RF Beacon tracking for GNSS-denied airframes across all modes (TKT-52)
        for d in living:
            if getattr(d, "gnss_denied", False):
                d_gcs = float(np.linalg.norm(d.pos - self.tether_anchor))
                howl_rssi = float(np.clip(-20.0 - 20.0 * math.log10(max(1.0, d_gcs)), -100.0, -20.0))
                d.rssi = round(howl_rssi, 1)
                d.link_quality = max(0.0, min(1.0, (d.rssi - (-100.0)) / 60.0))

    # ==========================================================================
    # 1b. Dynamic In-Flight Swarm Enrollment & P2P Handshakes (TKT-50)
    # ==========================================================================
    def _update_in_flight_enrollment(self, dt: float) -> None:
        """Evaluate in-flight dynamic P2P enrollment handshakes for unenrolled scouts (TKT-50).
        Scouts clear initial pad climb (Z >= 18.0m) and seek to handshake with active enrolled peers
        within RANGE_P2P (60.0m) or leader within RANGE_BARK (90.0m)."""
        living = [d for d in self.drones if not d.crashed]
        enrolled_peers = [d for d in living if getattr(d, "enrolled", True)]

        # If no living enrolled peers exist (e.g. all original fleet died),
        # an ascending scout that clears Z >= 20m assumes leadership and becomes enrolled
        if not enrolled_peers:
            for d in living:
                if not getattr(d, "enrolled", True) and d.pos[2] >= 20.0:
                    d.enrolled = True
                    d.enrollment_status = "ENROLLED"
                    d.is_leader = True
                    d.tier = TIER_LEADER
                    d.status = "FLEET_LEADER"
                    enrolled_peers.append(d)
                    break
            return

        for scout in living:
            # Advance slot settling timer
            if getattr(scout, "enrollment_timer", 0.0) > 0.0:
                scout.enrollment_timer = max(0.0, scout.enrollment_timer - dt)

            # Check unenrolled scouts that have climbed out of pad effect (Z >= 18.0m)
            if not getattr(scout, "enrolled", True) and scout.pos[2] >= 18.0:
                # Find peers within P2P mesh range (60m) or leader within Bark (90m)
                peers_in_range = []
                for peer in enrolled_peers:
                    d_peer = float(np.linalg.norm(scout.pos - peer.pos))
                    max_contact = RANGE_BARK if (peer.is_leader or peer.tier == TIER_LEADER) else RANGE_P2P
                    if d_peer <= max_contact and not self.is_packet_dropped(d_peer, is_hardline=False, max_range=max_contact):
                        peers_in_range.append(peer)

                if peers_in_range:
                    # In-flight P2P Handshake triggered!
                    scout.enrolled = True
                    scout.enrollment_status = "ENROLLED"
                    scout.enrollment_timer = 2.5  # Smooth slot settling ramp

                    # State synchronization
                    if self.active_brain_name == "GOSSIP_CONSENSUS" and hasattr(self.brain, "peer_states"):
                        best_peer = max(
                            peers_in_range,
                            key=lambda p: getattr(self.brain.peer_states.get(p.id), "active_seq", -1)
                        )
                        if best_peer.id in self.brain.peer_states:
                            bp_state = self.brain.peer_states[best_peer.id]
                            scout_state = self.brain._get_peer_state(scout.id)
                            scout_state.active_seq = bp_state.active_seq
                            scout_state.active_target = np.copy(bp_state.active_target)
                            scout_state.pending_seq = bp_state.pending_seq
                            scout_state.pending_target = np.copy(bp_state.pending_target)
                            scout_state.gossip_status = "COORDINATED"
                            scout.acknowledged_target = np.copy(bp_state.active_target)
                    elif self.active_brain_name == "LEADER_FOLLOWER":
                        scout.acknowledged_target = np.copy(self.get_acknowledged_target())

                    # Clear lost-link flag if previously isolated on pad
                    scout.lost_link = False
                    scout.last_heartbeat_time = self.sim_time
                    if scout.status == "LOST_LINK":
                        scout.status = "ACTIVE"

                    enrolled_peers.append(scout)

    # ==========================================================================
    # 2. Tether Avoidance & Severance Dynamics
    # ==========================================================================
    def _update_tether_dynamics(self) -> None:
        """Check cable length limits and physical collisions by follower airframes.
        Physical contact <= 1.0m snaps cable and triggers autonomic succession."""
        if self.comms_mode != "TETHERED_RELAY":
            return
        current_leader = self.leader
        if current_leader is None or self.tether_severed or current_leader.flight_state in ("LANDED", "GROUND_IDLE"):
            return

        # Check maximum cable length extension
        cable_vec = current_leader.pos - self.tether_anchor
        cable_len = float(np.linalg.norm(cable_vec))
        if cable_len > TETHER_MAX_LEN:
            self._snap_tether(current_leader, reason="TENSILE_SNAP_MAX_LENGTH")
            return

        # Check collision with cable by any follower
        for drone in self.drones:
            if drone.crashed or drone.is_leader or drone.flight_state in ("LANDED", "GROUND_IDLE") or getattr(drone, "gnss_settled", False):
                continue

            d_perp, _ = self._dist_to_segment(
                drone.pos, self.tether_anchor, current_leader.pos
            )
            if d_perp <= TETHER_SNAP_DIST:
                self._snap_tether(
                    current_leader,
                    reason=f"FOLLOWER_COLLISION_DRONE_{drone.id}",
                )
                break

    def _snap_tether(self, leader_drone: Drone, reason: str = "") -> None:
        """Snap tether, initiate umbilical emergency breakaway dive on leader,
        dynamically promote 2IC, and initiate Doctrine A loiter timer (TKT-16 / TKT-27)."""
        self.tether_severed = True
        leader_drone.tether_lost = True
        leader_drone.jettison_watchdog = 6.0
        leader_drone.jettison_timer = 0.0
        leader_drone.tether_jettisoned = False
        leader_drone.status = "BREAKAWAY_JETTISON"

        # Abort mission route in Tethered Relay mode (TKT-28)
        self.route_active = False
        for d in self.drones:
            d.mission_active = False

        # Demote leader and promote dynamic 2IC
        self._trigger_autonomic_succession(demote_old_leader=True)

        # Engage Fleet Doctrine A: 30s defensive loiter before automated RTB
        self.lost_link_loiter_timer = 30.0
        self.swarm_rtb_active = False

    def _compute_tether_avoidance(self) -> Dict[int, np.ndarray]:
        """Apply lateral repulsion when followers enter the 3.0m cable clearance envelope."""
        accs = {d.id: np.zeros(3, dtype=float) for d in self.drones}
        if self.comms_mode != "TETHERED_RELAY":
            return accs
        current_leader = self.leader

        if current_leader is None or self.tether_severed or current_leader.flight_state in ("LANDED", "GROUND_IDLE"):
            return accs

        for drone in self.drones:
            if drone.crashed or drone.is_leader or drone.flight_state in ("LANDED", "GROUND_IDLE") or getattr(drone, "gnss_settled", False):
                continue

            d_perp, r_perp = self._dist_to_segment(
                drone.pos, self.tether_anchor, current_leader.pos
            )
            TETHER_CLEARANCE = 4.5
            if d_perp < TETHER_CLEARANCE:
                if d_perp < 0.01:
                    # Coincident with cable: push in positive X
                    push_dir = np.array([1.0, 0.0, 0.0], dtype=float)
                else:
                    push_dir = r_perp / d_perp

                mag = 20.0 * ((TETHER_CLEARANCE - d_perp) / TETHER_CLEARANCE)
                # Relative-velocity approach damping: brake incoming momentum before collision
                v_closing = float(np.dot(drone.vel, -push_dir))
                if v_closing > 0:
                    mag += v_closing * 2.5
                accs[drone.id] = push_dir * mag

        return accs

    @staticmethod
    def _dist_to_segment(
        point: np.ndarray, seg_a: np.ndarray, seg_b: np.ndarray
    ) -> Tuple[float, np.ndarray]:
        """Calculate Euclidean distance and perpendicular offset vector from a point
        to a line segment [seg_a, seg_b]."""
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
    # 3. Guidance Targets & Formation Brain
    # ==========================================================================
    def _compute_guidance_accelerations(self) -> Dict[int, np.ndarray]:
        """Compute guidance steering forces toward target waypoint, formation slots,
        or Last Known Position (LKP) if link is lost."""
        accs = {d.id: np.zeros(3, dtype=float) for d in self.drones}
        current_leader = self.leader

        for drone in self.drones:
            if drone.crashed:
                continue

            # 1. Leader Guidance: Track global target waypoint
            if drone.is_leader:
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, self.target_pos, max_speed=V_CRUISE
                )
                continue

            # 2. Lost-Link Guidance: Autonomous navigation toward LKP
            if drone.lost_link:
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, drone.lkp, max_speed=V_LKP
                )
                continue

            # 3. Active Follower Formation Guidance
            if current_leader is not None:
                slot_pos = self._get_formation_slot(drone, current_leader)
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, slot_pos, max_speed=V_CRUISE
                )
            else:
                # No surviving leader: loiter in place
                accs[drone.id] = -drone.vel * 2.0

        return accs

    def _get_formation_slot(self, drone: Drone, leader: Drone) -> np.ndarray:
        """Calculate the desired 3D slot coordinate for a follower based on the active formation mode."""
        if hasattr(self, "brain") and hasattr(self.brain, "_get_formation_slot"):
            return self.brain._get_formation_slot(drone, leader)

        # Fallback to local calculation if brain not initialized
        living_followers = [
            d for d in self.drones if not d.crashed and not d.is_leader and getattr(d, "enrolled", True)
        ]
        try:
            k = living_followers.index(drone) + 1
        except ValueError:
            k = drone.id

        if self.formation_mode == "ECHELON_ESCORT":
            # Staggered V-wedge behind leader
            speed_xy = math.hypot(leader.vel[0], leader.vel[1])
            if speed_xy > 0.5:
                heading = np.array(
                    [leader.vel[0] / speed_xy, leader.vel[1] / speed_xy, 0.0]
                )
            else:
                heading = np.array(
                    [math.cos(leader.yaw), math.sin(leader.yaw), 0.0]
                )
            lateral = np.array([-heading[1], heading[0], 0.0])

            row = (k + 1) // 2
            side = 1.0 if (k % 2 == 1) else -1.0
            return (
                leader.pos
                - (row * 6.0 * heading)
                + (side * row * 6.0 * lateral)
                + np.array([0.0, 0.0, row * 0.5])
            )

        elif self.formation_mode == "ORBIT_VORTEX":
            # Helical cylinder revolving around leader
            radius = 14.0
            omega = 0.8
            angle = omega * self.sim_time + (2.0 * math.pi * (k - 1) / max(1, len(living_followers)))
            z_offset = 3.0 * math.sin(2.0 * angle)
            return (
                leader.pos
                + np.array([radius * math.cos(angle), radius * math.sin(angle), z_offset])
            )

        elif self.formation_mode == "PERIMETER_PATROL":
            # Static defensive sentry ring holding equidistant guard stations around target
            radius = 25.0
            angle = (2.0 * math.pi * (k - 1)) / max(1, len(living_followers))
            return (
                self.target_pos
                + np.array([radius * math.cos(angle), radius * math.sin(angle), 0.0])
            )

        else:
            # NATURAL_FLOCK default: follow behind leader with minor offset
            row = (k + 1) // 2
            side = 1.0 if (k % 2 == 1) else -1.0
            return leader.pos + np.array([-row * 5.0, side * 5.0, 0.0])

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
        if dist < 0.25:  # 0.25m arrival deadband stops micro-hunting/jitter
            return -current_vel * 3.0

        if dist < arrival_radius:
            desired_speed = max_speed * (dist / arrival_radius)
        else:
            desired_speed = max_speed

        desired_vel = (offset / dist) * desired_speed
        steering_force = (desired_vel - current_vel) * 2.5
        return steering_force

    # ==========================================================================
    # 4. Sane 4-Zone Spatial Safety (Relative-Velocity Damping)
    # ==========================================================================
    def _compute_4zone_avoidance(self) -> Dict[int, np.ndarray]:
        """Compute pairwise collision avoidance across the 4 physical zones:
        - Zone 1 (d > 8.0m): Nominal clear, 0 force.
        - Zone 2 (3.5m < d <= 8.0m): Linear separation push.
        - Zone 3 (1.5m < d <= 3.5m): Critical push + active relative-velocity braking.
        - Zone 4 (d <= 1.5m): Prop clash, motors killed, ballistic crash."""
        accs = {d.id: np.zeros(3, dtype=float) for d in self.drones}
        n = len(self.drones)

        for i in range(n):
            drone_i = self.drones[i]
            if drone_i.crashed or drone_i.flight_state in ("LANDED", "GROUND_IDLE"):
                continue

            for j in range(i + 1, n):
                drone_j = self.drones[j]
                if drone_j.crashed or drone_j.flight_state in ("LANDED", "GROUND_IDLE"):
                    continue

                diff = drone_i.pos - drone_j.pos
                dist = float(np.linalg.norm(diff))

                # Check Zone 4: Physical Contact / Prop Clash (d <= 1.5m)
                if dist <= ZONE4_CLASH:
                    drone_i.collision_inferred = True
                    drone_j.collision_inferred = True
                    self._trigger_crash(drone_i, reason=f"CLASH_WITH_DRONE_{drone_j.id}")
                    self._trigger_crash(drone_j, reason=f"CLASH_WITH_DRONE_{drone_i.id}")
                    continue

                # Normal vector from j to i
                if dist < 0.01:
                    n_ij = np.array([1.0, 0.0, 0.0], dtype=float)
                else:
                    n_ij = diff / dist

                # Check Zone 3: Critical Danger (1.5m < d <= 3.5m)
                if dist <= ZONE3_OUTER:
                    # Evasive push magnitude
                    push_mag = 16.0 + 12.0 * ((ZONE3_OUTER - dist) / (ZONE3_OUTER - ZONE4_CLASH))
                    a_push_i = push_mag * n_ij
                    a_push_j = -push_mag * n_ij

                    # Horizontal lateral breakout and vertical corridor expansion to prevent stacking plunge
                    d_xy = float(math.hypot(diff[0], diff[1]))
                    if d_xy < 2.8:
                        if d_xy > 0.05:
                            n_xy = np.array([diff[0] / d_xy, diff[1] / d_xy, 0.0], dtype=float)
                        else:
                            n_xy = np.array([1.0, 0.0, 0.0], dtype=float)
                        lat_push = 14.0 * ((2.8 - d_xy) / 2.8)
                        a_push_i += lat_push * n_xy
                        a_push_j -= lat_push * n_xy

                        # Vertical corridor expansion: push vertically apart if Z difference < 2.0m
                        z_diff = diff[2]
                        if abs(z_diff) < 2.0:
                            z_sign = 1.0 if z_diff >= 0.0 else -1.0
                            z_push = 16.0 * ((2.0 - abs(z_diff)) / 2.0)
                            a_push_i[2] += z_sign * z_push
                            a_push_j[2] -= z_sign * z_push

                    # Relative-velocity damping (brakes incoming momentum)
                    # v_rel = v_i - v_j. Project onto n_ij.
                    v_rel = drone_i.vel - drone_j.vel
                    v_closing = float(np.dot(v_rel, n_ij))
                    if v_closing < 0.0:
                        # Approaching each other: apply opposing braking force
                        a_damp_i = -3.5 * (v_closing * n_ij)
                        a_damp_j = 3.5 * (v_closing * n_ij)
                    else:
                        a_damp_i = np.zeros(3, dtype=float)
                        a_damp_j = np.zeros(3, dtype=float)

                    accs[drone_i.id] += a_push_i + a_damp_i
                    accs[drone_j.id] += a_push_j + a_damp_j

                    # P2P Yip RSSI Emergency Cushion (TKT-52):
                    # Inter-drone close-range proximity cushion (hat_d < 2.2m <=> RSSI > -38 dBm)
                    if dist <= 2.2:
                        cushion_mag = 22.0 * (2.2 - dist)
                        accs[drone_i.id] += cushion_mag * n_ij
                        accs[drone_j.id] -= cushion_mag * n_ij

                # Check Zone 2: Caution Buffer (3.5m < d <= 8.0m)
                elif dist <= ZONE2_OUTER:
                    sep_mag = 5.0 * ((ZONE2_OUTER - dist) / (ZONE2_OUTER - ZONE3_OUTER))
                    accs[drone_i.id] += sep_mag * n_ij
                    accs[drone_j.id] -= sep_mag * n_ij

        return accs

    def _trigger_crash(self, drone: Drone, reason: str = "") -> None:
        """Disengage drone motors and trigger ballistic tumble."""
        drone.crashed = True
        drone.crashed_while_linked = bool(drone.downlink_active)
        drone.flight_state = "CRASHED"
        drone.status = "CRASHED"
        if drone.downlink_active:
            drone.mayday_beacon = True
            drone.mayday_reason = reason or "IMPACT_DETECTED"
        if hasattr(self, "fleet_registry"):
            self.fleet_registry.record_casualty(
                drone_id=drone.id,
                lkp=list(drone.pos),
                reason=reason,
                sim_time=self.sim_time,
            )
        was_leader = drone.is_leader
        if (drone.is_leader or drone.id == 0) and self.comms_mode == "TETHERED_RELAY":
            if not self.tether_severed:
                self.tether_severed = True
                drone.tether_lost = True
                drone.jettison_timer = 0.0
                drone.jettison_watchdog = 0.0
                drone.tether_jettisoned = True
                if self.lost_link_loiter_timer <= 0.0:
                    self.lost_link_loiter_timer = 30.0
                    self.swarm_rtb_active = False
        if was_leader:
            drone.is_leader = False
            self._trigger_autonomic_succession(demote_old_leader=True, retired_leader=drone)
        drone.mission_active = False

    # ==========================================================================
    # 5. Autonomic Leadership Succession
    # ==========================================================================
    def _trigger_autonomic_succession(
        self, demote_old_leader: bool = True, retired_leader: Optional[Drone] = None
    ) -> Optional[Drone]:
        """Elect the highest-scoring surviving follower to take over fleet guidance.
        Succession Score = (HardwareTier * 1000) - DroneID."""
        # Prioritize actively airborne candidates to prevent electing landed/descending drones
        airborne_candidates = [
            d for d in self.drones
            if not d.crashed and not d.is_leader and not d.tether_lost and d.flight_state not in ("LANDED", "DESCENDING", "RETRIEVED", "GROUND_IDLE")
        ]
        if not airborne_candidates:
            airborne_candidates = [
                d for d in self.drones
                if not d.crashed and not d.is_leader and d.flight_state not in ("LANDED", "DESCENDING", "RETRIEVED", "GROUND_IDLE")
            ]

        candidates = airborne_candidates if airborne_candidates else [
            d for d in self.drones
            if not d.crashed and not d.is_leader and not d.tether_lost
        ]
        if not candidates:
            # Fallback to any living drone if all tethered
            candidates = [d for d in self.drones if not d.crashed and not d.is_leader]

        if not candidates:
            return None

        # Sort by succession score descending
        candidates.sort(key=lambda d: d.succession_score(), reverse=True)
        new_leader = candidates[0]
        old_leader: Optional[Drone] = retired_leader

        # Demote previous leader
        if demote_old_leader:
            for drone in self.drones:
                if drone.id != new_leader.id and drone.is_leader:
                    old_leader = drone
                    drone.is_leader = False
                    if not drone.crashed:
                        if getattr(drone, "tether_lost", False) and not getattr(drone, "tether_jettisoned", False):
                            drone.status = "BREAKAWAY_JETTISON"
                        else:
                            drone.status = "ACTIVE"
                        # Lateral Peel-Off Corridor Deconfliction (TKT-15)
                        speed_xy = math.hypot(drone.vel[0], drone.vel[1])
                        if speed_xy > 0.5:
                            heading = np.array([drone.vel[0] / speed_xy, drone.vel[1] / speed_xy, 0.0], dtype=float)
                        else:
                            heading = np.array([math.cos(drone.yaw), math.sin(drone.yaw), 0.0], dtype=float)
                        lat = np.array([-heading[1], heading[0], 0.0], dtype=float)

                        # Determine relative lateral displacement of promoted successor
                        rel_pos = new_leader.pos - drone.pos
                        d_lat = float(np.dot(rel_pos, lat))

                        # Peel away from successor's approach side (anti-jousting)
                        peel_dir = -lat if d_lat > 0.0 else lat
                        norm_peel = float(np.linalg.norm(peel_dir))
                        if norm_peel > 0.01:
                            peel_dir = peel_dir / norm_peel
                        else:
                            peel_dir = np.array([0.0, 1.0, 0.0], dtype=float)

                        drone.peel_off_timer = 2.5
                        drone.peel_off_dir = peel_dir

        # Promote new leader
        new_leader.is_leader = True
        new_leader.tier = TIER_LEADER
        new_leader.status = "PROMOTED_LEADER"
        new_leader.lost_link = False
        new_leader.promotion_timer = 1.5
        if old_leader is not None:
            new_leader.acknowledged_target = np.copy(old_leader.acknowledged_target)
            new_leader.mission_waypoints = [np.copy(wp) for wp in old_leader.mission_waypoints]
            new_leader.current_wp_idx = old_leader.current_wp_idx
            new_leader.mission_active = old_leader.mission_active
            new_leader.mission_loop = old_leader.mission_loop
            new_leader.wp_loiter_timer = getattr(old_leader, "wp_loiter_timer", 0.0)
            new_leader.wp_loitering = getattr(old_leader, "wp_loitering", False)

        # Notify active brain hook of succession
        if hasattr(self, "brain") and hasattr(self.brain, "on_succession"):
            self.brain.on_succession(old_leader, new_leader)

        return new_leader

    # ==========================================================================
    # 6. Mission Controls & Casualty Laboratory
    # ==========================================================================
    @staticmethod
    def get_pad_position(pad_idx: int) -> np.ndarray:
        """Calculate coordinates for recovery/launch pad index (0..7).
        Base pad layout outside the 3.0m tether clearance envelope.
        Centered at (0.0, -6.0, 0.5) with multi-drone grid layout (>= 2.5m separation)."""
        idx = pad_idx % 8
        pad_x = float((idx % 4 - 1.5) * 2.5)
        pad_y = float(-6.0 - (idx // 4) * 3.0)
        return np.array([pad_x, pad_y, 0.5], dtype=float)

    def is_in_comms_range(self) -> bool:
        """Check whether at least one living drone is within the GCS 120m RF radio horizon."""
        living = [d for d in self.drones if not d.crashed]
        if not living:
            return False
        min_dist = min(float(np.linalg.norm(d.pos - self.tether_anchor)) for d in living)
        return min_dist <= RADIO_RANGE

    def command_rtb_hold(self, force_autonomous: bool = False) -> Dict[str, Any]:
        """Stage 1 Recovery: Commanded RTB (Hold).
        Fleet transits to Base (0, 0, 25m) at cruise speed and settles into
        a stationary sentry ring at Z=20-25m outside 3m tether clearance cylinder.
        Enforces 120m Radio Horizon Airgap: dropped if fleet is outside comms range."""
        if not force_autonomous and not self.is_in_comms_range():
            return {
                "status": "DROPPED",
                "reason": "OUT_OF_RANGE",
                "error": f"Fleet is beyond GCS radio horizon ({RADIO_RANGE}m). RTB command dropped.",
            }

        self.overhead_marshal_active = True
        self.swarm_rtb_active = True
        self.recovery_stage = "OVERHEAD_MARSHAL"
        self.patrol_active = False
        self.set_target([0.0, 0.0, 25.0])
        for drone in self.drones:
            if not drone.crashed and drone.flight_state not in ("LANDED", "DESCENDING"):
                drone.flight_state = "OVERHEAD_HOLD"
                drone.status = "OVERHEAD_HOLD"
                drone.acknowledged_target = np.array([0.0, 0.0, 25.0], dtype=float)
                drone.lkp = np.array([0.0, 0.0, 25.0], dtype=float)
        return {"status": "OK", "recovery_stage": self.recovery_stage}

    def cancel_rtb(self) -> None:
        """Cancel RTB / overhead marshaling and return living airborne drones to mission cruise."""
        self.overhead_marshal_active = False
        self.swarm_rtb_active = False
        self.lost_link_loiter_timer = 0.0
        self.recovery_stage = "IDLE"
        self.turnaround_timer = 0.0
        self.atc_current_wave = 1
        self.atc_waiting_queue.clear()
        if self.tether_severed and self.comms_mode == "TETHERED_RELAY":
            self.comms_mode = "LEADER_FOLLOWER"
        for drone in self.drones:
            if not drone.crashed and drone.flight_state in ("OVERHEAD_HOLD", "DESCENDING"):
                drone.flight_state = "CRUISE"
                drone.status = "FLEET_LEADER" if drone.is_leader else "ACTIVE"
                drone.landing_stage = "NONE"
                drone.gated_by_drone_id = None
                drone.touchdown_confirmed = False
                if drone.landing_pad_idx is not None and drone.landing_pad_idx in self.landing_pads:
                    del self.landing_pads[drone.landing_pad_idx]
                drone.landing_pad_idx = None

    def command_land_swarm(self) -> Union[List[int], Dict[str, Any]]:
        """Stage 2 Recovery: Operator-authorized coordinated landing.
        Engages ATC pad reservation (max 8 pads), sequential staggered descent
        with Downwash Anti-Vortex Gate, and overhead marshal wave holding (TKT-42).
        Enforces 120m Radio Horizon Airgap: dropped if fleet is outside comms range."""
        if not self.is_in_comms_range():
            return {
                "status": "DROPPED",
                "reason": "OUT_OF_RANGE",
                "error": f"Fleet is beyond GCS radio horizon ({RADIO_RANGE}m). Landing command dropped.",
            }

        airborne_drones = [
            d for d in self.drones
            if not d.crashed and not getattr(d, "retrieved", False) and d.flight_state not in ("LANDED", "GROUND_IDLE", "DESCENDING", "RETRIEVED")
        ]
        if not airborne_drones:
            return []

        airborne_drones.sort(key=lambda d: d.id)
        self.recovery_stage = "LANDING"

        # Bounded to ATC pad capacity (0..7)
        available_pads = [p for p in range(self.atc_capacity) if p not in self.landing_pads]
        batch_drones = airborne_drones[:len(available_pads)]
        marshal_drones = airborne_drones[len(available_pads):]

        queued_ids = []
        for i, drone in enumerate(batch_drones):
            pad_idx = available_pads[i]
            self.landing_pads[pad_idx] = drone.id
            drone.landing_pad_idx = pad_idx
            drone.landing_pad = self.get_pad_position(pad_idx)
            drone.landing_wait_timer = i * 2.0  # 2.0s minimum clearance interval
            drone.flight_state = "DESCENDING"
            drone.status = "LANDING"
            drone.touchdown_confirmed = False
            if i == 0:
                drone.landing_stage = "APPROACH"
                drone.gated_by_drone_id = None
            else:
                drone.landing_stage = "WAITING_CLEARANCE"
                drone.gated_by_drone_id = batch_drones[i - 1].id
            queued_ids.append(drone.id)

            # If leader is descending, abdicate and trigger succession among remaining airborne drones
            if drone.is_leader:
                drone.is_leader = False
                self._trigger_autonomic_succession(demote_old_leader=True, retired_leader=drone)

        # Remaining airborne drones hold in circular orbit at overhead marshal shelves (Z in [25, 35]m)
        if marshal_drones:
            self.overhead_marshal_active = True
            for k, drone in enumerate(marshal_drones):
                drone.flight_state = "OVERHEAD_HOLD"
                drone.status = "MARSHAL_HOLD"
                drone.landing_stage = "MARSHAL_HOLD"
                drone.landing_pad_idx = None
                drone.landing_pad = None
                drone.touchdown_confirmed = False
                drone.gated_by_drone_id = None
                shelf_z = 25.0 + (k % 5) * 2.0  # Shelves in [25, 35]m
                angle = (2.0 * math.pi * k) / max(1, len(marshal_drones))
                orbit_r = 18.0
                drone.acknowledged_target = np.array([orbit_r * math.cos(angle), orbit_r * math.sin(angle), shelf_z], dtype=float)
                drone.target_station = np.copy(drone.acknowledged_target)
                if drone.id not in self.atc_waiting_queue:
                    self.atc_waiting_queue.append(drone.id)
        else:
            self.overhead_marshal_active = False
            self.atc_waiting_queue.clear()

        return queued_ids

    def command_land_drone(self, drone_id: int) -> bool:
        """Command an individual drone to land on an available recovery pad."""
        if not self.is_in_comms_range():
            return False
        for drone in self.drones:
            if drone.id == drone_id and not drone.crashed and drone.flight_state not in ("LANDED", "DESCENDING"):
                if not getattr(drone, "uplink_active", True):
                    return False
                used_pads = set(self.landing_pads.keys())
                available = [p for p in range(self.atc_capacity) if p not in used_pads]
                if not available:
                    return False
                pad_idx = available[0]
                self.landing_pads[pad_idx] = drone.id
                drone.landing_pad_idx = pad_idx
                drone.landing_pad = self.get_pad_position(pad_idx)
                drone.landing_wait_timer = 0.0
                drone.flight_state = "DESCENDING"
                drone.status = "LANDING"
                drone.landing_stage = "APPROACH"
                drone.gated_by_drone_id = None
                drone.touchdown_confirmed = False

                # If landing drone was fleet leader, abdicate and trigger succession immediately
                if drone.is_leader:
                    drone.is_leader = False
                    self._trigger_autonomic_succession(demote_old_leader=True, retired_leader=drone)
                return True
        return False

    def command_takeoff_drone(self, drone_id: int) -> bool:
        """Relaunch a landed or ground-idle drone from its pad via vertical takeoff."""
        if not self.is_in_comms_range():
            return False
        for drone in self.drones:
            if drone.id == drone_id and drone.flight_state in ("LANDED", "GROUND_IDLE"):
                drone.flight_state = "TAKEOFF"
                drone.status = "LOST_LINK" if drone.lost_link else "ACTIVE"
                drone.landing_stage = "NONE"
                drone.rotor_rpm = 5200.0
                drone.vel = np.zeros(3, dtype=float)
                drone.acc = np.zeros(3, dtype=float)
                if drone.landing_pad_idx is not None and drone.landing_pad_idx in self.landing_pads:
                    del self.landing_pads[drone.landing_pad_idx]
                return True
        return False

    def retrieve_drone_from_pad(self, pad_idx: int) -> Optional[int]:
        """Ground crew physically retrieves and stows an airframe or wreckage from a landing pad into the hangar."""
        pad_pos = self.get_pad_position(pad_idx)
        assigned_id = self.landing_pads.get(pad_idx, None)
        target_drone = self.get_drone_by_id(assigned_id) if assigned_id is not None else None

        if target_drone is None or getattr(target_drone, "retrieved", False):
            for d in self.drones:
                if getattr(d, "retrieved", False):
                    continue
                d_xy = float(np.linalg.norm(d.pos[:2] - pad_pos[:2]))
                if d_xy <= 1.5 and d.pos[2] <= 1.0:
                    target_drone = d
                    break

        if target_drone is not None and not getattr(target_drone, "retrieved", False):
            target_drone.retrieved = True
            target_drone.flight_state = "RETRIEVED"
            target_drone.status = "RETRIEVED"
            target_drone.pos = np.array([-100.0 - pad_idx * 5.0, -100.0, 0.0], dtype=float)
            target_drone.vel = np.zeros(3, dtype=float)
            target_drone.acc = np.zeros(3, dtype=float)
            target_drone.rotor_rpm = 0.0
            if pad_idx in self.landing_pads and self.landing_pads[pad_idx] == target_drone.id:
                del self.landing_pads[pad_idx]
            return target_drone.id

        return None

    def retrieve_all_landed_drones(self) -> List[int]:
        """Ground crew physically retrieves all landed/ground-idle airframes across all pads into hangar."""
        retrieved = []
        for pad_idx in range(self.atc_capacity):
            did = self.retrieve_drone_from_pad(pad_idx)
            if did is not None:
                retrieved.append(did)
        return retrieved

    def clear_pads(self, turnaround_rearm: bool = False) -> Dict[str, Any]:
        """Reset/clear occupied pads (salvaging/storing landed drones or re-arming)
        and release the next wave of airborne drones from overhead marshal hold to land (TKT-42)."""
        cleared_drones = []
        for pad_idx, drone_id in list(self.landing_pads.items()):
            d = self.get_drone_by_id(drone_id)
            if d and d.flight_state in ("LANDED", "GROUND_IDLE"):
                d.landing_pad_idx = None
                d.landing_stage = "NONE"
                if turnaround_rearm:
                    d.flight_state = "GROUND_IDLE"
                    d.status = "GROUND_IDLE"
                    d.rotor_rpm = 0.0
                else:
                    d.flight_state = "LANDED"
                    d.status = "LANDED"
                del self.landing_pads[pad_idx]
                cleared_drones.append(drone_id)

        self.turnaround_timer = 0.0

        waiting_airborne = [
            d for d in self.drones
            if not d.crashed and d.flight_state == "OVERHEAD_HOLD"
        ]

        released_ids = []
        if waiting_airborne:
            self.atc_current_wave += 1
            res = self.command_land_swarm()
            if isinstance(res, list):
                released_ids = res

        return {
            "status": "OK",
            "cleared_pads": len(cleared_drones),
            "cleared_drone_ids": cleared_drones,
            "released": len(released_ids),
            "released_drone_ids": released_ids,
            "current_wave": self.atc_current_wave,
        }

    def command_scramble_swarm(self, stagger_s: float = 1.2) -> Dict[str, Any]:
        """Initiate sequenced cold scramble from ground pads (TKT-41).
        Flagship #0 spins up and departs immediately; followers are staggered
        every stagger_s (default 1.2s) to maintain downwash clearance.
        Climb along designated pad corridor to Z >= 20.0m before mustering."""
        if not self.is_in_comms_range():
            return {
                "status": "DROPPED",
                "reason": "OUT_OF_RANGE",
                "error": f"Fleet is beyond GCS radio horizon ({RADIO_RANGE}m). Scramble command dropped.",
            }

        grounded = [d for d in self.drones if not d.crashed and d.flight_state in ("GROUND_IDLE", "LANDED")]
        if not grounded:
            return {"status": "NOOP", "reason": "NO_GROUNDED_DRONES"}

        # Sort: leader first if present, then followers by id
        leader_drones = [d for d in grounded if d.is_leader]
        follower_drones = sorted([d for d in grounded if not d.is_leader], key=lambda d: d.id)

        launch_sequence = leader_drones + follower_drones
        if not any(d.is_leader for d in self.drones if not d.crashed):
            if launch_sequence:
                launch_sequence[0].is_leader = True
                launch_sequence[0].tier = TIER_LEADER

        self.scramble_stagger = float(stagger_s)
        self.scramble_timer = 0.0
        self.scramble_active = True

        stagger_order = []
        for idx, drone in enumerate(launch_sequence):
            drone.scramble_time = round(idx * self.scramble_stagger, 2)
            stagger_order.append({"drone_id": drone.id, "scramble_time": drone.scramble_time})
            if drone.scramble_time <= 0.0:
                drone.flight_state = "TAKEOFF"
                drone.status = "ACTIVE"
                drone.landing_stage = "NONE"
                drone.rotor_rpm = 5200.0
                drone.vel = np.zeros(3, dtype=float)
                drone.acc = np.zeros(3, dtype=float)
                if drone.landing_pad_idx is not None and drone.landing_pad_idx in self.landing_pads:
                    del self.landing_pads[drone.landing_pad_idx]

        return {
            "status": "SCRAMBLE_INITIATED",
            "drone_count": len(launch_sequence),
            "stagger_s": self.scramble_stagger,
            "sequence": stagger_order,
        }

    def command_cold_start_swarm(self, count: Optional[int] = None) -> Dict[str, Any]:
        """Cold ground reset: resets or grounds fleet onto numbered ground launch pads (TKT-41)."""
        n = count if count is not None else len(self.drones)
        self.cold_start = True
        self._init_fleet(n, cold_start=True)
        return {"status": "COLD_START_ACTIVE", "drone_count": n}

    def spawn_reinforcement(
        self,
        rejoin_target: Optional[Union[np.ndarray, List[float]]] = None,
        auto_launch: bool = False,
    ) -> Drone:
        """Deploy a new reinforcement drone from the base launch pad (TKT-41 & TKT-50).
        Assigns the next sequential drone ID (max_id + 1).
        Spawns on the base launch pad as an unenrolled Scout.
        If auto_launch=False:
            - Places drone cold on pad in GROUND_IDLE (Z=0.05m, 0 RPM).
            - Generates tactical pre-briefing:
                - Active contact: 'FLEET_ACTIVE: Gang in contact at [pos]. Rendezvous target [target].'
                - Comms lost: 'FLEET_LOST: Gang lost near [pos] en route to [target].'
        If auto_launch=True:
            - Immediately spools up motors to 5200 RPM in TAKEOFF corridor climb.
        """
        current_max = max((d.id for d in self.drones), default=-1)
        self._highest_drone_id = max(getattr(self, "_highest_drone_id", -1), current_max) + 1
        new_id = self._highest_drone_id

        # Base launch pad coordinates outside the 3.0m tether clearance envelope
        pad_idx = new_id % 8
        spawn_pos = self.get_pad_position(pad_idx)
        if not auto_launch:
            spawn_pos[2] = 0.05

        # Allocate standby reserve from FleetRegistry (TKT-48)
        if hasattr(self, "fleet_registry"):
            reserve_rec = self.fleet_registry.allocate_reserve(
                drone_id=new_id, pad_id=pad_idx, sim_time=self.sim_time
            )
            hw_uid = reserve_rec.hardware_uid
            tail_num = reserve_rec.tail_number
        else:
            hw_uid = f"3842A10F9B20FF{new_id:02X}"
            tail_num = f"ALITHEIA-RES-{new_id:02d}"

        drone = Drone(
            drone_id=new_id,
            tier=TIER_ESCORT,
            pos=spawn_pos,
            is_leader=False,
            hardware_uid=hw_uid,
            tail_number=tail_num,
        )
        drone.enrolled = False
        drone.enrollment_status = "SCOUT"
        drone.vel = np.zeros(3, dtype=float)
        drone.acc = np.zeros(3, dtype=float)
        drone.landing_pad_idx = pad_idx
        drone.landing_pad = np.copy(spawn_pos)
        self.landing_pads[pad_idx] = drone.id

        current_leader = self.leader
        ack_tgt = self.get_acknowledged_target()

        # Generate pre-briefing
        if rejoin_target is not None:
            drone.rejoin_target = np.array(rejoin_target, dtype=float)
            drone.lkp = np.copy(drone.rejoin_target)
            drone.last_heartbeat_time = self.sim_time
            drone.lost_link = False
            drone.briefing = f"CUSTOM_MISSION: Rendezvous vector designated at [{drone.rejoin_target[0]:.1f}, {drone.rejoin_target[1]:.1f}, {drone.rejoin_target[2]:.1f}]."
        elif current_leader is not None and not current_leader.crashed:
            dist_to_leader = float(np.linalg.norm(spawn_pos - current_leader.pos))
            if dist_to_leader <= RANGE_HOWL and self.is_in_comms_range():
                drone.lkp = np.copy(current_leader.pos)
                drone.rejoin_target = np.copy(ack_tgt)
                drone.last_heartbeat_time = self.sim_time
                drone.lost_link = False
                lead_pos_str = f"[{current_leader.pos[0]:.1f}, {current_leader.pos[1]:.1f}, {current_leader.pos[2]:.1f}]"
                tgt_str = f"[{ack_tgt[0]:.1f}, {ack_tgt[1]:.1f}, {ack_tgt[2]:.1f}]"
                drone.briefing = f"FLEET_ACTIVE: Gang in contact at {lead_pos_str}. Rendezvous target {tgt_str}."
            else:
                lead_pos_str = f"[{current_leader.pos[0]:.1f}, {current_leader.pos[1]:.1f}, {current_leader.pos[2]:.1f}]"
                tgt_str = f"[{ack_tgt[0]:.1f}, {ack_tgt[1]:.1f}, {ack_tgt[2]:.1f}]"
                drone.lkp = np.array([0.0, 0.0, 20.0], dtype=float)
                drone.rejoin_target = np.array([0.0, 0.0, 20.0], dtype=float)
                drone.last_heartbeat_time = -100.0
                drone.lost_link = True
                drone.status = "LOST_LINK"
                drone.briefing = f"FLEET_LOST: Gang lost near {lead_pos_str} en route to {tgt_str}."
        else:
            drone.lkp = np.array([0.0, 0.0, 20.0], dtype=float)
            drone.rejoin_target = np.array([0.0, 0.0, 20.0], dtype=float)
            drone.last_heartbeat_time = -100.0
            drone.lost_link = True
            drone.status = "LOST_LINK"
            drone.briefing = "FLEET_OFFLINE: No active leader broadcast detected. Holding base corridor."

        if auto_launch:
            drone.flight_state = "TAKEOFF"
            drone.status = "LOST_LINK" if drone.lost_link else "ACTIVE"
            drone.rotor_rpm = 5200.0
            if drone.landing_pad_idx is not None and drone.landing_pad_idx in self.landing_pads:
                del self.landing_pads[drone.landing_pad_idx]
        else:
            drone.flight_state = "GROUND_IDLE"
            drone.status = "GROUND_IDLE"
            drone.rotor_rpm = 0.0

        self.drones.append(drone)

        # Do NOT register as an active peer in Gossip yet; enrollment occurs on P2P handshake!
        if hasattr(self.brain, "peer_states") and hasattr(self.brain, "_get_peer_state"):
            p_state = self.brain._get_peer_state(new_id)
            p_state.gossip_status = "SCOUT"

        return drone

    def launch_reinforcement(self, rejoin_target: Optional[Union[np.ndarray, List[float]]] = None) -> Drone:
        """Deploy reinforcement and immediately launch (backward-compatible alias)."""
        return self.spawn_reinforcement(rejoin_target=rejoin_target, auto_launch=True)

    def kill_leader(self) -> None:
        """Trigger emergency motor cut on active leader."""
        current_leader = self.leader
        if current_leader is not None:
            self._trigger_crash(current_leader, reason="OPERATOR_KILL_COMMAND")

    def kill_drone(self, drone_id: int) -> None:
        """Kill motors on a specific drone by ID."""
        for drone in self.drones:
            if drone.id == drone_id and not drone.crashed:
                self._trigger_crash(drone, reason="OPERATOR_TARGET_KILL")
                break

    def reboot_drone(self, drone_id: int) -> None:
        """Safely redeploy a crashed drone into the formation via vertical takeoff."""
        if hasattr(self, "fleet_registry"):
            self.fleet_registry.reboot_airframe(drone_id)
        for drone in self.drones:
            if drone.id == drone_id:
                ground_pos = np.array([drone.pos[0], drone.pos[1], 0.5], dtype=float)
                drone.reset_state(ground_pos, is_leader=False, flight_state="TAKEOFF")
                break

    def reboot_all_dead(self) -> List[int]:
        """Reboot and revive all crashed casualties in-place from ground via vertical takeoff
        without resetting surviving drones, simulation clock, or active targets."""
        revived_ids: List[int] = []
        crashed_drones = [d for d in self.drones if d.crashed]
        if not crashed_drones:
            return revived_ids

        has_living_leader = any(d.is_leader and not d.crashed for d in self.drones)

        for i, drone in enumerate(crashed_drones):
            if hasattr(self, "fleet_registry"):
                self.fleet_registry.reboot_airframe(drone.id)
            make_leader = (not has_living_leader and i == 0)
            ground_pos = np.array([drone.pos[0], drone.pos[1], 0.5], dtype=float)
            # Give slight stagger if multiple crashed at the exact same location
            ground_pos[0] += (drone.id % 3 - 1) * 2.0
            ground_pos[1] += ((drone.id // 3) % 3 - 1) * 2.0

            drone.reset_state(ground_pos, is_leader=make_leader, flight_state="TAKEOFF")
            drone.status = "PROMOTED_LEADER" if make_leader else "ACTIVE"
            if make_leader:
                has_living_leader = True
            revived_ids.append(drone.id)

        return revived_ids

    def clear_crashed_wreckage(self) -> List[int]:
        """Purge all destroyed/crashed drones from the simulation.
        Returns list of removed drone IDs."""
        purged_ids = [d.id for d in self.drones if d.crashed]
        if not purged_ids:
            return []

        self.drones = [d for d in self.drones if not d.crashed]

        # Clean up peer states if in gossip consensus brain mode
        if hasattr(self.brain, "peer_states"):
            for pid in purged_ids:
                self.brain.peer_states.pop(pid, None)

        # Clean up any pads held by purged drones
        for pid in list(self.landing_pads.keys()):
            if self.landing_pads[pid] in purged_ids:
                del self.landing_pads[pid]

        # If fleet has surviving drones but no active leader, elect one
        if self.leader is None and self.drones:
            self._trigger_autonomic_succession(demote_old_leader=False)

        return purged_ids

    def reboot_fleet(self) -> None:
        """Reset and reboot all drones back to nominal starting states."""
        self._init_fleet(len(self.drones))
        self.lost_link_loiter_timer = 0.0
        self.swarm_rtb_active = False
        self.overhead_marshal_active = False
        self.recovery_stage = "IDLE"
        self.landing_pads.clear()
        self.brain.reset()

    def toggle_gnss_denial(
        self, drone_id: Optional[int] = None, denied: Optional[bool] = None
    ) -> bool:
        """Toggle or explicitly set GNSS/GPS denial for a specific drone or the entire fleet (TKT-52).
        Returns new denial state (True if denied, False if nominal)."""
        if drone_id is not None:
            drone = self.get_drone_by_id(drone_id)
            if drone is None:
                return False
            new_state = (not drone.gnss_denied) if denied is None else bool(denied)
            self._set_drone_gnss_denial(drone, new_state)
            return new_state
        else:
            new_state = (not self.gnss_denied_all) if denied is None else bool(denied)
            self.gnss_denied_all = new_state
            for drone in self.drones:
                if not drone.crashed:
                    self._set_drone_gnss_denial(drone, new_state)
            return new_state

    def _set_drone_gnss_denial(self, drone: Drone, state: bool) -> None:
        """Apply or lift tactical GNSS denial on an individual airframe."""
        drone.gnss_denied = state
        if state:
            drone.gnss_denied_timer = 0.0
            # Lock entry track reciprocal heading: psi_reciprocal = (yaw + 180 deg) % 360 deg
            drone.reciprocal_heading = float((drone.yaw + math.pi) % (2.0 * math.pi))
            drone.rssi_prev = float(drone.rssi)
            drone.drift_radius = 0.5
            drone.gnss_settled = False
            drone.mission_active = False
            drone.status = "GNSS_DENIED"
            # Broadcast MAVLink STATUSTEXT
            if hasattr(self, "mavlink") and self.mavlink is not None:
                self.mavlink.send_mavlink_statustext(
                    "GNSS DENIED: DEAD-RECKONING RTB",
                    severity=2,  # MAV_SEVERITY_CRITICAL
                    src_sys=drone.id + 1,
                )
        else:
            drone.gnss_denied_timer = 0.0
            drone.drift_radius = 0.0
            drone.gnss_settled = False
            if drone.status == "GNSS_DENIED":
                drone.status = "FLEET_LEADER" if drone.is_leader else "ACTIVE"

    def cut_tether(self) -> None:
        """Operator manual tether severance directive."""
        current_leader = self.leader
        if current_leader is not None:
            self._snap_tether(current_leader, reason="OPERATOR_SEVER_TETHER")

    def set_formation(self, mode: str, force: bool = False) -> Dict[str, Any]:
        """Switch active formation flight mode. Gated by radio horizon unless forced."""
        if not force and not self.is_in_comms_range():
            return {
                "status": "DROPPED",
                "reason": "OUT_OF_RANGE",
                "error": f"Fleet is beyond GCS radio horizon ({RADIO_RANGE}m). Formation command dropped.",
                "formation": self.formation_mode,
            }

        mode_upper = mode.upper().replace("-", "_").strip()
        if mode_upper in ("BOIDS", "BOID", "FLOCK"):
            mode_upper = "NATURAL_FLOCK"

        valid_modes = [
            "ECHELON_ESCORT",
            "NATURAL_FLOCK",
            "ORBIT_VORTEX",
            "PERIMETER_PATROL",
        ]
        if mode_upper in valid_modes:
            self.formation_mode = mode_upper
            if hasattr(self.brain, "formation_mode"):
                self.brain.formation_mode = mode_upper
            return {"status": "OK", "formation": self.formation_mode}
        return {"status": "INVALID_MODE", "formation": self.formation_mode}

    def dispatch_target_uplink(self, target_pos: Union[List[float], np.ndarray]) -> Dict[str, Any]:
        """Attempt to deliver a commanded target waypoint across the simulated C2 link (tether or RF).
        If the physical channel is broken (severed tether or out of 120m RF range),
        the uplink packet is dropped and drone onboard buffers remain untouched."""
        target_arr = np.array(target_pos, dtype=float)

        # 1. TETHERED_RELAY: Hardline umbilical to Drone 0
        if self.comms_mode == "TETHERED_RELAY":
            if self.tether_severed:
                return {
                    "status": "DROPPED",
                    "reason": "TETHER_SEVERED",
                    "error": "Tactical umbilical is severed. Uplink packet dropped.",
                }
            # Hardline delivery to tethered leader
            for d in self.drones:
                if d.id == 0 or d.is_leader:
                    d.acknowledged_target = np.copy(target_arr)
            return {
                "status": "OK",
                "channel": "HARDLINE_UMBILICAL",
                "target": [round(float(v), 2) for v in target_arr],
            }

        # 2. LEADER_FOLLOWER: Wireless Star to active fleet leader
        elif self.comms_mode == "LEADER_FOLLOWER":
            current_leader = self.leader
            if current_leader is None or current_leader.crashed:
                return {
                    "status": "DROPPED",
                    "reason": "NO_LEADER",
                    "error": "No living fleet leader to receive wireless uplink.",
                }
            d_gcs = float(np.linalg.norm(current_leader.pos - self.tether_anchor))
            if d_gcs > RADIO_RANGE:
                return {
                    "status": "OUT_OF_RANGE",
                    "reason": "RADIO_RANGE_EXCEEDED",
                    "error": f"Leader Drone #{current_leader.id} ({d_gcs:.1f}m) out of GCS radio range ({RADIO_RANGE}m)",
                }
            current_leader.acknowledged_target = np.copy(target_arr)
            return {
                "status": "OK",
                "channel": "WIRELESS_STAR",
                "target": [round(float(v), 2) for v in target_arr],
            }

        # 3. GOSSIP_CONSENSUS: Wireless P2P Mesh Ingestion
        elif self.comms_mode == "GOSSIP_CONSENSUS":
            if hasattr(self.brain, "inject_target"):
                res = self.brain.inject_target(target_arr)
                if isinstance(res, dict) and res.get("status") == "OUT_OF_RANGE":
                    return res
                return res if isinstance(res, dict) else {
                    "status": "OK",
                    "channel": "P2P_MESH",
                    "target": [round(float(v), 2) for v in target_arr],
                }
            return {"status": "OK", "channel": "P2P_MESH"}

        return {"status": "ERROR", "error": f"Unknown comms mode: {self.comms_mode}"}

    def set_target(self, pos: Union[List[float], np.ndarray]) -> Dict[str, Any]:
        """Command a new 3D mission target waypoint from GCS and attempt uplink dispatch."""
        return self.c2.set_target(pos)

    def toggle_target_patrol(self, active: Optional[bool] = None) -> bool:
        """Toggle or explicitly set continuous 3D target auto-patrol loop."""
        return self.c2.toggle_target_patrol(active)

    def start_route_preset(self, preset_name: str) -> Dict[str, Any]:
        """Deploy and start a predefined route preset (TKT-28)."""
        return self.c2.start_route_preset(preset_name)

    def deploy_mission_route(
        self,
        waypoints: Union[List[List[float]], List[np.ndarray]],
        loop: bool = True,
        name: str = "CUSTOM",
    ) -> Dict[str, Any]:
        """Deploy discrete waypoint mission route to the swarm over physical comms uplink (TKT-28)."""
        return self.c2.deploy_mission_route(waypoints, loop=loop, name=name)

    def stop_mission_route(self) -> Dict[str, Any]:
        """Stop active mission route and hold current position (TKT-28)."""
        return self.c2.stop_mission_route()

    def is_target_reached(self, threshold: float = 2.5) -> bool:
        """Deterministic arrival check: whether active swarm guidance has arrived within threshold."""
        return self.c2.is_target_reached(threshold)

    def generate_safe_rng_target(self) -> np.ndarray:
        """Generate a high-variance, guaranteed-SAFE 3D target coordinate."""
        return self.c2.generate_safe_rng_target()

    def toggle_rng_recon(self, active: Optional[bool] = None) -> bool:
        """Toggle or explicitly set tactical RNG Touch-and-Go reconnaissance mode (TKT-29)."""
        return self.c2.toggle_rng_recon(active)

    def start_rng_recon(self) -> Dict[str, Any]:
        """Activate RNG touch-and-go reconnaissance mode (TKT-29)."""
        return self.c2.start_rng_recon()

    def stop_rng_recon(self) -> Dict[str, Any]:
        """Deactivate RNG touch-and-go reconnaissance mode (TKT-29)."""
        return self.c2.stop_rng_recon()

    # ==========================================================================
    # 7. Telemetry Snapshot Serialization
    # ==========================================================================
    def get_state_snapshot(self) -> Dict[str, Any]:
        """Generate high-throughput JSON-serializable dictionary for GCS and UDP."""
        current_leader = self.leader
        tether_len = 0.0
        if current_leader is not None and not self.tether_severed:
            tether_len = float(np.linalg.norm(current_leader.pos - self.tether_anchor))

        two_ic = self.get_2ic()
        two_ic_id = two_ic.id if two_ic else None

        drones_list = []
        sim_drones_list = []
        for d in self.drones:
            d_dict = d.to_dict()
            d_dict["is_2ic"] = (d.id == two_ic_id and not d.is_leader and not d.crashed)
            sim_drones_list.append(dict(d_dict))

            # Operator Airgap: If drone crashed while communication was down (beyond radio horizon / lost link),
            # GCS only knows it is [LOST_LINK] at its Last Known Position (LKP).
            if d.crashed and not getattr(d, "crashed_while_linked", False) and not d.downlink_active:
                d_dict["crashed"] = False
                d_dict["flight_state"] = "LOST_LINK"
                d_dict["status"] = "LOST_LINK"
                d_dict["pos"] = [round(float(v), 2) for v in d.lkp]
                d_dict["vel"] = [0.0, 0.0, 0.0]
                d_dict["lost_link"] = True

            drones_list.append(d_dict)

        ack_target = self.get_acknowledged_target()

        # Determine active route current waypoint index & loiter state (TKT-28 & TKT-30)
        if self.active_brain_name == "GOSSIP_CONSENSUS":
            current_wp_idx = getattr(self.brain, "current_route_wp_idx", 0)
            current_loitering = getattr(self.brain, "wp_loitering", False)
            current_loiter_timer = getattr(self.brain, "wp_loiter_timer", 0.0)
        elif current_leader is not None:
            current_wp_idx = current_leader.current_wp_idx
            current_loitering = getattr(current_leader, "wp_loitering", False)
            current_loiter_timer = getattr(current_leader, "wp_loiter_timer", 0.0)
        else:
            current_wp_idx = 0
            current_loitering = False
            current_loiter_timer = 0.0

        route_data = {
            "active": self.route_active,
            "name": self.route_name,
            "loop": self.route_loop,
            "current_wp_idx": current_wp_idx,
            "waypoints": [[round(float(v), 2) for v in wp] for wp in self.route_waypoints],
            "loitering": current_loitering,
            "loiter_timer": round(max(0.0, float(current_loiter_timer)), 1),
        }

        return {
            "sim_time": round(float(self.sim_time), 3),
            "dt": DT,
            "home_position": {
                "lat": self.home_lat,
                "lon": self.home_lon,
                "alt": self.home_alt,
            },
            "brain": self.active_brain_name,
            "comms_mode": self.comms_mode,
            "rf_jamming": round(float(self.rf_jamming_rate), 2),
            "rf_link_budget": {
                "howl_gcs_uplink": RANGE_HOWL,
                "bark_leader_downlink": RANGE_BARK,
                "yip_escort_downlink": RANGE_YIP,
                "p2p_mesh": RANGE_P2P,
            },
            "gcs_radio_range": RADIO_RANGE,
            "formation": getattr(self.brain, "formation_mode", self.formation_mode),
            "target": [round(float(v), 2) for v in self.target_pos],
            "commanded_target": [round(float(v), 2) for v in self.target_pos],
            "acknowledged_target": [round(float(v), 2) for v in ack_target],
            "target_reached": self.is_target_reached(),
            "mission_route": route_data,
            "patrol_active": self.patrol_active,
            "rng_recon": {
                "active": self.rng_recon_active,
                "dwelling": self.rng_dwelling,
                "dwell_timer": round(max(0.0, float(self.rng_dwell_timer)), 1),
                "dwell_time": float(self.rng_dwell_time),
                "station_count": int(self.rng_station_count),
                "status": self.rng_status,
                "paused": self.rng_paused,
            },
            "two_ic_id": two_ic_id,
            "lost_link_loiter_timer": round(max(0.0, float(self.lost_link_loiter_timer)), 1),
            "swarm_rtb_active": self.swarm_rtb_active,
            "overhead_marshal_active": self.overhead_marshal_active,
            "recovery_stage": self.recovery_stage,
            "cold_start": bool(getattr(self, "cold_start", False)),
            "scramble_active": bool(getattr(self, "scramble_active", False)),
            "scramble_timer": round(float(getattr(self, "scramble_timer", 0.0)), 2),
            "landing_pads": {int(k): v for k, v in self.landing_pads.items()},
            "atc": {
                "capacity": self.atc_capacity,
                "current_wave": self.atc_current_wave,
                "turnaround_timer": round(float(self.turnaround_timer), 1),
                "waiting_marshal_count": len([d for d in self.drones if not d.crashed and d.flight_state == "OVERHEAD_HOLD"]),
                "landed_count": len([d for d in self.drones if not d.crashed and d.flight_state in ("LANDED", "GROUND_IDLE")]),
                "pads_occupied": len(self.landing_pads),
            },
            "tether": {
                "anchor": [round(float(v), 2) for v in self.tether_anchor],
                "leader_id": current_leader.id if current_leader else -1,
                "severed": self.tether_severed if self.comms_mode == "TETHERED_RELAY" else False,
                "length": round(tether_len, 2),
                "deployed": (self.comms_mode == "TETHERED_RELAY" and (current_leader is not None and current_leader.flight_state not in ("LANDED", "GROUND_IDLE"))),
                "visible": (self.comms_mode == "TETHERED_RELAY" and not self.tether_severed and (current_leader is not None and current_leader.flight_state not in ("LANDED", "GROUND_IDLE"))),
            },
            "drones": drones_list,
            "sim_drones": sim_drones_list,
            "fleet_registry": self.fleet_registry.get_registry_snapshot() if hasattr(self, "fleet_registry") else {},
            "brain_telemetry": self.brain.get_telemetry_extra(),
        }


# ==============================================================================
# Backward-Compatible Class Alias
# ==============================================================================
SwarmSimulator = SITLWorldEngine

__all__ = [
    "DT",
    "BODY_RADIUS",
    "ZONE4_CLASH",
    "ZONE3_OUTER",
    "ZONE2_OUTER",
    "V_CRUISE",
    "V_MAX",
    "V_LKP",
    "A_MAX",
    "GRAVITY",
    "RANGE_HOWL",
    "RANGE_BARK",
    "RANGE_YIP",
    "RANGE_P2P",
    "RADIO_RANGE",
    "HEARTBEAT_INTERVAL",
    "LINK_TIMEOUT",
    "TETHER_ANCHOR",
    "TETHER_AVOID_DIST",
    "TETHER_SNAP_DIST",
    "TETHER_MAX_LEN",
    "TIER_LEADER",
    "TIER_ESCORT",
    "TIER_SCOUT",
    "ROUTE_PRESETS",
    "Drone",
    "SITLWorldEngine",
    "SwarmSimulator",
]


if __name__ == "__main__":
    import argparse
    import time
    from SIM.backend.mavlink_bridge import MavlinkBridge

    parser = argparse.ArgumentParser(description="SwarmTech3 SITL World Simulation Engine")
    parser.add_argument("--count", type=int, default=8, help="Number of drones")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="UDP host")
    parser.add_argument("--tx-port", type=int, default=9000, help="UDP Telemetry TX port")
    parser.add_argument("--rx-port", type=int, default=9001, help="UDP Command RX port")
    parser.add_argument("--mavlink-port", type=int, default=14550, help="UDP MAVLink port")
    args = parser.parse_args()

    engine = SITLWorldEngine(drone_count=args.count)
    bridge = MavlinkBridge(
        sim=engine,
        host=args.host,
        tx_port=args.tx_port,
        rx_port=args.rx_port,
        mavlink_port=args.mavlink_port,
    )

    print(f"[SITL] Engine active: {args.count} drones @ 60 FPS", flush=True)
    print(f"[SITL] Telemetry TX (UDP {args.tx_port}) | Command RX (UDP {args.rx_port}) | MAVLink (UDP {args.mavlink_port})", flush=True)

    try:
        while True:
            t0 = time.perf_counter()
            engine.step(DT)
            bridge.step()
            dt_step = time.perf_counter() - t0
            sleep_time = max(0.0, DT - dt_step)
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\n[SITL] Simulation engine terminated by operator.", flush=True)
    finally:
        bridge.close()


