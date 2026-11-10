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

from OBC.backend.drone_firmware import (
    Drone, GroundRepeater, FlightState, TIER_LEADER, TIER_ESCORT, TIER_SCOUT,
    BASE_TIER_T0, BASE_TIER_T1_5, BASE_TIER_T3,
    TETHER_NONE, TETHER_DATA_ONLY, TETHER_POWER_DATA,
    RF_MODE_YIP, RF_MODE_BORK, RF_MODE_HOWL,
)
from OBC.backend.brains import (
    BaseBrain,
    RadioPacket,
    HeardPeer,
    LocalTelemetry,
    FlightIntent,
    BRAIN_REGISTRY,
    LeaderFollowerBrain,
    GossipConsensusBrain,
)
from OBC.backend.brains.gossip_consensus import DronePeerState
from GCS.backend.c2_gateway import C2Gateway, ROUTE_PRESETS
from SIM.backend.fleet_registry import FleetRegistry
from SIM.backend.urban_geometry import (
    UrbanObstacle,
    point_to_aabb_distance,
    point_to_aabb_repulsion,
    ray_intersects_aabb,
    get_default_urban_obstacles,
    get_cyberpunk_obby_obstacles,
    get_surface_elevation,
    get_overhead_ceiling,
    is_on_obstacle_top,
)
from SIM.backend.rf_channel import (
    RFChannelManager,
    is_rf_occluded as rf_is_occluded,
    RANGE_HOWL,
    RANGE_BARK,
    RANGE_BORK,
    RANGE_YIP,
    RANGE_P2P,
    RADIO_RANGE,
    HEARTBEAT_INTERVAL,
    LINK_TIMEOUT,
    RF_MODE_BORK,
)
from SIM.backend.atc_recovery import (
    ATCRecoveryManager,
    get_pad_position,
    ATC_DEFAULT_CAPACITY,
)


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
        is_tethered: Optional[bool] = None,
        comms_mode: Optional[str] = None,
        urban_scenario: Optional[str] = None,
    ):
        if count is not None:
            drone_count = count
        if start_in_air is not None:
            cold_start = not start_in_air
        if seed is not None:
            np.random.seed(seed)

        self.sim_time: float = 0.0
        if is_tethered is False:
            self.comms_mode: str = "LEADER_FOLLOWER"
        elif comms_mode is not None:
            self.comms_mode = comms_mode
        else:
            self.comms_mode = "TETHERED_RELAY"  # "TETHERED_RELAY", "LEADER_FOLLOWER", "GOSSIP_CONSENSUS"

        # Reference Geographic Home Anchor (Bubble 3: World Ground Truth)
        # Default: Tokyo Bay / Shinagawa Canal Waterfront (Tennozu Isle)
        self.home_lat: float = float(home_lat if home_lat is not None else 35.625971)
        self.home_lon: float = float(home_lon if home_lon is not None else 139.734759)
        self.home_alt: float = float(home_alt if home_alt is not None else 20.0)  # Meters AMSL

        # Global Mission State
        self.formation_mode: str = "ECHELON_ESCORT"  # ECHELON_ESCORT, NATURAL_FLOCK, ORBIT_VORTEX, PERIMETER_PATROL
        self.tether_anchor: np.ndarray = np.copy(TETHER_ANCHOR)
        self.tether_severed: bool = False

        # Bubble 2 Subsystems: RF Channel & ATC Recovery Managers (TKT-61)
        self.rf_channel: RFChannelManager = RFChannelManager(tether_anchor=self.tether_anchor)
        self.rf_channel.enable_packet_loss = enable_packet_loss
        self.atc: ATCRecoveryManager = ATCRecoveryManager(capacity=8)

        # Cold-Start Pad Scramble & Sequenced Climb-Out Corridor (TKT-41)
        self.cold_start: bool = cold_start

        # Bubble 1: Tactical C2 Gateway (GCS Intent & Mission Layer)
        self.c2: C2Gateway = C2Gateway(self)
        self.battery_drain_multiplier: float = 1.0  # Tactical power drain multiplier (TKT-21 / TKT-58)
        self.tactical_power_drain: bool = True

        # Fleet Doctrine A (Lost-Link Failsafe Loiter & Automated RTB) (TKT-16)
        self.lost_link_loiter_timer: float = 0.0
        self.lost_link_policy: str = "DOCTRINE_A"

        # Tactical GNSS Denial & Dead-Reckoning RTB (TKT-52)
        self.gnss_denied_all: bool = False

        # Stationary Ground Repeaters & Dropped Payloads (TKT-59 / PLN-04)
        self.ground_repeaters: List[GroundRepeater] = []
        self.falling_payloads: List[Dict[str, Any]] = []

        # Urban 3D Geometry & Static Obstacles (PLN-08 / TKT-60)
        self.urban_obstacles: List[UrbanObstacle] = []
        if urban_scenario:
            self.load_urban_scenario(urban_scenario)

        # Build Drone Fleet
        self.drones: List[Drone] = []
        self._highest_drone_id: int = drone_count - 1
        self._init_fleet(drone_count)

        # Pluggable Guidance Brain Plugin (Default: LeaderFollowerBrain per drone)
        self.active_brain_name: str = "LEADER_FOLLOWER"
        self.brain: BaseBrain = _LeaderFollowerSimProxy(self)

    def _primary_drone_brain(self) -> Optional[BaseBrain]:
        """Return the leader's (or lowest-ID living drone's) onboard brain instance."""
        lead = self.leader
        if lead is not None and hasattr(lead, "brain") and lead.brain is not None:
            return lead.brain
        for d in self.drones:
            if not d.crashed and hasattr(d, "brain") and d.brain is not None:
                return d.brain
        return self.drones[0].brain if self.drones else None

    def _build_local_telemetry(self, drone: Drone) -> LocalTelemetry:
        """Build an immutable LocalTelemetry snapshot for a single airframe.
        Enforces GNSS Denial Airgap: masks horizontal pos[:2] to NaN when gnss_denied=True."""
        if getattr(drone, "gnss_denied", False):
            pos_obs = np.array([np.nan, np.nan, float(drone.pos[2])], dtype=float)
        else:
            pos_obs = np.copy(drone.pos)

        tether_active = (
            self.comms_mode == "TETHERED_RELAY"
            and not self.tether_severed
        )
        wps_tuple = tuple(np.copy(wp) for wp in getattr(drone, "mission_waypoints", []))

        return LocalTelemetry(
            drone_id=drone.id,
            tier=drone.tier,
            is_leader=drone.is_leader,
            enrolled=bool(getattr(drone, "enrolled", True)),
            flight_state=drone.flight_state,
            status=drone.status,
            clock_s=float(self.sim_time),
            pos=pos_obs,
            vel=np.copy(drone.vel),
            yaw=float(drone.yaw),
            reciprocal_heading=float(getattr(drone, "reciprocal_heading", 0.0)),
            rssi=float(drone.rssi),
            rssi_prev=float(getattr(drone, "rssi_prev", -100.0)),
            link_quality=float(drone.link_quality),
            lost_link=bool(drone.lost_link),
            uplink_active=bool(drone.uplink_active),
            downlink_active=bool(drone.downlink_active),
            gnss_denied=bool(getattr(drone, "gnss_denied", False)),
            gnss_denied_timer=float(getattr(drone, "gnss_denied_timer", 0.0)),
            drift_radius=float(getattr(drone, "drift_radius", 0.0)),
            gnss_settled=bool(getattr(drone, "gnss_settled", False)),
            battery=float(drone.battery),
            acknowledged_target=np.copy(drone.acknowledged_target),
            lkp=np.copy(drone.lkp),
            backtrack_target=np.copy(drone.backtrack_target) if getattr(drone, "backtrack_target", None) is not None else None,
            holding_at_horizon=bool(getattr(drone, "holding_at_horizon", False)),
            rejoin_target=np.copy(drone.rejoin_target) if getattr(drone, "rejoin_target", None) is not None else None,
            mission_active=bool(getattr(drone, "mission_active", False)),
            mission_paused=bool(getattr(drone, "mission_paused", False) or self.route_paused),
            mission_loop=bool(getattr(drone, "mission_loop", True)),
            mission_waypoints=wps_tuple,
            current_wp_idx=int(getattr(drone, "current_wp_idx", 0)),
            wp_loitering=bool(getattr(drone, "wp_loitering", False)),
            wp_loiter_timer=float(getattr(drone, "wp_loiter_timer", 0.0)),
            wp_total_loiter=float(getattr(drone, "wp_total_loiter", 0.0)),
            route_acceptance_radius=float(self.route_acceptance_radius),
            peel_off_timer=float(getattr(drone, "peel_off_timer", 0.0)),
            peel_off_dir=np.copy(getattr(drone, "peel_off_dir", np.zeros(3, dtype=float))),
            promotion_timer=float(getattr(drone, "promotion_timer", 0.0)),
            enrollment_timer=float(getattr(drone, "enrollment_timer", 0.0)),
            tether_active=bool(tether_active),
            tether_lost=bool(getattr(drone, "tether_lost", False)),
            tether_jettisoned=bool(getattr(drone, "tether_jettisoned", False)),
            tether_failsafe_policy=str(getattr(drone, "tether_failsafe_policy", "REJOIN")),
            tether_anchor_pos=np.copy(self.tether_anchor),
            overhead_marshal_active=bool(self.overhead_marshal_active),
            patrol_active=bool(self.patrol_active),
            tether_heartbeat_watchdog=float(getattr(drone, "tether_heartbeat_watchdog", 0.5)),
            corridor_target=np.copy(drone.corridor_target) if getattr(drone, "corridor_target", None) is not None else None,
            urban_obstacles=tuple(getattr(self, "urban_obstacles", [])),
            escort_leader_id=getattr(drone, "escort_leader_id", None),
            corridor_stack=tuple(drone.corridor_stack._stack) if hasattr(drone, "corridor_stack") else (),
        )

    def _build_transceiver_beacon(self, sender: Drone) -> RadioPacket:
        """Build an RF telemetry beacon packet from the airframe's local registers."""
        pos_list = None if getattr(sender, "gnss_denied", False) else [float(v) for v in sender.pos]
        b = getattr(sender, "brain", None)
        if sender.is_leader:
            return RadioPacket(
                sender_id=sender.id,
                msg_type="LEADER_HEARTBEAT",
                timestamp=float(self.sim_time),
                payload={
                    "pos": pos_list,
                    "vel": [float(v) for v in sender.vel],
                    "yaw": float(sender.yaw),
                    "is_leader": True,
                    "tier": int(sender.tier),
                    "crashed": bool(sender.crashed),
                    "enrolled": bool(getattr(sender, "enrolled", True)),
                    "flight_state": str(sender.flight_state),
                    "acknowledged_target": [float(v) for v in sender.acknowledged_target],
                    "formation_mode": getattr(b, "formation_mode", self.formation_mode),
                    "formation_heading": float(getattr(b, "formation_heading", 0.0)),
                    "assigned_slots": dict(getattr(b, "assigned_slots", {})),
                    "gcs_contact": bool(sender.uplink_active),
                    "gateway_id": getattr(sender, "gateway_id", None),
                },
            )
        return RadioPacket(
            sender_id=sender.id,
            msg_type="ESCORT_YIP",
            timestamp=float(self.sim_time),
            payload={
                "pos": pos_list,
                "vel": [float(v) for v in sender.vel],
                "yaw": float(sender.yaw),
                "is_leader": False,
                "tier": int(sender.tier),
                "crashed": bool(sender.crashed),
                "enrolled": bool(getattr(sender, "enrolled", True)),
                "flight_state": str(sender.flight_state),
                "acknowledged_target": [float(v) for v in sender.acknowledged_target],
                "gcs_contact": bool(sender.uplink_active),
                "gateway_id": getattr(sender, "gateway_id", None),
            },
        )

    def _deliver_radio_packets(self, dt: float) -> None:
        """Deliver queued outbox packets across physical RF medium to peer inboxes."""
        living = [
            d for d in self.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING")
        ]
        all_senders = [d for d in self.drones if d.outbox]
        for sender in all_senders:
            out_packets = list(sender.outbox)
            sender.outbox.clear()
            max_tx = RADIO_RANGE if self.active_brain_name == "LEADER_FOLLOWER" else (
                RANGE_BARK if (sender.is_leader or sender.tier == TIER_LEADER) else RANGE_P2P
            )
            for pkt in out_packets:
                for receiver in living:
                    if receiver.id == sender.id:
                        continue
                    dist = float(np.linalg.norm(sender.pos - receiver.pos))
                    r_limit = RANGE_P2P if pkt.msg_type in ("GOSSIP_ECHO", "PEER_BEACON") else max_tx
                    if dist <= r_limit and not self.is_packet_dropped(dist, is_hardline=False, max_range=r_limit):
                        receiver.inbox.append(pkt)

    def _execute_obc_step(self, dt: float) -> Dict[int, np.ndarray]:
        """Execute per-drone autonomous OBC step_drone(telemetry, inbox, dt) across the fleet."""
        accs: Dict[int, np.ndarray] = {d.id: np.zeros(3, dtype=float) for d in self.drones}
        living = [
            d for d in self.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING")
        ]
        ordered = sorted(living, key=lambda d: (0 if d.is_leader else 1, d.id))
        for drone in ordered:
            telemetry = self._build_local_telemetry(drone)
            inbox = list(drone.inbox)
            drone.inbox.clear()
            intent, outbox = drone.brain.step_drone(telemetry, inbox, dt)
            drone.outbox.extend(outbox)

            for key, val in intent.state_updates.items():
                setattr(drone, key, np.copy(val) if isinstance(val, np.ndarray) else val)
            if intent.target_yaw is not None:
                drone.target_yaw = float(intent.target_yaw)
            if intent.target_station is not None:
                drone.target_station = np.copy(intent.target_station)
            accs[drone.id] = np.array(intent.target_acc, dtype=float)

            if intent.state_updates.get("corridor_pop"):
                if hasattr(drone, "corridor_stack") and len(drone.corridor_stack) > 1:
                    drone.corridor_stack._stack.pop()
            if drone.is_leader and intent.state_updates.get("acknowledged_target") is not None:
                self.target_pos = np.copy(intent.state_updates["acknowledged_target"])
            if intent.state_updates.get("flight_state") == "OVERHEAD_HOLD" and drone.is_leader:
                self.overhead_marshal_active = True
                self.recovery_stage = "OVERHEAD_MARSHAL"

            # If leader or escort leader just stepped in LEADER_FOLLOWER mode, propagate fresh heartbeat
            # across the physical RF medium to followers/escortee before they execute their step.
            if (drone.is_leader or drone.flight_state == "ESCORT_LEAD") and self.active_brain_name == "LEADER_FOLLOWER" and drone.outbox:
                lead_pkts = list(drone.outbox)
                drone.outbox.clear()
                for pkt in lead_pkts:
                    for receiver in ordered:
                        if receiver.id == drone.id:
                            continue
                        dist = float(np.linalg.norm(drone.pos - receiver.pos))
                        if dist <= RADIO_RANGE and not self.is_packet_dropped(dist, is_hardline=False, max_range=RADIO_RANGE):
                            receiver.inbox = [
                                p for p in receiver.inbox
                                if not (p.sender_id == drone.id and p.msg_type == pkt.msg_type)
                            ]
                            receiver.inbox.append(pkt)
        return accs

    def set_brain(self, brain_name: str, force: bool = False) -> bool:
        """Hot-swap the guidance brain plugin at runtime across all airframes. Gated by comms range unless forced."""
        if not force and not self.is_in_comms_range():
            return False
        key = brain_name.upper()
        if key in BRAIN_REGISTRY:
            self.active_brain_name = key
            old_heading = getattr(self.brain, "formation_heading", 0.0)
            old_init = getattr(self.brain, "heading_initialized", False)
            old_formation = getattr(self.brain, "formation_mode", getattr(self, "formation_mode", "ECHELON_ESCORT"))
            old_slots = dict(getattr(self.brain, "assigned_slots", {}))
            if old_slots and key == "GOSSIP_CONSENSUS" and self.leader is not None:
                if self.leader.id not in old_slots:
                    old_slots[self.leader.id] = 0

            self._last_gossip_broadcast = -1.0

            # BUG-03 Part 1: Unified Consensus Target on Hot-Swap
            consensus_target = np.copy(self.target_pos)
            if self.leader is not None and not self.leader.crashed:
                consensus_target = np.copy(self.leader.acknowledged_target)

            for d in self.drones:
                if key == "GOSSIP_CONSENSUS":
                    d.acknowledged_target = np.copy(consensus_target)
                    d.brain = GossipConsensusBrain(
                        drone_id=d.id,
                        initial_target=consensus_target,
                        formation_mode=old_formation,
                    )
                else:
                    d.brain = BRAIN_REGISTRY[key](drone_id=d.id)
                    d.brain.formation_mode = old_formation

            if key == "GOSSIP_CONSENSUS":
                self.brain = _GossipSimProxy(self)
                self.comms_mode = "GOSSIP_CONSENSUS"
            else:
                self.brain = _LeaderFollowerSimProxy(self)
                if key == "LEADER_FOLLOWER" and self.comms_mode == "GOSSIP_CONSENSUS":
                    self.comms_mode = "LEADER_FOLLOWER"

            self.brain.formation_mode = old_formation
            if old_init:
                self.brain.formation_heading = old_heading
                self.brain.heading_initialized = True
            if old_slots:
                self.brain.assigned_slots = dict(old_slots)
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
            self.tether_severed = False
            for d in self.drones:
                d.tether_lost = False
                d.tether_jettisoned = False
            if self.active_brain_name != "LEADER_FOLLOWER":
                self.set_brain("LEADER_FOLLOWER")
            return True
        elif mode in ("GOSSIP_CONSENSUS", "P2P_MESH", "MESH", "GOSSIP"):
            self.comms_mode = "GOSSIP_CONSENSUS"
            self.lost_link_loiter_timer = 0.0  # Clear tether severance timer in mesh mode
            self.tether_severed = False
            for d in self.drones:
                d.tether_lost = False
                d.tether_jettisoned = False
            if self.active_brain_name != "GOSSIP_CONSENSUS":
                self.set_brain("GOSSIP_CONSENSUS")
            return True
        return False

    def set_rf_jamming(self, rate: float) -> float:
        """Set electronic warfare RF jamming rate [0.0, 0.5]."""
        return self.rf_channel.set_rf_jamming(rate)

    def calculate_packet_drop_prob(
        self, distance: float, is_hardline: bool = False, max_range: Optional[float] = None
    ) -> float:
        """Calculate packet loss probability using distance-dependent path loss and jamming."""
        return self.rf_channel.calculate_packet_drop_prob(distance, is_hardline=is_hardline, max_range=max_range)

    def is_packet_dropped(
        self, distance: float, is_hardline: bool = False, max_range: Optional[float] = None
    ) -> bool:
        """Stochastically determine if packet is lost given distance and jamming rate."""
        return self.rf_channel.is_packet_dropped(distance, is_hardline=is_hardline, max_range=max_range)

    def calculate_rssi(self, distance: float, is_hardline: bool = False) -> Tuple[float, float]:
        """Calculate RSSI (dBm) and normalized link quality [0.0, 1.0]."""
        return self.rf_channel.calculate_rssi(distance, is_hardline=is_hardline)

    def is_within_base_rf_dome(self, pos_or_dist: Union[float, List[float], np.ndarray]) -> bool:
        """Check whether position or scalar distance is within 50m High-Gain RF Dome (TKT-42)."""
        return self.rf_channel.is_within_base_rf_dome(pos_or_dist, self.tether_anchor)

    def calculate_base_rf_link(self, pos_or_dist: Union[float, List[float], np.ndarray]) -> Tuple[float, float, float]:
        """Compute (drop_prob, rssi, link_quality) incorporating 50m Base Horizon High-Gain RF Dome (TKT-42)."""
        return self.rf_channel.calculate_base_rf_link(pos_or_dist, self.tether_anchor)

    def load_urban_scenario(self, scenario_name: str = "default_canyon") -> List[UrbanObstacle]:
        """Load an urban obstacle scenario preset (PLN-03 / PLN-08 / TKT-60)."""
        name = scenario_name.lower().replace("-", "_").replace(" ", "_")
        if name in ("default_canyon", "canyon", "urban", "skybridge"):
            self.urban_obstacles = get_default_urban_obstacles()
            self.urban_scenario_name = "default_canyon"
        elif name in ("cyberpunk_obby", "obby", "stepped", "terraces", "terrain"):
            self.urban_obstacles = get_cyberpunk_obby_obstacles()
            self.urban_scenario_name = "cyberpunk_obby"
        elif name in ("empty", "clear", "none", "flat"):
            self.urban_obstacles = []
            self.urban_scenario_name = "empty"
        return self.urban_obstacles

    def set_urban_obstacles(self, obstacles: List[UrbanObstacle]) -> None:
        """Set or update the active urban obstacles list."""
        self.urban_obstacles = list(obstacles)

    def is_rf_occluded(self, origin: np.ndarray, target: np.ndarray) -> bool:
        """Check if direct line-of-sight ray between origin and target intersects any RF-blocking obstacle (PLN-08 / TKT-60)."""
        return rf_is_occluded(origin, target, getattr(self, "urban_obstacles", None))

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
        self.ground_repeaters.clear()
        self.falling_payloads.clear()
        self._highest_drone_id = count - 1

        # Fleet Hardware Identity Registry (TKT-48)
        self.fleet_registry = FleetRegistry(initial_count=count)

        if cold_start:
            # Cold-start on numbered ground launch pads (TKT-41)
            # Leader #0 positioned on Pad 0 (Z = 0.05m)
            rec0 = self.fleet_registry.get_by_drone_id(0)
            pad0_pos = self.get_pad_position(0)
            pad0_pos[2] = 0.05
            tether_type0 = TETHER_DATA_ONLY if self.comms_mode == "TETHERED_RELAY" else TETHER_NONE
            leader = Drone(
                drone_id=0,
                tier=TIER_LEADER,
                pos=pad0_pos,
                is_leader=True,
                hardware_uid=rec0.hardware_uid if rec0 else "3842A10F9B200001",
                tail_number=rec0.tail_number if rec0 else "ALITHEIA-V01",
                base_tier=BASE_TIER_T1_5,
                tether_type=tether_type0,
                rf_power_mode=RF_MODE_BORK,
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
                    base_tier=BASE_TIER_T1_5,
                    tether_type=TETHER_NONE,
                    rf_power_mode=RF_MODE_YIP,
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
            tether_type0 = TETHER_DATA_ONLY if self.comms_mode == "TETHERED_RELAY" else TETHER_NONE
            leader = Drone(
                drone_id=0,
                tier=TIER_LEADER,
                pos=np.array([0.0, 0.0, 20.0], dtype=float),
                is_leader=True,
                hardware_uid=rec0.hardware_uid if rec0 else "3842A10F9B200001",
                tail_number=rec0.tail_number if rec0 else "ALITHEIA-V01",
                base_tier=BASE_TIER_T1_5,
                tether_type=tether_type0,
                rf_power_mode=RF_MODE_BORK,
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
                    base_tier=BASE_TIER_T1_5,
                    tether_type=TETHER_NONE,
                    rf_power_mode=RF_MODE_YIP,
                )
                self.drones.append(drone)

        for d in self.drones:
            d.acknowledged_target = np.copy(self.target_pos)
            if getattr(self, "active_brain_name", "LEADER_FOLLOWER") == "GOSSIP_CONSENSUS":
                d.brain = GossipConsensusBrain(
                    drone_id=d.id,
                    initial_target=d.acknowledged_target,
                    formation_mode=self.formation_mode,
                )
            else:
                d.brain = LeaderFollowerBrain(drone_id=d.id)
                d.brain.formation_mode = self.formation_mode

    # ==========================================================================
    # Bubble 2 Subsystems: RF Channel & ATC Recovery Delegated Properties (TKT-61)
    # ==========================================================================
    @property
    def rf_jamming_rate(self) -> float:
        return self.rf_channel.rf_jamming_rate

    @rf_jamming_rate.setter
    def rf_jamming_rate(self, val: float) -> None:
        self.rf_channel.rf_jamming_rate = val

    @property
    def enable_packet_loss(self) -> bool:
        return self.rf_channel.enable_packet_loss

    @enable_packet_loss.setter
    def enable_packet_loss(self, val: bool) -> None:
        self.rf_channel.enable_packet_loss = val

    @property
    def last_heartbeat_broadcast(self) -> float:
        return self.rf_channel.last_heartbeat_broadcast

    @last_heartbeat_broadcast.setter
    def last_heartbeat_broadcast(self, val: float) -> None:
        self.rf_channel.last_heartbeat_broadcast = val

    @property
    def landing_pads(self) -> Dict[int, int]:
        return self.atc.landing_pads

    @landing_pads.setter
    def landing_pads(self, val: Dict[int, int]) -> None:
        self.atc.landing_pads = val

    @property
    def atc_waiting_queue(self) -> List[int]:
        return self.atc.atc_waiting_queue

    @atc_waiting_queue.setter
    def atc_waiting_queue(self, val: List[int]) -> None:
        self.atc.atc_waiting_queue = val

    @property
    def atc_current_wave(self) -> int:
        return self.atc.atc_current_wave

    @atc_current_wave.setter
    def atc_current_wave(self, val: int) -> None:
        self.atc.atc_current_wave = val

    @property
    def turnaround_timer(self) -> float:
        return self.atc.turnaround_timer

    @turnaround_timer.setter
    def turnaround_timer(self, val: float) -> None:
        self.atc.turnaround_timer = val

    @property
    def auto_turnaround_enabled(self) -> bool:
        return self.atc.auto_turnaround_enabled

    @auto_turnaround_enabled.setter
    def auto_turnaround_enabled(self, val: bool) -> None:
        self.atc.auto_turnaround_enabled = val

    @property
    def scramble_active(self) -> bool:
        return self.atc.scramble_active

    @scramble_active.setter
    def scramble_active(self, val: bool) -> None:
        self.atc.scramble_active = val

    @property
    def scramble_timer(self) -> float:
        return self.atc.scramble_timer

    @scramble_timer.setter
    def scramble_timer(self, val: float) -> None:
        self.atc.scramble_timer = val

    @property
    def scramble_stagger(self) -> float:
        return self.atc.scramble_stagger

    @scramble_stagger.setter
    def scramble_stagger(self, val: float) -> None:
        self.atc.scramble_stagger = val

    @property
    def recovery_stage(self) -> str:
        return self.atc.recovery_stage

    @recovery_stage.setter
    def recovery_stage(self, val: str) -> None:
        self.atc.recovery_stage = val

    @property
    def swarm_rtb_active(self) -> bool:
        return self.atc.swarm_rtb_active

    @swarm_rtb_active.setter
    def swarm_rtb_active(self, val: bool) -> None:
        self.atc.swarm_rtb_active = val

    @property
    def overhead_marshal_active(self) -> bool:
        return self.atc.overhead_marshal_active

    @overhead_marshal_active.setter
    def overhead_marshal_active(self, val: bool) -> None:
        self.atc.overhead_marshal_active = val

    @property
    def atc_capacity(self) -> int:
        return self.atc.capacity

    @atc_capacity.setter
    def atc_capacity(self, val: int) -> None:
        self.atc.capacity = val

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
    def route_paused(self) -> bool:
        return self.c2.route_paused

    @route_paused.setter
    def route_paused(self, val: bool) -> None:
        self.c2.route_paused = val

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

    # 1-Click Aux Mission: Patrol to BINGO RTB Delegations (TKT-58)
    @property
    def bingo_mission_active(self) -> bool:
        return self.c2.bingo_mission_active

    @property
    def bingo_threshold(self) -> float:
        return self.c2.bingo_threshold

    @property
    def bingo_triggered(self) -> bool:
        return self.c2.bingo_triggered

    @property
    def bingo_stage(self) -> str:
        return self.c2.bingo_stage

    def deploy_patrol_to_bingo(self, bingo_threshold: float = 25.0, pattern: str = "BOX_PATROL") -> Dict[str, Any]:
        return self.c2.deploy_patrol_to_bingo(bingo_threshold=bingo_threshold, pattern=pattern)

    def stop_patrol_to_bingo(self) -> Dict[str, Any]:
        return self.c2.stop_patrol_to_bingo()

    # 1-Click Aux Mission: Physical Courier / Search-and-Recall Rescue Mule Delegations (TKT-51)
    @property
    def courier_active(self) -> bool:
        return self.c2.courier_active

    @property
    def courier_stage(self) -> str:
        return self.c2.courier_stage

    def deploy_search_and_recall(self, target_drone_id: Optional[int] = None, courier_id: Optional[int] = None) -> Dict[str, Any]:
        return self.c2.deploy_search_and_recall(target_drone_id=target_drone_id, courier_id=courier_id)

    def stop_search_and_recall(self) -> Dict[str, Any]:
        return self.c2.stop_search_and_recall()

    # 1-Click Aux Mission: Cairn G1 Solar Repeater Payload Drop (TKT-59 / PLN-04)
    @property
    def payload_drop_active(self) -> bool:
        return self.c2.payload_drop_active

    @property
    def payload_drop_stage(self) -> str:
        return self.c2.payload_drop_stage

    def deploy_payload_drop(self, target_pos: Optional[Union[List[float], np.ndarray]] = None, carrier_id: Optional[int] = None) -> Dict[str, Any]:
        return self.c2.deploy_payload_drop(target_pos=target_pos, carrier_id=carrier_id)

    def stop_payload_drop(self) -> Dict[str, Any]:
        return self.c2.stop_payload_drop()

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
            for d in self.drones:
                if d.id != successor.id:
                    d.is_leader = False
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
            if not d.crashed and not d.is_leader and not d.tether_lost and d.succession_score() >= 0
        ]
        if not candidates:
            candidates = [d for d in self.drones if not d.crashed and not d.is_leader and d.succession_score() >= 0]
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

        # 0b. Advance Tether Severance Loiter & Automated RTB Timer (TKT-16 / TKT-19 / TKT-54)
        # Wireless modes rely strictly on autonomous airframe OBC watchdog & breadcrumb backtrack
        if self.comms_mode == "TETHERED_RELAY" and self.tether_severed and not self.swarm_rtb_active:
            if self.lost_link_loiter_timer <= 0.0:
                self.lost_link_loiter_timer = 30.0
            else:
                self.lost_link_loiter_timer = max(0.0, self.lost_link_loiter_timer - dt)
                if self.lost_link_loiter_timer <= 0.0:
                    # Loiter grace period expired: automated return to base (Stage 1 Overhead Marshal)
                    self.command_rtb_hold(force_autonomous=True)
        elif not (self.comms_mode == "TETHERED_RELAY" and self.tether_severed) and not self.swarm_rtb_active:
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
            if not drone.crashed and drone.flight_state in ("DESCENDING", "FIELD_DESCENDING"):
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

        # 0f. Advance Falling Payloads (TKT-59 / PLN-04)
        surviving_payloads = []
        for p in self.falling_payloads:
            p["vel"][2] -= 9.81 * dt
            p["pos"] += p["vel"] * dt
            if p["pos"][2] <= 0.05:
                # Touchdown on terrain!
                p["pos"][2] = 0.05
                rep_id = len(self.ground_repeaters) + 1
                repeater = GroundRepeater(
                    id=rep_id,
                    pos=np.array([p["pos"][0], p["pos"][1], 0.05], dtype=float),
                    active=True,
                    solar_powered=True,
                    radio_range=60.0,
                    deployed_at=float(self.sim_time),
                    carrier_id=p.get("carrier_id"),
                )
                self.ground_repeaters.append(repeater)
                if hasattr(self, "c2") and hasattr(self.c2, "log_event"):
                    self.c2.log_event("STATUS", f"[TOUCHDOWN: Cairn G1 Repeater #{rep_id} anchored at ({repeater.pos[0]:.1f}, {repeater.pos[1]:.1f}). Solar transceiver ACTIVE.]")
            else:
                surviving_payloads.append(p)
        self.falling_payloads = surviving_payloads

        # Reset GNSS dead-reckoning drift for drones passing near active ground repeaters (TKT-59)
        if self.ground_repeaters:
            for d in self.drones:
                if not d.crashed and getattr(d, "gnss_denied", False):
                    for rep in self.ground_repeaters:
                        if rep.active:
                            dist_rep = float(np.linalg.norm(d.pos - rep.pos))
                            if dist_rep <= 30.0:
                                d.gnss_denied_timer = 0.0
                                break

        # 1. Update Physical RF Channel, Asymmetric Link Budget & Mesh Gateway (TKT-49)
        self._update_radio_comms()

        # 1.1 Heartbeat Watchdog, Breadcrumb Backtrack & Perimeter Hold (TKT-54)
        self._update_heartbeat_watchdog_and_breadcrumbs(dt)

        # 1b. Update In-Flight Dynamic Swarm Enrollment & P2P Handshakes (TKT-50)
        self._update_in_flight_enrollment(dt)

        # 2. Comms & Packet Protocol from active Brain Plugin
        self.brain.update_comms(dt)

        # 2. Check Tether Dynamics & Severance (Tethered Relay Mode)
        if self.comms_mode == "TETHERED_RELAY":
            self._update_tether_dynamics(dt)

        # 3. Compute Guidance Targets from active Brain Plugin (includes OBC Zone 2/3 reflexes)
        guidance_accs = self.brain.compute_guidance(dt)

        # 4. Check Physical Collisions (Zone 4 Physical Contact & Structural Clash)
        self._check_zone4_physical_collisions()

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
                tether_accs[drone.id] = np.zeros(3, dtype=float)
                drone.vel = np.zeros(3, dtype=float)
                drone.acc = np.zeros(3, dtype=float)
                drone.rotor_rpm = 0.0

            elif drone.flight_state in ("DESCENDING", "FIELD_DESCENDING"):
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
                        v_target_xy = min(5.0, max(1.0, dist_xy * 1.8)) * dir_xy
                        a_target_x = float(np.clip(3.5 * (v_target_xy[0] - drone.vel[0]), -6.0, 6.0))
                        a_target_y = float(np.clip(3.5 * (v_target_xy[1] - drone.vel[1]), -6.0, 6.0))
                    else:
                        a_target_x = float(-5.0 * drone.vel[0])
                        a_target_y = float(-5.0 * drone.vel[1])

                    # 2. Vertical Guidance: 2-Stage Deceleration Sink Rate
                    # Stage 1 Sink: vz = -1.5 m/s down to Z = 1.0m
                    # Stage 2 Cushion: vz = -0.5 m/s from Z = 1.0m down to Z = 0.5m
                    current_z = drone.pos[2]
                    target_pad_z = float(pad_pos[2]) if pad_pos is not None else 0.5
                    rel_alt = current_z - target_pad_z
                    if dist_xy > 2.5:
                        # Maintain safe approach altitude while closing horizontal distance
                        target_vz = float(np.clip(2.0 * (15.0 - current_z), -1.5, 2.0))
                        drone.landing_stage = "APPROACH"
                    elif rel_alt > 1.0:
                        target_vz = -1.8
                        drone.landing_stage = "APPROACH"
                    else:
                        target_vz = -0.5
                        drone.landing_stage = "FINAL_DESCENT"

                    a_target_z = float(np.clip(4.0 * (target_vz - drone.vel[2]), -6.0, 6.0))
                    guidance_accs[drone.id] = np.array([a_target_x, a_target_y, a_target_z], dtype=float)

                    # Touchdown Check: Ground contact check
                    surf_z = get_surface_elevation(drone.pos, getattr(self, "urban_obstacles", None))
                    target_ground_z = pad_pos[2] if pad_pos is not None else (surf_z + 0.05 if drone.flight_state == "FIELD_DESCENDING" else surf_z + 0.5)
                    touchdown_z_thresh = target_ground_z + 0.08
                    if current_z <= touchdown_z_thresh and dist_xy <= 0.40:
                        # Ground Truth Pad Occupancy Check (Operator Incompetence Failsafe for Base pads):
                        if drone.flight_state != "FIELD_DESCENDING":
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
                        if drone.flight_state == "FIELD_DESCENDING":
                            drone.flight_state = "FIELD_LANDED"
                            drone.status = "FIELD_LANDED"
                        else:
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
                                if not d.crashed and d.flight_state not in ("LANDED", "FIELD_LANDED", "E_LANDED") and d.id != drone.id
                            ]
                            if living_airborne:
                                successor = max(living_airborne, key=lambda d: d.succession_score())
                                for d in self.drones:
                                    if d.id != successor.id:
                                        d.is_leader = False
                                successor.is_leader = True
                                successor.tier = TIER_LEADER
                                successor.status = "FLEET_LEADER"

            elif drone.flight_state == "EMERGENCY_LANDING":
                surf_z = get_surface_elevation(drone.pos, getattr(self, "urban_obstacles", None))
                target_spot = drone.e_landing_spot if drone.e_landing_spot is not None else np.array([drone.pos[0], drone.pos[1], surf_z + 0.05], dtype=float)
                drone.target_station = target_spot
                to_spot_xy = target_spot[:2] - drone.pos[:2]
                dist_xy = float(np.linalg.norm(to_spot_xy))

                if dist_xy > 0.2:
                    dir_xy = to_spot_xy / dist_xy
                    v_target_xy = min(3.0, max(0.5, dist_xy * 1.2)) * dir_xy
                    a_target_x = float(np.clip(3.0 * (v_target_xy[0] - drone.vel[0]), -5.0, 5.0))
                    a_target_y = float(np.clip(3.0 * (v_target_xy[1] - drone.vel[1]), -5.0, 5.0))
                else:
                    a_target_x = float(-4.0 * drone.vel[0])
                    a_target_y = float(-4.0 * drone.vel[1])

                current_z = drone.pos[2]
                target_ground_z = float(target_spot[2])
                rel_alt = current_z - target_ground_z
                target_vz = -1.2 if rel_alt > 1.0 else -0.3
                a_target_z = float(np.clip(4.0 * (target_vz - drone.vel[2]), -5.0, 5.0))
                guidance_accs[drone.id] = np.array([a_target_x, a_target_y, a_target_z], dtype=float)

                if current_z <= target_ground_z + 0.08 and dist_xy <= 0.45:
                    drone.pos = np.copy(target_spot)
                    drone.vel = np.zeros(3, dtype=float)
                    drone.acc = np.zeros(3, dtype=float)
                    drone.flight_state = "E_LANDED"
                    drone.status = "E_LANDED"
                    drone.touchdown_confirmed = True
                    drone.rotor_rpm = 0.0

                    if drone.is_leader:
                        drone.is_leader = False
                        living_airborne = [
                            d for d in self.drones
                            if not d.crashed and d.flight_state not in ("LANDED", "FIELD_LANDED", "E_LANDED", "GROUND_IDLE") and d.id != drone.id
                        ]
                        if living_airborne:
                            successor = max(living_airborne, key=lambda d: d.succession_score())
                            for d in self.drones:
                                if d.id != successor.id:
                                    d.is_leader = False
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

            if drone.flight_state in ("LANDED", "GROUND_IDLE", "FIELD_LANDED", "E_LANDED"):
                # Grounded on pad: motors disarmed, zero kinematics
                drone.acc = np.zeros(3, dtype=float)
                drone.vel = np.zeros(3, dtype=float)
                if drone.landing_pad is not None:
                    drone.pos = np.copy(drone.landing_pad)
                drone.rotor_rpm = 0.0
                continue

            # Airframe guidance intent from OBC (including Zone 2/3 reflexes) + external physical forces (tether cable)
            a_guide = guidance_accs[drone.id]
            total_acc = a_guide + tether_accs[drone.id]
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
            surf_elev = get_surface_elevation(drone.pos, getattr(self, "urban_obstacles", None))
            floor_limit = surf_elev + (0.05 if drone.flight_state in ("TAKEOFF", "FIELD_DESCENDING", "EMERGENCY_LANDING", "FIELD_LANDED", "E_LANDED") else 0.5)
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

            # Orientation update (TKT-55: Bounded yaw slew rate <= 45 deg/s)
            if not drone.crashed and drone.flight_state not in ("LANDED", "GROUND_IDLE"):
                speed_xy = math.hypot(drone.vel[0], drone.vel[1])
                target_yaw = getattr(drone, "target_yaw", None)
                if target_yaw is None:
                    if speed_xy > 0.2:
                        target_yaw = math.atan2(drone.vel[1], drone.vel[0])
                    else:
                        target_yaw = drone.yaw

                # Slew rate clamp: <= 45 deg/s in standard flight, <= 60 deg/s in patrol pursuit
                max_yaw_rate = math.radians(60.0) if getattr(self, "patrol_active", False) else math.radians(45.0)
                max_step = max_yaw_rate * dt
                yaw_diff = (target_yaw - drone.yaw + math.pi) % (2.0 * math.pi) - math.pi
                slew = float(np.clip(yaw_diff, -max_step, max_step))
                drone.yaw = (drone.yaw + slew + math.pi) % (2.0 * math.pi) - math.pi

            # Bank tilt from acceleration
            drone.pitch = float(np.clip(-drone.acc[0] * 0.04, -0.4, 0.4))
            drone.roll = float(np.clip(-drone.acc[1] * 0.04, -0.4, 0.4))

            # Battery drain (TKT-21 / TKT-58 Tactical Power Drain)
            if drone.tether_type == TETHER_POWER_DATA and not self.tether_severed:
                drone.battery = 100.0
            else:
                if drone.flight_state in ("LANDED", "GROUND_IDLE"):
                    base_rate = 0.001
                else:
                    speed = math.hypot(drone.vel[0], drone.vel[1])
                    base_rate = 0.25 if speed > 8.0 else 0.15
                drain = base_rate * getattr(self, "battery_drain_multiplier", 1.0) * dt
                drone.battery = max(0.0, drone.battery - drain)

            # Autonomous OBC BINGO Failsafe (Brain Airgap)
            if drone.battery <= getattr(drone, "bingo_threshold", 20.0) and not drone.crashed and drone.flight_state not in ("LANDED", "GROUND_IDLE", "RETRIEVED"):
                if getattr(drone, "lost_link", False) and not getattr(drone, "bingo_failsafe_triggered", False):
                    drone.bingo_failsafe_triggered = True
                    drone.flight_state = "CORRIDOR_ERTB"
                    drone.status = "CORRIDOR_ERTB"
                    if hasattr(drone, "corridor_stack"):
                        target = drone.corridor_stack.peek()
                        if target is not None:
                            drone.acknowledged_target = np.copy(target)
                            drone.target_station = np.copy(target)
                    if hasattr(self, "c2") and hasattr(self.c2, "log_event"):
                        self.c2.log_event("WARN", f"[OBC BINGO FAILSAFE: Drone #{drone.id} in LOST_LINK hit BINGO battery ({drone.battery:.1f}%). Autonomically returning along corridor.]")

            # Autonomous Beyond-BINGO E-Landing Failsafe (TKT-64 / PLN-05)
            # If battery is physically insufficient to reach Base or active FLZ without mid-air crash:
            home_target = self.atc.active_field_lz.coords if self.atc.active_field_lz is not None else np.array([0.0, 0.0, 0.0], dtype=float)
            if not drone.crashed and drone.flight_state not in ("LANDED", "GROUND_IDLE", "RETRIEVED", "FIELD_LANDED", "E_LANDED", "EMERGENCY_LANDING"):
                if hasattr(drone, "is_beyond_bingo") and drone.is_beyond_bingo(home_target, cruise_speed=12.0, drain_rate=0.25):
                    if drone.pos[2] > 0.5:
                        self.command_e_land(drone.id, reason="BEYOND_BINGO")
                        if hasattr(self, "c2") and hasattr(self.c2, "log_event"):
                            self.c2.log_event("WARN", f"[OBC BEYOND-BINGO: Drone #{drone.id} battery ({drone.battery:.1f}%) insufficient to reach home. Initiating autonomous safe E-LAND.]")

            # Emergency Low Battery Soft Settle Protection
            if drone.battery <= 3.0 and not drone.crashed and drone.flight_state not in ("LANDED", "GROUND_IDLE", "RETRIEVED"):
                drone.gnss_settled = True
                drone.status = "EMERGENCY_SETTLE"

            # Reset GNSS dead-reckoning drift if near active ground repeater (TKT-59)
            if self.ground_repeaters and getattr(drone, "gnss_denied", False):
                for rep in self.ground_repeaters:
                    if rep.active:
                        dist_rep = float(np.linalg.norm(drone.pos - rep.pos))
                        if dist_rep <= 30.0:
                            drone.gnss_denied_timer = 0.0
                            drone.drift_radius = 0.0
                            break

    # ==========================================================================
    # 1. Radio Comms (10 Hz Baud), Asymmetric Link Budget & Mesh Gateway (TKT-49 / TKT-60)
    # ==========================================================================
    def _update_radio_comms(self) -> None:
        """Simulate realistic 10 Hz radio broadcast packets, asymmetric link budget (Canine Taxonomy),
        Mesh Telemetry Gateway relay, and timeout-based Lost-Link state transitions (TKT-49 / TKT-60)."""
        self.rf_channel.update_radio_comms(
            sim_time=self.sim_time,
            comms_mode=self.comms_mode,
            drones=self.drones,
            ground_repeaters=self.ground_repeaters,
            tether_anchor=self.tether_anchor,
            tether_severed=self.tether_severed,
            urban_obstacles=getattr(self, "urban_obstacles", None),
            current_leader=self.leader,
        )

    # ==========================================================================
    # 1.1 Heartbeat Watchdog, Breadcrumb Backtrack & Perimeter Hold (TKT-54)
    # ==========================================================================
    def _update_heartbeat_watchdog_and_breadcrumbs(self, dt: float) -> None:
        """Update 10-second rolling position breadcrumbs (@ 2 Hz) and decrement
        heartbeat watchdogs. Triggers autonomous breadcrumb backtrack to radio horizon
        upon watchdog expiry unless radio silence is permitted."""
        for d in self.drones:
            if d.crashed:
                continue

            # 1. Rolling 10s Breadcrumbs buffer (@ 2 Hz = every 0.5s)
            if d.flight_state in ("CRUISE", "EN_ROUTE", "TAKEOFF", "OVERHEAD_HOLD") and not d.lost_link and not getattr(d, "holding_at_horizon", False):
                d.breadcrumb_record_timer += dt
                if d.breadcrumb_record_timer >= 0.5:
                    d.breadcrumb_record_timer = 0.0
                    d.record_breadcrumb(self.sim_time)

            # Record sparse corridor waypoints (BUG-06)
            if hasattr(d, "record_corridor") and d.flight_state in ("CRUISE", "EN_ROUTE", "TAKEOFF", "OVERHEAD_HOLD", "PERIMETER_HOLD") and not d.crashed:
                d.record_corridor(d.pos)

            # 2. Watchdog countdown
            d.gcs_heartbeat_watchdog = max(0.0, d.gcs_heartbeat_watchdog - dt)

            # Check if tether severance loiter is active (Fleet Doctrine A: 30s)
            is_tether_loitering = (self.lost_link_loiter_timer > 0.0)

            # 3. Watchdog tripped -> Backtrack state machine
            if d.gcs_heartbeat_watchdog <= 0.0:
                d.lost_link = True
                if not getattr(d, "radio_silence_permitted", False):
                    # Autonomous backtrack toward radio horizon (suppressed during tether loiter)
                    if not is_tether_loitering and d.flight_state not in ("TAKEOFF", "DESCENDING", "FIELD_DESCENDING", "EMERGENCY_LANDING", "BACKTRACK_HOLD", "PERIMETER_HOLD", "LANDED", "FIELD_LANDED", "E_LANDED", "GROUND_IDLE", "CRASHED", "BREAKAWAY_JETTISON", "CORRIDOR_ERTB", "COURIER_TRANSIT", "RENDEZVOUS_DWELL", "ESCORT_LEAD", "ESCORT_REJOIN", "PAYLOAD_TRANSIT", "PRECISION_DESCENT", "CARRIER_EGRESS"):
                        d.flight_state = "BACKTRACK_HOLD"
                        d.status = "BACKTRACK_HOLD"
                        d.backtrack_hold = True
                        d.backtrack_target = d.get_backtrack_point(retreat_distance=18.0)
                        if d.breadcrumbs:
                            d.lkp = np.copy(d.pos)
                        d.acknowledged_target = np.copy(d.backtrack_target)
                        d.target_station = np.copy(d.backtrack_target)
                        if hasattr(self, "c2") and hasattr(self.c2, "log_event"):
                            self.c2.log_event("WARN", f"Drone #{d.id} heartbeat watchdog expired! Backtracking along breadcrumbs to radio horizon.")
                else:
                    if d.status != "LOST_LINK" and d.flight_state not in ("LANDED", "GROUND_IDLE", "CRASHED", "COURIER_TRANSIT", "RENDEZVOUS_DWELL", "ESCORT_LEAD", "ESCORT_REJOIN", "PAYLOAD_TRANSIT", "PRECISION_DESCENT", "CARRIER_EGRESS"):
                        d.status = "LOST_LINK"

            # 4. Link restoration during backtrack or corridor ERTB (suppressed if operator commanded RTB)
            if not getattr(self.atc, "swarm_rtb_active", False) and d.flight_state in ("BACKTRACK_HOLD", "CORRIDOR_ERTB") and d.gcs_heartbeat_watchdog > 0.0 and d.uplink_active:
                d.flight_state = "PERIMETER_HOLD"
                d.status = "PERIMETER_HOLD"
                d.backtrack_hold = False
                d.holding_at_horizon = True
                d.acknowledged_target = np.copy(d.pos)
                d.target_station = np.copy(d.pos)
                d.vel = np.zeros(3, dtype=float)
                if hasattr(self, "c2") and hasattr(self.c2, "log_event"):
                    self.c2.log_event("STATUS", f"[LINK RESTORED: Drone #{d.id} holding station at radio horizon. Commanded target was out of range.]")

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
    def _update_tether_dynamics(self, dt: float = 0.016) -> None:
        """Check cable length limits, strain tension, t-GCSH watchdog, and physical collisions.
        Physical contact <= 1.0m, tension > 35N, or t-GCSH timeout snaps cable and triggers autonomic succession."""
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

        # Check tether heartbeat watchdog (t-GCSH 100Hz hardline carrier, 0.5s timeout) (BUG-06)
        if not getattr(self, "t_gcsh_active", True):
            current_leader.tether_heartbeat_watchdog = max(0.0, current_leader.tether_heartbeat_watchdog - dt)
        if getattr(current_leader, "tether_heartbeat_watchdog", 0.5) <= 0.0:
            self._snap_tether(current_leader, reason="TETHER_HEARTBEAT_TIMEOUT")
            return

        # Check cable strain sensor threshold (T > 35N) (BUG-06)
        cable_tension = getattr(current_leader, "cable_tension", 0.0)
        if cable_tension > 35.0:
            self._snap_tether(current_leader, reason="CABLE_STRAIN_EXCEEDED")
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

    def sever_tether(self, reason: str = "MANUAL_SEVERANCE") -> None:
        """Explicitly sever tether umbilical on current leader (TKT-16 / TKT-57)."""
        if self.leader is not None:
            self._snap_tether(self.leader, reason=reason)

    @property
    def is_tethered(self) -> bool:
        """True if operating in TETHERED_RELAY mode and umbilical is intact."""
        return self.comms_mode == "TETHERED_RELAY" and not self.tether_severed

    def _snap_tether(self, leader_drone: Drone, reason: str = "") -> None:
        """Snap tether, initiate umbilical emergency breakaway dive on leader,
        dynamically promote 2IC, and initiate Doctrine A loiter timer (TKT-16 / TKT-27)."""
        self.tether_severed = True
        leader_drone.tether_lost = True
        leader_drone.tether_type = TETHER_NONE
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
    # 4. Sane 4-Zone Spatial Safety (Physical Zone 4 Damage vs OBC Reflexes)
    # ==========================================================================
    def _check_zone4_physical_collisions(self) -> None:
        """Physical prop clash and structural impact detection (SITL Physics Airgap).
        Zone 4 (d <= 1.5m): Prop clash destroys airframe motors and triggers ballistic tumble.
        SIM strictly detects physical damage; Zone 2 & 3 evasion reflexes run in OBC firmware."""
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

        # Check static urban obstacle physical impact (d <= ZONE4_CLASH)
        if getattr(self, "urban_obstacles", None):
            for drone in self.drones:
                if drone.crashed or drone.flight_state in ("LANDED", "GROUND_IDLE", "FIELD_LANDED", "E_LANDED"):
                    continue
                min_obs_dist = 999.0
                min_obs_id = None
                is_landing_or_takeoff = drone.flight_state in ("TAKEOFF", "FIELD_DESCENDING", "EMERGENCY_LANDING", "DESCENDING")
                for obs in self.urban_obstacles:
                    if getattr(obs, "is_landable", False) and is_on_obstacle_top(drone.pos, obs):
                        if is_landing_or_takeoff or drone.pos[2] >= obs.z_ceiling - 0.05:
                            continue
                    dist, _ = point_to_aabb_repulsion(
                        drone.pos, drone.vel, obs.bounds, zone3_outer=ZONE3_OUTER, zone4_clash=ZONE4_CLASH
                    )
                    if dist < min_obs_dist:
                        min_obs_dist = dist
                        min_obs_id = obs.id
                    if dist <= ZONE4_CLASH:
                        drone.collision_inferred = True
                        self._trigger_crash(drone, reason=f"STRUCTURAL_CLASH_{obs.id}")
                        break
                drone.nearest_obstacle_dist = min_obs_dist
                drone.nearest_obstacle_id = min_obs_id

    def _compute_4zone_avoidance(self) -> Dict[int, np.ndarray]:
        """Compute pairwise collision avoidance across the 4 physical zones:
        - Zone 1 (d > 8.0m): Nominal clear, 0 force.
        - Zone 2 (3.5m < d <= 8.0m): Linear separation push.
        - Zone 3 (1.5m < d <= 3.5m): Critical push + active relative-velocity braking.
        - Zone 4 (d <= 1.5m): Prop clash, motors killed, ballistic crash.
        Delegates Zone 4 physical collision checking to _check_zone4_physical_collisions(),
        and queries OBC firmware avoidance reflexes for diagnostic and test inspection."""
        self._check_zone4_physical_collisions()
        accs = {}
        for d in self.drones:
            if d.crashed or d.flight_state in ("LANDED", "GROUND_IDLE", "FIELD_LANDED", "E_LANDED"):
                accs[d.id] = np.zeros(3, dtype=float)
                continue
            telem = self._build_local_telemetry(d)
            if hasattr(d, "brain") and d.brain is not None and hasattr(d.brain, "compute_inter_drone_avoidance"):
                synthetic_peers = [
                    HeardPeer(
                        drone_id=other.id,
                        pos=np.copy(other.pos),
                        vel=np.copy(other.vel),
                        crashed=bool(other.crashed),
                        enrolled=bool(getattr(other, "enrolled", True)),
                        flight_state=str(other.flight_state),
                    )
                    for other in self.drones
                    if other.id != d.id
                ]
                a_peers = d.brain.compute_inter_drone_avoidance(telem, peers=synthetic_peers)
                a_obs = d.brain.compute_obstacle_avoidance(telem)
                accs[d.id] = a_peers + a_obs
            else:
                accs[d.id] = np.zeros(3, dtype=float)
        return accs

    def _trigger_crash(self, drone: Drone, reason: str = "") -> None:
        """Disengage drone motors and trigger ballistic tumble."""
        was_leader = drone.is_leader
        drone.crashed = True
        drone.is_leader = False
        drone.crashed_while_linked = bool(drone.downlink_active)
        drone.flight_state = FlightState.CRASHED
        drone.status = "CRASHED"
        if drone.downlink_active:
            drone.lkp = np.copy(drone.pos)
            drone.mayday_beacon = True
            drone.mayday_reason = reason or "IMPACT_DETECTED"
            drone.outbox.append(
                RadioPacket(
                    sender_id=drone.id,
                    msg_type="PEER_BEACON",
                    timestamp=float(self.sim_time),
                    payload={"id": int(drone.id), "crashed": True},
                )
            )
        if hasattr(self, "fleet_registry"):
            self.fleet_registry.record_casualty(
                drone_id=drone.id,
                lkp=list(drone.pos),
                reason=reason,
                sim_time=self.sim_time,
            )
        if (was_leader or drone.id == 0) and self.comms_mode == "TETHERED_RELAY":
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
            drone.rf_power_mode = RF_MODE_YIP
            drone.tether_type = TETHER_NONE
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

        # Sort by succession score descending, enforcing Flagship authorization (BUG-06)
        eligible = [c for c in candidates if c.succession_score() >= 0]
        if not eligible:
            return None
        eligible.sort(key=lambda d: d.succession_score(), reverse=True)
        new_leader = eligible[0]
        old_leader: Optional[Drone] = retired_leader

        # Demote previous leader
        if demote_old_leader:
            if retired_leader is not None:
                retired_leader.is_leader = False
                retired_leader.rf_power_mode = RF_MODE_YIP
                retired_leader.tether_type = TETHER_NONE
            for drone in self.drones:
                if drone.id != new_leader.id and drone.is_leader:
                    old_leader = drone
                    drone.is_leader = False
                    drone.rf_power_mode = RF_MODE_YIP
                    drone.tether_type = TETHER_NONE
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
        new_leader.rf_power_mode = RF_MODE_BORK
        new_leader.tether_type = TETHER_NONE
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
    # 6. Mission Controls, Batched ATC Recovery & Casualty Laboratory (TKT-41 / TKT-42 / TKT-61)
    # ==========================================================================
    @staticmethod
    def get_pad_position(pad_idx: int) -> np.ndarray:
        """Calculate coordinates for recovery/launch pad index (0..7)."""
        return get_pad_position(pad_idx)

    def is_in_comms_range(self) -> bool:
        """Check whether at least one living drone is within the GCS radio horizon."""
        return self.atc.is_in_comms_range(self.drones, self.tether_anchor, RADIO_RANGE)

    def command_rtb_hold(self, force_autonomous: bool = False, destination: str = "BASE") -> Dict[str, Any]:
        """Stage 1 Recovery: Commanded RTB (Hold)."""
        res = self.atc.command_rtb_hold(
            drones=self.drones,
            tether_anchor=self.tether_anchor,
            radio_range=RADIO_RANGE,
            force_autonomous=force_autonomous,
            set_target_fn=self.set_target,
            destination=destination,
            obstacles=self.urban_obstacles,
        )
        if res.get("status") == "OK":
            self.patrol_active = False
        return res

    def command_corridor_ertb(self) -> Dict[str, Any]:
        """Execute autonomous corridor emergency return-to-base (ERTB) (BUG-06)."""
        self.patrol_active = False
        log_fn = self.c2.log_event if hasattr(self, "c2") and hasattr(self.c2, "log_event") else None
        def set_lead_tgt(tgt):
            self.target_pos = np.copy(tgt)
        return self.atc.command_corridor_ertb(self.drones, log_event_fn=log_fn, set_target_fn=set_lead_tgt)

    def cancel_rtb(self) -> None:
        """Cancel RTB / overhead marshaling and return living airborne drones to mission cruise."""
        self.atc.cancel_rtb(self.drones)
        self.lost_link_loiter_timer = 0.0
        if self.tether_severed and self.comms_mode == "TETHERED_RELAY":
            self.comms_mode = "LEADER_FOLLOWER"

    def command_land_swarm(self, destination: str = "BASE") -> Union[List[int], Dict[str, Any]]:
        """Stage 2 Recovery: Operator-authorized coordinated landing."""
        return self.atc.command_land_swarm(
            drones=self.drones,
            tether_anchor=self.tether_anchor,
            radio_range=RADIO_RANGE,
            succession_fn=self._trigger_autonomic_succession,
            destination=destination,
            obstacles=self.urban_obstacles,
        )

    def set_field_lz(
        self,
        coords: Union[List[float], np.ndarray],
        name: str = "FLZ-1",
        radius: float = 15.0,
        capacity: int = 16,
        slot_spacing: float = 4.0,
    ) -> Dict[str, Any]:
        return self.atc.set_field_lz(coords, name, radius, capacity, slot_spacing)

    def clear_field_lz(self) -> Dict[str, Any]:
        return self.atc.clear_field_lz()

    def command_e_land(self, drone_id: int, reason: str = "MANUAL_OPERATOR") -> bool:
        return self.atc.command_e_land(
            drone_id=drone_id,
            drones=self.drones,
            obstacles=self.urban_obstacles,
            reason=reason,
            succession_fn=self._trigger_autonomic_succession,
        )

    def command_e_land_swarm(self, reason: str = "SWARM_EMERGENCY") -> List[int]:
        return self.atc.command_e_land_swarm(
            drones=self.drones,
            obstacles=self.urban_obstacles,
            reason=reason,
            succession_fn=self._trigger_autonomic_succession,
        )

    @property
    def field_lz(self) -> Optional[Dict[str, Any]]:
        if self.atc.active_field_lz is not None:
            return {
                "id": self.atc.active_field_lz.id,
                "name": self.atc.active_field_lz.name,
                "coords": [round(float(v), 2) for v in self.atc.active_field_lz.coords],
                "radius": self.atc.active_field_lz.radius,
                "capacity": self.atc.active_field_lz.capacity,
                "active": self.atc.active_field_lz.active,
            }
        return None

    def command_land_drone(self, drone_id: int) -> bool:
        """Command an individual drone to land on an available recovery pad."""
        return self.atc.command_land_drone(
            drone_id=drone_id,
            drones=self.drones,
            tether_anchor=self.tether_anchor,
            radio_range=RADIO_RANGE,
            succession_fn=self._trigger_autonomic_succession,
        )

    def command_takeoff_drone(self, drone_id: int) -> bool:
        """Relaunch a landed or ground-idle drone from its pad via vertical takeoff."""
        return self.atc.command_takeoff_drone(
            drone_id=drone_id,
            drones=self.drones,
            tether_anchor=self.tether_anchor,
            radio_range=RADIO_RANGE,
        )

    def retrieve_drone_from_pad(self, pad_idx: int) -> Optional[int]:
        """Ground crew physically retrieves and stows an airframe or wreckage from a landing pad into the hangar."""
        return self.atc.retrieve_drone_from_pad(pad_idx, self.drones)

    def retrieve_all_landed_drones(self) -> List[int]:
        """Ground crew physically retrieves all landed/ground-idle airframes across all pads into hangar."""
        return self.atc.retrieve_all_landed_drones(self.drones)

    def clear_pads(self, turnaround_rearm: bool = False) -> Dict[str, Any]:
        """Reset/clear occupied pads and release next wave of airborne drones from overhead marshal hold to land."""
        return self.atc.clear_pads(
            drones=self.drones,
            turnaround_rearm=turnaround_rearm,
            land_swarm_fn=self.command_land_swarm,
        )

    def command_scramble_swarm(self, stagger_s: float = 1.2) -> Dict[str, Any]:
        """Initiate sequenced cold scramble from ground pads (TKT-41 & TKT-68)."""
        res = self.atc.command_scramble_swarm(
            drones=self.drones,
            tether_anchor=self.tether_anchor,
            stagger_s=stagger_s,
            radio_range=RADIO_RANGE,
            tier_leader=TIER_LEADER,
        )
        if res.get("status") == "SCRAMBLE_INITIATED":
            self.target_pos = np.array([0.0, 0.0, 20.0], dtype=float)
        return res

    def command_cold_start_swarm(self, count: Optional[int] = None) -> Dict[str, Any]:
        """Cold ground reset: resets or grounds fleet onto numbered ground launch pads (TKT-41 & TKT-68)."""
        n = count if count is not None else len(self.drones)
        self.cold_start = True
        self.target_pos = np.array([0.0, 0.0, 20.0], dtype=float)
        self._init_fleet(n, cold_start=True)
        return {"status": "COLD_START_ACTIVE", "drone_count": n}

    def spawn_reinforcement(
        self,
        rejoin_target: Optional[Union[np.ndarray, List[float]]] = None,
        auto_launch: bool = False,
        base_tier: str = BASE_TIER_T1_5,
        capabilities: Optional[List[str]] = None,
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
            base_tier=base_tier,
            capabilities=capabilities,
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

        if self.active_brain_name == "GOSSIP_CONSENSUS":
            drone.brain = GossipConsensusBrain(
                drone_id=new_id,
                initial_target=drone.acknowledged_target,
                formation_mode=self.formation_mode,
            )
            drone.brain.peer_state.gossip_status = "SCOUT"
        else:
            drone.brain = LeaderFollowerBrain(drone_id=new_id)
            drone.brain.formation_mode = self.formation_mode

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
        elif mode_upper in ("PERIMETER_PATROL_IN", "PERIMETER_IN"):
            mode_upper = "PERIMETER_IN"
        elif mode_upper in ("PERIMETER_PATROL_OUT", "PERIMETER_OUT"):
            mode_upper = "PERIMETER_OUT"
        elif mode_upper == "PERIMETER":
            mode_upper = "PERIMETER_PATROL"

        valid_modes = [
            "ECHELON_ESCORT",
            "NATURAL_FLOCK",
            "ORBIT_VORTEX",
            "PERIMETER_PATROL",
            "PERIMETER_IN",
            "PERIMETER_OUT",
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
        if hasattr(self, "c2") and self.c2 is not None:
            self.c2._target_pos = np.copy(target_arr)

        # 1. TETHERED_RELAY: Hardline umbilical to Drone 0
        if self.comms_mode == "TETHERED_RELAY":
            if self.tether_severed:
                return {
                    "status": "DROPPED",
                    "reason": "TETHER_SEVERED",
                    "error": "Tactical umbilical is severed. Uplink packet dropped.",
                }
            # Hardline delivery of GCS_UPLINK packet to tethered leader
            pkt = RadioPacket(
                sender_id=-1,
                msg_type="GCS_UPLINK",
                timestamp=float(self.sim_time),
                payload={"cmd": "SET_TARGET", "target": [float(v) for v in target_arr]},
            )
            for d in self.drones:
                if d.id == 0 or d.is_leader:
                    d.inbox.append(pkt)
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
            pkt = RadioPacket(
                sender_id=-1,
                msg_type="GCS_UPLINK",
                timestamp=float(self.sim_time),
                payload={"cmd": "SET_TARGET", "target": [float(v) for v in target_arr]},
            )
            current_leader.inbox.append(pkt)
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
        mission_type: str = "PATROL",
    ) -> Dict[str, Any]:
        """Deploy discrete waypoint mission route to the swarm over physical comms uplink (TKT-28)."""
        return self.c2.deploy_mission_route(waypoints, loop=loop, name=name, mission_type=mission_type)

    def stop_mission_route(self) -> Dict[str, Any]:
        """Stop active mission route and hold current position (TKT-28)."""
        return self.c2.stop_mission_route()

    def pause_mission_route(self) -> Dict[str, Any]:
        """Pause active mission route; drones hold station at current waypoint (TKT-56)."""
        return self.c2.pause_mission_route()

    def resume_mission_route(self) -> Dict[str, Any]:
        """Resume active mission route from current waypoint (TKT-56)."""
        return self.c2.resume_mission_route()

    def skip_mission_waypoint(self) -> Dict[str, Any]:
        """Skip currently targeted waypoint and advance immediately to next (TKT-56)."""
        return self.c2.skip_mission_waypoint()

    def check_mission_authorization(
        self,
        mission_type: str,
        required_capabilities: Optional[List[str]] = None,
        target_drone_id: Optional[int] = None,
    ) -> Dict[str, Any]:
        """Verify whether enrolled fleet airframes meet capability requirements (TKT-57)."""
        return self.c2.check_mission_authorization(
            mission_type=mission_type,
            required_capabilities=required_capabilities,
            target_drone_id=target_drone_id,
        )

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

            # Operator Airgap:
            # 1. If drone crashed while communication was down (beyond radio horizon / lost link),
            #    GCS only knows it is [LOST_LINK] at its Last Known Position (LKP).
            if d.crashed and not getattr(d, "crashed_while_linked", False) and not d.downlink_active:
                d_dict["crashed"] = False
                d_dict["flight_state"] = "LOST_LINK"
                d_dict["status"] = "LOST_LINK"
                d_dict["pos"] = [round(float(v), 2) for v in d.lkp]
                d_dict["vel"] = [0.0, 0.0, 0.0]
                d_dict["lost_link"] = True
                d_dict["is_leader"] = False
            elif not d.crashed and not d.downlink_active and d.flight_state not in ("LANDED", "GROUND_IDLE"):
                # 2. Living airborne drone has lost downlink contact (beyond Bork/Yip horizon or lost link).
                #    GCS receives NO live telemetry updates; coordinates freeze at Last Known Position (LKP).
                d_dict["flight_state"] = "LOST_LINK"
                d_dict["status"] = "LOST_LINK"
                d_dict["pos"] = [round(float(v), 2) for v in d.lkp]
                d_dict["vel"] = [0.0, 0.0, 0.0]
                d_dict["lost_link"] = True

            drones_list.append(d_dict)

        ack_target = self.get_acknowledged_target()

        # Determine active route current waypoint index & loiter state (TKT-28, TKT-30, TKT-56)
        if self.active_brain_name == "GOSSIP_CONSENSUS":
            current_wp_idx = getattr(self.brain, "current_route_wp_idx", 0)
            current_loitering = getattr(self.brain, "wp_loitering", False)
            current_loiter_timer = getattr(self.brain, "wp_loiter_timer", 0.0)
            current_total_loiter = getattr(self.brain, "wp_total_loiter", 0.0)
            is_paused = getattr(self.c2, "route_paused", False) or getattr(self.brain, "route_paused", False)
        elif current_leader is not None:
            current_wp_idx = current_leader.current_wp_idx
            current_loitering = getattr(current_leader, "wp_loitering", False)
            current_loiter_timer = getattr(current_leader, "wp_loiter_timer", 0.0)
            current_total_loiter = getattr(current_leader, "wp_total_loiter", 0.0)
            is_paused = getattr(self.c2, "route_paused", False) or getattr(current_leader, "mission_paused", False)
        else:
            current_wp_idx = 0
            current_loitering = False
            current_loiter_timer = 0.0
            current_total_loiter = 0.0
            is_paused = getattr(self.c2, "route_paused", False)

        route_data = {
            "active": self.route_active,
            "paused": is_paused,
            "name": self.route_name,
            "loop": self.route_loop,
            "current_wp_idx": current_wp_idx,
            "total_wps": len(self.route_waypoints),
            "waypoints": [[round(float(v), 2) for v in wp] for wp in self.route_waypoints],
            "loitering": current_loitering,
            "loiter_timer": round(max(0.0, float(current_loiter_timer)), 1),
            "total_loiter": round(max(0.0, float(current_total_loiter)), 1),
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
                "bork_amplified_downlink": RANGE_BORK,
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
            "patrol_to_bingo": {
                "active": self.bingo_mission_active,
                "threshold": float(self.bingo_threshold),
                "triggered": bool(self.bingo_triggered),
                "stage": self.bingo_stage,
                "pattern": getattr(self.c2, "bingo_pattern", "BOX_PATROL"),
            },
            "search_and_recall": {
                "active": self.c2.courier_active,
                "stage": self.c2.courier_stage,
                "courier_id": self.c2.courier_drone_id,
                "target_drone_id": self.c2.courier_target_drone_id,
                "target_lkp": [round(float(v), 2) for v in self.c2.courier_target_lkp] if self.c2.courier_target_lkp is not None else None,
                "dwell_timer": round(float(self.c2.courier_dwell_timer), 1),
            },
            "payload_deposit": self.c2.get_payload_drop_state(),
            "ground_repeaters": [r.to_dict() for r in self.ground_repeaters],
            "urban_scenario": getattr(self, "urban_scenario_name", "default_canyon"),
            "urban_obstacles": [o.to_dict() for o in self.urban_obstacles] if getattr(self, "urban_obstacles", None) else [],
            "field_lz": self.field_lz,
            "two_ic_id": two_ic_id,
            "lost_link_loiter_timer": round(max(0.0, float(self.lost_link_loiter_timer)), 1),
            "swarm_rtb_active": self.swarm_rtb_active,
            "overhead_marshal_active": self.overhead_marshal_active,
            "fleet_holding_at_horizon": any(getattr(d, "holding_at_horizon", False) for d in self.drones if not d.crashed),
            "fleet_backtracking": any(getattr(d, "backtrack_hold", False) for d in self.drones if not d.crashed),
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
# Per-Drone OBC Brain Compatibility Proxies for SITL Inspection (Option A)
# ==============================================================================
class _LeaderFollowerSimProxy(LeaderFollowerBrain):
    """Compatibility proxy on sim.brain that delegates inspection and C2 hooks
    to the per-drone LeaderFollowerBrain instances without leaking SIM into OBC."""

    def __init__(self, engine: SITLWorldEngine) -> None:
        object.__setattr__(self, "_engine", engine)
        object.__setattr__(self, "_formation_mode", "ECHELON_ESCORT")
        object.__setattr__(self, "_formation_heading", 0.0)
        object.__setattr__(self, "_heading_initialized", False)
        object.__setattr__(self, "_assigned_slots", {})
        super().__init__(drone_id=0)

    def _lead_brain(self) -> Optional[LeaderFollowerBrain]:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        b = eng._primary_drone_brain()
        return b if isinstance(b, LeaderFollowerBrain) else None

    @property
    def formation_mode(self) -> str:
        b = self._lead_brain()
        return b.formation_mode if b is not None else object.__getattribute__(self, "_formation_mode")

    @formation_mode.setter
    def formation_mode(self, val: str) -> None:
        object.__setattr__(self, "_formation_mode", val)
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        pkt = RadioPacket(
            sender_id=-1,
            msg_type="C2_COMMAND",
            timestamp=float(eng.sim_time),
            payload={"cmd": "SET_FORMATION", "mode": val},
        )
        if hasattr(eng, "drones"):
            for d in eng.drones:
                d.inbox.append(pkt)
                if hasattr(d, "brain") and d.brain is not None:
                    d.brain.handle_command(dict(pkt.payload))

    @property
    def formation_heading(self) -> float:
        b = self._lead_brain()
        return b.formation_heading if b is not None else object.__getattribute__(self, "_formation_heading")

    @formation_heading.setter
    def formation_heading(self, val: float) -> None:
        object.__setattr__(self, "_formation_heading", float(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and d.brain is not None:
                    d.brain.formation_heading = float(val)

    @property
    def heading_initialized(self) -> bool:
        b = self._lead_brain()
        return b.heading_initialized if b is not None else object.__getattribute__(self, "_heading_initialized")

    @heading_initialized.setter
    def heading_initialized(self, val: bool) -> None:
        object.__setattr__(self, "_heading_initialized", bool(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and d.brain is not None:
                    d.brain.heading_initialized = bool(val)

    @property
    def assigned_slots(self) -> Dict[int, int]:
        b = self._lead_brain()
        return b.assigned_slots if b is not None else object.__getattribute__(self, "_assigned_slots")

    @assigned_slots.setter
    def assigned_slots(self, val: Dict[int, int]) -> None:
        object.__setattr__(self, "_assigned_slots", dict(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and d.brain is not None:
                    d.brain.assigned_slots = dict(val)

    def reset(self) -> None:
        object.__setattr__(self, "_formation_mode", "ECHELON_ESCORT")
        object.__setattr__(self, "_formation_heading", 0.0)
        object.__setattr__(self, "_heading_initialized", False)
        object.__setattr__(self, "_assigned_slots", {})
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and d.brain is not None:
                    d.brain.reset()

    def update_comms(self, dt: float) -> None:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        eng._deliver_radio_packets(dt)

    def compute_guidance(self, dt: float) -> Dict[int, np.ndarray]:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        return eng._execute_obc_step(dt)

    def on_succession(self, old_leader: Optional[Drone], new_leader: Drone) -> None:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        old_id = old_leader.id if old_leader is not None else None
        if (
            old_leader is not None
            and isinstance(getattr(old_leader, "brain", None), LeaderFollowerBrain)
            and isinstance(getattr(new_leader, "brain", None), LeaderFollowerBrain)
        ):
            if old_leader.brain.assigned_slots and not new_leader.brain.assigned_slots:
                new_leader.brain.assigned_slots = dict(old_leader.brain.assigned_slots)
            new_leader.brain.formation_heading = old_leader.brain.formation_heading
            new_leader.brain.heading_initialized = old_leader.brain.heading_initialized
        for d in eng.drones:
            if hasattr(d, "brain") and d.brain is not None:
                d.brain.on_succession(old_id, new_leader.id)

    def _get_formation_slot(
        self,
        drone: Any,
        leader: Any,
        n_followers: Optional[int] = None,
        overhead_marshal: bool = False,
        clock_s: Optional[float] = None,
    ) -> np.ndarray:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        b = self._lead_brain() or self
        living_followers = [
            d for d in eng.drones
            if not d.crashed and not d.is_leader and getattr(d, "enrolled", True)
        ]
        living_followers.sort(key=lambda d: d.id)
        n_f = len(living_followers) if n_followers is None else int(n_followers)
        k = b.assigned_slots.get(int(getattr(drone, "id", 1)))
        if k is None:
            for idx, f in enumerate(living_followers):
                if f.id == getattr(drone, "id", -1):
                    k = idx + 1
                    break
            if k is None:
                k = max(1, int(getattr(drone, "id", 1)))
        l_pos = np.array(getattr(leader, "pos", [0.0, 0.0, 20.0]), dtype=float)
        l_vel = np.array(getattr(leader, "vel", [0.0, 0.0, 0.0]), dtype=float)
        l_yaw = float(getattr(leader, "yaw", 0.0))
        c_target = np.array(getattr(leader, "acknowledged_target", l_pos), dtype=float)
        return b._compute_slot_geometry(
            k=k,
            n_followers=max(1, n_f),
            leader_pos=l_pos,
            leader_vel=l_vel,
            leader_yaw=l_yaw,
            center_target=c_target,
            overhead_marshal=bool(overhead_marshal or eng.overhead_marshal_active),
            clock_s=float(eng.sim_time if clock_s is None else clock_s),
        )

    def _compute_boid_acceleration(
        self,
        drone: Any,
        leader: Any,
        peers: Optional[List[Any]] = None,
        is_tethered: bool = True,
        tether_anchor: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        b = self._lead_brain() or self
        peer_list = peers if peers is not None else [
            d for d in eng.drones if not d.crashed and getattr(d, "enrolled", True)
        ]
        teth = (eng.comms_mode == "TETHERED_RELAY" and not eng.tether_severed)
        return b._compute_boid_acceleration(
            drone=drone,
            leader=leader,
            peers=peer_list,
            is_tethered=teth,
            tether_anchor=eng.tether_anchor,
        )

    def handle_command(self, cmd_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        res = None
        for d in eng.drones:
            if hasattr(d, "brain") and d.brain is not None:
                r = d.brain.handle_command(cmd_dict)
                if r is not None:
                    res = r
        cmd = str(cmd_dict.get("cmd", "")).upper()
        if cmd == "PAUSE_ROUTE":
            self.route_paused = True
            for d in eng.drones:
                d.mission_paused = True
        elif cmd == "RESUME_ROUTE":
            self.route_paused = False
            for d in eng.drones:
                d.mission_paused = False
        return res


class _GossipSimProxy(GossipConsensusBrain):
    """Compatibility proxy on sim.brain that delegates inspection and C2 hooks
    to the per-drone GossipConsensusBrain instances without leaking SIM into OBC."""

    def __init__(self, engine: SITLWorldEngine) -> None:
        object.__setattr__(self, "_engine", engine)
        object.__setattr__(self, "_formation_mode", getattr(engine, "formation_mode", "PERIMETER_PATROL"))
        object.__setattr__(self, "_formation_heading", 0.0)
        object.__setattr__(self, "_heading_initialized", False)
        object.__setattr__(self, "_assigned_slots", {})
        object.__setattr__(self, "_current_seq", 1)
        object.__setattr__(self, "_route_active", False)
        object.__setattr__(self, "_route_paused", False)
        object.__setattr__(self, "_route_loop", True)
        object.__setattr__(self, "_route_waypoints", [])
        object.__setattr__(self, "_current_route_wp_idx", 0)
        object.__setattr__(self, "_wp_loitering", False)
        object.__setattr__(self, "_wp_loiter_timer", 0.0)
        object.__setattr__(self, "_wp_total_loiter", 0.0)
        super().__init__(
            drone_id=0,
            initial_target=getattr(engine, "target_pos", np.array([60.0, 40.0, 30.0])),
            formation_mode=getattr(engine, "formation_mode", "PERIMETER_PATROL"),
        )

    def _apex_brain(self) -> Optional[GossipConsensusBrain]:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            living = sorted(
                [d for d in eng.drones if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING")],
                key=lambda d: d.id,
            )
            for d in living:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    return d.brain
        b = eng._primary_drone_brain()
        return b if isinstance(b, GossipConsensusBrain) else None

    @property
    def peer_states(self) -> Dict[int, DronePeerState]:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        states: Dict[int, DronePeerState] = {}
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    states[d.id] = d.brain.peer_state
        return states

    @peer_states.setter
    def peer_states(self, val: Dict[int, DronePeerState]) -> None:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if d.id in val and hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    d.brain.peer_state = val[d.id]

    def _get_peer_state(self, drone_id: Optional[int] = None) -> DronePeerState:
        did = 0 if drone_id is None else int(drone_id)
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if d.id == did and hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    return d.brain.peer_state
        return DronePeerState(did, getattr(eng, "target_pos", np.array([60.0, 40.0, 30.0])))

    @property
    def current_seq(self) -> int:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        max_seq = object.__getattribute__(self, "_current_seq")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    max_seq = max(max_seq, d.brain.current_seq, d.brain.peer_state.pending_seq, d.brain.peer_state.active_seq)
        return max_seq

    @current_seq.setter
    def current_seq(self, val: int) -> None:
        object.__setattr__(self, "_current_seq", int(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    d.brain.current_seq = int(val)

    @property
    def formation_mode(self) -> str:
        b = self._apex_brain()
        return b.formation_mode if b is not None else object.__getattribute__(self, "_formation_mode")

    @formation_mode.setter
    def formation_mode(self, val: str) -> None:
        object.__setattr__(self, "_formation_mode", val)
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        pkt = RadioPacket(
            sender_id=-1,
            msg_type="C2_COMMAND",
            timestamp=float(eng.sim_time),
            payload={"cmd": "SET_FORMATION", "mode": val},
        )
        if hasattr(eng, "drones"):
            for d in eng.drones:
                d.inbox.append(pkt)
                if hasattr(d, "brain") and d.brain is not None:
                    d.brain.handle_command(dict(pkt.payload))

    @property
    def formation_heading(self) -> float:
        b = self._apex_brain()
        return b.formation_heading if b is not None else object.__getattribute__(self, "_formation_heading")

    @formation_heading.setter
    def formation_heading(self, val: float) -> None:
        object.__setattr__(self, "_formation_heading", float(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and d.brain is not None:
                    d.brain.formation_heading = float(val)

    @property
    def heading_initialized(self) -> bool:
        b = self._apex_brain()
        return b.heading_initialized if b is not None else object.__getattribute__(self, "_heading_initialized")

    @heading_initialized.setter
    def heading_initialized(self, val: bool) -> None:
        object.__setattr__(self, "_heading_initialized", bool(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and d.brain is not None:
                    d.brain.heading_initialized = bool(val)

    @property
    def assigned_slots(self) -> Dict[int, int]:
        b = self._apex_brain()
        return b.assigned_slots if b is not None else object.__getattribute__(self, "_assigned_slots")

    @assigned_slots.setter
    def assigned_slots(self, val: Dict[int, int]) -> None:
        object.__setattr__(self, "_assigned_slots", dict(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and d.brain is not None:
                    d.brain.assigned_slots = dict(val)

    @property
    def route_active(self) -> bool:
        b = self._apex_brain()
        return b.route_active if b is not None else object.__getattribute__(self, "_route_active")

    @route_active.setter
    def route_active(self, val: bool) -> None:
        object.__setattr__(self, "_route_active", bool(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    d.brain.route_active = bool(val)

    @property
    def route_paused(self) -> bool:
        b = self._apex_brain()
        return b.route_paused if b is not None else object.__getattribute__(self, "_route_paused")

    @route_paused.setter
    def route_paused(self, val: bool) -> None:
        object.__setattr__(self, "_route_paused", bool(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    d.brain.route_paused = bool(val)

    @property
    def route_loop(self) -> bool:
        b = self._apex_brain()
        return b.route_loop if b is not None else object.__getattribute__(self, "_route_loop")

    @route_loop.setter
    def route_loop(self, val: bool) -> None:
        object.__setattr__(self, "_route_loop", bool(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    d.brain.route_loop = bool(val)

    @property
    def route_waypoints(self) -> List[np.ndarray]:
        b = self._apex_brain()
        return b.route_waypoints if b is not None else object.__getattribute__(self, "_route_waypoints")

    @route_waypoints.setter
    def route_waypoints(self, val: List[np.ndarray]) -> None:
        object.__setattr__(self, "_route_waypoints", [np.copy(w) for w in val])
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    d.brain.route_waypoints = [np.copy(w) for w in val]

    @property
    def current_route_wp_idx(self) -> int:
        b = self._apex_brain()
        return b.current_route_wp_idx if b is not None else object.__getattribute__(self, "_current_route_wp_idx")

    @current_route_wp_idx.setter
    def current_route_wp_idx(self, val: int) -> None:
        object.__setattr__(self, "_current_route_wp_idx", int(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    d.brain.current_route_wp_idx = int(val)

    @property
    def wp_loitering(self) -> bool:
        b = self._apex_brain()
        return b.wp_loitering if b is not None else object.__getattribute__(self, "_wp_loitering")

    @wp_loitering.setter
    def wp_loitering(self, val: bool) -> None:
        object.__setattr__(self, "_wp_loitering", bool(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    d.brain.wp_loitering = bool(val)

    @property
    def wp_loiter_timer(self) -> float:
        b = self._apex_brain()
        return b.wp_loiter_timer if b is not None else object.__getattribute__(self, "_wp_loiter_timer")

    @wp_loiter_timer.setter
    def wp_loiter_timer(self, val: float) -> None:
        object.__setattr__(self, "_wp_loiter_timer", float(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    d.brain.wp_loiter_timer = float(val)

    @property
    def wp_total_loiter(self) -> float:
        b = self._apex_brain()
        return b.wp_total_loiter if b is not None else object.__getattribute__(self, "_wp_total_loiter")

    @wp_total_loiter.setter
    def wp_total_loiter(self, val: float) -> None:
        object.__setattr__(self, "_wp_total_loiter", float(val))
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    d.brain.wp_total_loiter = float(val)

    def on_route_deployed(self, clean_wps: List[np.ndarray], loop: bool = True) -> None:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if not clean_wps:
            return
        first_target = np.array(clean_wps[0][:3], dtype=float)
        first_loiter = float(clean_wps[0][3]) if len(clean_wps[0]) > 3 else 0.0
        self.current_seq += 1
        seq = self.current_seq
        self.route_waypoints = [np.copy(w) for w in clean_wps]
        self.route_active = True
        self.route_paused = False
        self.route_loop = bool(loop)
        self.current_route_wp_idx = 0
        self.wp_loitering = False
        self.wp_loiter_timer = 0.0
        self.wp_total_loiter = first_loiter

        for d in eng.drones:
            if d.crashed:
                continue
            d.mission_waypoints = [np.copy(w) for w in clean_wps]
            d.current_wp_idx = 0
            d.mission_active = True
            d.mission_paused = False
            d.mission_loop = bool(loop)
            d.wp_loitering = False
            d.wp_loiter_timer = 0.0
            d.wp_total_loiter = first_loiter
            d.acknowledged_target = np.copy(first_target)
            if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                d.brain.current_seq = seq
                d.brain.peer_state.active_seq = seq
                d.brain.peer_state.pending_seq = seq
                d.brain.peer_state.active_target = np.copy(first_target)
                d.brain.peer_state.pending_target = np.copy(first_target)
                d.brain.peer_state.gossip_status = "COORDINATED"

    def skip_waypoint(self) -> int:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        wps = self.route_waypoints
        if not wps:
            return 0
        idx = self.current_route_wp_idx
        if idx + 1 < len(wps):
            new_idx = idx + 1
        elif self.route_loop:
            new_idx = 0
        else:
            new_idx = idx
            self.route_active = False

        next_wp = np.array(wps[new_idx][:3], dtype=float)
        next_loiter = float(wps[new_idx][3]) if len(wps[new_idx]) > 3 else 0.0
        self.current_route_wp_idx = new_idx
        self.wp_loitering = False
        self.wp_loiter_timer = 0.0
        self.wp_total_loiter = next_loiter
        self.current_seq += 1
        seq = self.current_seq
        eng.c2._target_pos = np.copy(next_wp)

        for d in eng.drones:
            if d.crashed:
                continue
            d.current_wp_idx = new_idx
            d.acknowledged_target = np.copy(next_wp)
            d.wp_loitering = False
            d.wp_loiter_timer = 0.0
            d.wp_total_loiter = next_loiter
            if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                d.brain.current_seq = seq
                d.brain.peer_state.active_seq = seq
                d.brain.peer_state.pending_seq = seq
                d.brain.peer_state.active_target = np.copy(next_wp)
                d.brain.peer_state.pending_target = np.copy(next_wp)
                d.brain.peer_state.gossip_status = "COORDINATED"
        return new_idx

    def _get_formation_slot(
        self,
        drone: Any,
        k_or_leader: Any = 0,
        N: Optional[int] = None,
        target_pos: Optional[Any] = None,
        overhead_marshal: Optional[bool] = None,
        clock_s: Optional[float] = None,
    ) -> np.ndarray:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        b = self._apex_brain() or self
        if N is None and target_pos is None:
            living = sorted(
                [d for d in eng.drones if not d.crashed and getattr(d, "enrolled", True)],
                key=lambda d: d.id,
            )
            n_val = max(1, len(living))
            ids = [d.id for d in living]
            did = int(getattr(drone, "id", 0))
            k_val = self.assigned_slots.get(did, ids.index(did) if did in ids else 0)
            t_pos = eng.get_acknowledged_target()
        else:
            k_val = int(k_or_leader)
            n_val = int(N) if N is not None else max(1, len(eng.drones))
            t_pos = np.array(target_pos if target_pos is not None else eng.get_acknowledged_target(), dtype=float)
        ovh = eng.overhead_marshal_active if overhead_marshal is None else bool(overhead_marshal)
        clk = eng.sim_time if clock_s is None else float(clock_s)
        return GossipConsensusBrain._get_formation_slot(
            b,
            drone,
            k_val,
            n_val,
            t_pos,
            overhead_marshal=ovh,
            clock_s=clk,
        )

    def reset(self) -> None:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        if hasattr(eng, "drones"):
            for d in eng.drones:
                if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                    d.brain._default_target = np.copy(eng.target_pos)
                    d.brain.reset()

    def update_comms(self, dt: float) -> None:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        eng._deliver_radio_packets(dt)

    def compute_guidance(self, dt: float) -> Dict[int, np.ndarray]:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        return eng._execute_obc_step(dt)

    def inject_target(self, target_pos: Any) -> Dict[str, Any]:
        """Deliver a GCS C2 target Howl uplink to the closest mesh node within RANGE_HOWL."""
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        target_arr = np.array(target_pos, dtype=float)
        active_drones = [d for d in eng.drones if not d.crashed]
        if not active_drones:
            return {"status": "ERROR", "error": "No active drones in fleet"}

        gcs_pos = eng.tether_anchor
        seed_drone = min(
            active_drones,
            key=lambda d: float(np.linalg.norm(d.pos - gcs_pos)),
        )
        min_dist = float(np.linalg.norm(seed_drone.pos - gcs_pos))

        if min_dist > RANGE_HOWL:
            return {
                "status": "OUT_OF_RANGE",
                "reason": "RADIO_RANGE_EXCEEDED",
                "error": f"Nearest mesh node ({min_dist:.1f}m) out of GCS Howl range ({RANGE_HOWL}m)",
            }

        if not eng.route_active:
            self.route_active = False

        seq = self.current_seq + 1
        self.current_seq = seq
        pkt = RadioPacket(
            sender_id=-1,
            msg_type="GCS_UPLINK",
            timestamp=float(eng.sim_time),
            seq=seq,
            payload={
                "cmd": "SET_TARGET",
                "target": [float(v) for v in target_arr],
                "seq": seq,
                "multi_node": len(active_drones) > 1,
            },
        )
        seed_drone.inbox.append(pkt)
        if hasattr(seed_drone, "brain") and isinstance(seed_drone.brain, GossipConsensusBrain):
            tel = eng._build_local_telemetry(seed_drone)
            seed_drone.brain.ingest_gossip_packets(tel, [pkt])

        return {
            "status": "OK",
            "seed_drone_id": seed_drone.id,
            "seq": seq,
            "target_seq": seq,
            "dist_to_gcs": round(min_dist, 1),
        }

    def handle_command(self, cmd_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        cmd = str(cmd_dict.get("cmd", "")).upper()
        if cmd == "SET_TARGET":
            pos = cmd_dict.get("pos", [60.0, 40.0, 30.0])
            return self.inject_target(pos)
        elif cmd == "SKIP_WAYPOINT":
            new_idx = self.skip_waypoint()
            return {"status": "OK", "action": "WAYPOINT_SKIPPED", "current_wp_idx": new_idx}
        res = None
        for d in eng.drones:
            if hasattr(d, "brain") and d.brain is not None:
                r = d.brain.handle_command(cmd_dict)
                if r is not None:
                    res = r
        if cmd == "SET_FORMATION" and isinstance(res, dict) and "formation" in res:
            object.__setattr__(self, "_formation_mode", res["formation"])
            eng.formation_mode = res["formation"]
        elif cmd == "PAUSE_ROUTE":
            self.route_paused = True
            for d in eng.drones:
                d.mission_paused = True
        elif cmd == "RESUME_ROUTE":
            self.route_paused = False
            for d in eng.drones:
                d.mission_paused = False
        return res

    def get_telemetry_extra(self) -> Dict[str, Any]:
        eng: SITLWorldEngine = object.__getattribute__(self, "_engine")
        active_drones = [d for d in eng.drones if not d.crashed]
        propagating_count = 0
        for d in active_drones:
            if hasattr(d, "brain") and isinstance(d.brain, GossipConsensusBrain):
                if d.brain.peer_state.gossip_status == "PROPAGATING":
                    propagating_count += 1
        return {
            "brain_mode": "GOSSIP_CONSENSUS",
            "formation": self.formation_mode,
            "formation_heading_deg": round(math.degrees(self.formation_heading) % 360.0, 1),
            "target_seq": self.current_seq,
            "propagating_drones": propagating_count,
            "consensus_reached": (propagating_count == 0),
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


