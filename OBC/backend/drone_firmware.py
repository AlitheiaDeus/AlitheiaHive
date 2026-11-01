"""
SwarmTech3_UpRising: Autonomous Drone Firmware & OBC State Bubble (drone_firmware.py)
Represents the autonomous Onboard Computer (OBC) for an individual airframe.
Maintains kinematics state, telemetry, local acknowledged target mailbox,
discrete waypoint queue, altitude-gated breakaway dive, and local failsafes.
"""

from __future__ import annotations
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any, Union
import numpy as np

from OBC.backend.brains import BaseBrain, LeaderFollowerBrain, RadioPacket

# Hardware Tiers & Succession (TKT-48, TKT-57)
TIER_LEADER: int = 3
TIER_ESCORT: int = 2
TIER_SCOUT: int = 1

# Modular Base Tiers (TKT-57)
BASE_TIER_T0: str = "T0"         # Stationary Anchor / Heavy Ground Station
BASE_TIER_T1_5: str = "T1.5"     # Standard Tactical Multi-Role Baseline (Beta)
BASE_TIER_T3: str = "T3"         # Heavy Lift / Compute Specialist (Kodiak)

# Modular Tether Prefixes (TKT-57)
TETHER_NONE: str = "NONE"             # Untethered wireless
TETHER_DATA_ONLY: str = "DATA_ONLY"   # t- prefix: Hair-thin micro-fiber, onboard LiPo, zero RF
TETHER_POWER_DATA: str = "POWER_DATA" # T- prefix: Full ground umbilical, infinite loiter

# Dynamic RF Comms Profiles (TKT-57)
RF_MODE_YIP: str = "YIP"         # 50m Low-power whisper mode (escort standby)
RF_MODE_BORK: str = "BORK"       # 65m Amplified PA boost mode (active leader / relay)
RF_MODE_HOWL: str = "HOWL"       # 120m GCS mast uplink

# Default Capabilities Matrix (TKT-57)
DEFAULT_CAPABILITIES_T1_5: List[str] = [
    "FORMATION_FLIGHT",
    "PATROL",
    "RECON",
    "S&R_OPTICAL",
    "BORK_AMPLIFY",
]

DEFAULT_CAPABILITIES_T3: List[str] = [
    "FORMATION_FLIGHT",
    "HEAVY_PAYLOAD",
    "LIDAR_SCAN",
    "JAMMER_TRIANGULATION",
    "DROP_SERVO",
]

DEFAULT_CAPABILITIES_T0: List[str] = [
    "BASE_ANCHOR",
    "HIGH_GAIN_HOWL",
    "INFINITE_LOITER",
]


class Drone:
    """Represents an autonomous quadcopter agent with physical kinematics,
    telemetry state, comms link, and navigation brain."""

    def __init__(
        self,
        drone_id: int,
        tier: int = TIER_ESCORT,
        pos: Optional[np.ndarray] = None,
        is_leader: bool = False,
        hardware_uid: str = "",
        tail_number: str = "",
        base_tier: str = BASE_TIER_T1_5,
        tether_type: str = TETHER_NONE,
        rf_power_mode: str = RF_MODE_YIP,
        capabilities: Optional[List[str]] = None,
    ):
        self.id: int = drone_id
        self.tier: int = tier
        self.is_leader: bool = is_leader

        # Airframe Hardware Identity (TKT-48)
        self.hardware_uid: str = hardware_uid or f"3842A10F9B20{drone_id + 1:04X}"
        self.tail_number: str = tail_number or f"ALITHEIA-V{drone_id + 1:02d}"

        # Modular Hardware Tier & Tether Prefixes (TKT-57)
        self.base_tier: str = base_tier
        self.tether_type: str = tether_type
        self.rf_power_mode: str = rf_power_mode if not is_leader else RF_MODE_BORK
        if capabilities is not None:
            self.capabilities: List[str] = list(capabilities)
        elif self.base_tier == BASE_TIER_T3:
            self.capabilities: List[str] = list(DEFAULT_CAPABILITIES_T3)
        elif self.base_tier == BASE_TIER_T0:
            self.capabilities: List[str] = list(DEFAULT_CAPABILITIES_T0)
        else:
            self.capabilities: List[str] = list(DEFAULT_CAPABILITIES_T1_5)

        # Kinematics
        self.pos: np.ndarray = (
            np.array(pos, dtype=float) if pos is not None else np.zeros(3, dtype=float)
        )
        self.vel: np.ndarray = np.zeros(3, dtype=float)
        self.acc: np.ndarray = np.zeros(3, dtype=float)

        # Orientation & Visuals
        self.yaw: float = 0.0          # Radians (0 = +X axis)
        self.target_yaw: Optional[float] = None # Commanded heading from guidance brain (rad) (TKT-55)
        self.pitch: float = 0.0        # Radians
        self.roll: float = 0.0         # Radians
        self.rotor_rpm: float = 5200.0 # Visual rotor speed
        self.scramble_time: float = 0.0 # Scheduled scramble launch timestamp (TKT-41)
        self.briefing: str = ""         # Mission pre-briefing / rendezvous text (TKT-41)

        # Comms & Fault Tolerance
        self.crashed: bool = False
        self.flight_state: str = "CRUISE"  # "TAKEOFF", "EN_ROUTE", "CRUISE", "CRASHED", "LANDED", "GROUND_IDLE"
        self.lost_link: bool = False
        self.lkp: np.ndarray = np.copy(self.pos)
        self.last_heartbeat_time: float = 0.0
        self.rssi: float = -45.0           # Signal strength (dBm) (TKT-17)
        self.link_quality: float = 1.0     # Normalized link quality [0.0, 1.0] (TKT-17)
        self.uplink_active: bool = True    # True if within GCS Howl range (TKT-49)
        self.downlink_active: bool = True  # True if direct or gateway relay to GCS (TKT-49)
        self.yip_lost: bool = False        # True if uplink received but downlink silent (TKT-49)
        self.gateway_id: Optional[int] = None # ID of mesh node relaying to GCS (TKT-49)

        # Status & Battery
        self.battery: float = 100.0
        self.status: str = "ACTIVE" if not is_leader else "FLEET_LEADER"
        self.tether_lost: bool = False

        # Succession Deconfliction & Lateral Peel-Off (TKT-15)
        self.peel_off_timer: float = 0.0
        self.peel_off_dir: np.ndarray = np.zeros(3, dtype=float)
        self.promotion_timer: float = 0.0

        # Tether Failsafes & Cable Jettison (TKT-16 / TKT-27)
        self.jettison_timer: float = 0.0
        self.jettison_watchdog: float = 6.0
        self.tether_jettisoned: bool = False
        self.tether_failsafe_policy: str = "REJOIN"  # "REJOIN" or "RTB"

        # Autonomous Swarm RTB & Coordinated Soft-Landing (TKT-19)
        self.landing_pad_idx: Optional[int] = None
        self.landing_pad: Optional[np.ndarray] = None
        self.landing_wait_timer: float = 0.0
        self.landing_stage: str = "NONE"  # "NONE", "WAITING_CLEARANCE", "APPROACH", "FINAL_DESCENT", "LANDED", "TOUCHDOWN_CONFIRMED"
        self.touchdown_confirmed: bool = False
        self.gated_by_drone_id: Optional[int] = None

        # Onboard Target Buffer (TKT-25: Ground Truth vs Autopilot Decoupling)
        self.acknowledged_target: np.ndarray = (
            np.array(pos, dtype=float) if pos is not None else np.array([60.0, 40.0, 30.0], dtype=float)
        )
        self.target_station: Optional[np.ndarray] = None

        # Discrete Waypoint Mission Queue & Loiter Timers (TKT-28, TKT-30, TKT-56)
        self.mission_waypoints: List[np.ndarray] = []
        self.current_wp_idx: int = 0
        self.mission_active: bool = False
        self.mission_loop: bool = True
        self.mission_name: str = ""
        self.mission_paused: bool = False
        self.wp_loiter_timer: float = 0.0
        self.wp_loitering: bool = False
        self.wp_total_loiter: float = 0.0

        # Dynamic Swarm Enrollment & Solo Scout Rejoin (TKT-50)
        self.enrolled: bool = True
        self.enrollment_status: str = "ENROLLED"  # "ENROLLED", "SCOUT", "REJOINING"
        self.enrollment_timer: float = 0.0
        self.rejoin_target: Optional[np.ndarray] = None

        # Tactical GNSS Denial & Dead-Reckoning RTB (TKT-52)
        self.gnss_denied: bool = False
        self.gnss_denied_timer: float = 0.0
        self.reciprocal_heading: float = 0.0  # Locked entry track reciprocal heading (radians)
        self.drift_radius: float = 0.0       # Time-scaled drift error circle (meters)
        self.rssi_prev: float = -100.0       # Previous scalar Howl RSSI for gradient ascent (dBm)
        self.gnss_settled: bool = False      # Emergency blind soft-settle flag
        self.crashed_while_linked: bool = False
        self.retrieved: bool = False
        self.declared_destroyed: bool = False
        self.mayday_beacon: bool = False
        self.mayday_reason: str = ""
        self.collision_inferred: bool = False

        # Heartbeat Watchdog & Breadcrumb Backtrack (TKT-54)
        self.gcs_heartbeat_watchdog: float = 2.0
        self.radio_silence_permitted: bool = False
        self.breadcrumbs: List[Tuple[float, np.ndarray]] = []
        self.breadcrumb_record_timer: float = 0.0
        self.holding_at_horizon: bool = False
        self.backtrack_target: Optional[np.ndarray] = None
        self.backtrack_hold: bool = False

        # Per-Drone Autonomous Brain & Packet Mailboxes (TLK-01 Pure I/O Boundary)
        self.inbox: List[RadioPacket] = []
        self.outbox: List[RadioPacket] = []
        self.brain: BaseBrain = LeaderFollowerBrain(drone_id=self.id)

    def reset_state(
        self,
        pos: np.ndarray,
        is_leader: bool = False,
        flight_state: str = "TAKEOFF",
        hardware_uid: Optional[str] = None,
        tail_number: Optional[str] = None,
    ) -> None:
        """Reset drone state for relaunch or reboot."""
        if hardware_uid is not None:
            self.hardware_uid = hardware_uid
        if tail_number is not None:
            self.tail_number = tail_number
        self.pos = np.array(pos, dtype=float)
        self.vel = np.zeros(3, dtype=float)
        self.acc = np.zeros(3, dtype=float)
        self.yaw = 0.0
        self.target_yaw = None
        self.pitch = 0.0
        self.roll = 0.0
        self.rotor_rpm = 5200.0
        self.crashed = False
        self.flight_state = flight_state
        self.lost_link = False
        self.lkp = np.copy(self.pos)
        self.last_heartbeat_time = 0.0
        self.rssi = -45.0
        self.link_quality = 1.0
        self.uplink_active = True
        self.downlink_active = True
        self.yip_lost = False
        self.gateway_id = None
        self.battery = 100.0
        self.is_leader = is_leader
        self.status = "FLEET_LEADER" if is_leader else "ACTIVE"
        self.tether_lost = False
        self.peel_off_timer = 0.0
        self.peel_off_dir = np.zeros(3, dtype=float)
        self.promotion_timer = 0.0
        self.jettison_timer = 0.0
        self.jettison_watchdog = 6.0
        self.tether_jettisoned = False
        self.tether_failsafe_policy = "REJOIN"
        self.landing_pad_idx = None
        self.landing_pad = None
        self.landing_wait_timer = 0.0
        self.landing_stage = "NONE"
        self.touchdown_confirmed = False
        self.gated_by_drone_id = None
        self.acknowledged_target = np.array(pos, dtype=float)
        self.target_station = None
        self.mission_waypoints = []
        self.current_wp_idx = 0
        self.mission_active = False
        self.mission_loop = True
        self.mission_name = ""
        self.mission_paused = False
        self.wp_loiter_timer = 0.0
        self.wp_loitering = False
        self.wp_total_loiter = 0.0
        self.enrolled = True
        self.enrollment_status = "ENROLLED"
        self.enrollment_timer = 0.0
        self.rejoin_target = None
        self.gnss_denied = False
        self.gnss_denied_timer = 0.0
        self.reciprocal_heading = 0.0
        self.drift_radius = 0.0
        self.rssi_prev = -100.0
        self.gnss_settled = False
        self.crashed_while_linked = False
        self.retrieved = False
        self.declared_destroyed = False
        self.mayday_beacon = False
        self.mayday_reason = ""
        self.collision_inferred = False
        self.gcs_heartbeat_watchdog = 2.0
        self.radio_silence_permitted = False
        self.breadcrumbs = []
        self.breadcrumb_record_timer = 0.0
        self.holding_at_horizon = False
        self.backtrack_target = None
        self.backtrack_hold = False
        self.inbox.clear()
        self.outbox.clear()
        if hasattr(self, "brain") and self.brain is not None:
            self.brain.reset()

    def record_breadcrumb(self, clock_s: float) -> None:
        """Record current 3D position in the rolling 10-second (2 Hz) breadcrumb buffer."""
        self.breadcrumbs.append((clock_s, np.copy(self.pos)))
        if len(self.breadcrumbs) > 20:
            self.breadcrumbs.pop(0)

    def get_backtrack_point(self, retreat_distance: float = 18.0) -> np.ndarray:
        """Find a breadcrumb point approximately retreat_distance back along the trail."""
        if not self.breadcrumbs:
            return np.copy(self.lkp)
        for _, b_pos in reversed(self.breadcrumbs):
            if float(np.linalg.norm(self.pos - b_pos)) >= retreat_distance:
                return np.copy(b_pos)
        return np.copy(self.breadcrumbs[0][1])

    @property
    def target_reached(self) -> bool:
        """Whether this drone has arrived within 2.5m acceptance radius of its acknowledged target."""
        if self.crashed:
            return False
        return float(np.linalg.norm(self.pos - self.acknowledged_target)) <= 2.5

    @property
    def full_tier_designation(self) -> str:
        """Modular tier designation with tether prefix (TKT-57):
        - 'T-' for Power+Data umbilical (e.g. 'T-T0')
        - 't-' for Data-only micro-fiber (e.g. 't-T1.5')
        - No prefix for untethered wireless (e.g. 'T1.5', 'T3')"""
        if self.tether_type == TETHER_POWER_DATA:
            prefix = "T-"
        elif self.tether_type == TETHER_DATA_ONLY:
            prefix = "t-"
        else:
            prefix = ""
        return f"{prefix}{self.base_tier}"

    def succession_score(self) -> int:
        """Calculate autonomic succession eligibility score: (Tier * 1000) - ID."""
        return (self.tier * 1000) - self.id

    def to_dict(self) -> Dict[str, Any]:
        """Serialize state for WebSocket JSON telemetry frame."""
        return {
            "id": self.id,
            "sysid": self.id + 1,
            "hardware_uid": self.hardware_uid,
            "tail_number": self.tail_number,
            "tier": self.tier,
            "base_tier": self.base_tier,
            "tether_type": self.tether_type,
            "full_tier_designation": self.full_tier_designation,
            "rf_power_mode": self.rf_power_mode,
            "capabilities": list(self.capabilities),
            "is_leader": self.is_leader,
            "pos": [round(float(v), 2) for v in self.pos],
            "vel": [round(float(v), 2) for v in self.vel],
            "yaw": round(float(self.yaw), 3),
            "target_yaw": round(float(self.target_yaw), 3) if self.target_yaw is not None else None,
            "pitch": round(float(self.pitch), 3),
            "roll": round(float(self.roll), 3),
            "crashed": self.crashed,
            "flight_state": self.flight_state,
            "lost_link": self.lost_link,
            "lkp": [round(float(v), 2) for v in self.lkp],
            "rssi": round(float(self.rssi), 1),
            "link_quality": round(float(self.link_quality), 2),
            "uplink_active": self.uplink_active,
            "downlink_active": self.downlink_active,
            "yip_lost": self.yip_lost,
            "gateway_id": self.gateway_id,
            "battery": round(float(self.battery), 1),
            "status": self.status,
            "rotor_rpm": round(float(self.rotor_rpm), 0),
            "jettison_timer": round(float(self.jettison_timer), 2),
            "jettison_watchdog": round(float(self.jettison_watchdog), 2),
            "tether_jettisoned": self.tether_jettisoned,
            "tether_lost": self.tether_lost,
            "tether_failsafe_policy": self.tether_failsafe_policy,
            "landing_pad_idx": self.landing_pad_idx,
            "landing_stage": self.landing_stage,
            "touchdown_confirmed": self.touchdown_confirmed,
            "gated_by_drone_id": self.gated_by_drone_id,
            "acknowledged_target": [round(float(v), 2) for v in self.acknowledged_target],
            "mission_active": self.mission_active,
            "current_wp_idx": self.current_wp_idx,
            "wp_count": len(self.mission_waypoints),
            "wp_loiter_timer": round(float(self.wp_loiter_timer), 2),
            "wp_loitering": self.wp_loitering,
            "target_reached": self.target_reached,
            "enrolled": self.enrolled,
            "enrollment_status": self.enrollment_status,
            "enrollment_timer": round(float(self.enrollment_timer), 2),
            "rejoin_target": [round(float(v), 2) for v in self.rejoin_target] if self.rejoin_target is not None else None,
            "briefing": self.briefing,
            "scramble_time": round(float(self.scramble_time), 2),
            "gnss_denied": self.gnss_denied,
            "gnss_denied_timer": round(float(self.gnss_denied_timer), 2),
            "reciprocal_heading": round(float(self.reciprocal_heading), 3),
            "drift_radius": round(float(self.drift_radius), 2),
            "gnss_settled": self.gnss_settled,
            "crashed_while_linked": self.crashed_while_linked,
            "retrieved": self.retrieved,
            "declared_destroyed": self.declared_destroyed,
            "mayday_beacon": self.mayday_beacon,
            "mayday_reason": self.mayday_reason,
            "collision_inferred": self.collision_inferred,
            "gcs_heartbeat_watchdog": round(float(self.gcs_heartbeat_watchdog), 2),
            "radio_silence_permitted": self.radio_silence_permitted,
            "holding_at_horizon": self.holding_at_horizon,
            "backtrack_hold": self.backtrack_hold,
            "backtrack_target": [round(float(v), 2) for v in self.backtrack_target] if self.backtrack_target is not None else None,
            "mission_active": self.mission_active,
            "mission_name": self.mission_name,
            "mission_paused": self.mission_paused,
            "current_wp_idx": self.current_wp_idx,
            "wp_loitering": self.wp_loitering,
            "wp_loiter_timer": round(float(self.wp_loiter_timer), 1),
            "wp_total_loiter": round(float(self.wp_total_loiter), 1),
        }
