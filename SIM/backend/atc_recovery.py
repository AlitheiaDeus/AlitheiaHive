"""
SwarmTech3_UpRising: Batched ATC Pad Recovery & Scramble Manager (atc_recovery.py)
Bubble 2: SITL World Simulator.
Encapsulates airport ATC landing pad allocation, overhead marshalling stacks,
staggered descent clearance gates (Downwash Anti-Vortex), turnaround reset, and sequenced cold scrambles (TKT-41 / TKT-42).
"""

from __future__ import annotations
import math
from typing import Dict, List, Optional, Tuple, Any, Union, Callable
import numpy as np

from OBC.backend.drone_firmware import FlightState

# ATC Pad Layout Constants
ATC_DEFAULT_CAPACITY: int = 8
PAD_CLEARANCE_RADIUS: float = 3.0


def get_pad_position(pad_idx: int) -> np.ndarray:
    """Calculate coordinates for recovery/launch pad index (0..7).
    Base pad layout outside the 3.0m tether clearance envelope.
    Centered at (0.0, -6.0, 0.5) with multi-drone grid layout (>= 2.5m separation)."""
    idx = pad_idx % 8
    pad_x = float((idx % 4 - 1.5) * 2.5)
    pad_y = float(-6.0 - (idx // 4) * 3.0)
    return np.array([pad_x, pad_y, 0.5], dtype=float)


class ATCRecoveryManager:
    """Manages multi-pad landing allocations, overhead marshal stacks,
    automatic turnaround servicing, and staggered scrambles for the swarm fleet."""

    def __init__(self, capacity: int = ATC_DEFAULT_CAPACITY) -> None:
        self.capacity: int = capacity
        self.landing_pads: Dict[int, int] = {}       # pad_idx -> drone_id
        self.atc_waiting_queue: List[int] = []       # Drone IDs holding in marshal stack
        self.atc_current_wave: int = 1
        self.turnaround_timer: float = 0.0
        self.auto_turnaround_enabled: bool = True
        self.scramble_active: bool = False
        self.scramble_timer: float = 0.0
        self.scramble_stagger: float = 1.2
        self.recovery_stage: str = "IDLE"            # "IDLE", "OVERHEAD_MARSHAL", "LANDING"
        self.swarm_rtb_active: bool = False
        self.overhead_marshal_active: bool = False

    def is_in_comms_range(self, drones: List[Any], tether_anchor: np.ndarray, radio_range: float = 140.0) -> bool:
        """Check whether at least one living drone is within the GCS radio horizon."""
        living = [d for d in drones if not d.crashed]
        if not living:
            return False
        min_dist = min(float(np.linalg.norm(d.pos - tether_anchor)) for d in living)
        return min_dist <= radio_range

    def command_rtb_hold(
        self,
        drones: List[Any],
        tether_anchor: np.ndarray,
        radio_range: float = 140.0,
        force_autonomous: bool = False,
        set_target_fn: Optional[Callable[[List[float]], Any]] = None,
    ) -> Dict[str, Any]:
        """Stage 1 Recovery: Commanded RTB (Hold).
        Fleet transits to Base (0, 0, 25m) at cruise speed and settles into
        a stationary sentry ring at Z=20-25m outside 3m tether clearance cylinder."""
        if not force_autonomous and not self.is_in_comms_range(drones, tether_anchor, radio_range):
            return {
                "status": "DROPPED",
                "reason": "OUT_OF_RANGE",
                "error": f"Fleet is beyond GCS radio horizon ({radio_range}m). RTB command dropped.",
            }

        self.overhead_marshal_active = True
        self.swarm_rtb_active = True
        self.recovery_stage = "OVERHEAD_MARSHAL"

        base_target = [0.0, 0.0, 25.0]
        if set_target_fn is not None:
            set_target_fn(base_target)

        for drone in drones:
            if not drone.crashed and drone.flight_state not in ("LANDED", "DESCENDING"):
                drone.flight_state = FlightState.OVERHEAD_HOLD
                drone.status = "OVERHEAD_HOLD"
                drone.acknowledged_target = np.array(base_target, dtype=float)
                drone.lkp = np.array(base_target, dtype=float)
                if hasattr(drone, "brain") and hasattr(drone.brain, "peer_state"):
                    drone.brain.peer_state.active_target = np.array(base_target, dtype=float)
                    drone.brain.peer_state.pending_target = np.array(base_target, dtype=float)
                    drone.brain.peer_state.gossip_status = "COORDINATED"
        return {"status": "OK", "recovery_stage": self.recovery_stage}

    def command_corridor_ertb(
        self,
        drones: List[Any],
        log_event_fn: Optional[Callable[[str, str], Any]] = None,
        set_target_fn: Optional[Callable[[np.ndarray], Any]] = None,
    ) -> Dict[str, Any]:
        """Execute autonomous corridor emergency return-to-base (ERTB) (BUG-06).
        Airframes traverse popped IngressCorridorStack waypoints in reverse order back to Base [0, 0, 25m]."""
        self.swarm_rtb_active = True
        for drone in drones:
            if not drone.crashed and drone.flight_state not in ("LANDED", "DESCENDING", "GROUND_IDLE"):
                drone.flight_state = FlightState.CORRIDOR_ERTB
                drone.status = "CORRIDOR_ERTB"
                if hasattr(drone, "corridor_stack"):
                    target = drone.corridor_stack.peek()
                    if target is not None:
                        drone.corridor_target = target
                        drone.acknowledged_target = np.copy(target)
                        drone.target_station = np.copy(target)
                        if drone.is_leader and set_target_fn is not None:
                            set_target_fn(np.copy(target))
        if log_event_fn is not None:
            log_event_fn("WARN", "[EMERGENCY RTB: Swarm executing Corridor ERTB along ingress breadcrumbs back to Base.]")
        return {"status": "OK", "flight_state": "CORRIDOR_ERTB"}

    def cancel_rtb(self, drones: List[Any]) -> None:
        """Cancel RTB / overhead marshaling and return living airborne drones to mission cruise."""
        self.overhead_marshal_active = False
        self.swarm_rtb_active = False
        self.recovery_stage = "IDLE"
        self.turnaround_timer = 0.0
        self.atc_current_wave = 1
        self.atc_waiting_queue.clear()
        for drone in drones:
            if not drone.crashed and drone.flight_state in ("OVERHEAD_HOLD", "DESCENDING"):
                drone.flight_state = "CRUISE"
                drone.status = "FLEET_LEADER" if drone.is_leader else "ACTIVE"
                drone.landing_stage = "NONE"
                drone.gated_by_drone_id = None
                drone.touchdown_confirmed = False
                if drone.landing_pad_idx is not None and drone.landing_pad_idx in self.landing_pads:
                    del self.landing_pads[drone.landing_pad_idx]
                drone.landing_pad_idx = None

    def command_land_swarm(
        self,
        drones: List[Any],
        tether_anchor: np.ndarray,
        radio_range: float = 140.0,
        succession_fn: Optional[Callable[..., Any]] = None,
    ) -> Union[List[int], Dict[str, Any]]:
        """Stage 2 Recovery: Operator-authorized coordinated landing.
        Engages ATC pad reservation, sequential staggered descent
        with Downwash Anti-Vortex Gate, and overhead marshal wave holding (TKT-42)."""
        if not self.is_in_comms_range(drones, tether_anchor, radio_range):
            return {
                "status": "DROPPED",
                "reason": "OUT_OF_RANGE",
                "error": f"Fleet is beyond GCS radio horizon ({radio_range}m). Landing command dropped.",
            }

        airborne_drones = [
            d for d in drones
            if not d.crashed and not getattr(d, "retrieved", False) and d.flight_state not in ("LANDED", "GROUND_IDLE", "DESCENDING", "RETRIEVED")
        ]
        if not airborne_drones:
            return []

        airborne_drones.sort(key=lambda d: d.id)
        self.recovery_stage = "LANDING"

        available_pads = [p for p in range(self.capacity) if p not in self.landing_pads]
        batch_drones = airborne_drones[:len(available_pads)]
        marshal_drones = airborne_drones[len(available_pads):]

        queued_ids = []
        for i, drone in enumerate(batch_drones):
            pad_idx = available_pads[i]
            self.landing_pads[pad_idx] = drone.id
            drone.landing_pad_idx = pad_idx
            drone.landing_pad = get_pad_position(pad_idx)
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

            if drone.is_leader and succession_fn is not None:
                drone.is_leader = False
                succession_fn(demote_old_leader=True, retired_leader=drone)

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
                shelf_z = 25.0 + (k % 5) * 2.0
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

    def command_land_drone(
        self,
        drone_id: int,
        drones: List[Any],
        tether_anchor: np.ndarray,
        radio_range: float = 140.0,
        succession_fn: Optional[Callable[..., Any]] = None,
    ) -> bool:
        """Command an individual drone to land on an available recovery pad."""
        if not self.is_in_comms_range(drones, tether_anchor, radio_range):
            return False
        for drone in drones:
            if drone.id == drone_id and not drone.crashed and drone.flight_state not in ("LANDED", "DESCENDING"):
                if not getattr(drone, "uplink_active", True):
                    return False
                used_pads = set(self.landing_pads.keys())
                available = [p for p in range(self.capacity) if p not in used_pads]
                if not available:
                    return False
                pad_idx = available[0]
                self.landing_pads[pad_idx] = drone.id
                drone.landing_pad_idx = pad_idx
                drone.landing_pad = get_pad_position(pad_idx)
                drone.landing_wait_timer = 0.0
                drone.flight_state = "DESCENDING"
                drone.status = "LANDING"
                drone.landing_stage = "APPROACH"
                drone.gated_by_drone_id = None
                drone.touchdown_confirmed = False

                if drone.is_leader and succession_fn is not None:
                    drone.is_leader = False
                    succession_fn(demote_old_leader=True, retired_leader=drone)
                return True
        return False

    def command_takeoff_drone(
        self,
        drone_id: int,
        drones: List[Any],
        tether_anchor: np.ndarray,
        radio_range: float = 140.0,
    ) -> bool:
        """Relaunch a landed or ground-idle drone from its pad via vertical takeoff."""
        if not self.is_in_comms_range(drones, tether_anchor, radio_range):
            return False
        for drone in drones:
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

    def retrieve_drone_from_pad(self, pad_idx: int, drones: List[Any]) -> Optional[int]:
        """Ground crew physically retrieves and stows an airframe or wreckage from a landing pad into the hangar."""
        pad_pos = get_pad_position(pad_idx)
        assigned_id = self.landing_pads.get(pad_idx, None)
        target_drone = next((d for d in drones if d.id == assigned_id), None) if assigned_id is not None else None

        if target_drone is None or getattr(target_drone, "retrieved", False):
            for d in drones:
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

    def retrieve_all_landed_drones(self, drones: List[Any]) -> List[int]:
        """Ground crew physically retrieves all landed/ground-idle airframes across all pads into hangar."""
        retrieved = []
        for pad_idx in range(self.capacity):
            did = self.retrieve_drone_from_pad(pad_idx, drones)
            if did is not None:
                retrieved.append(did)
        return retrieved

    def clear_pads(
        self,
        drones: List[Any],
        turnaround_rearm: bool = False,
        land_swarm_fn: Optional[Callable[[], Any]] = None,
    ) -> Dict[str, Any]:
        """Reset/clear occupied pads (salvaging/storing landed drones or re-arming)
        and release the next wave of airborne drones from overhead marshal hold to land (TKT-42)."""
        cleared_drones = []
        for pad_idx, drone_id in list(self.landing_pads.items()):
            d = next((x for x in drones if x.id == drone_id), None)
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
            d for d in drones
            if not d.crashed and d.flight_state == "OVERHEAD_HOLD"
        ]

        released_ids = []
        if waiting_airborne and land_swarm_fn is not None:
            self.atc_current_wave += 1
            res = land_swarm_fn()
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

    def command_scramble_swarm(
        self,
        drones: List[Any],
        tether_anchor: np.ndarray,
        stagger_s: float = 1.2,
        radio_range: float = 140.0,
        tier_leader: str = "TIER_1_LEADER",
    ) -> Dict[str, Any]:
        """Initiate sequenced cold scramble from ground pads (TKT-41).
        Flagship #0 spins up and departs immediately; followers are staggered
        every stagger_s (default 1.2s) to maintain downwash clearance."""
        if not self.is_in_comms_range(drones, tether_anchor, radio_range):
            return {
                "status": "DROPPED",
                "reason": "OUT_OF_RANGE",
                "error": f"Fleet is beyond GCS radio horizon ({radio_range}m). Scramble command dropped.",
            }

        grounded = [d for d in drones if not d.crashed and d.flight_state in ("GROUND_IDLE", "LANDED")]
        if not grounded:
            return {"status": "NOOP", "reason": "NO_GROUNDED_DRONES"}

        leader_drones = [d for d in grounded if d.is_leader]
        follower_drones = sorted([d for d in grounded if not d.is_leader], key=lambda d: d.id)

        launch_sequence = leader_drones + follower_drones
        if not any(d.is_leader for d in drones if not d.crashed):
            if launch_sequence:
                launch_sequence[0].is_leader = True
                launch_sequence[0].tier = tier_leader

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
