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

from dataclasses import dataclass, field
from OBC.backend.drone_firmware import FlightState
from OBC.backend.brains.base import RadioPacket


@dataclass
class FieldLandingZone:
    """Designated Field Landing Zone (FLZ) for remote or off-base recovery (TKT-64 / PLN-05)."""
    id: str
    name: str
    coords: np.ndarray  # Center coordinates [x, y, z] (z typically ground level ~0.05m)
    radius: float = 15.0  # Field perimeter radius in meters
    active: bool = True
    capacity: int = 16  # Max simultaneous field landing slots
    slot_spacing: float = 4.0  # Separation between adjacent airframes in field

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

        # Designated Field Landing Zone & Emergency Landings (TKT-64 / PLN-05)
        self.active_field_lz: Optional[FieldLandingZone] = None
        self.field_lz_slots: Dict[int, int] = {}  # slot_idx -> drone_id
        self.pad_denied_override: bool = False

    def is_in_comms_range(self, drones: List[Any], tether_anchor: np.ndarray, radio_range: float = 140.0) -> bool:
        """Check whether at least one living drone is within the GCS radio horizon."""
        living = [d for d in drones if not d.crashed]
        if not living:
            return False
        min_dist = min(float(np.linalg.norm(d.pos - tether_anchor)) for d in living)
        return min_dist <= radio_range

    def set_field_lz(
        self,
        coords: Union[List[float], np.ndarray],
        name: str = "FLZ-1",
        radius: float = 15.0,
        capacity: int = 16,
        slot_spacing: float = 4.0,
    ) -> Dict[str, Any]:
        """Designate a Field Landing Zone for mission recovery or pad denial diversion."""
        c = np.array(coords, dtype=float)
        if len(c) == 2:
            c = np.array([c[0], c[1], 0.05], dtype=float)
        self.active_field_lz = FieldLandingZone(
            id=f"FLZ_{name.upper()}",
            name=name,
            coords=c,
            radius=float(radius),
            active=True,
            capacity=capacity,
            slot_spacing=slot_spacing,
        )
        self.field_lz_slots.clear()
        return {
            "status": "OK",
            "field_lz": {
                "id": self.active_field_lz.id,
                "name": self.active_field_lz.name,
                "coords": [round(float(v), 2) for v in self.active_field_lz.coords],
                "radius": self.active_field_lz.radius,
                "capacity": self.active_field_lz.capacity,
            }
        }

    def clear_field_lz(self) -> Dict[str, Any]:
        """Deactivate and clear the current Field Landing Zone."""
        self.active_field_lz = None
        self.field_lz_slots.clear()
        return {"status": "OK", "field_lz": None}

    def get_field_slot_position(self, slot_idx: int, flz: Optional[FieldLandingZone] = None) -> np.ndarray:
        """Calculate coordinates for a field landing slot in an open radial sentry circle.
        Centered at flz.coords with radius = 8.0m and wide spacing (>= 4.0m) at ground Z=0.05m."""
        active_flz = flz or self.active_field_lz
        if active_flz is None:
            return get_pad_position(0)

        center = active_flz.coords
        cap = max(1, active_flz.capacity)
        ring_r = max(6.0, (cap * active_flz.slot_spacing) / (2.0 * math.pi))
        ring_r = min(ring_r, active_flz.radius - 2.0)
        angle = (2.0 * math.pi * slot_idx) / cap
        sx = center[0] + ring_r * math.cos(angle)
        sy = center[1] + ring_r * math.sin(angle)
        sz = center[2] if center[2] > 0.0 else 0.05
        return np.array([sx, sy, sz], dtype=float)

    def check_pad_denial(self, obstacles: Optional[List[Any]] = None, pads: Optional[List[np.ndarray]] = None) -> bool:
        """Check whether Base pads are obstructed or denied by urban obstacles or manual flag."""
        if self.pad_denied_override:
            return True
        if not obstacles:
            return False
        from SIM.backend.urban_geometry import point_to_aabb_distance
        check_pads = pads or [get_pad_position(i) for i in range(self.capacity)]
        for pad in check_pads:
            for obs in obstacles:
                if hasattr(obs, "bounds"):
                    d = point_to_aabb_distance(pad, obs.bounds)
                    if d < 2.0:
                        return True
        return False

    def command_rtb_hold(
        self,
        drones: List[Any],
        tether_anchor: np.ndarray,
        radio_range: float = 140.0,
        force_autonomous: bool = False,
        set_target_fn: Optional[Callable[[List[float]], Any]] = None,
        destination: str = "BASE",
        obstacles: Optional[List[Any]] = None,
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

        # Check if Base is denied or FLZ was explicitly requested
        target_dest = (destination or "BASE").upper()
        if target_dest == "BASE":
            if self.check_pad_denial(obstacles or []):
                if self.active_field_lz is not None:
                    target_dest = "FIELD_LZ"

        self.overhead_marshal_active = True
        self.swarm_rtb_active = True
        self.recovery_stage = "OVERHEAD_MARSHAL"

        if target_dest == "FIELD_LZ" and self.active_field_lz is not None:
            flz_center = self.active_field_lz.coords
            target_hold = [float(flz_center[0]), float(flz_center[1]), 25.0]
            hold_state = FlightState.FIELD_HOLD
            status_text = "FIELD_MARSHAL_HOLD"
        else:
            target_hold = [0.0, 0.0, 25.0]
            hold_state = FlightState.OVERHEAD_HOLD
            status_text = "OVERHEAD_HOLD"

        if set_target_fn is not None:
            set_target_fn(target_hold)

        for drone in drones:
            if not drone.crashed and drone.flight_state not in ("LANDED", "GROUND_IDLE", "FIELD_LANDED", "E_LANDED", "DESCENDING", "FIELD_DESCENDING", "EMERGENCY_LANDING"):
                drone.flight_state = hold_state
                drone.status = status_text
                drone.acknowledged_target = np.array(target_hold, dtype=float)
                drone.lkp = np.array(target_hold, dtype=float)
                pkt = RadioPacket(
                    sender_id=-1,
                    msg_type="C2_COMMAND",
                    timestamp=0.0,
                    payload={
                        "cmd": "HOLD_POSITION",
                        "target": [float(v) for v in target_hold],
                    },
                )
                drone.inbox.append(pkt)
                if hasattr(drone, "brain") and drone.brain is not None:
                    drone.brain.handle_command(dict(pkt.payload))
        return {"status": "OK", "recovery_stage": self.recovery_stage, "destination": target_dest}

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
        destination: str = "BASE",
        obstacles: Optional[List[Any]] = None,
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

        target_dest = (destination or "BASE").upper()
        if target_dest == "BASE" and self.check_pad_denial(obstacles or []):
            if self.active_field_lz is not None:
                target_dest = "FIELD_LZ"

        queued_ids = []
        if target_dest == "FIELD_LZ" and self.active_field_lz is not None:
            # Field Landing Zone Allocation (Open Radial Sentry Ring)
            flz_cap = self.active_field_lz.capacity
            avail_slots = [s for s in range(flz_cap) if s not in self.field_lz_slots]
            batch_drones = airborne_drones[:len(avail_slots)]
            marshal_drones = airborne_drones[len(avail_slots):]

            for i, drone in enumerate(batch_drones):
                slot_idx = avail_slots[i]
                self.field_lz_slots[slot_idx] = drone.id
                drone.field_lz_slot = slot_idx
                drone.landing_pad_idx = None
                drone.landing_pad = self.get_field_slot_position(slot_idx)
                drone.landing_wait_timer = i * 1.5  # 1.5s field clearance stagger
                drone.flight_state = FlightState.FIELD_DESCENDING
                drone.status = "FIELD_LANDING"
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
        else:
            # Nominal Base Pad Allocation
            available_pads = [p for p in range(self.capacity) if p not in self.landing_pads]
            batch_drones = airborne_drones[:len(available_pads)]
            marshal_drones = airborne_drones[len(available_pads):]

            for i, drone in enumerate(batch_drones):
                pad_idx = available_pads[i]
                self.landing_pads[pad_idx] = drone.id
                drone.landing_pad_idx = pad_idx
                drone.field_lz_slot = None
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
                pkt = RadioPacket(
                    sender_id=-1,
                    msg_type="C2_COMMAND",
                    timestamp=0.0,
                    payload={
                        "cmd": "TAKEOFF",
                        "flight_state": "TAKEOFF",
                        "status": "LOST_LINK" if drone.lost_link else "ACTIVE",
                    },
                )
                drone.inbox.append(pkt)
                if hasattr(drone, "brain") and drone.brain is not None:
                    drone.brain.handle_command(dict(pkt.payload))
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

        # Swarm assembles in Base Overhead Muster Hold [0, 0, 20] on cold scramble (TKT-68)
        muster_target = np.array([0.0, 0.0, 20.0], dtype=float)
        for d in launch_sequence:
            d.acknowledged_target = np.copy(muster_target)
            if hasattr(d, "target_station"):
                d.target_station = np.copy(muster_target)

        stagger_order = []
        for idx, drone in enumerate(launch_sequence):
            drone.scramble_time = round(idx * self.scramble_stagger, 2)
            stagger_order.append({"drone_id": drone.id, "scramble_time": drone.scramble_time})
            pkt = RadioPacket(
                sender_id=-1,
                msg_type="C2_COMMAND",
                timestamp=0.0,
                payload={
                    "cmd": "SCRAMBLE",
                    "target": [0.0, 0.0, 20.0],
                    "scramble_time": drone.scramble_time,
                },
            )
            drone.inbox.append(pkt)
            if hasattr(drone, "brain") and drone.brain is not None:
                drone.brain.handle_command(dict(pkt.payload))
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

    # ==========================================================================
    # Autonomous Emergency Landing (E-Land) Engine (TKT-64 / PLN-05)
    # ==========================================================================
    def find_safe_flat_space(
        self,
        drone_pos: np.ndarray,
        obstacles: List[Any],
        drones: List[Any],
        search_radius: float = 20.0,
        ground_z: float = 0.05,
    ) -> np.ndarray:
        """Find the nearest clear, flat ground position for an emergency landing.
        Searches outward in concentric rings from current position, ensuring
        >= 3.0m clearance from any UrbanObstacle AABB and other grounded drones."""
        from SIM.backend.urban_geometry import point_to_aabb_distance, get_surface_elevation

        cur_x, cur_y = float(drone_pos[0]), float(drone_pos[1])
        surf_z = get_surface_elevation(drone_pos, obstacles)
        platform_z = surf_z + 0.05 if surf_z > 0.1 else ground_z

        def is_spot_safe(cx: float, cy: float, cz: float) -> bool:
            spot = np.array([cx, cy, cz], dtype=float)
            for obs in obstacles:
                if hasattr(obs, "bounds"):
                    if getattr(obs, "is_landable", False) and cz >= obs.z_ceiling - 0.1:
                        if (obs.x_min + 2.0 <= cx <= obs.x_max - 2.0) and (obs.y_min + 2.0 <= cy <= obs.y_max - 2.0):
                            continue
                    d = point_to_aabb_distance(spot, obs.bounds)
                    if d < 3.0:
                        return False
            for other in drones:
                if getattr(other, "crashed", False) or getattr(other, "retrieved", False):
                    continue
                d_other = float(np.linalg.norm(other.pos[:2] - spot[:2]))
                if d_other < 3.0 and abs(other.pos[2] - spot[2]) <= 2.0:
                    return False
            return True

        # 1. Test directly underneath current position (at platform or ground altitude)
        if is_spot_safe(cur_x, cur_y, platform_z):
            return np.array([cur_x, cur_y, platform_z], dtype=float)

        # 2. Concentric radial scan outward on platform (if elevated)
        if platform_z > ground_z:
            for r in np.arange(2.0, min(12.0, search_radius) + 0.1, 2.0):
                num_samples = max(8, int(r * 2))
                for k in range(num_samples):
                    theta = (2.0 * math.pi * k) / num_samples
                    cand_x = cur_x + r * math.cos(theta)
                    cand_y = cur_y + r * math.sin(theta)
                    if is_spot_safe(cand_x, cand_y, platform_z):
                        return np.array([cand_x, cand_y, platform_z], dtype=float)

        # 3. Concentric radial scan outward on ground level
        for r in np.arange(2.0, search_radius + 0.1, 2.0):
            num_samples = max(8, int(r * 2))
            for k in range(num_samples):
                theta = (2.0 * math.pi * k) / num_samples
                cand_x = cur_x + r * math.cos(theta)
                cand_y = cur_y + r * math.sin(theta)
                if is_spot_safe(cand_x, cand_y, ground_z):
                    return np.array([cand_x, cand_y, ground_z], dtype=float)

        # 4. Fallback: outward nudge
        norm_xy = math.hypot(cur_x, cur_y)
        offset_dir = np.array([cur_x, cur_y], dtype=float) / max(0.1, norm_xy)
        return np.array([cur_x + offset_dir[0] * 5.0, cur_y + offset_dir[1] * 5.0, ground_z], dtype=float)

    def command_e_land(
        self,
        drone_id: int,
        drones: List[Any],
        obstacles: List[Any],
        reason: str = "MANUAL_OPERATOR",
        succession_fn: Optional[Callable[..., Any]] = None,
    ) -> bool:
        """Command an emergency landing for an individual compromised or beyond-BINGO drone."""
        for drone in drones:
            if drone.id == drone_id and not drone.crashed and drone.flight_state not in ("LANDED", "GROUND_IDLE", "FIELD_LANDED", "E_LANDED"):
                spot = self.find_safe_flat_space(drone.pos, obstacles, drones)
                drone.e_landing_spot = spot
                drone.e_landing_reason = reason
                drone.flight_state = FlightState.EMERGENCY_LANDING
                drone.status = "E_LANDING"
                drone.landing_stage = "FINAL_DESCENT"
                drone.touchdown_confirmed = False
                drone.acknowledged_target = np.copy(spot)
                drone.target_station = np.copy(spot)

                pkt = RadioPacket(
                    sender_id=-1,
                    msg_type="C2_COMMAND",
                    timestamp=0.0,
                    payload={
                        "cmd": "E_LAND",
                        "target": [float(v) for v in spot],
                        "spot": [float(v) for v in spot],
                        "reason": reason,
                        "flight_state": "EMERGENCY_LANDING",
                        "status": "E_LANDING",
                    },
                )
                drone.inbox.append(pkt)
                if hasattr(drone, "brain") and drone.brain is not None:
                    drone.brain.handle_command(dict(pkt.payload))

                if drone.is_leader and succession_fn is not None:
                    drone.is_leader = False
                    succession_fn(demote_old_leader=True, retired_leader=drone)
                return True
        return False

    def command_e_land_swarm(
        self,
        drones: List[Any],
        obstacles: List[Any],
        reason: str = "SWARM_EMERGENCY",
        succession_fn: Optional[Callable[..., Any]] = None,
    ) -> List[int]:
        """Emergency-land all living airborne drones to nearest safe flat coordinates."""
        initiated = []
        for drone in drones:
            if not drone.crashed and drone.flight_state not in ("LANDED", "GROUND_IDLE", "FIELD_LANDED", "E_LANDED"):
                if self.command_e_land(drone.id, drones, obstacles, reason, succession_fn):
                    initiated.append(drone.id)
        return initiated
