"""
SwarmTech3_UpRising: Tactical Ground Control Station (GCS) C2 Gateway (c2_gateway.py)
Bubble 1: Operator Airgap.
Owns mission route planning, RNG touch-and-go recon generator, target patrol trajectory math,
dwell countdowns, commanded target beacons, and operator UI event dispatching.
Communicates strictly across the Canyon Airgap via network packets / protocol interfaces.
"""

from __future__ import annotations
import math
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any, Union
import numpy as np

# Radio Comms & Range Constants (Canine Taxonomy - TKT-49)
RANGE_HOWL: float = 140.0
RANGE_BARK: float = 90.0
RANGE_BORK: float = 70.0
RANGE_YIP: float = 50.0
RANGE_P2P: float = 60.0
RF_MODE_BORK: str = "BORK"
RADIO_RANGE: float = RANGE_HOWL
TETHER_ANCHOR = np.array([0.0, 0.0, 1.5], dtype=float)

# Mission Type to Required Capability Tags (TKT-57)
MISSION_CAPABILITY_REQUIREMENTS: Dict[str, List[str]] = {
    "PATROL": ["FORMATION_FLIGHT", "PATROL"],
    "BOX_PATROL": ["FORMATION_FLIGHT", "PATROL"],
    "DIAMOND_SURVEY": ["FORMATION_FLIGHT", "PATROL"],
    "HEXAGON_SWEEP": ["FORMATION_FLIGHT", "PATROL"],
    "PATROL_TO_BINGO": ["FORMATION_FLIGHT", "PATROL"],
    "RECON": ["RECON", "S&R_OPTICAL"],
    "S&R": ["RECON", "S&R_OPTICAL"],
    "SEARCH_AND_RESCUE": ["RECON", "S&R_OPTICAL"],
    "SEARCH_AND_RECALL": ["RECON", "S&R_OPTICAL"],
    "COURIER": ["RECON", "S&R_OPTICAL"],
    "COURIER_ESCORT": ["RECON", "S&R_OPTICAL"],
    "PAYLOAD_DROP": ["HEAVY_PAYLOAD", "DROP_SERVO"],
    "DEPOSIT": ["HEAVY_PAYLOAD", "DROP_SERVO"],
    "LIDAR_SCAN": ["LIDAR_SCAN"],
    "POINTCLOUD_MAPPING": ["LIDAR_SCAN"],
    "SIGINT_TRIANGULATION": ["JAMMER_TRIANGULATION"],
    "EW_HUNT": ["JAMMER_TRIANGULATION"],
}

# Discrete Waypoint Mission Route Presets (TKT-28 & TKT-30)
# Schema: [x, y, z, loiter_s]
ROUTE_PRESETS: Dict[str, List[List[float]]] = {
    "BOX_PATROL": [
        [40.0, -20.0, 25.0, 4.0],
        [40.0, 40.0, 28.0, 4.0],
        [-20.0, 40.0, 25.0, 4.0],
        [-20.0, -20.0, 22.0, 4.0],
    ],
    "DIAMOND_SURVEY": [
        [50.0, 10.0, 28.0, 3.0],
        [15.0, 45.0, 25.0, 3.0],
        [-20.0, 10.0, 24.0, 3.0],
        [15.0, -25.0, 26.0, 3.0],
    ],
    "HEXAGON_SWEEP": [
        [45.0, 25.0, 26.0, 2.0],
        [20.0, 45.0, 28.0, 2.0],
        [-15.0, 30.0, 25.0, 2.0],
        [-25.0, -5.0, 24.0, 2.0],
        [-5.0, -30.0, 26.0, 2.0],
        [30.0, -20.0, 27.0, 2.0],
    ],
}


class C2Gateway:
    """Tactical C2 Mission Gateway running at the GCS operator layer.
    Manages operator flight intents, discrete route queues, continuous figure-8 patrol,
    and semi-autonomous RNG touch-and-go exploratory reconnaissance."""

    def __init__(self, sim: Optional[Any] = None):
        self.sim: Optional[Any] = sim

        # Commanded Target (GCS intent)
        self._target_pos: np.ndarray = np.array([60.0, 40.0, 30.0], dtype=float)

        # Dynamic Target Auto-Patrol Loop (TKT-12)
        self.patrol_active: bool = False
        self.patrol_speed: float = 0.08  # Radians per second (~78s per loop)
        self.patrol_time: float = 0.0
        self.patrol_center: np.ndarray = np.array([20.0, 20.0, 28.0], dtype=float)
        self.patrol_scale: np.ndarray = np.array([50.0, 35.0, 10.0], dtype=float)
        self.last_gossip_inject_time: float = 0.0

        # Discrete Waypoint Mission Routes (TKT-28 & TKT-56)
        self.route_active: bool = False
        self.route_paused: bool = False
        self.route_name: str = "NONE"
        self.route_waypoints: List[np.ndarray] = []
        self.route_loop: bool = True
        self.route_acceptance_radius: float = 4.0

        # Tactical RNG Touch-and-Go Reconnaissance (TKT-29 & TKT-31)
        self.rng_recon_active: bool = False
        self.rng_dwelling: bool = False
        self.rng_dwell_time: float = 2.0         # 2.0s tactical dwell / sensor scan countdown
        self.rng_dwell_timer: float = 0.0
        self.rng_station_count: int = 0
        self.rng_geofence_min: np.ndarray = np.array([-50.0, -50.0, 10.0], dtype=float)
        self.rng_geofence_max: np.ndarray = np.array([70.0, 70.0, 35.0], dtype=float)
        self.rng_min_variance: float = 15.0      # Minimum transit jump distance (m)
        self.rng_tether_clearance: float = 3.0   # Horizontal clearance from anchor (m)
        self.rng_status: str = "IDLE"            # "IDLE", "TRACKING", "DWELLING", "UPLINK_OFFLINE"
        self.rng_paused: bool = False            # True when uplink fails or link is lost

        # 1-Click Aux Mission: Patrol to BINGO RTB (TKT-58 / PLN-02)
        self.bingo_mission_active: bool = False
        self.bingo_threshold: float = 25.0       # Critical battery percentage triggering automated RTB
        self.bingo_triggered: bool = False       # True once battery drops below threshold and RTB commences
        self.bingo_auto_land: bool = True        # Auto-initiate Stage 2 coordinated landing upon marshal arrival
        self.bingo_stage: str = "IDLE"           # "IDLE", "PATROLLING", "RTB_TRANSIT", "MARSHAL_HOLD", "LANDING", "COMPLETED"
        self.bingo_pattern: str = "BOX_PATROL"

        # Physical Courier / Search-and-Recall Rescue Mule (TKT-51 / PLN-02)
        self.courier_active: bool = False
        self.courier_drone_id: Optional[int] = None
        self.courier_target_drone_id: Optional[int] = None
        self.courier_target_lkp: Optional[np.ndarray] = None
        self.courier_stage: str = "IDLE"          # "IDLE", "TRANSIT_OUTBOUND", "RENDEZVOUS_DWELL", "ESCORT_INBOUND", "MIA_RETURN", "HANDOVER_COMPLETE"
        self.courier_dwell_timer: float = 0.0
        self.courier_dwell_timeout: float = 15.0

    @property
    def target_pos(self) -> np.ndarray:
        return self._target_pos

    @target_pos.setter
    def target_pos(self, value: Union[List[float], np.ndarray]) -> None:
        self._target_pos = np.array(value, dtype=float)
        if self.sim:
            if not getattr(self.sim, "tether_severed", False) and getattr(self.sim, "comms_mode", "") == "TETHERED_RELAY":
                for d in getattr(self.sim, "drones", []):
                    if d.is_leader or d.id == 0:
                        d.acknowledged_target = np.copy(self._target_pos)
            elif getattr(self.sim, "comms_mode", "") == "LEADER_FOLLOWER":
                current_leader = getattr(self.sim, "leader", None)
                if current_leader is not None and not current_leader.crashed:
                    anchor = getattr(self.sim, "tether_anchor", TETHER_ANCHOR)
                    d_gcs = float(np.linalg.norm(current_leader.pos - anchor))
                    if d_gcs <= RADIO_RANGE:
                        current_leader.acknowledged_target = np.copy(self._target_pos)
            elif getattr(self.sim, "comms_mode", "") in ("GOSSIP_CONSENSUS", "P2P_MESH", "P2P_GOSSIP_MESH", "MESH"):
                if hasattr(self.sim, "brain") and hasattr(self.sim.brain, "inject_target"):
                    self.sim.brain.inject_target(self._target_pos)

    def attach_sim(self, sim: Any) -> None:
        """Attach underlying simulation or network bridge interface."""
        self.sim = sim

    def step(self, dt: float) -> None:
        """Advance C2 layer state machine: auto-patrol generator, RNG recon dwell & generation."""
        if self.sim is None:
            return

        sim_time = getattr(self.sim, "sim_time", 0.0)

        # 1. Continuous Auto-Patrol Loop (TKT-12)
        if self.patrol_active:
            self.patrol_time += dt * self.patrol_speed
            t = self.patrol_time
            patrol_x = float(self.patrol_center[0] + self.patrol_scale[0] * math.cos(t))
            patrol_y = float(self.patrol_center[1] + self.patrol_scale[1] * math.sin(2.0 * t) * 0.75)
            patrol_z = float(max(15.0, self.patrol_center[2] + self.patrol_scale[2] * math.sin(t * 1.5)))

            # Update Commanded Target at GCS
            self._target_pos = np.array([patrol_x, patrol_y, patrol_z], dtype=float)

            should_update_uplink = True
            active_brain = getattr(self.sim, "active_brain_name", "")
            if active_brain == "GOSSIP_CONSENSUS":
                time_since_last = sim_time - self.last_gossip_inject_time
                brain = getattr(self.sim, "brain", None)
                if brain and hasattr(brain, "get_telemetry_extra"):
                    gossip_extra = brain.get_telemetry_extra()
                    consensus_reached = gossip_extra.get("consensus_reached", True)
                    if not consensus_reached and time_since_last < 2.5:
                        should_update_uplink = False
                    elif time_since_last < 1.2:
                        should_update_uplink = False

            if should_update_uplink:
                res = self.dispatch_target_uplink(self._target_pos)
                if active_brain == "GOSSIP_CONSENSUS" and res.get("status") == "OK":
                    self.last_gossip_inject_time = sim_time

        # 2. Tactical RNG Touch-and-Go Reconnaissance Generator (TKT-29 & TKT-31)
        elif self.rng_recon_active:
            link_ok = self.is_link_active()

            if not link_ok:
                # Comms channel broken (tether cut, out of RF range, jammed)
                self.rng_status = "UPLINK_OFFLINE"
                self.rng_paused = True
                # Freeze commanded target, halt dwell countdown, do NOT re-roll coordinates
                return

            # Link healthy: if previously paused in UPLINK_OFFLINE, attempt graceful recovery
            if self.rng_paused:
                res = self.dispatch_target_uplink(self._target_pos)
                if res.get("status") == "OK":
                    self.rng_paused = False
                    self.rng_status = "TRACKING"
                else:
                    return

            target_reached = self.is_target_reached()
            if target_reached:
                if not self.rng_dwelling:
                    # Arrival detected: initiate tactical dwell / sensor scan countdown
                    self.rng_dwelling = True
                    self.rng_dwell_timer = self.rng_dwell_time
                    self.rng_station_count += 1
                    self.rng_status = "DWELLING"
                else:
                    self.rng_dwell_timer = max(0.0, self.rng_dwell_timer - dt)
                    if self.rng_dwell_timer <= 0.0:
                        # Dwell complete: compute new randomized, high-variance SAFE target
                        new_target = self.generate_safe_rng_target()
                        res = self.dispatch_target_uplink(new_target)
                        if res.get("status") == "OK":
                            self._target_pos = np.copy(new_target)
                            self.rng_dwelling = False
                            self.rng_status = "TRACKING"
                            self.rng_paused = False
                            active_brain = getattr(self.sim, "active_brain_name", "")
                            if active_brain == "GOSSIP_CONSENSUS":
                                self.last_gossip_inject_time = sim_time
                        else:
                            # Uplink failed: Freeze Commanded Target, enter UPLINK_OFFLINE hold
                            self.rng_status = "UPLINK_OFFLINE"
                            self.rng_paused = True
            else:
                self.rng_dwelling = False
                self.rng_status = "TRACKING"

        # 3. Patrol to BINGO Watchdog & 2-Stage RTB Coordinator (TKT-58)
        if self.bingo_mission_active:
            if not self.bingo_triggered:
                # Comms gate: GCS evaluates battery strictly from downlinked telemetry (Operator Airgap)
                if self.is_link_active():
                    drones = getattr(self.sim, "drones", [])
                    living_airborne = [
                        d for d in drones
                        if not d.crashed and d.flight_state not in ("LANDED", "GROUND_IDLE") and getattr(d, "downlink_active", True)
                    ]
                    if living_airborne:
                        min_battery = min(float(d.battery) for d in living_airborne)
                        if min_battery <= self.bingo_threshold:
                            self.bingo_triggered = True
                            self.bingo_stage = "RTB_TRANSIT"
                            # Stop active route / patrol pattern
                            self.route_active = False
                            self.route_paused = False
                            if self.sim:
                                for d in getattr(self.sim, "drones", []):
                                    d.mission_active = False
                                    d.mission_paused = False
                                    d.wp_loitering = False
                            self.patrol_active = False
                            # Initiate Stage 1: Overhead Marshal RTB Hold
                            if hasattr(self.sim, "command_rtb_hold"):
                                self.sim.command_rtb_hold()
                            self.log_event(
                                "WARN",
                                f"[BINGO TRIGGERED: Fleet battery ({min_battery:.1f}%) reached threshold ({self.bingo_threshold:.1f}%). Commencing 2-Stage RTB.]"
                            )
            else:
                # BINGO RTB already triggered: monitor progress through Stage 1 -> Stage 2
                if self.bingo_stage == "RTB_TRANSIT":
                    leader = getattr(self.sim, "leader", None)
                    drones = getattr(self.sim, "drones", [])
                    living_airborne = [
                        d for d in drones
                        if not d.crashed and d.flight_state not in ("LANDED", "GROUND_IDLE", "RETRIEVED")
                    ]
                    anchor = getattr(self.sim, "tether_anchor", TETHER_ANCHOR)
                    leader_dist = float(np.linalg.norm(leader.pos[:2] - anchor[:2])) if leader else 0.0
                    all_near_base = all(float(np.linalg.norm(d.pos[:2] - anchor[:2])) <= 25.0 for d in living_airborne) if living_airborne else True
                    is_marshaling = getattr(self.sim, "overhead_marshal_active", False) or getattr(self.sim, "recovery_stage", "") == "OVERHEAD_MARSHAL"

                    if (leader and leader_dist <= 18.0 and all_near_base) or (is_marshaling and leader and leader_dist <= 18.0):
                        self.bingo_stage = "MARSHAL_HOLD"
                        if self.bingo_auto_land:
                            self.bingo_stage = "LANDING"
                            if hasattr(self.sim, "command_land_swarm"):
                                self.sim.command_land_swarm()
                            self.log_event(
                                "CMD",
                                "[BINGO STAGE 2: Fleet established in overhead marshal. Auto-initiating ATC pad landing.]"
                            )
                elif self.bingo_stage == "LANDING":
                    drones = getattr(self.sim, "drones", [])
                    living_airborne = [
                        d for d in drones
                        if not d.crashed and d.flight_state not in ("LANDED", "GROUND_IDLE", "RETRIEVED")
                    ]
                    if not living_airborne:
                        self.bingo_stage = "COMPLETED"
                        self.bingo_mission_active = False

        # 4. Physical Courier / Search-and-Recall Rescue Mule (TKT-51 / PLN-02)
        if self.courier_active and self.sim is not None:
            courier = self.sim.get_drone_by_id(self.courier_drone_id) if hasattr(self.sim, "get_drone_by_id") else None
            target_drone = self.sim.get_drone_by_id(self.courier_target_drone_id) if hasattr(self.sim, "get_drone_by_id") else None
            anchor = getattr(self.sim, "tether_anchor", TETHER_ANCHOR)

            if courier is None or courier.crashed:
                self.courier_stage = "COURIER_LOST"
                self.courier_active = False
                self.log_event("WARN", f"[COURIER FAILURE: Courier #{self.courier_drone_id} crashed or missing in action.]")
            else:
                if self.courier_stage == "TRANSIT_OUTBOUND":
                    if self.courier_target_lkp is not None:
                        dist_to_lkp = float(np.linalg.norm(courier.pos - self.courier_target_lkp))
                        if dist_to_lkp <= 5.0:
                            self.courier_stage = "RENDEZVOUS_DWELL"
                            self.courier_dwell_timer = 0.0
                            courier.flight_state = "RENDEZVOUS_DWELL"
                            courier.status = "RENDEZVOUS_DWELL"
                            courier.acknowledged_target = np.copy(self.courier_target_lkp)
                            courier.target_station = np.copy(self.courier_target_lkp)
                            self.log_event("CMD", f"[COURIER ARRIVAL: Courier #{courier.id} arrived at LKP ({dist_to_lkp:.1f}m). Stationary dwell & P2P rescue beacon engaged.]")

                elif self.courier_stage == "RENDEZVOUS_DWELL":
                    # Check P2P contact with target drone (within RANGE_P2P = 60m)
                    contact_made = False
                    if target_drone is not None and not target_drone.crashed:
                        d_p2p = float(np.linalg.norm(courier.pos - target_drone.pos))
                        if d_p2p <= RANGE_P2P:
                            contact_made = True
                            self.courier_stage = "ESCORT_INBOUND"
                            courier.flight_state = "ESCORT_LEAD"
                            courier.status = "ESCORT_LEAD"
                            courier.courier_escortee_id = target_drone.id
                            courier.acknowledged_target = np.array([0.0, 0.0, 25.0], dtype=float)
                            courier.target_station = np.array([0.0, 0.0, 25.0], dtype=float)

                            target_drone.flight_state = "ESCORT_REJOIN"
                            target_drone.status = "ESCORT_REJOIN"
                            target_drone.escort_leader_id = courier.id
                            target_drone.backtrack_hold = False
                            target_drone.holding_at_horizon = False

                            self.log_event("CMD", f"[RESCUE HANDSHAKE: Courier #{courier.id} established P2P contact with Drone #{target_drone.id} ({d_p2p:.1f}m). Inbound tandem escort initiated.]")

                    if not contact_made:
                        self.courier_dwell_timer += dt
                        if self.courier_dwell_timer >= self.courier_dwell_timeout:
                            # MIA Timeout
                            self.courier_stage = "MIA_RETURN"
                            courier.flight_state = "CRUISE"
                            courier.status = "COURIER_MIA_RETURN"
                            courier.acknowledged_target = np.array([0.0, 0.0, 25.0], dtype=float)
                            courier.target_station = np.array([0.0, 0.0, 25.0], dtype=float)
                            self.log_event("WARN", f"[COURIER TIMEOUT: Target Drone #{self.courier_target_drone_id} MIA at LKP after 15s. Courier #{courier.id} returning to Base alone.]")

                elif self.courier_stage == "ESCORT_INBOUND":
                    # Check if target drone has entered GCS radio horizon
                    target_dist_gcs = float(np.linalg.norm(target_drone.pos - anchor)) if target_drone else 999.0
                    r_howl = getattr(self.sim, "radio_range", RANGE_HOWL)
                    target_linked = (target_dist_gcs <= r_howl and getattr(target_drone, "downlink_active", True))

                    if target_linked:
                        self.courier_stage = "HANDOVER_COMPLETE"
                        self.courier_active = False
                        courier.flight_state = "CRUISE"
                        courier.status = "ACTIVE"
                        courier.courier_mission_active = False
                        courier.courier_escortee_id = None

                        if target_drone is not None:
                            target_drone.flight_state = "CRUISE"
                            target_drone.status = "ACTIVE"
                            target_drone.lost_link = False
                            target_drone.escort_leader_id = None
                            target_drone.gcs_heartbeat_watchdog = 2.0
                            target_drone.last_heartbeat_time = getattr(self.sim, "sim_time", 0.0)
                            target_drone.acknowledged_target = np.copy(courier.acknowledged_target)
                            target_drone.target_station = np.copy(courier.acknowledged_target)

                        self.log_event("STATUS", f"[RESCUE SUCCESS: Drone #{target_drone.id} penetrated GCS radio horizon ({target_dist_gcs:.1f}m). Downlink restored, fleet re-enrolled.]")

                elif self.courier_stage == "MIA_RETURN":
                    dist_base = float(np.linalg.norm(courier.pos[:2] - anchor[:2]))
                    if dist_base <= 15.0:
                        self.courier_stage = "MIA_COMPLETE"
                        self.courier_active = False
                        courier.flight_state = "CRUISE"
                        courier.courier_mission_active = False

    def is_link_active(self) -> bool:
        """Check whether the bidirectional communications link between GCS and fleet is healthy.
        Enforces Operator Airgap: if physical umbilical is cut or RF horizon is exceeded,
        the GCS loses telemetry reception and cannot receive arrival pings."""
        if self.sim is None:
            return False

        comms_mode = getattr(self.sim, "comms_mode", "TETHERED_RELAY")
        tether_severed = getattr(self.sim, "tether_severed", False)
        anchor = getattr(self.sim, "tether_anchor", TETHER_ANCHOR)
        drones = getattr(self.sim, "drones", [])
        active_brain = getattr(self.sim, "active_brain_name", "")

        # 1. Hardline Tether: Cable severed means zero comms between GCS and Drone 0
        if comms_mode == "TETHERED_RELAY":
            if tether_severed:
                return False
            leader = getattr(self.sim, "leader", None)
            return leader is not None and not leader.crashed

        # 2. Wireless Star: Dependent on RF line-of-sight and range
        elif comms_mode == "LEADER_FOLLOWER":
            leader = getattr(self.sim, "leader", None)
            if leader is None or leader.crashed or getattr(leader, "lost_link", False):
                return False
            d_gcs = float(np.linalg.norm(leader.pos - anchor))
            r_down = RANGE_BORK if getattr(leader, "rf_power_mode", None) == RF_MODE_BORK or getattr(leader, "tier", None) == 1.5 else RANGE_BARK
            if d_gcs > r_down:
                return False
            if hasattr(leader, "downlink_active") and not leader.downlink_active:
                return False
            if hasattr(self.sim, "is_packet_dropped") and self.sim.is_packet_dropped(d_gcs, is_hardline=False, max_range=r_down):
                return False
            return True

        # 3. P2P Gossip Mesh: Needs at least one non-crashed gateway drone with active downlink
        elif active_brain == "GOSSIP_CONSENSUS" or comms_mode in ("GOSSIP_CONSENSUS", "P2P_GOSSIP_MESH", "P2P_MESH", "MESH"):
            living = [d for d in drones if not d.crashed]
            if not living:
                return False
            min_dist = min(float(np.linalg.norm(d.pos - anchor)) for d in living)
            if min_dist > RANGE_HOWL:
                return False
            if living and hasattr(living[0], "downlink_active"):
                return any(d.downlink_active for d in living)
            if hasattr(self.sim, "is_packet_dropped") and self.sim.is_packet_dropped(min_dist, is_hardline=False):
                return False
            return True

        return False

    def is_target_reached(self, threshold: float = 2.5) -> bool:
        """Deterministic arrival check: whether active swarm guidance has arrived within threshold.
        Enforces Operator Airgap:
        1. If comms link is lost/severed, GCS receives NO arrival packets (returns False).
        2. Reads arrival strictly from downlinked telemetry handshake."""
        if self.sim is None:
            return False

        # Telemetry link gate: severed tether or lost RF link suppresses arrival downlink
        if not self.is_link_active():
            return False

        active_brain = getattr(self.sim, "active_brain_name", "")
        drones = getattr(self.sim, "drones", [])

        if active_brain == "GOSSIP_CONSENSUS":
            active = [
                d for d in drones
                if not d.crashed and getattr(d, "flight_state", "") not in ("LANDED", "DESCENDING")
            ]
            if not active:
                return False
            centroid = np.mean([d.pos for d in active], axis=0)
            target = self.get_acknowledged_target()
            return float(np.linalg.norm(centroid - target)) <= threshold
        else:
            leader = getattr(self.sim, "leader", None)
            if leader is None or leader.crashed:
                return False
            # Telemetry handshake from leader downlinked state
            if hasattr(leader, "target_reached"):
                return bool(leader.target_reached)
            target = leader.acknowledged_target
            return float(np.linalg.norm(leader.pos - target)) <= threshold

    def get_acknowledged_target(self) -> np.ndarray:
        """Get the target currently acknowledged by the swarm."""
        if self.sim is None:
            return self._target_pos
        if hasattr(self.sim, "get_acknowledged_target"):
            return self.sim.get_acknowledged_target()
        leader = getattr(self.sim, "leader", None)
        if leader:
            return leader.acknowledged_target
        return self._target_pos

    def generate_safe_rng_target(self) -> np.ndarray:
        """Generate a high-variance, guaranteed-SAFE 3D target coordinate."""
        current_target = np.copy(self.get_acknowledged_target())
        anchor = getattr(self.sim, "tether_anchor", TETHER_ANCHOR)

        for _ in range(200):
            rx = float(np.random.uniform(self.rng_geofence_min[0], self.rng_geofence_max[0]))
            ry = float(np.random.uniform(self.rng_geofence_min[1], self.rng_geofence_max[1]))
            rz = float(np.random.uniform(self.rng_geofence_min[2], self.rng_geofence_max[2]))
            cand = np.array([rx, ry, rz], dtype=float)

            # High variance: ensure jump distance >= 15m
            if float(np.linalg.norm(cand - current_target)) < self.rng_min_variance:
                continue

            # Safe tether clearance: outside 3m cylinder around anchor
            anchor_dist_xy = float(np.hypot(cand[0] - anchor[0], cand[1] - anchor[1]))
            if anchor_dist_xy < self.rng_tether_clearance:
                continue

            return cand

        # Deterministic safe offset fallback
        dx = 20.0 if current_target[0] <= 10.0 else -20.0
        dy = 20.0 if current_target[1] <= 10.0 else -20.0
        dz = 25.0
        return np.array([current_target[0] + dx, current_target[1] + dy, dz], dtype=float)

    def dispatch_target_uplink(self, target_pos: Union[List[float], np.ndarray]) -> Dict[str, Any]:
        """Dispatch commanded target across simulated link to fleet."""
        if self.sim is None:
            return {"status": "ERROR", "reason": "NO_SIM_ATTACHED"}
        if hasattr(self.sim, "dispatch_target_uplink"):
            return self.sim.dispatch_target_uplink(target_pos)
        return {"status": "OK", "target": list(target_pos)}

    def set_target(self, pos: Union[List[float], np.ndarray]) -> Dict[str, Any]:
        """Command a new 3D mission target waypoint from GCS and attempt uplink dispatch."""
        target_arr = np.array(pos, dtype=float)
        self._target_pos = target_arr

        # Manual target dispatch disengages active mission route, auto-patrol, and RNG recon
        if self.route_active:
            self.route_active = False
            if self.sim:
                for d in getattr(self.sim, "drones", []):
                    d.mission_active = False
        if self.rng_recon_active:
            self.rng_recon_active = False
            self.rng_dwelling = False
            self.rng_status = "IDLE"
            self.rng_paused = False
        if self.bingo_mission_active and not np.allclose(self._target_pos, [0.0, 0.0, 25.0], atol=1.0):
            self.bingo_mission_active = False
            self.bingo_triggered = False
            self.bingo_stage = "IDLE"

        uplink_res = self.dispatch_target_uplink(self._target_pos)

        # Disengage RTB marshal if commanded away from base
        if not np.allclose(self._target_pos, [0.0, 0.0, 25.0], atol=1.0):
            if self.sim:
                overhead = getattr(self.sim, "overhead_marshal_active", False)
                rtb = getattr(self.sim, "swarm_rtb_active", False)
                if (overhead or rtb) and uplink_res.get("status") == "OK":
                    if hasattr(self.sim, "cancel_rtb"):
                        self.sim.cancel_rtb()

        return uplink_res

    def toggle_target_patrol(self, active: Optional[bool] = None) -> bool:
        """Toggle or explicitly set continuous 3D target auto-patrol loop."""
        if active is not None:
            self.patrol_active = bool(active)
        else:
            self.patrol_active = not self.patrol_active
        if self.patrol_active:
            self.last_gossip_inject_time = 0.0
            if self.route_active:
                self.route_active = False
                if self.sim:
                    for d in getattr(self.sim, "drones", []):
                        d.mission_active = False
            if self.rng_recon_active:
                self.rng_recon_active = False
                self.rng_dwelling = False
                self.rng_status = "IDLE"
                self.rng_paused = False
            if self.bingo_mission_active:
                self.bingo_mission_active = False
                self.bingo_triggered = False
                self.bingo_stage = "IDLE"
        return self.patrol_active

    def start_route_preset(self, preset_name: str) -> Dict[str, Any]:
        """Deploy and start a predefined route preset (TKT-28)."""
        name = preset_name.upper().replace("-", "_").replace(" ", "_")
        if name not in ROUTE_PRESETS:
            return {"status": "ERROR", "reason": f"Unknown preset: {preset_name}"}
        wps = ROUTE_PRESETS[name]
        return self.deploy_mission_route(wps, loop=True, name=name)

    def check_mission_authorization(
        self,
        mission_type: str,
        required_capabilities: Optional[List[str]] = None,
        target_drone_id: Optional[int] = None,
    ) -> Dict[str, Any]:
        """Verify whether enrolled fleet airframes meet capability requirements (TKT-57).
        Enforces Operator Airgap: evaluates downlinked telemetry capabilities."""
        m_key = mission_type.upper().replace("-", "_").replace(" ", "_")
        reqs = list(required_capabilities) if required_capabilities else list(MISSION_CAPABILITY_REQUIREMENTS.get(m_key, ["FORMATION_FLIGHT"]))

        drones = getattr(self.sim, "drones", []) if self.sim else []
        enrolled_living = [
            d for d in drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING", "RETRIEVED")
        ]

        if not enrolled_living:
            return {
                "status": "REJECTED",
                "authorized": False,
                "reason": "NO_AIRBORNE_AIRFRAMES",
                "mission_type": mission_type,
                "required_capabilities": reqs,
                "missing_capabilities": reqs,
            }

        # If a specific target drone is commanded
        if target_drone_id is not None:
            candidate = next((d for d in enrolled_living if d.id == target_drone_id), None)
            if candidate is None:
                return {
                    "status": "REJECTED",
                    "authorized": False,
                    "reason": "TARGET_DRONE_UNAVAILABLE",
                    "target_drone_id": target_drone_id,
                }
            drone_caps = set(getattr(candidate, "capabilities", []))
            missing = [r for r in reqs if r not in drone_caps]
            if missing:
                return {
                    "status": "REJECTED",
                    "authorized": False,
                    "reason": "TIER_AUTHORIZATION_FAILED",
                    "mission_type": mission_type,
                    "target_drone_id": target_drone_id,
                    "tier": getattr(candidate, "full_tier_designation", "UNKNOWN"),
                    "required_capabilities": reqs,
                    "missing_capabilities": missing,
                }
            return {
                "status": "AUTHORIZED",
                "authorized": True,
                "mission_type": mission_type,
                "target_drone_id": target_drone_id,
                "allocated_drone_id": target_drone_id,
                "allocated_drone_tier": getattr(candidate, "full_tier_designation", "UNKNOWN"),
                "tier": getattr(candidate, "full_tier_designation", "UNKNOWN"),
                "allocated_drone_ids": [target_drone_id],
            }

        # Fleet-wide search for eligible airframes
        eligible_drones = []
        for d in enrolled_living:
            drone_caps = set(getattr(d, "capabilities", []))
            if all(r in drone_caps for r in reqs):
                eligible_drones.append(d.id)

        if not eligible_drones:
            available_caps = set()
            for d in enrolled_living:
                available_caps.update(getattr(d, "capabilities", []))
            missing = [r for r in reqs if r not in available_caps]
            return {
                "status": "REJECTED",
                "authorized": False,
                "reason": "TIER_AUTHORIZATION_FAILED",
                "mission_type": mission_type,
                "required_capabilities": reqs,
                "missing_capabilities": missing,
                "enrolled_tiers": [getattr(d, "full_tier_designation", "UNKNOWN") for d in enrolled_living],
            }

        first_eligible = next((d for d in enrolled_living if d.id == eligible_drones[0]), None)
        return {
            "status": "AUTHORIZED",
            "authorized": True,
            "mission_type": mission_type,
            "required_capabilities": reqs,
            "allocated_drone_id": eligible_drones[0],
            "allocated_drone_tier": getattr(first_eligible, "full_tier_designation", "UNKNOWN") if first_eligible else "UNKNOWN",
            "allocated_drone_ids": eligible_drones,
        }

    def deploy_mission_route(
        self,
        waypoints: Union[List[List[float]], List[np.ndarray], List[Dict[str, float]]],
        loop: bool = True,
        name: str = "CUSTOM",
        mission_type: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Deploy discrete waypoint mission route to the swarm over physical comms uplink (TKT-28 & TKT-30).
        Supports 4D waypoints: [x, y, z, loiter_s] or {'x': x, 'y': y, 'z': z, 'loiter': loiter_s}."""
        if not waypoints:
            return {"status": "ERROR", "reason": "Empty waypoints list"}

        # Pre-Flight Capability Authorization Check (TKT-57)
        m_type = (mission_type or name).upper().replace("-", "_").replace(" ", "_")
        if m_type in MISSION_CAPABILITY_REQUIREMENTS or mission_type is not None:
            auth_check = self.check_mission_authorization(m_type)
            if not auth_check.get("authorized", True) or auth_check.get("status") == "REJECTED":
                return {
                    "status": "ERROR",
                    "code": "TIER_AUTHORIZATION_FAILED",
                    "reason": auth_check.get("reason", "TIER_AUTHORIZATION_FAILED"),
                    "missing_capabilities": auth_check.get("missing_capabilities", []),
                    "mission_type": m_type,
                }

        clean_wps: List[np.ndarray] = []
        for wp in waypoints:
            if isinstance(wp, dict):
                clean_wps.append(np.array([
                    float(wp.get("x", 0.0)),
                    float(wp.get("y", 0.0)),
                    float(wp.get("z", 20.0)),
                    float(wp.get("loiter", 0.0)),
                ], dtype=float))
            else:
                arr = list(wp)
                if len(arr) == 3:
                    clean_wps.append(np.array([float(arr[0]), float(arr[1]), float(arr[2]), 0.0], dtype=float))
                else:
                    clean_wps.append(np.array([float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3])], dtype=float))

        if self.sim is None:
            self.route_active = True
            self.route_name = name
            self.route_waypoints = clean_wps
            self.route_loop = loop
            self._target_pos = np.copy(clean_wps[0][:3])
            return {"status": "OK", "name": name, "count": len(clean_wps), "loop": loop}

        drones = getattr(self.sim, "drones", [])
        active_brain = getattr(self.sim, "active_brain_name", "")
        tether_anchor = getattr(self.sim, "tether_anchor", TETHER_ANCHOR)
        comms_mode = getattr(self.sim, "comms_mode", "TETHERED_RELAY")
        tether_severed = getattr(self.sim, "tether_severed", False)

        # Determine gateway drone for uplink
        if active_brain == "GOSSIP_CONSENSUS":
            living_drones = [d for d in drones if not d.crashed]
            if not living_drones:
                return {"status": "DROPPED", "reason": "NO_LIVING_DRONES"}
            target_drone = min(
                living_drones,
                key=lambda d: float(np.linalg.norm(d.pos - tether_anchor)),
            )
        else:
            target_drone = getattr(self.sim, "leader", None)
            if target_drone is None or target_drone.crashed:
                return {"status": "DROPPED", "reason": "NO_ACTIVE_LEADER"}

        # Calculate link distance and drop probability
        dist_gcs = float(np.linalg.norm(target_drone.pos - tether_anchor))
        is_hardline = (comms_mode == "TETHERED_RELAY" and not tether_severed and target_drone.id == 0)

        if not is_hardline:
            if comms_mode == "TETHERED_RELAY" and tether_severed:
                return {"status": "DROPPED", "reason": "TETHER_SEVERED", "error": "Tactical umbilical is severed."}
            if dist_gcs > RADIO_RANGE:
                return {"status": "OUT_OF_RANGE", "reason": "RADIO_RANGE_EXCEEDED", "error": f"Gateway node ({dist_gcs:.1f}m) out of GCS radio range ({RADIO_RANGE}m)"}
            if hasattr(self.sim, "is_packet_dropped") and self.sim.is_packet_dropped(dist_gcs, is_hardline=False):
                return {"status": "DROPPED", "reason": "PACKET_LOSS"}

        # Link healthy: deploy to GCS C2 state
        self.route_active = True
        self.route_paused = False
        self.route_name = name
        self.route_waypoints = clean_wps
        self.route_loop = loop
        self.patrol_active = False
        self.rng_recon_active = False
        self.rng_dwelling = False
        self.rng_status = "IDLE"
        self.rng_paused = False

        # Upload to drone OBC
        target_drone.mission_waypoints = [np.copy(wp) for wp in clean_wps]
        target_drone.current_wp_idx = 0
        target_drone.mission_active = True
        target_drone.mission_name = name
        target_drone.mission_paused = False
        target_drone.mission_loop = loop
        target_drone.acknowledged_target = np.copy(clean_wps[0][:3])
        target_drone.wp_loiter_timer = 0.0
        target_drone.wp_loitering = False
        target_drone.wp_total_loiter = float(clean_wps[0][3]) if len(clean_wps[0]) > 3 else 0.0

        # If Gossip brain, disseminate route to peers
        brain = getattr(self.sim, "brain", None)
        if active_brain == "GOSSIP_CONSENSUS" and brain and hasattr(brain, "on_route_deployed"):
            brain.on_route_deployed(clean_wps, loop=loop)

        # Update Commanded Target at GCS
        self._target_pos = np.copy(clean_wps[0][:3])

        return {
            "status": "OK",
            "name": self.route_name,
            "waypoints": [list(wp) for wp in self.route_waypoints],
            "count": len(self.route_waypoints),
            "loop": self.route_loop,
        }

    def pause_mission_route(self) -> Dict[str, Any]:
        """Pause active mission route; drones hold station at current waypoint (TKT-56)."""
        if not self.route_active:
            return {"status": "ERROR", "reason": "No active route to pause"}
        self.route_paused = True
        if self.sim:
            for d in getattr(self.sim, "drones", []):
                d.mission_paused = True
            brain = getattr(self.sim, "brain", None)
            if brain and hasattr(brain, "route_paused"):
                brain.route_paused = True
        return {"status": "OK", "action": "ROUTE_PAUSED"}

    def resume_mission_route(self) -> Dict[str, Any]:
        """Resume active mission route from current waypoint (TKT-56)."""
        if not self.route_active:
            return {"status": "ERROR", "reason": "No active route to resume"}
        self.route_paused = False
        if self.sim:
            for d in getattr(self.sim, "drones", []):
                d.mission_paused = False
            brain = getattr(self.sim, "brain", None)
            if brain and hasattr(brain, "route_paused"):
                brain.route_paused = False
        return {"status": "OK", "action": "ROUTE_RESUMED"}

    def skip_mission_waypoint(self) -> Dict[str, Any]:
        """Skip currently targeted waypoint and advance immediately to next (TKT-56)."""
        if not self.route_active or not self.route_waypoints:
            return {"status": "ERROR", "reason": "No active route to skip waypoint"}

        brain = getattr(self.sim, "brain", None)
        active_brain = getattr(self.sim, "active_brain_name", "")
        if active_brain == "GOSSIP_CONSENSUS" and brain and hasattr(brain, "skip_waypoint"):
            new_idx = brain.skip_waypoint()
        else:
            leader = getattr(self.sim, "leader", None)
            if leader is None:
                return {"status": "ERROR", "reason": "No active leader to skip waypoint"}
            wps = leader.mission_waypoints if leader.mission_waypoints else self.route_waypoints
            idx = leader.current_wp_idx
            if idx + 1 < len(wps):
                leader.current_wp_idx += 1
            elif leader.mission_loop:
                leader.current_wp_idx = 0
            else:
                leader.mission_active = False
            new_idx = leader.current_wp_idx
            leader.acknowledged_target = np.array(wps[new_idx][:3], dtype=float)
            leader.wp_loitering = False
            leader.wp_loiter_timer = 0.0
            leader.wp_total_loiter = float(wps[new_idx][3]) if len(wps[new_idx]) > 3 else 0.0
            self._target_pos = np.copy(leader.acknowledged_target)

        return {"status": "OK", "action": "WAYPOINT_SKIPPED", "current_wp_idx": new_idx}

    def stop_mission_route(self) -> Dict[str, Any]:
        """Stop active mission route and hold current position (TKT-28 & TKT-56)."""
        self.route_active = False
        self.route_paused = False
        if self.bingo_mission_active and not np.allclose(self._target_pos, [0.0, 0.0, 25.0], atol=1.0):
            self.bingo_mission_active = False
            self.bingo_triggered = False
            self.bingo_stage = "IDLE"
        if self.sim:
            for d in getattr(self.sim, "drones", []):
                d.mission_active = False
                d.mission_paused = False
                d.wp_loitering = False
            brain = getattr(self.sim, "brain", None)
            if brain:
                if hasattr(brain, "route_active"):
                    brain.route_active = False
                if hasattr(brain, "route_paused"):
                    brain.route_paused = False
                if hasattr(brain, "wp_loitering"):
                    brain.wp_loitering = False
        return {"status": "OK", "action": "ROUTE_STOPPED"}

    def toggle_rng_recon(self, active: Optional[bool] = None) -> bool:
        """Toggle or explicitly set tactical RNG Touch-and-Go reconnaissance mode (TKT-29 & TKT-31)."""
        if active is not None:
            self.rng_recon_active = bool(active)
        else:
            self.rng_recon_active = not self.rng_recon_active

        if self.rng_recon_active:
            self.patrol_active = False
            self.stop_mission_route()
            self.rng_dwelling = False
            self.rng_dwell_timer = 0.0

            if not self.is_link_active():
                self.rng_status = "UPLINK_OFFLINE"
                self.rng_paused = True
            elif self.is_target_reached():
                self.rng_dwelling = True
                self.rng_dwell_timer = self.rng_dwell_time
                self.rng_station_count += 1
                self.rng_status = "DWELLING"
                self.rng_paused = False
            else:
                self.rng_status = "TRACKING"
                self.rng_paused = False
        else:
            self.rng_dwelling = False
            self.rng_dwell_timer = 0.0
            self.rng_status = "IDLE"
            self.rng_paused = False

        return self.rng_recon_active

    def start_rng_recon(self) -> Dict[str, Any]:
        """Activate RNG touch-and-go reconnaissance mode (TKT-29)."""
        self.toggle_rng_recon(True)
        return {
            "status": "OK",
            "action": "RNG_RECON_STARTED",
            "station_count": self.rng_station_count,
            "dwell_time": self.rng_dwell_time,
        }

    def stop_rng_recon(self) -> Dict[str, Any]:
        """Deactivate RNG touch-and-go reconnaissance mode (TKT-29)."""
        self.toggle_rng_recon(False)
        return {
            "status": "OK",
            "action": "RNG_RECON_STOPPED",
            "station_count": self.rng_station_count,
        }

    def clear_pads(self, turnaround_rearm: bool = False) -> Dict[str, Any]:
        """Operator action: reset/clear recovery pads and release the next wave of holding drones (TKT-42)."""
        if self.sim and hasattr(self.sim, "clear_pads"):
            return self.sim.clear_pads(turnaround_rearm=turnaround_rearm)
        return {"status": "ERROR", "reason": "No simulation engine attached"}

    def log_event(self, level: str, message: str) -> None:
        """Log event to attached session manager or console."""
        if self.sim and hasattr(self.sim, "session_manager") and self.sim.session_manager:
            self.sim.session_manager.add_event(level, message, sim_time=getattr(self.sim, "sim_time", 0.0))

    def deploy_patrol_to_bingo(
        self,
        bingo_threshold: float = 25.0,
        pattern: str = "BOX_PATROL",
    ) -> Dict[str, Any]:
        """Deploy 1-Click Aux Mission: Continuous Patrol to BINGO Battery RTB (TKT-58 / PLN-02).
        Verifies FORMATION_FLIGHT and PATROL pre-flight capabilities before uploading pattern route.
        Monitors fleet battery; automatically executes 2-stage recovery upon BINGO trigger."""
        pat = pattern.upper().replace("-", "_").replace(" ", "_")
        if pat not in ROUTE_PRESETS:
            pat = "BOX_PATROL"

        # Pre-Flight Capability Authorization Check (TKT-57)
        auth_check = self.check_mission_authorization("PATROL_TO_BINGO")
        if not auth_check.get("authorized", True) or auth_check.get("status") == "REJECTED":
            return {
                "status": "ERROR",
                "code": "TIER_AUTHORIZATION_FAILED",
                "reason": auth_check.get("reason", "TIER_AUTHORIZATION_FAILED"),
                "missing_capabilities": auth_check.get("missing_capabilities", []),
                "mission_type": "PATROL_TO_BINGO",
            }

        # Pre-Flight Battery Level Check: Reject if battery is already at or below threshold
        drones = getattr(self.sim, "drones", []) if self.sim else []
        living = [
            d for d in drones
            if not d.crashed and d.flight_state not in ("LANDED", "GROUND_IDLE")
        ]
        if living:
            min_battery = min(float(d.battery) for d in living)
            if min_battery <= float(bingo_threshold):
                return {
                    "status": "ERROR",
                    "code": "INSUFFICIENT_BATTERY",
                    "reason": f"Fleet battery ({min_battery:.1f}%) is already at or below BINGO threshold ({float(bingo_threshold):.1f}%)",
                    "min_battery": min_battery,
                    "bingo_threshold": float(bingo_threshold),
                }

        # Deploy waypoint route preset with loop=True
        wps = ROUTE_PRESETS[pat]
        route_res = self.deploy_mission_route(
            wps,
            loop=True,
            name=f"PATROL_TO_BINGO_{pat}",
            mission_type="PATROL_TO_BINGO",
        )
        if route_res.get("status") != "OK":
            return route_res

        self.bingo_mission_active = True
        self.bingo_threshold = float(bingo_threshold)
        self.bingo_triggered = False
        self.bingo_auto_land = True
        self.bingo_stage = "PATROLLING"
        self.bingo_pattern = pat

        self.log_event("CMD", f"[AUX MISSION: Patrol to BINGO armed. Pattern: {pat}, BINGO Threshold: {self.bingo_threshold:.1f}%]")

        return {
            "status": "OK",
            "action": "PATROL_TO_BINGO_DEPLOYED",
            "pattern": pat,
            "bingo_threshold": self.bingo_threshold,
            "waypoints_count": len(wps),
            "route_name": self.route_name,
        }

    def stop_patrol_to_bingo(self) -> Dict[str, Any]:
        """Abort active Patrol to BINGO aux mission (TKT-58)."""
        if not self.bingo_mission_active:
            return {"status": "ERROR", "reason": "No active Patrol to BINGO mission"}
        self.bingo_mission_active = False
        self.bingo_triggered = False
        self.bingo_stage = "IDLE"
        self.stop_mission_route()
        self.log_event("CMD", "[AUX MISSION: Patrol to BINGO ABORTED by operator]")
        return {"status": "OK", "action": "PATROL_TO_BINGO_STOPPED"}

    def deploy_search_and_recall(
        self,
        target_drone_id: Optional[int] = None,
        courier_id: Optional[int] = None,
    ) -> Dict[str, Any]:
        """Deploy 1-Click Aux Mission: Physical Courier / Search-and-Recall Rescue Mule to LKP (TKT-51 / PLN-02).
        Verifies RECON and S&R_OPTICAL pre-flight capabilities for the courier.
        Navigates autonomously to target LKP, dwells with 60m P2P rescue ping, and escorts lost wingman back."""
        if self.sim is None:
            return {"status": "ERROR", "reason": "No simulation engine attached"}

        drones = getattr(self.sim, "drones", [])
        if not drones:
            return {"status": "ERROR", "reason": "NO_DRONES"}

        # 1. Identify target lost drone
        target = None
        if target_drone_id is not None:
            target = self.sim.get_drone_by_id(target_drone_id) if hasattr(self.sim, "get_drone_by_id") else next((d for d in drones if d.id == target_drone_id), None)
            if target is None:
                return {"status": "ERROR", "reason": f"Target drone #{target_drone_id} not found"}
        else:
            # Find first living drone that is in LOST_LINK or lost downlink
            for d in drones:
                if not d.crashed and (getattr(d, "lost_link", False) or not getattr(d, "downlink_active", True) or d.flight_state in ("LOST_LINK", "BACKTRACK_HOLD")):
                    target = d
                    break
            if target is None:
                return {"status": "ERROR", "reason": "NO_LOST_DRONES_DETECTED", "error": "No dark or lost-link airframes currently detected."}

        # 2. Identify and allocate courier drone
        courier = None
        if courier_id is not None:
            courier = self.sim.get_drone_by_id(courier_id) if hasattr(self.sim, "get_drone_by_id") else next((d for d in drones if d.id == courier_id), None)
            if courier is None or courier.crashed:
                return {"status": "ERROR", "reason": f"Courier drone #{courier_id} unavailable"}
        else:
            # Find eligible airborne or available drone (not target, not crashed) with S&R capabilities
            candidates = [
                d for d in drones
                if d.id != target.id and not d.crashed and d.flight_state not in ("CRASHED",) and getattr(d, "uplink_active", True)
            ]
            for c in candidates:
                caps = set(getattr(c, "capabilities", []))
                if "RECON" in caps or "S&R_OPTICAL" in caps:
                    courier = c
                    break
            if courier is None and candidates:
                courier = candidates[0]
            if courier is None:
                return {"status": "ERROR", "reason": "NO_ELIGIBLE_COURIER", "error": "No available airframes to dispatch as courier."}

        # Pre-Flight Capability Check on courier
        auth_check = self.check_mission_authorization("SEARCH_AND_RECALL", target_drone_id=courier.id)
        if not auth_check.get("authorized", True):
            return {
                "status": "ERROR",
                "code": "TIER_AUTHORIZATION_FAILED",
                "reason": auth_check.get("reason", "TIER_AUTHORIZATION_FAILED"),
                "missing_capabilities": auth_check.get("missing_capabilities", []),
                "courier_id": courier.id,
            }

        # Setup Courier flight intent
        lkp = np.copy(target.lkp)
        courier.courier_mission_active = True
        courier.courier_target_drone_id = target.id
        courier.courier_target_lkp = np.copy(lkp)
        courier.courier_dwell_timer = 0.0
        courier.courier_dwell_timeout = 15.0
        courier.courier_escortee_id = None
        courier.flight_state = "COURIER_TRANSIT"
        courier.status = "COURIER_TRANSIT"
        courier.radio_silence_permitted = True
        courier.acknowledged_target = np.copy(lkp)
        courier.target_station = np.copy(lkp)

        self.courier_active = True
        self.courier_drone_id = courier.id
        self.courier_target_drone_id = target.id
        self.courier_target_lkp = np.copy(lkp)
        self.courier_stage = "TRANSIT_OUTBOUND"
        self.courier_dwell_timer = 0.0

        self.log_event(
            "CMD",
            f"[AUX MISSION: S&R Courier #{courier.id} dispatched to LKP ({lkp[0]:.1f}, {lkp[1]:.1f}, {lkp[2]:.1f}) for dark Drone #{target.id}. Radio silence once past horizon.]"
        )

        return {
            "status": "OK",
            "action": "SEARCH_AND_RECALL_DEPLOYED",
            "courier_id": courier.id,
            "target_drone_id": target.id,
            "target_lkp": [round(float(v), 2) for v in lkp],
            "stage": self.courier_stage,
        }

    def stop_search_and_recall(self) -> Dict[str, Any]:
        """Abort active Search and Recall courier mission (TKT-51)."""
        if not self.courier_active:
            return {"status": "ERROR", "reason": "No active Search and Recall mission"}

        if self.sim:
            courier = self.sim.get_drone_by_id(self.courier_drone_id) if hasattr(self.sim, "get_drone_by_id") else None
            if courier:
                courier.courier_mission_active = False
                courier.courier_escortee_id = None
                courier.flight_state = "CRUISE"
                courier.status = "ACTIVE"
                courier.radio_silence_permitted = False
            target_drone = self.sim.get_drone_by_id(self.courier_target_drone_id) if hasattr(self.sim, "get_drone_by_id") else None
            if target_drone and target_drone.flight_state == "ESCORT_REJOIN":
                target_drone.escort_leader_id = None

        self.courier_active = False
        self.courier_stage = "IDLE"
        self.log_event("CMD", "[AUX MISSION: Search and Recall ABORTED by operator]")
        return {"status": "OK", "action": "SEARCH_AND_RECALL_STOPPED"}
