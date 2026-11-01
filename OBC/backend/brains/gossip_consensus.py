"""
SwarmTech3_UpRising: P2P Gossip Consensus Brain (gossip_consensus.py)
Implements decentralized Peer-to-Peer (P2P) mesh communications,
multi-hop target gossip propagation, quorum barrier consensus,
and synchronized bank transitions without a single point of failure.
"""

from __future__ import annotations
import math
from typing import Dict, List, Optional, Set, Any, TYPE_CHECKING
import numpy as np

from .base import BaseBrain
from .hungarian import hungarian_assignment

if TYPE_CHECKING:
    from SIM.backend.sitl_engine import SwarmSimulator
    from OBC.backend.drone_firmware import Drone

# Radio Comms & Asymmetric Link Budget (Canine Acoustic Taxonomy - TKT-49)
RANGE_HOWL: float = 140.0              # High-power GCS mast TX (blasting uplink) (m)
RANGE_BARK: float = 90.0               # Tier-1 Flagship onboard TX (leader downlink) (m)
RANGE_YIP: float = 50.0                # Tier-2 Escort micro-transceiver (escort downlink) (m)
RANGE_P2P: float = 60.0                # Inter-drone P2P mesh gossip radio propagation limit (m)
RADIO_RANGE: float = RANGE_HOWL        # Backward-compatible generic radio horizon alias (m)
HEARTBEAT_INTERVAL: float = 0.1
V_CRUISE: float = 12.0
V_LKP: float = 6.0
QUORUM_RATIO: float = 0.75  # 75% consensus threshold required before transition


DEADBAND_RADIUS: float = 0.35
MAX_SLEW_RATE: float = math.radians(45.0)
FREEZE_SPEED: float = 0.5   # Cruising speed threshold below which heading freezes (m/s)
FREEZE_DIST: float = 2.0    # Distance to target below which heading freezes against jitter (m)


class DronePeerState:
    """Decentralized state tracked on each individual drone."""

    def __init__(self, drone_id: int, initial_target: np.ndarray):
        self.drone_id: int = drone_id
        self.active_target: np.ndarray = np.array(initial_target, dtype=float)
        self.active_seq: int = 1
        self.pending_target: np.ndarray = np.array(initial_target, dtype=float)
        self.pending_seq: int = 1
        self.echo_ack_peers: Set[int] = {drone_id}
        self.gossip_status: str = "COORDINATED"  # COORDINATED, PROPAGATING
        self.last_tx_time: float = 0.0


class GossipConsensusBrain(BaseBrain):
    """Decentralized P2P Gossip Consensus Swarm Brain.
    - No single fleet leader required.
    - Mission targets injected into any drone in range are echoed across the mesh.
    - Drones continue safe current trajectories while echoing (no sudden collision stops).
    - Once quorum (>= 75%) is reached, all drones smoothly bank toward the new target.
    - TKT-11: Structured decentralized swarm formations (Sentry Shell, Orbit Vortex,
      Chevron Echelon, and Dispersed Lattice) computed deterministically from peer ID rankings."""

    brain_name: str = "GOSSIP_CONSENSUS"

    def __init__(self, sim: SwarmSimulator):
        super().__init__(sim)
        self.current_seq: int = 1
        self.peer_states: Dict[int, DronePeerState] = {}
        self.last_broadcast_tick: float = -1.0
        self.formation_mode: str = getattr(sim, "formation_mode", "PERIMETER_PATROL")
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
        self.reset()

    def reset(self) -> None:
        self.current_seq = 1
        self.peer_states.clear()
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
        for drone in self.sim.drones:
            self.peer_states[drone.id] = DronePeerState(
                drone.id, self.sim.target_pos
            )

    def on_route_deployed(self, clean_wps: List[np.ndarray], loop: bool = True) -> None:
        """Receive newly uploaded discrete mission route (TKT-28 & TKT-30)."""
        self.route_waypoints = [np.copy(wp) for wp in clean_wps]
        self.current_route_wp_idx = 0
        self.route_active = True
        self.route_paused = False
        self.route_loop = loop
        self.wp_loiter_timer = 0.0
        self.wp_total_loiter = float(clean_wps[0][3]) if len(clean_wps[0]) > 3 else 0.0
        self.wp_loitering = False
        self.current_seq += 1
        first_wp = clean_wps[0][:3]
        for drone in self.sim.drones:
            drone.mission_waypoints = [np.copy(wp) for wp in clean_wps]
            drone.current_wp_idx = 0
            drone.mission_active = True
            drone.mission_paused = False
            drone.mission_loop = loop
            drone.acknowledged_target = np.copy(first_wp)
            drone.wp_loiter_timer = 0.0
            drone.wp_total_loiter = self.wp_total_loiter
            drone.wp_loitering = False
            if drone.id in self.peer_states:
                state = self.peer_states[drone.id]
                state.pending_seq = self.current_seq
                state.pending_target = np.copy(first_wp)
                state.active_seq = self.current_seq
                state.active_target = np.copy(first_wp)
                state.echo_ack_peers = {d.id for d in self.sim.drones if not d.crashed}
                state.gossip_status = "COORDINATED"

    # ==========================================================================
    # 1. 10 Hz P2P Mesh Broadcast & Gossip Propagation
    # ==========================================================================
    def update_comms(self, dt: float) -> None:
        current_time = self.sim.sim_time

        # Check 10 Hz broadcast timer
        if current_time - self.last_broadcast_tick < HEARTBEAT_INTERVAL:
            return
        self.last_broadcast_tick = current_time

        active_drones = [
            d for d in self.sim.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING") and getattr(d, "enrolled", True) and not getattr(d, "gnss_denied", False)
        ]
        living_count = len(active_drones)
        if living_count == 0:
            return

        # Build 10 Hz heartbeat packets for all active drones
        packets = {}
        for drone in active_drones:
            state = self._get_peer_state(drone.id)
            packets[drone.id] = {
                "id": drone.id,
                "pos": np.copy(drone.pos),
                "vel": np.copy(drone.vel),
                "active_seq": state.active_seq,
                "echo_seq": state.pending_seq,
                "echo_target": np.copy(state.pending_target),
            }

        # Compute RSSI & Link Quality for all airborne drones based on proximity to nearest peer or GCS anchor
        for drone in self.sim.drones:
            if drone.crashed or drone.flight_state in ("LANDED", "DESCENDING"):
                continue
            distances = [float(np.linalg.norm(drone.pos - self.sim.tether_anchor))]
            for peer in self.sim.drones:
                if peer.id != drone.id and not peer.crashed and peer.flight_state not in ("LANDED", "DESCENDING"):
                    distances.append(float(np.linalg.norm(drone.pos - peer.pos)))
            min_dist = min(distances) if distances else 0.0
            drone.rssi, drone.link_quality = self.sim.calculate_rssi(min_dist, is_hardline=False)

        # Simulate decentralized P2P RF broadcast over Euclidean RADIO_RANGE (120m)
        for drone in active_drones:
            drone_state = self._get_peer_state(drone.id)

            for peer in active_drones:
                if peer.id == drone.id:
                    continue

                dist = float(np.linalg.norm(drone.pos - peer.pos))
                if dist > RANGE_P2P:
                    continue  # Out of inter-drone P2P mesh radio range (60m)

                # Physics-based packet drop from path loss & EW jamming
                if self.sim.is_packet_dropped(dist, is_hardline=False, max_range=RANGE_P2P):
                    continue

                peer_pkt = packets[peer.id]

                # Check if peer has newer target directive
                if peer_pkt["echo_seq"] > drone_state.pending_seq:
                    # Adopt newer pending target and begin echoing
                    drone_state.pending_seq = peer_pkt["echo_seq"]
                    drone_state.pending_target = np.copy(peer_pkt["echo_target"])
                    drone_state.echo_ack_peers = {drone.id, peer.id}
                    drone_state.gossip_status = "PROPAGATING"

                elif peer_pkt["echo_seq"] == drone_state.pending_seq:
                    # Peer is acknowledging the same pending sequence
                    drone_state.echo_ack_peers.add(peer.id)

            # Check Quorum Barrier Synchronization
            # Once quorum ratio reached, transition active target
            living_peer_ids = {d.id for d in active_drones}
            acks_in_range = drone_state.echo_ack_peers.intersection(living_peer_ids)
            ack_ratio = len(acks_in_range) / float(living_count)

            if drone_state.pending_seq > drone_state.active_seq:
                if ack_ratio >= QUORUM_RATIO:
                    # Consensus achieved: synchronized transition!
                    drone_state.active_seq = drone_state.pending_seq
                    drone_state.active_target = np.copy(drone_state.pending_target)
                    drone_state.gossip_status = "COORDINATED"

    def _get_peer_state(self, drone_id: int) -> DronePeerState:
        if drone_id not in self.peer_states:
            self.peer_states[drone_id] = DronePeerState(
                drone_id, self.sim.target_pos
            )
        return self.peer_states[drone_id]

    # ==========================================================================
    # 2. Decentralized Guidance Vector Computation (TKT-11 Formations)
    # ==========================================================================
    def compute_guidance(self, dt: float) -> Dict[int, np.ndarray]:
        """Compute guidance steering forces:
        1. Mission Arrival Steering toward consensus geometric slot offset.
        2. Local Flocking Cohesion & Alignment with nearby peers."""
        accs = {d.id: np.zeros(3, dtype=float) for d in self.sim.drones}

        # Guidance for GNSS-denied airframes (TKT-52)
        gnss_denied_drones = [
            d for d in self.sim.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING") and getattr(d, "gnss_denied", False)
        ]
        for d in gnss_denied_drones:
            accs[d.id] = self.compute_gnss_denied_guidance(d, dt)

        # Guidance for unenrolled solo scouts (TKT-50)
        unenrolled_drones = [
            d for d in self.sim.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING") and not getattr(d, "enrolled", True) and not getattr(d, "gnss_denied", False)
        ]
        for scout in unenrolled_drones:
            if getattr(scout, "lost_link", False):
                scout.target_station = scout.lkp
                accs[scout.id] = self._steer_toward(
                    scout.pos, scout.vel, scout.lkp, max_speed=V_LKP
                )
            elif scout.rejoin_target is not None:
                scout.target_station = scout.rejoin_target
                accs[scout.id] = self._steer_toward(
                    scout.pos, scout.vel, scout.rejoin_target, max_speed=V_CRUISE
                )
            else:
                scout.target_station = scout.pos
                accs[scout.id] = self._steer_toward(
                    scout.pos, scout.vel, scout.pos, max_speed=0.0
                )

        active_drones = [
            d for d in self.sim.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING") and getattr(d, "enrolled", True) and not getattr(d, "gnss_denied", False)
        ]
        living_count = len(active_drones)
        if living_count == 0:
            return accs

        # Deterministic decentralized slot indexing based on living drone IDs
        sorted_ids = sorted([d.id for d in active_drones])
        active_ids = {d.id for d in active_drones}
        for did in list(self.assigned_slots.keys()):
            if did not in active_ids:
                del self.assigned_slots[did]

        is_paused = getattr(self, "route_paused", False)

        # Check Mission Route waypoint advancement for Gossip Quorum (TKT-28, TKT-30, TKT-56)
        if self.route_active and self.route_waypoints:
            wps = self.route_waypoints
            idx = self.current_route_wp_idx
            if 0 <= idx < len(wps):
                raw_wp = wps[idx]
                target_wp = np.array(raw_wp[:3], dtype=float)
                loiter_duration = float(raw_wp[3]) if len(raw_wp) > 3 else 0.0
                centroid = np.mean([d.pos for d in active_drones], axis=0)
                dist_centroid = float(np.linalg.norm(centroid - target_wp))
                min_peer_dist = min(float(np.linalg.norm(d.pos - target_wp)) for d in active_drones) if active_drones else 999.0
                arrival_dist = min(dist_centroid, min_peer_dist)

                for d in active_drones:
                    d.wp_total_loiter = loiter_duration
                    d.wp_loiter_timer = self.wp_loiter_timer
                    d.wp_loitering = self.wp_loitering
                    d.mission_paused = is_paused

                if not is_paused and arrival_dist <= self.sim.route_acceptance_radius:
                    peers_settled = all(
                        float(np.linalg.norm(d.pos - self._get_formation_slot(d, self.assigned_slots.get(d.id, sorted_ids.index(d.id)), living_count, target_wp))) <= 4.5
                        for d in active_drones
                    ) if living_count > 1 else True

                    if not self.wp_loitering and loiter_duration > 0.0:
                        self.wp_loitering = True
                        self.wp_loiter_timer = max(0.0, loiter_duration - dt)
                        self.wp_total_loiter = loiter_duration
                        for d in active_drones:
                            d.wp_loitering = True
                            d.wp_loiter_timer = self.wp_loiter_timer
                            d.wp_total_loiter = loiter_duration
                    elif self.wp_loitering:
                        self.wp_loiter_timer = max(0.0, self.wp_loiter_timer - dt)
                        for d in active_drones:
                            d.wp_loiter_timer = self.wp_loiter_timer
                        if self.wp_loiter_timer <= 0.0 and peers_settled:
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
                                next_wp = target_wp
                                next_loiter = 0.0

                            self.wp_total_loiter = next_loiter
                            if self.route_active:
                                self.current_seq += 1
                                for d in active_drones:
                                    d.current_wp_idx = self.current_route_wp_idx
                                    d.acknowledged_target = np.copy(next_wp)
                                    d.wp_loitering = False
                                    d.wp_loiter_timer = 0.0
                                    d.wp_total_loiter = next_loiter
                                    if d.id in self.peer_states:
                                        p_state = self.peer_states[d.id]
                                        p_state.active_seq = self.current_seq
                                        p_state.pending_seq = self.current_seq
                                        p_state.active_target = np.copy(next_wp)
                                        p_state.pending_target = np.copy(next_wp)
                    else:
                        # Zero-duration fly-through waypoint
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
                            next_wp = target_wp
                            next_loiter = 0.0

                        self.wp_total_loiter = next_loiter
                        if self.route_active:
                            self.current_seq += 1
                            for d in active_drones:
                                d.current_wp_idx = self.current_route_wp_idx
                                d.acknowledged_target = np.copy(next_wp)
                                d.wp_loitering = False
                                d.wp_loiter_timer = 0.0
                                d.wp_total_loiter = next_loiter
                                if d.id in self.peer_states:
                                    p_state = self.peer_states[d.id]
                                    p_state.active_seq = self.current_seq
                                    p_state.pending_seq = self.current_seq
                                    p_state.active_target = np.copy(next_wp)
                                    p_state.pending_target = np.copy(next_wp)

        # Update virtual swarm heading toward consensus target
        consensus_target = self.sim.get_acknowledged_target()
        self._update_formation_heading(active_drones, consensus_target, dt)

        for drone in active_drones:
            # 0. Backtrack & Lost-link failsafe (TKT-54)
            if getattr(drone, "flight_state", "") == "BACKTRACK_HOLD" and getattr(drone, "backtrack_target", None) is not None:
                drone.target_station = drone.backtrack_target
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, drone.backtrack_target, max_speed=8.0
                )
                continue
            elif getattr(drone, "flight_state", "") == "PERIMETER_HOLD" or getattr(drone, "holding_at_horizon", False):
                drone.target_station = drone.acknowledged_target
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, drone.acknowledged_target, max_speed=2.0
                )
                continue
            if getattr(drone, "lost_link", False):
                drone.target_station = drone.lkp
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, drone.lkp, max_speed=V_LKP
                )
                continue

            state = self._get_peer_state(drone.id)
            drone.acknowledged_target = np.copy(state.active_target)
            if self.formation_mode == "NATURAL_FLOCK":
                drone.target_station = state.active_target
                accs[drone.id] = self._compute_boid_acceleration(
                    drone, active_drones, state.active_target
                )
            else:
                if getattr(self.sim, "overhead_marshal_active", False):
                    k = sorted_ids.index(drone.id)
                else:
                    k = self.assigned_slots.get(drone.id, sorted_ids.index(drone.id))
                slot_pos = self._get_formation_slot(drone, k, living_count, state.active_target)
                drone.target_station = slot_pos

                # 1. Target Pursuit: Arrive steering toward decentralized geometric slot
                target_speed = V_CRUISE
                if is_paused or getattr(self, "wp_loitering", False):
                    target_speed = 2.0
                elif getattr(drone, "enrollment_timer", 0.0) > 0.0:
                    ramp = max(0.0, min(1.0, 1.0 - (drone.enrollment_timer / 2.5)))
                    target_speed = 7.0 + (V_CRUISE - 7.0) * ramp

                target_acc = self._steer_toward(
                    drone.pos, drone.vel, slot_pos, max_speed=target_speed
                )

                # 2. Local Flocking Alignment (match velocity of peers within 15m)
                flock_acc = np.zeros(3, dtype=float)
                neighbor_count = 0
                for peer in active_drones:
                    if peer.id == drone.id:
                        continue
                    d = float(np.linalg.norm(drone.pos - peer.pos))
                    if d <= 15.0:
                        flock_acc += (peer.vel - drone.vel) * 0.3
                        neighbor_count += 1

                if neighbor_count > 0:
                    flock_acc /= float(neighbor_count)

                accs[drone.id] = target_acc + flock_acc

        # 3. Tactical Formation Orientations & Headings (TKT-55)
        for drone in active_drones:
            speed_xy = math.hypot(drone.vel[0], drone.vel[1])

            # Failsafe modes face velocity heading
            if getattr(drone, "gnss_denied", False) or getattr(drone, "lost_link", False) or getattr(drone, "flight_state", "") == "BACKTRACK_HOLD":
                if speed_xy > 0.5:
                    drone.target_yaw = math.atan2(drone.vel[1], drone.vel[0])
                continue

            state = self._get_peer_state(drone.id)
            center = getattr(state, "active_target", consensus_target)

            if self.formation_mode in ("PERIMETER_PATROL", "PERIMETER_IN", "PERIMETER_OUT"):
                r_xy = math.hypot(drone.pos[0] - center[0], drone.pos[1] - center[1])
                if self.formation_mode == "PERIMETER_OUT":
                    psi_target = math.atan2(drone.pos[1] - center[1], drone.pos[0] - center[0]) if r_xy > 0.5 else self.formation_heading
                else:  # PERIMETER_PATROL or PERIMETER_IN
                    psi_target = math.atan2(center[1] - drone.pos[1], center[0] - drone.pos[0]) if r_xy > 0.5 else self.formation_heading

                dist_to_slot = float(np.linalg.norm(drone.pos - getattr(drone, "target_station", drone.pos)))
                if dist_to_slot > 6.0 and speed_xy > 2.0:
                    drone.target_yaw = math.atan2(drone.vel[1], drone.vel[0])
                else:
                    drone.target_yaw = psi_target

            elif self.formation_mode == "ECHELON_ESCORT":
                drone.target_yaw = self.formation_heading

            elif self.formation_mode == "ORBIT_VORTEX":
                drone.target_yaw = math.atan2(drone.vel[1], drone.vel[0]) if speed_xy > 0.5 else self.formation_heading

            elif self.formation_mode == "NATURAL_FLOCK":
                drone.target_yaw = math.atan2(drone.vel[1], drone.vel[0]) if speed_xy > 0.2 else float(drone.yaw)

        return accs

    def _update_formation_heading(self, active_drones: List[Drone], target_pos: np.ndarray, dt: float) -> None:
        """Update virtual swarm heading vector toward consensus target with smooth slew.
        Uses mean consensus flight velocity and apex-target vector to eliminate
        pinwheel spinning when wing airframe counts are asymmetric/off-balance."""
        if not active_drones:
            return

        # Cruising velocity of the swarm
        mean_vel = np.mean([d.vel for d in active_drones], axis=0)
        speed_xy = float(math.hypot(mean_vel[0], mean_vel[1]))

        # Apex drone (slot k=0 in sorted IDs) position as geometric reference (unbiased by wings)
        sorted_drones = sorted(active_drones, key=lambda d: d.id)
        apex_drone = sorted_drones[0]
        to_target = target_pos - apex_drone.pos
        dist_to_target = float(math.hypot(to_target[0], to_target[1]))

        # 1. Initial bootstrap if heading has not been established yet
        if not self.heading_initialized:
            target_ref = getattr(self.sim, "target_pos", target_pos)
            to_target_init = target_ref - apex_drone.pos
            if speed_xy >= FREEZE_SPEED:
                self.formation_heading = math.atan2(mean_vel[1], mean_vel[0])
            else:
                dist_xy = math.hypot(to_target_init[0], to_target_init[1])
                if dist_xy > 1.0:
                    self.formation_heading = math.atan2(to_target_init[1], to_target_init[0])
                else:
                    self.formation_heading = float(apex_drone.yaw)
            self.heading_initialized = True
            return

        self.inversion_cooldown = max(0.0, self.inversion_cooldown - dt)

        # Check if active mission target indicates a sharp reversal (> 120 deg) (TKT-43)
        dist_xy = math.hypot(to_target[0], to_target[1])
        if (
            dist_xy > 5.0
            and self.inversion_cooldown <= 0.0
            and self.formation_mode == "ECHELON_ESCORT"
            and not getattr(self.sim, "overhead_marshal_active", False)
        ):
            to_tgt_dir = np.array([to_target[0] / dist_xy, to_target[1] / dist_xy])
            h_dir = np.array([math.cos(self.formation_heading), math.sin(self.formation_heading)])
            if float(np.dot(to_tgt_dir, h_dir)) < -0.5:
                rev_heading = math.atan2(to_target[1], to_target[0])
                if self.invert_formation_slots(active_drones, target_pos, rev_heading):
                    self.formation_heading = rev_heading
                    return

        patrol_active = getattr(self.sim, "patrol_active", False)

        # 2. Anti-jitter freeze: when swarm is settling, hovering near target, or decelerating
        # Freeze heading at last stable cruising angle to prevent asymmetric wing pinwheel spin!
        if not patrol_active and (speed_xy < FREEZE_SPEED or dist_to_target < FREEZE_DIST):
            return

        # 3. Target heading: follow flight velocity vector during cruise, or apex-to-target when starting
        if speed_xy >= FREEZE_SPEED:
            desired_yaw = math.atan2(mean_vel[1], mean_vel[0])
        elif dist_to_target > 1.0:
            desired_yaw = math.atan2(to_target[1], to_target[0])
        else:
            return

        yaw_diff = (desired_yaw - self.formation_heading + math.pi) % (2.0 * math.pi) - math.pi

        # Dynamic Hungarian Formation Inversion on sharp reversal (TKT-43)
        if (
            abs(yaw_diff) > math.radians(120.0)
            and self.inversion_cooldown <= 0.0
            and self.formation_mode == "ECHELON_ESCORT"
        ):
            if self.invert_formation_slots(active_drones, target_pos, desired_yaw):
                self.formation_heading = desired_yaw
                return

        effective_slew_rate = math.radians(60.0) if patrol_active else MAX_SLEW_RATE
        max_step = effective_slew_rate * dt
        slew = float(np.clip(yaw_diff, -max_step, max_step))
        self.formation_heading = (self.formation_heading + slew + math.pi) % (2.0 * math.pi) - math.pi

    def invert_formation_slots(
        self, active_drones: List[Drone], target_pos: np.ndarray, target_heading: float
    ) -> bool:
        """Perform Hungarian minimal-displacement slot re-assignment on heading reversal (TKT-43)."""
        n = len(active_drones)
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
        for i, drone in enumerate(active_drones):
            for j, s_pos in enumerate(candidate_slots):
                diff = drone.pos - s_pos
                cost_matrix[i, j] = float(np.dot(diff, diff))

        row_ind, col_ind = hungarian_assignment(cost_matrix)
        for r, c in zip(row_ind, col_ind):
            drone = active_drones[r]
            self.assigned_slots[drone.id] = c

        self.inversion_cooldown = 2.5
        return True

    def _get_formation_slot(
        self, drone: Drone, k: int, N: int, target_pos: np.ndarray
    ) -> np.ndarray:
        """Compute 3D position of decentralized geometric slot k in [0, N-1] (TKT-11).
        Preserves wing-side continuity (odd k -> right wing, even k -> left wing)
        to prevent cross-formation reciprocal collisions."""
        if k == 0:
            azimuth = 0.0
        else:
            side = 1.0 if (k % 2 == 1) else -1.0
            pair = (k + 1) // 2
            azimuth = side * pair * (2.0 * math.pi / max(1, N))

        # Stage 1 Overhead Marshal: Equidistant holding ring around Base anchor (0, 0, 25m)
        if getattr(self.sim, "overhead_marshal_active", False):
            radius = 20.0
            return np.array([radius * math.cos(azimuth), radius * math.sin(azimuth), 25.0], dtype=float)

        if self.formation_mode in ("PERIMETER_PATROL", "PERIMETER_IN", "PERIMETER_OUT"):
            # Static defensive sentry ring holding equidistant guard stations around target (R = 25m) (TKT-55)
            radius = 25.0
            return (
                target_pos
                + np.array([radius * math.cos(azimuth), radius * math.sin(azimuth), 0.0], dtype=float)
            )

        elif self.formation_mode == "ORBIT_VORTEX":
            # Dynamic rotating ring around consensus target with altitude undulation
            radius = 20.0
            omega = 0.6
            angle = omega * self.sim.sim_time + azimuth
            z_offset = 3.0 * math.sin(2.0 * angle)
            return (
                target_pos
                + np.array([radius * math.cos(angle), radius * math.sin(angle), z_offset], dtype=float)
            )

        elif self.formation_mode == "ECHELON_ESCORT":
            # Chevron / V-formation centered on lead apex peer oriented along virtual heading
            heading = np.array(
                [math.cos(self.formation_heading), math.sin(self.formation_heading), 0.0], dtype=float
            )
            lateral = np.array([-heading[1], heading[0], 0.0], dtype=float)

            if k == 0:
                return target_pos

            # Find apex drone (slot 0) to align chevron with lead airframe position during transit
            apex_drone = None
            if hasattr(self, "assigned_slots") and hasattr(self.sim, "drones"):
                for d in self.sim.drones:
                    if not d.crashed and self.assigned_slots.get(d.id) == 0:
                        apex_drone = d
                        break

            # Center chevron on live apex airframe position during transit (fallback: target_pos)
            apex_pos = apex_drone.pos if apex_drone is not None else target_pos

            row = (k + 1) // 2
            side = 1.0 if (k % 2 == 1) else -1.0
            return (
                apex_pos
                - (row * 6.0 * heading)
                + (side * row * 6.0 * lateral)
                + np.array([0.0, 0.0, row * 0.5], dtype=float)
            )

        else:
            # NATURAL_FLOCK representative station for external inspection
            return target_pos

    def _compute_boid_acceleration(
        self,
        drone: Drone,
        active_drones: List[Drone],
        consensus_target: np.ndarray,
    ) -> np.ndarray:
        """Compute true decentralized 3D Reynolds boid flocking acceleration (TKT-23).
        Forces:
        1. Separation: Inverse-square 3D repulsion (d < 5.0m) + Zone 3 momentum damping (d < 3.5m).
        2. Alignment: P2P velocity matching with neighbors (d < 14.0m).
        3. Cohesion: Attraction toward local cluster centroid (d < 20.0m).
        4. Goal Migration: Pull toward the consensus target.
        5. Vertical Z-Space Staggering & Safety Floor (Z >= 10.0m).
        """
        # 1. Goal Migration toward consensus target
        a_goal = self._steer_toward(
            drone.pos, drone.vel, consensus_target, max_speed=V_CRUISE, arrival_radius=8.0
        )

        # 2. Separation (d < 5.0m)
        a_sep = np.zeros(3, dtype=float)
        R_SEP = 5.0
        for peer in active_drones:
            if peer.id == drone.id:
                continue
            diff = drone.pos - peer.pos
            d = float(np.linalg.norm(diff))
            if 0.001 < d < R_SEP:
                rep_strength = (R_SEP - d) / (d + 0.1)
                a_sep += (diff / d) * rep_strength * 4.0

                # 3D Z-Space staggering: if horizontally close, push vertically apart to break 2D planes
                d_xy = float(math.hypot(diff[0], diff[1]))
                if d_xy < 3.5:
                    z_sign = 1.0 if diff[2] >= 0.0 else -1.0
                    a_sep[2] += z_sign * (3.5 - d_xy) * 3.0

                # Zone 3 relative-velocity approach damping (brake incoming momentum)
                if d < 3.5:
                    v_rel = drone.vel - peer.vel
                    closing = float(np.dot(v_rel, diff / d))
                    if closing < 0:
                        a_sep += -(diff / d) * closing * 2.5

        # 3. Alignment (d < 14.0m) - velocity matching with local peers
        a_align = np.zeros(3, dtype=float)
        R_ALIGN = 14.0
        align_weight_sum = 0.0
        vel_sum = np.zeros(3, dtype=float)
        for peer in active_drones:
            if peer.id == drone.id:
                continue
            d = float(np.linalg.norm(drone.pos - peer.pos))
            if d < R_ALIGN:
                w = 1.0 - (d / R_ALIGN)
                vel_sum += peer.vel * w
                align_weight_sum += w

        if align_weight_sum > 0:
            mean_vel = vel_sum / align_weight_sum
            a_align = (mean_vel - drone.vel) * 1.5

        # 4. Cohesion (d < 20.0m) - pull toward local peer centroid
        a_coh = np.zeros(3, dtype=float)
        R_COH = 20.0
        pos_sum = np.zeros(3, dtype=float)
        coh_count = 0
        for peer in active_drones:
            if peer.id == drone.id:
                continue
            d = float(np.linalg.norm(drone.pos - peer.pos))
            if d < R_COH:
                pos_sum += peer.pos
                coh_count += 1

        if coh_count > 0:
            centroid = pos_sum / float(coh_count)
            offset_cent = centroid - drone.pos
            d_cent = float(np.linalg.norm(offset_cent))
            if d_cent > 3.0:
                a_coh = (offset_cent / d_cent) * min(V_CRUISE, d_cent * 1.2) - drone.vel
                a_coh *= 0.8

        # 5. Altitude Floor & Ceiling Guard (Anisotropic 3D)
        a_z_guard = 0.0
        if drone.pos[2] < 12.0:
            a_z_guard += max(0.0, (14.0 - drone.pos[2]) * 4.0 - drone.vel[2] * 2.0)
        elif drone.pos[2] > 40.0:
            a_z_guard -= max(0.0, (drone.pos[2] - 38.0) * 4.0 + drone.vel[2] * 2.0)

        # 6. Tether Clearance: 3D Repulsion from tether umbilical if active
        a_tether = np.zeros(3, dtype=float)
        if (
            getattr(self.sim, "comms_mode", "TETHERED_RELAY") == "TETHERED_RELAY"
            and not getattr(self.sim, "tether_severed", False)
            and not getattr(drone, "is_leader", False)
        ):
            current_leader = getattr(self.sim, "leader", None)
            if current_leader is not None and hasattr(self.sim, "_dist_to_segment"):
                d_perp, r_perp = self.sim._dist_to_segment(
                    drone.pos, self.sim.tether_anchor, current_leader.pos
                )
                TETHER_ENVELOPE = 5.0
                if d_perp < TETHER_ENVELOPE:
                    push_dir = r_perp / max(0.01, d_perp)
                    push_mag = (TETHER_ENVELOPE - d_perp) * 12.0
                    v_closing = float(np.dot(drone.vel, -push_dir))
                    if v_closing > 0:
                        push_mag += v_closing * 4.0
                    a_tether = push_dir * push_mag

                    # Safety invariant: inhibit goal and cohesion forces pulling toward the cable
                    inward_goal = float(np.dot(a_goal, -push_dir))
                    if inward_goal > 0:
                        a_goal -= (-push_dir) * inward_goal
                    inward_coh = float(np.dot(a_coh, -push_dir))
                    if inward_coh > 0:
                        a_coh -= (-push_dir) * inward_coh

        # Weighted combination of Reynolds boid forces
        total_acc = (
            2.0 * a_goal +
            3.5 * a_sep +
            1.5 * a_align +
            1.0 * a_coh +
            a_tether
        )
        total_acc[2] += a_z_guard

        # Clamp to physical motor acceleration limit
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

    # ==========================================================================
    # 3. Mission Directive Ingestion (Gossip Trigger)
    # ==========================================================================
    def inject_target(self, new_target: List[float]) -> Dict[str, Any]:
        """Ingest new target command from GCS. Delivers to nearest drone within GCS radio range (120m),
        which immediately begins echoing to its peers across the mesh."""
        new_target_arr = np.array(new_target, dtype=float)

        # Find closest active drone within GCS Howl radio range (140m)
        active_drones = [d for d in self.sim.drones if not d.crashed]
        if not active_drones:
            return {"status": "NO_ACTIVE_DRONES", "error": "No living drones in swarm"}

        drones_in_range = []
        for d in active_drones:
            dist_to_gcs = float(np.linalg.norm(d.pos - self.sim.tether_anchor))
            if dist_to_gcs <= RANGE_HOWL:
                drones_in_range.append((dist_to_gcs, d))

        if not drones_in_range:
            return {
                "status": "OUT_OF_RANGE",
                "error": f"No active drones within GCS Howl radio range ({RANGE_HOWL}m)",
            }

        # Seed target into closest drone to GCS anchor
        drones_in_range.sort(key=lambda item: item[0])
        seed_drone = drones_in_range[0][1]

        self.current_seq += 1
        self.sim.target_pos = new_target_arr

        state = self._get_peer_state(seed_drone.id)
        state.pending_seq = self.current_seq
        state.pending_target = np.copy(new_target_arr)
        state.echo_ack_peers = {seed_drone.id}
        state.gossip_status = "PROPAGATING"

        return {
            "status": "OK",
            "cmd": "INJECT_TARGET",
            "seq": self.current_seq,
            "seed_drone_id": seed_drone.id,
            "target": [round(float(v), 2) for v in new_target_arr],
        }

    def skip_waypoint(self) -> int:
        """Skip current waypoint and transition active gossip quorum immediately to the next (TKT-56)."""
        if not self.route_active or not self.route_waypoints:
            return self.current_route_wp_idx

        wps = self.route_waypoints
        idx = self.current_route_wp_idx
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
            next_wp = np.array(wps[idx][:3], dtype=float)
            next_loiter = 0.0

        self.wp_loitering = False
        self.wp_loiter_timer = 0.0
        self.wp_total_loiter = next_loiter
        self.current_seq += 1

        active_drones = [
            d for d in self.sim.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING")
        ]
        for d in active_drones:
            d.current_wp_idx = self.current_route_wp_idx
            d.acknowledged_target = np.copy(next_wp)
            d.wp_loitering = False
            d.wp_loiter_timer = 0.0
            d.wp_total_loiter = next_loiter
            if d.id in self.peer_states:
                p_state = self.peer_states[d.id]
                p_state.active_seq = self.current_seq
                p_state.pending_seq = self.current_seq
                p_state.active_target = np.copy(next_wp)
                p_state.pending_target = np.copy(next_wp)
                p_state.echo_ack_peers = {p.id for p in active_drones}
                p_state.gossip_status = "COORDINATED"

        return self.current_route_wp_idx

    def handle_command(self, cmd_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        cmd = cmd_dict.get("cmd", "").upper()
        if cmd in ("SET_TARGET", "DISPATCH_TARGET"):
            pos = cmd_dict.get("pos", [60.0, 40.0, 30.0])
            return self.inject_target(pos)
        elif cmd == "SET_FORMATION":
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
            for d in self.sim.drones:
                d.mission_paused = True
            return {"status": "OK", "action": "ROUTE_PAUSED"}
        elif cmd == "RESUME_ROUTE":
            self.route_paused = False
            for d in self.sim.drones:
                d.mission_paused = False
            return {"status": "OK", "action": "ROUTE_RESUMED"}
        elif cmd == "SKIP_WAYPOINT":
            new_idx = self.skip_waypoint()
            return {"status": "OK", "action": "WAYPOINT_SKIPPED", "current_wp_idx": new_idx}
        elif cmd in ("ABORT_ROUTE", "STOP_ROUTE"):
            self.route_active = False
            self.route_paused = False
            self.wp_loitering = False
            self.wp_loiter_timer = 0.0
            for d in self.sim.drones:
                d.mission_active = False
                d.mission_paused = False
                d.wp_loitering = False
                d.mission_waypoints = []
            return {"status": "OK", "action": "ROUTE_ABORTED"}
        return None

    def get_telemetry_extra(self) -> Dict[str, Any]:
        active_drones = [d for d in self.sim.drones if not d.crashed]
        propagating_count = 0
        for d in active_drones:
            if self._get_peer_state(d.id).gossip_status == "PROPAGATING":
                propagating_count += 1

        return {
            "brain_mode": "GOSSIP_CONSENSUS",
            "formation": self.formation_mode,
            "formation_heading_deg": round(math.degrees(self.formation_heading) % 360.0, 1),
            "target_seq": self.current_seq,
            "propagating_drones": propagating_count,
            "consensus_reached": (propagating_count == 0),
        }
