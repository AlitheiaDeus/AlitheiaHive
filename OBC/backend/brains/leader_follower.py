"""
SwarmTech3_UpRising: Leader-Follower Guidance Brain (leader_follower.py)
Implements tethered flight behind a fleet leader, 10 Hz star-topology comms,
timeout-based Lost-Link state machine with LKP navigation, and autonomic succession.
"""

from __future__ import annotations
import math
from typing import Dict, List, Optional, Any, TYPE_CHECKING
import numpy as np

from .base import BaseBrain
from .hungarian import hungarian_assignment

if TYPE_CHECKING:
    from SIM.backend.sitl_engine import SwarmSimulator
    from OBC.backend.drone_firmware import Drone

RADIO_RANGE: float = 120.0
HEARTBEAT_INTERVAL: float = 0.1
LINK_TIMEOUT: float = 1.0
V_CRUISE: float = 12.0
V_LKP: float = 8.0
TIER_LEADER: int = 3
TIER_ESCORT: int = 2

# Echelon Anti-Jitter & Heading Slew Parameters
FREEZE_SPEED: float = 1.5        # m/s: leader speeds below this freeze echelon heading
FREEZE_DIST: float = 8.0         # meters: leader distance to target below this freezes echelon heading
DEADBAND_RADIUS: float = 0.25    # meters: arrival deadband stops micro-hunting/jitter
MAX_SLEW_RATE: float = math.radians(45.0)  # 45 deg/s = ~0.7854 rad/s slew limit


class LeaderFollowerBrain(BaseBrain):
    """Classic tactical swarm brain: Followers form up behind an active Leader.
    Comms are transmitted at 10 Hz over RF. Drones losing contact navigate
    autonomously to their Last Known Position (LKP)."""

    brain_name: str = "LEADER_FOLLOWER"

    def __init__(self, sim: SwarmSimulator):
        super().__init__(sim)
        self.last_heartbeat_broadcast: float = -1.0
        self.formation_mode: str = "ECHELON_ESCORT"  # ECHELON_ESCORT, NATURAL_FLOCK, ORBIT_VORTEX, PERIMETER_PATROL
        self.formation_heading: float = 0.0
        self.heading_initialized: bool = False
        self.assigned_slots: Dict[int, int] = {}
        self.inversion_cooldown: float = 0.0
        self.route_paused: bool = False

    def reset(self) -> None:
        self.last_heartbeat_broadcast = -1.0
        self.formation_heading = 0.0
        self.heading_initialized = False
        self.assigned_slots.clear()
        self.inversion_cooldown = 0.0
        self.route_paused = False

    def on_succession(self, old_leader: Optional[Drone], new_leader: Drone) -> None:
        """Deconflict slot assignments during leadership transitions (TKT-15).
        Transfers the promoted leader's vacated slot to the demoted leader
        so existing followers maintain wing affinity without crossing."""
        if new_leader.id in self.assigned_slots:
            vacated_slot = self.assigned_slots.pop(new_leader.id)
            if old_leader is not None and not old_leader.crashed:
                self.assigned_slots[old_leader.id] = vacated_slot

    # ==========================================================================
    # 1. Radio Comms & Link Budget Handling
    # ==========================================================================
    def update_comms(self, dt: float) -> None:
        """Physical RF propagation, asymmetric link budget & gateway relays are
        maintained by SITL World Engine _update_radio_comms() across all modes (TKT-49)."""
        pass

    # ==========================================================================
    # 2. Guidance Target Computation
    # ==========================================================================
    def compute_guidance(self, dt: float) -> Dict[int, np.ndarray]:
        accs = {d.id: np.zeros(3, dtype=float) for d in self.sim.drones}
        current_leader = self.sim.leader

        # 0. Smooth and freeze echelon heading based on leader kinematics
        if current_leader is not None and not current_leader.crashed:
            self._update_formation_heading(current_leader, dt)

        for drone in self.sim.drones:
            if drone.crashed or drone.flight_state in ("LANDED", "DESCENDING"):
                continue

            # 0b. Tactical GNSS Denial & Dead-Reckoning RTB Override (TKT-52)
            if getattr(drone, "gnss_denied", False):
                accs[drone.id] = self.compute_gnss_denied_guidance(drone, dt)
                continue

            # 1. Leader guidance: steer toward global mission target
            if drone.is_leader:
                if drone.flight_state == "BACKTRACK_HOLD" and getattr(drone, "backtrack_target", None) is not None:
                    drone.target_station = drone.backtrack_target
                    accs[drone.id] = self._steer_toward(
                        drone.pos, drone.vel, drone.backtrack_target, max_speed=8.0
                    )
                    continue
                elif drone.flight_state == "PERIMETER_HOLD" or getattr(drone, "holding_at_horizon", False):
                    drone.target_station = drone.acknowledged_target
                    accs[drone.id] = self._steer_toward(
                        drone.pos, drone.vel, drone.acknowledged_target, max_speed=2.0
                    )
                    continue
                elif drone.lost_link:
                    accs[drone.id] = self._steer_toward(
                        drone.pos, drone.vel, drone.lkp, max_speed=V_LKP
                    )
                    continue

                # Smooth acceleration ramp if newly promoted to give retiring leader corridor clearance
                if getattr(drone, "promotion_timer", 0.0) > 0.0:
                    drone.promotion_timer = max(0.0, drone.promotion_timer - dt)
                    ramp = 1.0 - (drone.promotion_timer / 1.5)
                    leader_speed = 7.0 + (V_CRUISE - 7.0) * ramp
                else:
                    leader_speed = V_CRUISE

                # Discrete Waypoint Mission Route Progression (TKT-28, TKT-30, TKT-56)
                if getattr(drone, "mission_active", False) and getattr(drone, "mission_waypoints", None):
                    is_paused = getattr(drone, "mission_paused", False) or getattr(self, "route_paused", False)
                    wps = drone.mission_waypoints
                    idx = drone.current_wp_idx
                    if 0 <= idx < len(wps):
                        raw_wp = wps[idx]
                        target_wp = np.array(raw_wp[:3], dtype=float)
                        loiter_duration = float(raw_wp[3]) if len(raw_wp) > 3 else 0.0
                        drone.acknowledged_target = target_wp
                        dist = float(np.linalg.norm(drone.pos - target_wp))

                        if is_paused:
                            # Hold station at current waypoint target without countdown
                            leader_speed = 2.0
                        elif dist <= self.sim.route_acceptance_radius:
                            # Check escorts formation settlement gate (<= 4.5m)
                            living_escorts = [
                                f for f in self.sim.drones
                                if not f.crashed and not f.is_leader and f.flight_state not in ("LANDED", "DESCENDING") and not f.lost_link and getattr(f, "enrolled", True)
                            ]
                            escorts_settled = all(
                                float(np.linalg.norm(f.pos - self._get_formation_slot(f, drone))) <= 4.5
                                for f in living_escorts
                            ) if living_escorts else True

                            if not getattr(drone, "wp_loitering", False) and loiter_duration > 0.0:
                                # Enter WAYPOINT_LOITER state and initialize countdown
                                drone.wp_loitering = True
                                drone.wp_loiter_timer = max(0.0, loiter_duration - dt)
                                drone.wp_total_loiter = loiter_duration
                            elif getattr(drone, "wp_loitering", False):
                                # Count down loiter timer
                                drone.wp_loiter_timer = max(0.0, drone.wp_loiter_timer - dt)

                                # Advance only if loiter timer elapsed AND formation is settled
                                if drone.wp_loiter_timer <= 0.0 and escorts_settled:
                                    drone.wp_loitering = False
                                    drone.wp_loiter_timer = 0.0
                                    if idx + 1 < len(wps):
                                        drone.current_wp_idx += 1
                                        next_wp = wps[drone.current_wp_idx]
                                        drone.acknowledged_target = np.array(next_wp[:3], dtype=float)
                                        drone.wp_total_loiter = float(next_wp[3]) if len(next_wp) > 3 else 0.0
                                    elif getattr(drone, "mission_loop", True):
                                        drone.current_wp_idx = 0
                                        drone.acknowledged_target = np.array(wps[0][:3], dtype=float)
                                        drone.wp_total_loiter = float(wps[0][3]) if len(wps[0]) > 3 else 0.0
                                    else:
                                        drone.mission_active = False
                            else:
                                # Zero-duration fly-through waypoint: advance immediately
                                drone.wp_loitering = False
                                drone.wp_loiter_timer = 0.0
                                if idx + 1 < len(wps):
                                    drone.current_wp_idx += 1
                                    next_wp = wps[drone.current_wp_idx]
                                    drone.acknowledged_target = np.array(next_wp[:3], dtype=float)
                                    drone.wp_total_loiter = float(next_wp[3]) if len(next_wp) > 3 else 0.0
                                elif getattr(drone, "mission_loop", True):
                                    drone.current_wp_idx = 0
                                    next_wp = wps[0]
                                    drone.acknowledged_target = np.array(next_wp[:3], dtype=float)
                                    drone.wp_total_loiter = float(next_wp[3]) if len(next_wp) > 3 else 0.0
                                else:
                                    drone.mission_active = False

                        # If loitering, hold station at target_wp with gentle braking speed
                        if getattr(drone, "wp_loitering", False):
                            leader_speed = 2.0

                target_dest = getattr(drone, "acknowledged_target", self.sim.target_pos)
                drone.target_station = target_dest
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, target_dest, max_speed=leader_speed
                )
                continue

            # 1b. Unenrolled solo scout guidance (TKT-50)
            if not getattr(drone, "enrolled", True):
                if drone.flight_state == "BACKTRACK_HOLD" and getattr(drone, "backtrack_target", None) is not None:
                    drone.target_station = drone.backtrack_target
                    accs[drone.id] = self._steer_toward(
                        drone.pos, drone.vel, drone.backtrack_target, max_speed=8.0
                    )
                    continue
                elif drone.flight_state == "PERIMETER_HOLD" or getattr(drone, "holding_at_horizon", False):
                    drone.target_station = drone.acknowledged_target
                    accs[drone.id] = self._steer_toward(
                        drone.pos, drone.vel, drone.acknowledged_target, max_speed=2.0
                    )
                    continue
                elif drone.lost_link:
                    drone.target_station = drone.lkp
                    accs[drone.id] = self._steer_toward(
                        drone.pos, drone.vel, drone.lkp, max_speed=V_LKP
                    )
                elif drone.rejoin_target is not None:
                    drone.target_station = drone.rejoin_target
                    accs[drone.id] = self._steer_toward(
                        drone.pos, drone.vel, drone.rejoin_target, max_speed=V_CRUISE
                    )
                else:
                    drone.target_station = drone.pos
                    accs[drone.id] = self._steer_toward(
                        drone.pos, drone.vel, drone.pos, max_speed=0.0
                    )
                continue

            # 2. Backtrack & Lost-link guidance
            if drone.flight_state == "BACKTRACK_HOLD" and getattr(drone, "backtrack_target", None) is not None:
                drone.target_station = drone.backtrack_target
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, drone.backtrack_target, max_speed=8.0
                )
                continue
            elif drone.flight_state == "PERIMETER_HOLD" or getattr(drone, "holding_at_horizon", False):
                drone.target_station = drone.acknowledged_target
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, drone.acknowledged_target, max_speed=2.0
                )
                continue

            if drone.lost_link:
                drone.target_station = drone.lkp
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, drone.lkp, max_speed=V_LKP
                )
                continue

            # 2b. Breakaway Dive & Cable Jettison Guidance (TKT-16 / TKT-27)
            if drone.status == "BREAKAWAY_JETTISON" and not getattr(drone, "tether_jettisoned", False):
                # Vertical dive corridor: step down 4.0m below formation plane with strict safety floor
                lead_z = current_leader.pos[2] if current_leader is not None else drone.pos[2]
                z_floor = max(3.0, lead_z - 4.5)
                target_z = max(z_floor, lead_z - 4.0)

                # Smoothly cushion sink rate near safety floor to prevent inertia overshoot
                dist_to_floor = max(0.0, drone.pos[2] - z_floor)
                max_sink = min(6.5, max(0.8, dist_to_floor * 2.5))
                desired_vz = float(np.clip(3.5 * (target_z - drone.pos[2]), -max_sink, 2.0))
                a_dive_z = float(np.clip(5.0 * (desired_vz - drone.vel[2]), -18.0, 15.0))

                # Lateral push away from formation
                heading = np.array([math.cos(self.formation_heading), math.sin(self.formation_heading), 0.0], dtype=float)
                peel_dir = getattr(drone, "peel_off_dir", np.zeros(3, dtype=float))
                if float(np.linalg.norm(peel_dir)) < 0.01:
                    peel_dir = np.array([-heading[1], heading[0], 0.0], dtype=float)
                v_peel_target = peel_dir * 6.0
                a_peel_lat = (v_peel_target - drone.vel) * 2.5

                # Longitudinal forward speed braking
                v_long = float(np.dot(drone.vel, heading))
                a_brake_long = -v_long * 2.5 * heading

                accs[drone.id] = np.array([
                    a_peel_lat[0] + a_brake_long[0],
                    a_peel_lat[1] + a_brake_long[1],
                    a_dive_z
                ], dtype=float)
                continue

            # 2c. Tether Failsafe Policy: Autonomous RTB override if configured
            if getattr(drone, "tether_failsafe_policy", "REJOIN") == "RTB" and getattr(drone, "tether_lost", False):
                accs[drone.id] = self._steer_toward(
                    drone.pos, drone.vel, np.array([0.0, 0.0, 25.0], dtype=float), max_speed=V_CRUISE
                )
                continue

            # 3. Lateral Peel-Off Corridor Guidance (Anti-Jousting)
            if getattr(drone, "peel_off_timer", 0.0) > 0.0 and current_leader is not None:
                drone.peel_off_timer = max(0.0, drone.peel_off_timer - dt)
                heading = np.array([math.cos(self.formation_heading), math.sin(self.formation_heading), 0.0], dtype=float)
                rel_to_leader = drone.pos - current_leader.pos
                long_dist = float(np.dot(rel_to_leader, heading))

                # Once drone has passed longitudinally behind the new leader, the jousting risk is over
                if long_dist < -2.0:
                    drone.peel_off_timer = 0.0
                else:
                    # Lateral yield push away from flight corridor
                    v_peel_target = drone.peel_off_dir * 8.0
                    a_peel_lat = (v_peel_target - drone.vel) * 3.0

                    # Vertical corridor separation (step down 1.5m below leader)
                    target_z = max(1.0, current_leader.pos[2] - 1.5)
                    z_err = target_z - drone.pos[2]
                    a_peel_z = np.clip(z_err * 3.0 - drone.vel[2] * 2.0, -8.0, 8.0)

                    # Longitudinal forward speed damping so leader passes cleanly
                    v_long = float(np.dot(drone.vel, heading))
                    a_peel_long = -v_long * 2.0 * heading

                    accs[drone.id] = np.array([
                        a_peel_lat[0] + a_peel_long[0],
                        a_peel_lat[1] + a_peel_long[1],
                        a_peel_z
                    ], dtype=float)
                    continue

            # 4. Normal active follower formation guidance
            if current_leader is not None:
                if self.formation_mode == "NATURAL_FLOCK":
                    heading = np.array([math.cos(self.formation_heading), math.sin(self.formation_heading), 0.0], dtype=float)
                    drone.target_station = current_leader.pos - heading * 8.0
                    accs[drone.id] = self._compute_boid_acceleration(drone, current_leader)
                else:
                    slot_pos = self._get_formation_slot(drone, current_leader)
                    drone.target_station = slot_pos
                    target_speed = V_CRUISE
                    if getattr(drone, "enrollment_timer", 0.0) > 0.0:
                        ramp = max(0.0, min(1.0, 1.0 - (drone.enrollment_timer / 2.5)))
                        target_speed = 7.0 + (V_CRUISE - 7.0) * ramp
                    accs[drone.id] = self._steer_toward(
                        drone.pos, drone.vel, slot_pos, max_speed=target_speed
                    )
            else:
                accs[drone.id] = -drone.vel * 2.0

        # 5. Tactical Formation Orientations & Headings (TKT-55)
        for drone in self.sim.drones:
            if drone.crashed or drone.flight_state in ("LANDED", "DESCENDING"):
                continue

            speed_xy = math.hypot(drone.vel[0], drone.vel[1])

            # Failsafe modes face velocity heading
            if getattr(drone, "gnss_denied", False) or drone.lost_link or drone.flight_state == "BACKTRACK_HOLD":
                if speed_xy > 0.5:
                    drone.target_yaw = math.atan2(drone.vel[1], drone.vel[0])
                continue

            if self.formation_mode in ("PERIMETER_PATROL", "PERIMETER_IN", "PERIMETER_OUT"):
                center = (
                    current_leader.acknowledged_target
                    if current_leader is not None and hasattr(current_leader, "acknowledged_target") and current_leader.acknowledged_target is not None
                    else self.sim.target_pos
                )
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

    def _update_formation_heading(self, leader: Drone, dt: float) -> None:
        """Update and smooth echelon formation heading with anti-jitter freeze
        and bounded slew rate filter (<= 45 deg/s)."""
        speed_xy = math.hypot(leader.vel[0], leader.vel[1])
        leader_target = getattr(leader, "acknowledged_target", self.sim.target_pos)
        to_target = leader_target - leader.pos
        dist_to_target = float(np.linalg.norm(to_target))

        # Initial bootstrap if heading has not been established yet
        if not self.heading_initialized:
            if speed_xy >= FREEZE_SPEED:
                self.formation_heading = math.atan2(leader.vel[1], leader.vel[0])
                self.heading_initialized = True
            else:
                dist_xy = math.hypot(to_target[0], to_target[1])
                if dist_xy > 1.0:
                    self.formation_heading = math.atan2(to_target[1], to_target[0])
                else:
                    self.formation_heading = float(leader.yaw)
                self.heading_initialized = True
            return

        self.inversion_cooldown = max(0.0, self.inversion_cooldown - dt)

        # Check if active mission target indicates a sharp reversal (> 120 deg) (TKT-43)
        dist_xy = math.hypot(to_target[0], to_target[1])
        if dist_xy > 5.0 and self.inversion_cooldown <= 0.0 and self.formation_mode == "ECHELON_ESCORT":
            to_tgt_dir = np.array([to_target[0] / dist_xy, to_target[1] / dist_xy])
            h_dir = np.array([math.cos(self.formation_heading), math.sin(self.formation_heading)])
            if float(np.dot(to_tgt_dir, h_dir)) < -0.5:
                rev_heading = math.atan2(to_target[1], to_target[0])
                if self.invert_formation_slots(rev_heading):
                    self.formation_heading = rev_heading
                    return

        patrol_active = getattr(self.sim, "patrol_active", False)

        # Anti-jitter freeze: when leader decelerates/reverses near stationary target or slows down
        if not patrol_active and (speed_xy < FREEZE_SPEED or dist_to_target < FREEZE_DIST):
            # Freeze heading at last stable cruising angle
            return

        # Cruising: target heading follows instantaneous velocity vector,
        # or points towards target beacon if leader slows down in transit
        if speed_xy >= FREEZE_SPEED:
            target_heading = math.atan2(leader.vel[1], leader.vel[0])
        elif dist_to_target > 1.0:
            target_heading = math.atan2(to_target[1], to_target[0])
        else:
            return

        # Apply bounded slew filter (<= 45 deg/s in standard cruise, <= 60 deg/s in patrol pursuit)
        effective_slew_rate = math.radians(60.0) if patrol_active else MAX_SLEW_RATE
        yaw_diff = (target_heading - self.formation_heading + math.pi) % (2.0 * math.pi) - math.pi

        # Dynamic Hungarian Formation Inversion on sharp heading reversal (TKT-43)
        if (
            abs(yaw_diff) > math.radians(120.0)
            and self.inversion_cooldown <= 0.0
            and self.formation_mode == "ECHELON_ESCORT"
            and not getattr(self.sim, "overhead_marshal_active", False)
        ):
            if self.invert_formation_slots(target_heading):
                self.formation_heading = target_heading
                return

        max_step = effective_slew_rate * dt
        slew = float(np.clip(yaw_diff, -max_step, max_step))
        self.formation_heading = (self.formation_heading + slew + math.pi) % (2.0 * math.pi) - math.pi

    def invert_formation_slots(self, target_heading: float) -> bool:
        """Perform Hungarian minimal-displacement slot re-assignment on heading reversal (TKT-43).
        Matches current follower positions to candidate slot positions in the new heading,
        minimizing total transit distance and eliminating cross-flight prop clash hazards."""
        current_leader = self.sim.leader
        if current_leader is None or current_leader.crashed:
            return False

        living_followers = [
            d for d in self.sim.drones
            if not d.crashed and not d.is_leader and d.flight_state not in ("LANDED", "DESCENDING") and getattr(d, "enrolled", True)
        ]
        n = len(living_followers)
        if n <= 1:
            return False

        # Compute candidate slot positions for new target heading
        heading_vec = np.array([math.cos(target_heading), math.sin(target_heading), 0.0], dtype=float)
        lateral_vec = np.array([-heading_vec[1], heading_vec[0], 0.0], dtype=float)

        candidate_slots = []
        for k in range(1, n + 1):
            row = (k + 1) // 2
            side = 1.0 if (k % 2 == 1) else -1.0
            slot_pos = (
                current_leader.pos
                - (row * 6.0 * heading_vec)
                + (side * row * 6.0 * lateral_vec)
                + np.array([0.0, 0.0, row * 0.5], dtype=float)
            )
            candidate_slots.append(slot_pos)

        # Construct cost matrix: squared Euclidean distance
        cost_matrix = np.zeros((n, n), dtype=float)
        for i, drone in enumerate(living_followers):
            for j, s_pos in enumerate(candidate_slots):
                diff = drone.pos - s_pos
                cost_matrix[i, j] = float(np.dot(diff, diff))

        # Solve optimal bipartite matching
        row_ind, col_ind = hungarian_assignment(cost_matrix)

        # Assign new deconflicted slots (k = col_idx + 1)
        for r, c in zip(row_ind, col_ind):
            drone = living_followers[r]
            self.assigned_slots[drone.id] = c + 1

        self.inversion_cooldown = 2.5
        return True

    def _get_formation_slot(self, drone: Drone, leader: Drone) -> np.ndarray:
        living_followers = [
            d for d in self.sim.drones
            if not d.crashed and not d.is_leader and d.flight_state not in ("LANDED", "DESCENDING") and getattr(d, "enrolled", True)
        ]
        living_ids = {d.id for d in living_followers}

        # Clean up dead, promoted, or landed drones from assigned slots
        for did in list(self.assigned_slots.keys()):
            if did not in living_ids:
                del self.assigned_slots[did]

        # Retrieve or assign persistent deconfliction slot
        if drone.id in self.assigned_slots:
            k = self.assigned_slots[drone.id]
        else:
            used = set(self.assigned_slots.values())
            try:
                suggested_k = living_followers.index(drone) + 1
            except ValueError:
                suggested_k = 1
            if suggested_k not in used:
                k = suggested_k
            else:
                k = 1
                while k in used:
                    k += 1
            self.assigned_slots[drone.id] = k

        # Stage 1 Overhead Marshal: Equidistant holding ring around base anchor (0, 0, 25m)
        if getattr(self.sim, "overhead_marshal_active", False):
            radius = 20.0
            k_marshal = living_followers.index(drone) + 1
            angle = (2.0 * math.pi * (k_marshal - 1)) / max(1, len(living_followers))
            return np.array([radius * math.cos(angle), radius * math.sin(angle), 25.0], dtype=float)

        if self.formation_mode == "ECHELON_ESCORT":
            heading = np.array(
                [math.cos(self.formation_heading), math.sin(self.formation_heading), 0.0]
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
            radius = 14.0
            omega = 0.8
            angle = omega * self.sim.sim_time + (2.0 * math.pi * (k - 1) / max(1, len(living_followers)))
            z_offset = 3.0 * math.sin(2.0 * angle)
            return (
                leader.pos
                + np.array([radius * math.cos(angle), radius * math.sin(angle), z_offset])
            )

        elif self.formation_mode in ("PERIMETER_PATROL", "PERIMETER_IN", "PERIMETER_OUT"):
            # Static defensive sentry ring holding equidistant guard stations around target (TKT-55)
            radius = 25.0
            angle = (2.0 * math.pi * (k - 1)) / max(1, len(living_followers))
            center = (
                leader.acknowledged_target
                if hasattr(leader, "acknowledged_target")
                else self.sim.target_pos
            )
            return (
                center
                + np.array([radius * math.cos(angle), radius * math.sin(angle), 0.0])
            )

        else:
            # NATURAL_FLOCK representative station for external inspection
            row = (k + 1) // 2
            side = 1.0 if (k % 2 == 1) else -1.0
            return leader.pos + np.array([-row * 6.0, side * 6.0, 0.0])

    def _compute_boid_acceleration(self, drone: Drone, leader: Drone) -> np.ndarray:
        """Compute true 3D Reynolds boid flocking acceleration (TKT-23).
        Forces:
        1. Separation: Inverse-square 3D repulsion (d < 5.0m) + Zone 3 relative-velocity brake (d < 3.5m).
        2. Alignment: Velocity matching with neighbors and leader (d < 14.0m).
        3. Cohesion: Attraction toward local flock center of mass (d < 20.0m).
        4. Goal Migration: Follow the leader's station trailing cushion.
        5. Vertical Z-Space Staggering & Safety Floor (Z >= 10.0m).
        6. Tether Clearance: Minimum 3.0m radial clearance around tether umbilical.
        """
        active_flock = [
            d for d in self.sim.drones
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING")
        ]

        # 1. Goal Migration: Dispersed trailing envelope so boids do not converge onto a single point
        heading = np.array([math.cos(self.formation_heading), math.sin(self.formation_heading), 0.0], dtype=float)
        lateral = np.array([-heading[1], heading[0], 0.0], dtype=float)
        k = getattr(drone, "id", 1)
        row = (k + 1) // 2
        side = 1.0 if (k % 2 == 1) else -1.0
        z_offset = (row * 0.8) if (side > 0) else (-row * 0.6)
        target_station = (
            leader.pos
            - (heading * (6.0 + row * 4.0))
            + (lateral * (side * row * 4.5))
            + np.array([0.0, 0.0, z_offset], dtype=float)
        )
        a_goal = self._steer_toward(drone.pos, drone.vel, target_station, max_speed=V_CRUISE, arrival_radius=8.0)

        # 2. Separation (d < 5.0m)
        a_sep = np.zeros(3, dtype=float)
        R_SEP = 5.0
        for peer in active_flock:
            if peer.id == drone.id:
                continue
            diff = drone.pos - peer.pos
            d = float(np.linalg.norm(diff))
            if 0.001 < d < R_SEP:
                rep_strength = (R_SEP - d) / (d + 0.1)
                a_sep += (diff / d) * rep_strength * 6.0

                # 3D Z-Space staggering: if horizontally close, push vertically apart to break 2D planes
                d_xy = float(math.hypot(diff[0], diff[1]))
                if d_xy < 3.5:
                    z_sign = 1.0 if diff[2] >= 0.0 else -1.0
                    a_sep[2] += z_sign * (3.5 - d_xy) * 4.0

                # Zone 3 relative-velocity approach damping (brake incoming momentum)
                if d < 3.5:
                    v_rel = drone.vel - peer.vel
                    closing = float(np.dot(v_rel, diff / d))
                    if closing < 0:
                        a_sep += -(diff / d) * closing * 3.5

        # 3. Alignment (d < 14.0m) - velocity matching with flockmates
        a_align = np.zeros(3, dtype=float)
        R_ALIGN = 14.0
        align_weight_sum = 0.0
        vel_sum = np.zeros(3, dtype=float)
        for peer in active_flock:
            if peer.id == drone.id:
                continue
            d = float(np.linalg.norm(drone.pos - peer.pos))
            if d < R_ALIGN:
                w = 1.0 - (d / R_ALIGN)
                if peer.is_leader:
                    w *= 2.0
                vel_sum += peer.vel * w
                align_weight_sum += w

        if align_weight_sum > 0:
            mean_vel = vel_sum / align_weight_sum
            a_align = (mean_vel - drone.vel) * 1.5

        # 4. Cohesion (d < 20.0m) - group centroid pull
        a_coh = np.zeros(3, dtype=float)
        R_COH = 20.0
        pos_sum = np.zeros(3, dtype=float)
        coh_count = 0
        for peer in active_flock:
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
            and not drone.is_leader
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
    # 3. Brain Command Hook
    # ==========================================================================
    def handle_command(self, cmd_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        cmd = cmd_dict.get("cmd", "").upper()
        if cmd == "SET_FORMATION":
            mode = cmd_dict.get("mode", "ECHELON_ESCORT")
            mode_upper = mode.upper().replace("-", "_").strip()
            if mode_upper in ("BOIDS", "BOID", "FLOCK"):
                mode_upper = "NATURAL_FLOCK"
            elif mode_upper in ("PERIMETER_PATROL_IN", "PERIMETER_IN"):
                mode_upper = "PERIMETER_IN"
            elif mode_upper in ("PERIMETER_PATROL_OUT", "PERIMETER_OUT"):
                mode_upper = "PERIMETER_OUT"
            elif mode_upper == "PERIMETER":
                mode_upper = "PERIMETER_PATROL"

            valid = ["ECHELON_ESCORT", "NATURAL_FLOCK", "ORBIT_VORTEX", "PERIMETER_PATROL", "PERIMETER_IN", "PERIMETER_OUT"]
            if mode_upper in valid:
                self.formation_mode = mode_upper
                return {"status": "OK", "formation": self.formation_mode}

        elif cmd == "PAUSE_ROUTE":
            self.route_paused = True
            if self.sim:
                for d in self.sim.drones:
                    d.mission_paused = True
            return {"status": "OK", "action": "ROUTE_PAUSED"}

        elif cmd == "RESUME_ROUTE":
            self.route_paused = False
            if self.sim:
                for d in self.sim.drones:
                    d.mission_paused = False
            return {"status": "OK", "action": "ROUTE_RESUMED"}

        elif cmd == "SKIP_WAYPOINT":
            leader = getattr(self.sim, "leader", None)
            if leader and getattr(leader, "mission_waypoints", None):
                wps = leader.mission_waypoints
                idx = leader.current_wp_idx
                if idx + 1 < len(wps):
                    leader.current_wp_idx += 1
                elif getattr(leader, "mission_loop", True):
                    leader.current_wp_idx = 0
                else:
                    leader.mission_active = False
                new_idx = leader.current_wp_idx
                leader.acknowledged_target = np.array(wps[new_idx][:3], dtype=float)
                leader.wp_loitering = False
                leader.wp_loiter_timer = 0.0
                leader.wp_total_loiter = float(wps[new_idx][3]) if len(wps[new_idx]) > 3 else 0.0
                return {"status": "OK", "action": "WAYPOINT_SKIPPED", "current_wp_idx": new_idx}
            return {"status": "ERROR", "reason": "No active mission waypoints"}

        elif cmd in ("ABORT_ROUTE", "STOP_ROUTE"):
            self.route_paused = False
            if self.sim:
                for d in self.sim.drones:
                    d.mission_active = False
                    d.mission_paused = False
                    d.wp_loitering = False
                    d.mission_waypoints = []
            return {"status": "OK", "action": "ROUTE_ABORTED"}
        return None

    def get_telemetry_extra(self) -> Dict[str, Any]:
        return {
            "brain_mode": "LEADER_FOLLOWER",
            "formation": self.formation_mode,
            "formation_heading_deg": round(math.degrees(self.formation_heading) % 360.0, 1),
        }
