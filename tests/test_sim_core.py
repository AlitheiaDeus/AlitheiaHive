"""
Automated Unit Tests for SwarmTech3_UpRising Simulation Core (sim_core.py)
Tests 4-Zone physics, Zone 3 momentum damping, tether dynamics & snap,
10 Hz Baud & LKP recovery, autonomic succession, and formations.
"""

import unittest
import numpy as np
import json

from SIM.backend.sitl_engine import (
    SwarmSimulator,
    DT,
    ZONE4_CLASH,
    ZONE3_OUTER,
    ZONE2_OUTER,
    RADIO_RANGE,
    LINK_TIMEOUT,
    TETHER_SNAP_DIST,
    TETHER_AVOID_DIST,
    TETHER_MAX_LEN,
    TIER_LEADER,
    TIER_ESCORT,
)
from OBC.backend.drone_firmware import Drone


class TestSimCore(unittest.TestCase):
    """Test suite verifying physics, safety zones, tether, comms, and succession."""

    def setUp(self):
        self.sim = SwarmSimulator(drone_count=6, enable_packet_loss=False, seed=42)

    def test_initialization(self):
        """Verify initial fleet composition and states."""
        self.assertEqual(len(self.sim.drones), 6)
        leader = self.sim.leader
        self.assertIsNotNone(leader)
        self.assertEqual(leader.id, 0)
        self.assertTrue(leader.is_leader)
        self.assertEqual(leader.tier, TIER_LEADER)
        self.assertFalse(self.sim.tether_severed)

        for drone in self.sim.drones[1:]:
            self.assertFalse(drone.is_leader)
            self.assertEqual(drone.tier, TIER_ESCORT)
            self.assertFalse(drone.crashed)
            self.assertFalse(drone.lost_link)

    def test_zone4_prop_clash(self):
        """Zone 4 (d <= 1.5m): Prop clash causes motor cut and ballistic crash."""
        d1 = self.sim.drones[1]
        d2 = self.sim.drones[2]

        # Place them at 1.2m separation (< 1.5m)
        d1.pos = np.array([10.0, 10.0, 20.0], dtype=float)
        d2.pos = np.array([10.0, 11.2, 20.0], dtype=float)

        self.sim.step()

        self.assertTrue(d1.crashed)
        self.assertTrue(d2.crashed)
        self.assertEqual(d1.status, "CRASHED")
        self.assertEqual(d2.status, "CRASHED")

    def test_zone3_relative_velocity_damping(self):
        """Zone 3 (1.5m < d <= 3.5m): Relative velocity damping prevents penetration."""
        d1 = self.sim.drones[1]
        d2 = self.sim.drones[2]

        # Position at 2.5m separation in Zone 3
        d1.pos = np.array([20.0, 0.0, 20.0], dtype=float)
        d2.pos = np.array([22.5, 0.0, 20.0], dtype=float)

        # Head-on closing velocities
        d1.vel = np.array([5.0, 0.0, 0.0], dtype=float)
        d2.vel = np.array([-5.0, 0.0, 0.0], dtype=float)

        avoidance = self.sim._compute_4zone_avoidance()
        acc_d1 = avoidance[d1.id]
        acc_d2 = avoidance[d2.id]

        # d1 should be strongly accelerated backward (-X)
        self.assertLess(acc_d1[0], -10.0)
        # d2 should be strongly accelerated forward (+X)
        self.assertGreater(acc_d2[0], 10.0)

        # Step forward 5 frames: verify relative velocity damping brakes closing speed
        initial_closing_speed = d1.vel[0] - d2.vel[0]  # 10.0 m/s
        for _ in range(5):
            self.sim.step()

        final_closing_speed = d1.vel[0] - d2.vel[0]
        self.assertLess(final_closing_speed, initial_closing_speed)
        # Ensure they did not clash
        self.assertFalse(d1.crashed)
        self.assertFalse(d2.crashed)

    def test_zone2_separation(self):
        """Zone 2 (3.5m < d <= 8.0m): Smooth linear separation repulsion."""
        d1 = self.sim.drones[1]
        d2 = self.sim.drones[2]

        # Separation = 5.0m (in Zone 2)
        d1.pos = np.array([30.0, 0.0, 20.0], dtype=float)
        d2.pos = np.array([35.0, 0.0, 20.0], dtype=float)
        d1.vel = np.zeros(3, dtype=float)
        d2.vel = np.zeros(3, dtype=float)

        avoidance = self.sim._compute_4zone_avoidance()
        # d1 should experience negative X push (away from d2)
        self.assertLess(avoidance[d1.id][0], 0.0)
        # d2 should experience positive X push (away from d1)
        self.assertGreater(avoidance[d2.id][0], 0.0)

    def test_zone1_nominal_clear(self):
        """Zone 1 (d > 8.0m): Nominal clear flight, zero repulsion force."""
        d1 = self.sim.drones[1]
        d2 = self.sim.drones[2]

        d1.pos = np.array([0.0, 0.0, 20.0], dtype=float)
        d2.pos = np.array([20.0, 0.0, 20.0], dtype=float)  # 20m > 8.0m

        avoidance = self.sim._compute_4zone_avoidance()
        np.testing.assert_allclose(avoidance[d1.id], [0.0, 0.0, 0.0], atol=1e-5)
        np.testing.assert_allclose(avoidance[d2.id], [0.0, 0.0, 0.0], atol=1e-5)

    def test_tether_avoidance(self):
        """Followers inside 3.0m tether envelope experience lateral repulsion."""
        leader = self.sim.leader
        leader.pos = np.array([0.0, 0.0, 50.0], dtype=float)
        # Tether segment is [0, 0, 1.5] to [0, 0, 50.0]

        follower = self.sim.drones[1]
        follower.pos = np.array([2.0, 0.0, 25.0], dtype=float)  # d_perp = 2.0m < 3.0m

        tether_accs = self.sim._compute_tether_avoidance()
        acc = tether_accs[follower.id]

        # Should push radially away in +X
        self.assertGreater(acc[0], 0.0)
        self.assertAlmostEqual(acc[1], 0.0, places=3)

    def test_tether_collision_snap_and_succession(self):
        """Follower penetrating d_perp <= 1.0m snaps tether and triggers succession."""
        leader = self.sim.leader
        old_leader_id = leader.id
        leader.pos = np.array([0.0, 0.0, 40.0], dtype=float)

        # Place follower right next to tether cable (0.5m away)
        follower = self.sim.drones[1]
        follower.pos = np.array([0.5, 0.0, 20.0], dtype=float)

        self.sim.step()

        self.assertTrue(self.sim.tether_severed)
        self.assertTrue(self.sim.drones[old_leader_id].tether_lost)
        # Autonomic succession should have elected a new leader
        new_leader = self.sim.leader
        self.assertIsNotNone(new_leader)
        self.assertNotEqual(new_leader.id, old_leader_id)
        self.assertTrue(new_leader.is_leader)

    def test_tether_tensile_snap_max_length(self):
        """Moving leader beyond 300m snaps cable under tensile limit."""
        leader = self.sim.leader
        leader.pos = np.array([0.0, 0.0, 310.0], dtype=float)  # > 300m

        self.sim.step()
        self.assertTrue(self.sim.tether_severed)

    def test_radio_baud_and_lkp_recovery(self):
        """10 Hz Baud & LKP: Follower loses link >120m, navigates to LKP, recovers on re-entry."""
        leader = self.sim.leader
        follower = self.sim.drones[1]

        # 1. Start within radio range (at 20m)
        leader.pos = np.array([0.0, 0.0, 20.0], dtype=float)
        follower.pos = np.array([10.0, 0.0, 20.0], dtype=float)

        # Run for 0.2s (12 frames) to receive at least one heartbeat
        for _ in range(12):
            self.sim.step()

        self.assertFalse(follower.lost_link)
        np.testing.assert_allclose(follower.lkp, follower.pos, atol=1.0)

        # 2. Teleport leader out of range (150m > 120m)
        recorded_lkp = np.copy(follower.lkp)
        leader.pos = np.array([150.0, 0.0, 20.0], dtype=float)

        # Advance 1.2 seconds (> 1.0s LINK_TIMEOUT = 72 ticks at 60 FPS)
        for _ in range(75):
            self.sim.step()

        self.assertTrue(follower.lost_link)
        self.assertEqual(follower.status, "LOST_LINK")

        # 3. Follower should steer toward its LKP
        # Distance to recorded LKP should decrease or remain near LKP
        dist_to_lkp = np.linalg.norm(follower.pos - recorded_lkp)
        self.assertLess(dist_to_lkp, 25.0)

        # 4. Bring leader back within range (< 120m)
        leader.pos = np.array([30.0, 0.0, 20.0], dtype=float)
        for _ in range(12):
            self.sim.step()

        self.assertFalse(follower.lost_link)
        self.assertEqual(follower.status, "ACTIVE")

    def test_autonomic_succession_leader_kill(self):
        """Killing the active leader promotes the highest-scoring follower."""
        # Initial leader is Drone #0
        self.assertEqual(self.sim.leader.id, 0)

        # Succession score = Tier * 1000 - ID
        # Living followers are #1, #2, #3, #4, #5 with Tier 2
        # Drone #1 has highest score: (2000 - 1) = 1999
        self.sim.kill_leader()

        self.assertTrue(self.sim.drones[0].crashed)
        self.assertTrue(self.sim.tether_severed)
        self.assertTrue(self.sim.drones[0].tether_lost)
        new_leader = self.sim.leader
        self.assertIsNotNone(new_leader)
        self.assertEqual(new_leader.id, 1)
        self.assertEqual(new_leader.status, "PROMOTED_LEADER")
        self.assertTrue(new_leader.is_leader)
        snapshot = self.sim.get_state_snapshot()
        self.assertTrue(snapshot["tether"]["severed"])
        self.assertFalse(snapshot["tether"]["visible"])

    def test_multi_mode_formations(self):
        """Verify dynamic switching between all 4 flight formation modes."""
        modes = [
            "ECHELON_ESCORT",
            "NATURAL_FLOCK",
            "ORBIT_VORTEX",
            "PERIMETER_PATROL",
        ]
        for mode in modes:
            self.sim.set_formation(mode)
            self.assertEqual(self.sim.formation_mode, mode)
            # Step 10 ticks to ensure no numerical singularities or NaN
            for _ in range(10):
                self.sim.step()
                for d in self.sim.drones:
                    self.assertFalse(np.isnan(d.pos).any())
                    self.assertFalse(np.isnan(d.vel).any())
                    self.assertFalse(np.isnan(d.acc).any())

    def test_state_snapshot_serialization(self):
        """Verify get_state_snapshot() is JSON serializable and contains required fields."""
        self.sim.step()
        snapshot = self.sim.get_state_snapshot()

        # Check JSON serialization
        json_str = json.dumps(snapshot)
        self.assertIsInstance(json_str, str)

        # Check fields
        self.assertIn("sim_time", snapshot)
        self.assertIn("dt", snapshot)
        self.assertIn("formation", snapshot)
        self.assertIn("target", snapshot)
        self.assertIn("tether", snapshot)
        self.assertIn("drones", snapshot)
        self.assertEqual(len(snapshot["drones"]), 6)

    def test_reboot_all_dead(self):
        """Verify reboot_all_dead revives crashed drones without resetting survivors or clock."""
        # Advance simulation time
        for _ in range(20):
            self.sim.step()
        sim_time_before = self.sim.sim_time
        self.assertGreater(sim_time_before, 0.0)

        # Kill 2 escort drones (#2, #4)
        self.sim.kill_drone(2)
        self.sim.kill_drone(4)
        self.assertTrue(self.sim.drones[2].crashed)
        self.assertTrue(self.sim.drones[4].crashed)
        self.assertFalse(self.sim.drones[0].crashed)
        self.assertFalse(self.sim.drones[1].crashed)

        # Position of Drone #0 (surviving leader) before reboot
        leader_pos_before = np.copy(self.sim.drones[0].pos)

        # Execute reboot_all_dead
        revived = self.sim.reboot_all_dead()
        self.assertEqual(sorted(revived), [2, 4])

        # Dead drones should now be active and not crashed
        self.assertFalse(self.sim.drones[2].crashed)
        self.assertFalse(self.sim.drones[4].crashed)
        self.assertEqual(self.sim.drones[2].status, "ACTIVE")
        self.assertEqual(self.sim.drones[4].status, "ACTIVE")

        # Living drones remain undisturbed
        np.testing.assert_allclose(self.sim.drones[0].pos, leader_pos_before)
        self.assertFalse(self.sim.drones[0].crashed)
        self.assertTrue(self.sim.drones[0].is_leader)

        # Sim clock was NOT reset
        self.assertEqual(self.sim.sim_time, sim_time_before)

        # Step sim forward to ensure renewed flight without error
        for _ in range(10):
            self.sim.step()
            for d in self.sim.drones:
                self.assertFalse(np.isnan(d.pos).any())

    def test_launch_reinforcement_sequential_id(self):
        """Verify launch_reinforcement creates sequential drone IDs with safe pad clearance."""
        initial_count = len(self.sim.drones)
        self.assertEqual(initial_count, 6)

        # Launch Drone #6
        drone6 = self.sim.launch_reinforcement()
        self.assertEqual(drone6.id, 6)
        self.assertEqual(drone6.flight_state, "TAKEOFF")
        self.assertEqual(drone6.pos[2], 0.5)
        self.assertFalse(drone6.crashed)
        self.assertEqual(len(self.sim.drones), 7)

        # Distance to tether anchor must be >= 3.0m
        dist_to_anchor = float(np.linalg.norm(drone6.pos[:2] - self.sim.tether_anchor[:2]))
        self.assertGreaterEqual(dist_to_anchor, 3.0)

        # Launch Drone #7
        drone7 = self.sim.launch_reinforcement()
        self.assertEqual(drone7.id, 7)
        self.assertEqual(len(self.sim.drones), 8)

        # Pad separation between Drone #6 and Drone #7 must be >= 2.0m (no Zone 4 clash)
        pad_dist = float(np.linalg.norm(drone6.pos - drone7.pos))
        self.assertGreaterEqual(pad_dist, 2.0)

    def test_vertical_takeoff_flight_profile(self):
        """Verify 3-phase state machine: pure vertical climb -> en route -> cruise."""
        drone = self.sim.launch_reinforcement()
        initial_xy = np.copy(drone.pos[:2])

        # Step 30 frames (0.5s) while ascending
        for _ in range(30):
            self.sim.step()

        # Altitude must have increased
        self.assertGreater(drone.pos[2], 1.0)
        # Horizontal drift during low altitude must remain negligible (< 0.2m)
        drift_xy = float(np.linalg.norm(drone.pos[:2] - initial_xy))
        self.assertLess(drift_xy, 0.2)
        # Vertical velocity must be positive climbing
        self.assertGreater(drone.vel[2], 0.0)

        # Step forward until altitude clears 20.0m
        steps = 0
        while drone.pos[2] < 20.0 and steps < 600:
            self.sim.step()
            steps += 1

        self.assertGreaterEqual(drone.pos[2], 20.0)
        # Must transition out of TAKEOFF to EN_ROUTE (or CRUISE)
        self.assertIn(drone.flight_state, ["EN_ROUTE", "CRUISE"])

    def test_reinforcement_takeover_when_all_dead(self):
        """Verify reinforcement ascends and assumes flagship leadership when fleet is dead."""
        # Kill all drones
        for d in self.sim.drones:
            self.sim.kill_drone(d.id)
        self.assertIsNone(self.sim.leader)

        # Launch reinforcement drone #8
        drone = self.sim.launch_reinforcement()
        self.assertFalse(drone.is_leader)

        # Climb to 20m
        steps = 0
        while drone.pos[2] < 20.0 and steps < 600:
            self.sim.step()
            steps += 1

        # Must be promoted to leader upon clearing transition altitude
        self.assertTrue(drone.is_leader)
        self.assertEqual(drone.status, "FLEET_LEADER")
        self.assertEqual(self.sim.leader.id, drone.id)

    def test_clear_crashed_wreckage(self):
        """Verify battlefield salvage purges destroyed drones and preserves monotonic IDs."""
        # 1. No dead drones: returns empty list
        purged = self.sim.clear_crashed_wreckage()
        self.assertEqual(purged, [])
        self.assertEqual(len(self.sim.drones), 6)

        # 2. Kill 2 drones (#2 and #4)
        self.sim.kill_drone(2)
        self.sim.kill_drone(4)
        self.assertTrue(self.sim.drones[2].crashed)
        self.assertTrue(self.sim.drones[4].crashed)

        # 3. Clear wreckage
        purged = self.sim.clear_crashed_wreckage()
        self.assertEqual(sorted(purged), [2, 4])
        self.assertEqual(len(self.sim.drones), 4)

        remaining_ids = [d.id for d in self.sim.drones]
        self.assertEqual(remaining_ids, [0, 1, 3, 5])
        for d in self.sim.drones:
            self.assertFalse(d.crashed)

        # 4. Monotonic ID generation after wreckage purge (no reuse of 2 or 4)
        new_drone = self.sim.launch_reinforcement()
        self.assertEqual(new_drone.id, 6)
        self.assertEqual(len(self.sim.drones), 5)

        # 5. Clear all dead drones and ensure sim handles empty fleet safely
        for d in list(self.sim.drones):
            self.sim.kill_drone(d.id)
        self.assertEqual(len([d for d in self.sim.drones if d.crashed]), 5)

        all_purged = self.sim.clear_crashed_wreckage()
        self.assertEqual(len(all_purged), 5)
        self.assertEqual(len(self.sim.drones), 0)
        self.assertIsNone(self.sim.leader)

        # Launching reinforcement into cleared battlefield
        survivor = self.sim.launch_reinforcement()
        self.assertEqual(survivor.id, 7)
        self.assertEqual(len(self.sim.drones), 1)

    def test_target_auto_patrol_loop(self):
        """Verify dynamic 3D target auto-patrol smoothly updates target position."""
        self.assertFalse(self.sim.patrol_active)

        # Engage auto-patrol
        active = self.sim.toggle_target_patrol(True)
        self.assertTrue(active)
        self.assertTrue(self.sim.patrol_active)

        # Step 60 ticks (1.0s)
        initial_target = np.copy(self.sim.target_pos)
        for _ in range(60):
            self.sim.step()

        # Target must have moved continuously
        dist_moved = float(np.linalg.norm(self.sim.target_pos - initial_target))
        self.assertGreater(dist_moved, 0.5)

        # Altitude must remain safe (Z >= 15.0m)
        self.assertGreaterEqual(self.sim.target_pos[2], 15.0)

        # Disengage auto-patrol
        active = self.sim.toggle_target_patrol(False)
        self.assertFalse(active)
        self.assertFalse(self.sim.patrol_active)

        # Target should now remain fixed on step
        frozen_target = np.copy(self.sim.target_pos)
        for _ in range(30):
            self.sim.step()
        np.testing.assert_allclose(self.sim.target_pos, frozen_target)

    def test_uplink_mailbox_tether_severance_drops_packets(self):
        """Severed tether drops GCS target uplinks; drone acknowledged target is not modified (TKT-25)."""
        sim = SwarmSimulator(drone_count=6)
        initial_ack = np.copy(sim.leader.acknowledged_target)

        # Sever tether
        sim.cut_tether()
        self.assertTrue(sim.tether_severed)

        # Attempt to command new target waypoint from GCS
        res = sim.set_target([100.0, 100.0, 30.0])
        self.assertEqual(res.get("status"), "DROPPED")
        self.assertEqual(res.get("reason"), "TETHER_SEVERED")

        # Commanded target at GCS was updated
        np.testing.assert_allclose(sim.target_pos, [100.0, 100.0, 30.0])

        # But living successor leader's onboard target buffer remained untouched
        np.testing.assert_allclose(sim.leader.acknowledged_target, initial_ack)

    def test_auto_patrol_severance_prevents_2ic_dead_wire_chase(self):
        """Severing tether during auto-patrol prevents 2IC from chasing moving patrol target (TKT-25)."""
        sim = SwarmSimulator(drone_count=6)
        sim.toggle_target_patrol(True)
        self.assertTrue(sim.patrol_active)

        # Step 30 ticks to begin patrol transit
        for _ in range(30):
            sim.step()

        # Sever tether
        sim.cut_tether()
        self.assertTrue(sim.tether_severed)

        # Dynamic 2IC is now leader
        successor = sim.leader
        self.assertIsNotNone(successor)
        promoted_loiter_target = np.copy(successor.acknowledged_target)

        # Step 120 ticks (2.0s) with auto-patrol running in GCS
        for _ in range(120):
            sim.step()

        # Commanded target at GCS continues moving along figure-8
        dist_gcs_moved = float(np.linalg.norm(sim.target_pos - promoted_loiter_target))
        self.assertGreater(dist_gcs_moved, 1.0, "GCS commanded target should advance along patrol route")

        # But 2IC's onboard target buffer MUST remain frozen at the cut loiter target!
        np.testing.assert_allclose(
            successor.acknowledged_target,
            promoted_loiter_target,
            err_msg="2IC must not chase moving GCS patrol target across severed tether!",
        )

        # All living drones must maintain safe separation (0 prop clashes)
        living = [d for d in sim.drones if not d.crashed]
        for i in range(len(living)):
            for j in range(i + 1, len(living)):
                d = float(np.linalg.norm(living[i].pos - living[j].pos))
                self.assertGreater(d, 1.5, "No prop clashes permitted during post-severance loiter!")

    def test_uplink_mailbox_radio_range_gating(self):
        """In LEADER_FOLLOWER, target uplink packets are dropped if leader exceeds 140m GCS Howl range (TKT-25/49)."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_comms_mode("LEADER_FOLLOWER")

        # Move leader beyond 140m GCS radio range
        sim.leader.pos = np.array([150.0, 0.0, 25.0], dtype=float)
        initial_ack = np.copy(sim.leader.acknowledged_target)

        # Attempt uplink dispatch
        res = sim.set_target([80.0, 80.0, 30.0])
        self.assertEqual(res.get("status"), "OUT_OF_RANGE")
        # Onboard target buffer remains untouched
        np.testing.assert_allclose(sim.leader.acknowledged_target, initial_ack)

        # Move leader back within 120m range
        sim.leader.pos = np.array([60.0, 0.0, 25.0], dtype=float)
        res = sim.set_target([80.0, 80.0, 30.0])
        self.assertEqual(res.get("status"), "OK")
        np.testing.assert_allclose(sim.leader.acknowledged_target, [80.0, 80.0, 30.0])


if __name__ == "__main__":
    unittest.main()

