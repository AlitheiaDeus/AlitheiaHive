"""
Unit test suite for TKT-56: Discrete 4D Waypoint Queue & Loiter Progress Engine.
Validates 4D waypoint execution [x, y, z, loiter_s], dwell countdowns,
formation escort settlement gates, pause/resume/skip controls, and
telemetry snapshot streaming across both LeaderFollower and GossipConsensus brains.
"""

import unittest
import numpy as np

from SIM.backend.sitl_engine import SwarmSimulator
from tests.quarantine import quarantine_until


class TestWaypointEngine(unittest.TestCase):
    """Test suite validating discrete 4D waypoint queues, loiter countdowns, and settlement gates."""

    def test_4d_waypoint_queue_deployment(self):
        """Deploying a 4D route correctly loads coordinates and loiter duration into leader and GCS."""
        sim = SwarmSimulator(drone_count=4)
        wps_4d = [
            [20.0, 10.0, 20.0, 3.0],
            [30.0, 20.0, 25.0, 5.0],
            [10.0, 30.0, 22.0, 0.0],
        ]
        res = sim.deploy_mission_route(wps_4d, loop=True, name="ALPHA_4D")
        self.assertEqual(res.get("status"), "OK")
        self.assertTrue(sim.route_active)
        self.assertFalse(sim.route_paused)

        leader = sim.leader
        self.assertIsNotNone(leader)
        self.assertTrue(leader.mission_active)
        self.assertEqual(leader.current_wp_idx, 0)
        self.assertEqual(leader.wp_total_loiter, 3.0)
        np.testing.assert_allclose(leader.acknowledged_target, [20.0, 10.0, 20.0])

    def test_loiter_countdown_and_dwell(self):
        """Entering waypoint acceptance radius triggers loiter countdown and station hold."""
        sim = SwarmSimulator(drone_count=4)
        wps_4d = [
            [20.0, 0.0, 20.0, 2.0],  # 2.0s loiter
            [40.0, 0.0, 20.0, 0.0],
        ]
        sim.deploy_mission_route(wps_4d, loop=False, name="LOITER_TEST")
        leader = sim.leader

        # Position entire swarm at WP 0 so escorts are settled
        leader.pos = np.array([20.1, 0.1, 20.0], dtype=float)
        leader.flight_state = "CRUISE"
        brain = sim.brain
        for f in sim.drones:
            f.flight_state = "CRUISE"
            if not f.is_leader:
                slot = brain._get_formation_slot(f, leader)
                f.pos = np.copy(slot)
                f.vel = np.zeros(3)

        # First step: enters loiter state
        sim.step()
        self.assertTrue(leader.wp_loitering)
        self.assertAlmostEqual(leader.wp_total_loiter, 2.0, places=1)
        self.assertLess(leader.wp_loiter_timer, 2.0)
        self.assertEqual(leader.current_wp_idx, 0)

        # Step 60 frames (1.0s at dt=0.0166) -> loiter timer should be ~1.0s
        for _ in range(60):
            # Keep followers stationed at their slots
            for f in sim.drones:
                if not f.is_leader:
                    f.pos = np.copy(brain._get_formation_slot(f, leader))
                    f.vel = np.zeros(3)
            sim.step()

        self.assertTrue(leader.wp_loitering)
        self.assertAlmostEqual(leader.wp_loiter_timer, 1.0, delta=0.15)
        self.assertEqual(leader.current_wp_idx, 0)

    def test_escort_settlement_gate_holds_departure(self):
        """Leader holds departure at loiter station even after timer expires if escorts are not settled."""
        sim = SwarmSimulator(drone_count=3)
        wps_4d = [
            [20.0, 0.0, 20.0, 0.5],  # 0.5s loiter
            [50.0, 0.0, 25.0, 0.0],
        ]
        sim.deploy_mission_route(wps_4d, loop=False, name="SETTLEMENT_TEST")
        leader = sim.leader
        leader.pos = np.array([20.0, 0.0, 20.0], dtype=float)
        leader.flight_state = "CRUISE"
        for d in sim.drones:
            d.flight_state = "CRUISE"

        # Followers 1 and 2 are positioned far away (> 4.5m from slots, and deconflicted from each other)
        sim.drones[1].pos = np.array([0.0, -10.0, 20.0], dtype=float)
        sim.drones[2].pos = np.array([0.0, 10.0, 20.0], dtype=float)

        # Step 60 frames (1.0s) -> 0.5s loiter timer elapses to 0.0
        for _ in range(60):
            sim.step()

        # Timer has elapsed to 0.0, BUT escorts are still detached (> 4.5m)
        self.assertLessEqual(leader.wp_loiter_timer, 0.0)
        # Formation settlement gate must prevent advancement to WP 1!
        self.assertEqual(leader.current_wp_idx, 0)
        self.assertTrue(leader.mission_active)

        # Now bring followers into their formation stations (<= 4.5m)
        brain = sim.brain
        for f in sim.drones:
            if not f.is_leader:
                f.pos = np.copy(brain._get_formation_slot(f, leader))
                f.vel = np.zeros(3)

        # Step one frame: now that escorts are settled, departure gate opens and advances to WP 1!
        sim.step()
        self.assertEqual(leader.current_wp_idx, 1)
        np.testing.assert_allclose(leader.acknowledged_target, [50.0, 0.0, 25.0])

    def test_pause_and_resume_route(self):
        """Pausing the route freezes countdown and advancement; resuming restores execution."""
        sim = SwarmSimulator(drone_count=4)
        wps_4d = [
            [20.0, 0.0, 20.0, 3.0],
            [40.0, 0.0, 20.0, 0.0],
        ]
        sim.deploy_mission_route(wps_4d, loop=True, name="PAUSE_TEST")
        leader = sim.leader
        leader.pos = np.array([20.0, 0.0, 20.0], dtype=float)

        # Enter loiter
        sim.step()
        self.assertTrue(leader.wp_loitering)

        # Pause mission route
        res_pause = sim.pause_mission_route()
        self.assertEqual(res_pause.get("status"), "OK")
        self.assertTrue(sim.route_paused)
        self.assertTrue(leader.mission_paused)

        timer_at_pause = leader.wp_loiter_timer

        # Step 30 frames (0.5s) while paused
        for _ in range(30):
            sim.step()

        # Loiter countdown must freeze while paused
        self.assertEqual(leader.wp_loiter_timer, timer_at_pause)
        self.assertEqual(leader.current_wp_idx, 0)

        # Resume mission route
        res_resume = sim.resume_mission_route()
        self.assertEqual(res_resume.get("status"), "OK")
        self.assertFalse(sim.route_paused)
        self.assertFalse(leader.mission_paused)

        # Step 10 frames: countdown resumes
        for _ in range(10):
            sim.step()
        self.assertLess(leader.wp_loiter_timer, timer_at_pause)

    def test_skip_mission_waypoint(self):
        """Operator can skip active waypoint manually to advance immediately to next."""
        sim = SwarmSimulator(drone_count=3)
        wps_4d = [
            [10.0, 10.0, 20.0, 10.0],  # Long 10s loiter
            [25.0, 25.0, 22.0, 5.0],
            [40.0, 40.0, 25.0, 0.0],
        ]
        sim.deploy_mission_route(wps_4d, loop=True, name="SKIP_TEST")
        leader = sim.leader
        self.assertEqual(leader.current_wp_idx, 0)

        # Trigger skip
        res_skip = sim.skip_mission_waypoint()
        self.assertEqual(res_skip.get("status"), "OK")
        self.assertEqual(res_skip.get("current_wp_idx"), 1)
        self.assertEqual(leader.current_wp_idx, 1)
        np.testing.assert_allclose(leader.acknowledged_target, [25.0, 25.0, 22.0])
        self.assertEqual(leader.wp_total_loiter, 5.0)
        self.assertFalse(leader.wp_loitering)

    @quarantine_until(heartbeat=3, reason="Requires Heartbeat 3 P2P gossip 4D loiter and settlement")
    def test_gossip_consensus_4d_loiter_and_settlement(self):
        """GossipConsensus brain correctly executes 4D loiter countdowns and escort settlement."""
        sim = SwarmSimulator(drone_count=4)
        sim.set_brain("GOSSIP_CONSENSUS")
        brain = sim.brain

        wps_4d = [
            [20.0, 10.0, 20.0, 1.5],  # 1.5s loiter
            [35.0, 25.0, 25.0, 0.0],
        ]
        sim.deploy_mission_route(wps_4d, loop=False, name="GOSSIP_4D")

        self.assertTrue(brain.route_active)
        self.assertEqual(brain.current_route_wp_idx, 0)
        self.assertEqual(brain.wp_total_loiter, 1.5)

        # Teleport drones close to WP 0 so centroid enters acceptance sphere
        target_wp = np.array([20.0, 10.0, 20.0], dtype=float)
        living_count = len(sim.drones)
        sorted_ids = sorted([d.id for d in sim.drones])
        for d in sim.drones:
            d.flight_state = "CRUISE"
            slot = brain._get_formation_slot(d, sorted_ids.index(d.id), living_count, target_wp)
            d.pos = np.copy(slot)
            d.vel = np.zeros(3)

        # Step 1: centroid enters acceptance sphere, activates loiter
        sim.step()
        self.assertTrue(brain.wp_loitering)
        self.assertAlmostEqual(brain.wp_total_loiter, 1.5, places=1)
        self.assertLess(brain.wp_loiter_timer, 1.5)

        # Step 60 frames (1.0s) with drones holding station
        for _ in range(60):
            for d in sim.drones:
                slot = brain._get_formation_slot(d, sorted_ids.index(d.id), living_count, target_wp)
                d.pos = np.copy(slot)
                d.vel = np.zeros(3)
            sim.step()

        self.assertTrue(brain.wp_loitering)
        self.assertAlmostEqual(brain.wp_loiter_timer, 0.5, delta=0.15)
        self.assertEqual(brain.current_route_wp_idx, 0)

        # Step remaining 40 frames (0.66s) -> loiter timer expires and advances to WP 1!
        for _ in range(40):
            for d in sim.drones:
                slot = brain._get_formation_slot(d, sorted_ids.index(d.id), living_count, target_wp)
                d.pos = np.copy(slot)
                d.vel = np.zeros(3)
            sim.step()

        self.assertFalse(brain.wp_loitering)
        self.assertEqual(brain.current_route_wp_idx, 1)
        next_target = np.array([35.0, 25.0, 25.0], dtype=float)
        for d in sim.drones:
            np.testing.assert_allclose(d.acknowledged_target, next_target)

    @quarantine_until(heartbeat=3, reason="Requires Heartbeat 3 P2P gossip route pause and skip")
    def test_gossip_consensus_pause_and_skip(self):
        """GossipConsensus supports route pause and manual waypoint skip."""
        sim = SwarmSimulator(drone_count=4)
        sim.set_brain("GOSSIP_CONSENSUS")
        brain = sim.brain

        wps_4d = [
            [20.0, 10.0, 20.0, 5.0],
            [35.0, 25.0, 25.0, 2.0],
        ]
        sim.deploy_mission_route(wps_4d, loop=True, name="GOSSIP_CTRL")

        # Pause
        sim.pause_mission_route()
        self.assertTrue(brain.route_paused)
        self.assertTrue(sim.route_paused)

        # Resume
        sim.resume_mission_route()
        self.assertFalse(brain.route_paused)
        self.assertFalse(sim.route_paused)

        # Skip
        new_idx = sim.skip_mission_waypoint()
        self.assertEqual(new_idx.get("current_wp_idx"), 1)
        self.assertEqual(brain.current_route_wp_idx, 1)
        next_target = np.array([35.0, 25.0, 25.0], dtype=float)
        for d in sim.drones:
            np.testing.assert_allclose(d.acknowledged_target, next_target)

    def test_state_snapshot_telemetry_fields(self):
        """Telemetry snapshot contains all required 4D route and loiter fields."""
        sim = SwarmSimulator(drone_count=4)
        wps_4d = [
            [20.0, 10.0, 20.0, 4.0],
            [30.0, 20.0, 25.0, 0.0],
        ]
        sim.deploy_mission_route(wps_4d, loop=True, name="TELEMETRY_SNAP")
        snap = sim.get_state_snapshot()

        route_info = snap.get("mission_route")
        self.assertIsNotNone(route_info)
        self.assertTrue(route_info.get("active"))
        self.assertFalse(route_info.get("paused"))
        self.assertEqual(route_info.get("name"), "TELEMETRY_SNAP")
        self.assertEqual(route_info.get("current_wp_idx"), 0)
        self.assertEqual(route_info.get("total_wps"), 2)
        self.assertIn("loitering", route_info)
        self.assertIn("loiter_timer", route_info)
        self.assertIn("total_loiter", route_info)


if __name__ == "__main__":
    unittest.main()
