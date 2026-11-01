"""
Unit tests for TKT-28: Discrete Waypoint Mission Queue & Custom Route Deployment.
Validates discrete route upload, arrival advancement, wrap-around looping,
tether-severance abort hierarchy, manual interrupt, and leadership succession inheritance.
"""

import unittest
import numpy as np

from SIM.backend.sitl_engine import SwarmSimulator, ROUTE_PRESETS


class TestMissionRoutes(unittest.TestCase):
    """Test suite validating discrete waypoint mission queues and route deployment."""

    def test_route_preset_deployment_and_obc_upload(self):
        """Deploying a route preset uploads waypoints to leader OBC and activates route."""
        sim = SwarmSimulator(drone_count=6)
        res = sim.start_route_preset("BOX_PATROL")

        self.assertEqual(res.get("status"), "OK")
        self.assertEqual(res.get("name"), "BOX_PATROL")
        self.assertTrue(sim.route_active)
        self.assertEqual(len(sim.route_waypoints), 4)

        leader = sim.leader
        self.assertIsNotNone(leader)
        self.assertTrue(leader.mission_active)
        self.assertEqual(len(leader.mission_waypoints), 4)
        self.assertEqual(leader.current_wp_idx, 0)
        np.testing.assert_allclose(leader.acknowledged_target, ROUTE_PRESETS["BOX_PATROL"][0][:3])

    def test_waypoint_arrival_advancement_and_loop(self):
        """Leader advances to next waypoint upon entering 2.5m acceptance sphere and loops."""
        sim = SwarmSimulator(drone_count=4)
        custom_wps = [
            [20.0, 0.0, 20.0],
            [30.0, 10.0, 22.0],
            [20.0, 20.0, 25.0],
        ]
        res = sim.deploy_mission_route(custom_wps, loop=True, name="TRIANGLE_LOOP")
        self.assertEqual(res.get("status"), "OK")

        leader = sim.leader
        self.assertEqual(leader.current_wp_idx, 0)
        np.testing.assert_allclose(leader.acknowledged_target, custom_wps[0])

        # Teleport leader within acceptance sphere of WP 0
        leader.pos = np.array([20.5, 0.2, 20.1], dtype=float)
        sim.step()

        # Should have advanced to WP 1
        self.assertEqual(leader.current_wp_idx, 1)
        np.testing.assert_allclose(leader.acknowledged_target, custom_wps[1])

        # Teleport leader within acceptance sphere of WP 1
        leader.pos = np.array([30.1, 9.9, 22.2], dtype=float)
        sim.step()

        # Should have advanced to WP 2
        self.assertEqual(leader.current_wp_idx, 2)
        np.testing.assert_allclose(leader.acknowledged_target, custom_wps[2])

        # Teleport leader within acceptance sphere of final WP 2 -> should wrap around to WP 0
        leader.pos = np.array([19.9, 20.1, 24.9], dtype=float)
        sim.step()

        self.assertEqual(leader.current_wp_idx, 0)
        np.testing.assert_allclose(leader.acknowledged_target, custom_wps[0])
        self.assertTrue(leader.mission_active)

    def test_non_looping_route_deactivates_on_final_waypoint(self):
        """A route with loop=False terminates mission_active upon arriving at the final waypoint."""
        sim = SwarmSimulator(drone_count=4)
        wps = [
            [15.0, 0.0, 20.0],
            [25.0, 0.0, 22.0],
        ]
        sim.deploy_mission_route(wps, loop=False, name="POINT_TO_POINT")
        leader = sim.leader

        # Reach WP 0
        leader.pos = np.array([15.1, 0.0, 20.0], dtype=float)
        sim.step()
        self.assertEqual(leader.current_wp_idx, 1)

        # Reach WP 1 (final)
        leader.pos = np.array([25.0, 0.1, 22.0], dtype=float)
        sim.step()

        self.assertFalse(leader.mission_active)

    def test_tether_severance_aborts_mission_route(self):
        """Severing tether in TETHERED_RELAY mode immediately aborts active route and engages Doctrine A."""
        sim = SwarmSimulator(drone_count=6)
        sim.start_route_preset("DIAMOND_SURVEY")
        self.assertTrue(sim.route_active)
        self.assertTrue(sim.drones[0].mission_active)

        # Sever tether
        sim.cut_tether()

        # Route must be aborted to prevent infinite battery drain
        self.assertFalse(sim.route_active)
        for d in sim.drones:
            self.assertFalse(d.mission_active)

        # Doctrine A 30s loiter initiated
        self.assertEqual(sim.lost_link_loiter_timer, 30.0)

    def test_manual_target_dispatch_interrupts_route(self):
        """Operator dispatching a single target coordinate cancels the active mission route."""
        sim = SwarmSimulator(drone_count=4)
        sim.start_route_preset("BOX_PATROL")
        self.assertTrue(sim.route_active)

        sim.set_target([70.0, 70.0, 30.0])
        self.assertFalse(sim.route_active)
        for d in sim.drones:
            self.assertFalse(d.mission_active)

    def test_comms_loss_uplink_drop(self):
        """Deploying a route when tether is severed or RF range > 120m drops the uplink packet."""
        sim = SwarmSimulator(drone_count=4)
        # Sever tether
        sim.cut_tether()

        res = sim.start_route_preset("BOX_PATROL")
        self.assertEqual(res.get("status"), "DROPPED")
        self.assertEqual(res.get("reason"), "TETHER_SEVERED")

        # In wireless mode out of range
        sim.set_comms_mode("LEADER_FOLLOWER")
        sim.leader.pos = np.array([200.0, 0.0, 30.0], dtype=float)  # > 120m
        res2 = sim.start_route_preset("BOX_PATROL")
        self.assertEqual(res2.get("status"), "OUT_OF_RANGE")

    def test_leader_succession_inherits_route_in_wireless_mode(self):
        """In wireless LEADER_FOLLOWER mode, promoted 2IC inherits mission route and waypoint index."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_comms_mode("LEADER_FOLLOWER")
        sim.start_route_preset("HEXAGON_SWEEP")

        d0 = sim.drones[0]
        d1 = sim.drones[1]
        self.assertTrue(d0.is_leader)
        self.assertTrue(d0.mission_active)
        d0.current_wp_idx = 2  # At WP 2

        # Simulate leader motor cut
        sim.kill_leader()

        # Drone 1 should now be promoted leader and hold the same mission state
        self.assertTrue(d1.is_leader)
        self.assertTrue(d1.mission_active)
        self.assertEqual(d1.current_wp_idx, 2)
        self.assertEqual(len(d1.mission_waypoints), len(ROUTE_PRESETS["HEXAGON_SWEEP"]))

    def test_gossip_consensus_route_progression(self):
        """In Gossip consensus mode, swarm centroid arrival advances the consensus waypoint."""
        sim = SwarmSimulator(drone_count=4)
        sim.set_brain("GOSSIP_CONSENSUS")
        custom_wps = [
            [20.0, 0.0, 20.0],
            [35.0, 20.0, 25.0],
        ]
        res = sim.deploy_mission_route(custom_wps, loop=True, name="GOSSIP_ROUTE")
        self.assertEqual(res.get("status"), "OK")
        self.assertEqual(sim.brain.current_route_wp_idx, 0)

        # Position drones around WP 0 with safe physical separation (> 3.5m) so centroid arrives at WP 0
        offsets = [
            np.array([2.0, 0.0, 0.0], dtype=float),
            np.array([-2.0, 0.0, 0.0], dtype=float),
            np.array([0.0, 2.0, 0.0], dtype=float),
            np.array([0.0, -2.0, 0.0], dtype=float),
        ]
        for d, off in zip(sim.drones, offsets):
            d.pos = np.array([20.0, 0.0, 20.0], dtype=float) + off

        sim.step()

        # Gossip brain should have advanced to WP 1
        self.assertEqual(sim.brain.current_route_wp_idx, 1)
        np.testing.assert_allclose(sim.get_acknowledged_target(), custom_wps[1][:3])

    def test_4d_waypoint_loiter_countdown_and_advancement(self):
        """Leader enters loiter at station, counts down wp_loiter_timer, and advances only after loiter expires."""
        sim = SwarmSimulator(drone_count=4)
        wps = [
            [20.0, 0.0, 20.0, 1.5],   # 1.5s loiter
            [35.0, 15.0, 25.0, 2.0],  # 2.0s loiter
        ]
        sim.deploy_mission_route(wps, loop=False, name="LOITER_ROUTE")
        leader = sim.leader

        # Teleport leader to WP 0
        leader.pos = np.array([20.1, 0.0, 20.0], dtype=float)
        # Position escorts directly in their formation slots to satisfy stabilization gate
        for f in sim.drones:
            if not f.is_leader:
                f.flight_state = "CRUISE"
                slot = sim.brain._get_formation_slot(f, leader)
                f.pos = np.copy(slot)

        sim.step()

        # Leader should have entered loiter state
        self.assertTrue(leader.wp_loitering)
        self.assertAlmostEqual(leader.wp_loiter_timer, 1.5 - (1.0 / 60.0), delta=0.05)
        self.assertEqual(leader.current_wp_idx, 0)

        # Step 30 frames (0.5s)
        for _ in range(30):
            sim.step()

        self.assertTrue(leader.wp_loitering)
        self.assertAlmostEqual(leader.wp_loiter_timer, 1.0, delta=0.1)
        self.assertEqual(leader.current_wp_idx, 0)

        # Step remaining 70 frames (> 1.1s)
        for _ in range(70):
            sim.step()

        # Loiter elapsed: leader advances to WP 1
        self.assertFalse(leader.wp_loitering)
        self.assertEqual(leader.current_wp_idx, 1)
        np.testing.assert_allclose(leader.acknowledged_target, [35.0, 15.0, 25.0])

    def test_dict_format_4d_waypoints(self):
        """Routes defined with dict objects {x, y, z, loiter} are parsed and executed identically."""
        sim = SwarmSimulator(drone_count=4)
        dict_wps = [
            {"x": 25.0, "y": 10.0, "z": 22.0, "loiter": 2.5},
            {"x": 40.0, "y": -15.0, "z": 24.0, "loiter": 1.0},
        ]
        res = sim.deploy_mission_route(dict_wps, loop=True, name="DICT_ROUTE")
        self.assertEqual(res.get("status"), "OK")
        self.assertEqual(len(sim.route_waypoints), 2)
        np.testing.assert_allclose(sim.route_waypoints[0], [25.0, 10.0, 22.0, 2.5])
        np.testing.assert_allclose(sim.route_waypoints[1], [40.0, -15.0, 24.0, 1.0])

    def test_formation_stabilization_gate_holds_leader(self):
        """Leader loitering at waypoint will hold station if escorts are lagging (> 4.0m from slot)."""
        sim = SwarmSimulator(drone_count=4)
        wps = [
            [20.0, 0.0, 20.0, 0.5],   # 0.5s loiter (30 frames)
            [40.0, 20.0, 25.0, 0.0],
        ]
        sim.deploy_mission_route(wps, loop=False, name="GATE_ROUTE")
        leader = sim.leader
        leader.pos = np.array([20.0, 0.0, 20.0], dtype=float)

        # Displace escorts far away from slots with mutual separation to prevent prop clash
        offsets = [
            np.array([-15.0, 0.0, 0.0], dtype=float),
            np.array([0.0, -15.0, 0.0], dtype=float),
            np.array([-15.0, -15.0, 0.0], dtype=float),
        ]
        followers = [d for d in sim.drones if not d.is_leader]
        for f, off in zip(followers, offsets):
            f.flight_state = "CRUISE"
            slot = sim.brain._get_formation_slot(f, leader)
            f.pos = slot + off

        # Step 40 frames (> 0.5s timer)
        for _ in range(40):
            sim.step()

        # Timer has run down to 0.0s, but escorts are NOT settled -> leader holds at WP 0
        self.assertEqual(leader.wp_loiter_timer, 0.0)
        self.assertTrue(leader.wp_loitering)
        self.assertEqual(leader.current_wp_idx, 0)

        # Now snap escorts to their assigned slots
        for f in followers:
            slot = sim.brain._get_formation_slot(f, leader)
            f.pos = np.copy(slot)

        # Step 1 frame
        sim.step()

        # Escorts settled: gate clears and leader advances to WP 1
        self.assertFalse(leader.wp_loitering)
        self.assertEqual(leader.current_wp_idx, 1)

    def test_snapshot_telemetry_includes_loiter_fields(self):
        """State snapshot correctly exports loitering boolean and real-time loiter_timer."""
        sim = SwarmSimulator(drone_count=4)
        sim.start_route_preset("BOX_PATROL")
        snapshot = sim.get_state_snapshot()
        route = snapshot["mission_route"]
        self.assertIn("loitering", route)
        self.assertIn("loiter_timer", route)
        self.assertIsInstance(route["loitering"], bool)
        self.assertIsInstance(route["loiter_timer"], float)


if __name__ == "__main__":
    unittest.main()
