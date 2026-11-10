"""
Unit tests for Tactical RNG Touch-and-Go Reconnaissance Generator (TKT-29).
Validates deterministic target arrival detection, GCS-side dwell countdown,
safe high-variance target generation, channel-bound uplink failsafes,
and exclusive mode switching.
"""

import unittest
import numpy as np
from SIM.backend.sitl_engine import SwarmSimulator
from SIM.backend.mavlink_bridge import MavlinkBridge


class TestRngRecon(unittest.TestCase):
    def test_is_target_reached_leader_follower(self):
        """In LEADER_FOLLOWER mode, arrival is determined by leader proximity to target."""
        sim = SwarmSimulator(drone_count=4)
        target = np.array([30.0, 30.0, 20.0], dtype=float)
        sim.set_target(target)
        sim.step(0.016)

        leader = sim.leader
        self.assertIsNotNone(leader)

        # Leader far from target
        leader.pos = np.array([0.0, 0.0, 20.0], dtype=float)
        self.assertFalse(sim.is_target_reached())
        self.assertFalse(leader.target_reached)

        # Leader within 2.5m acceptance sphere
        leader.pos = np.array([30.5, 30.2, 20.1], dtype=float)
        self.assertTrue(sim.is_target_reached())
        self.assertTrue(leader.target_reached)

    def test_is_target_reached_gossip_consensus(self):
        """In GOSSIP_CONSENSUS mode, arrival is determined by swarm centroid proximity to target."""
        sim = SwarmSimulator(drone_count=4)
        sim.set_brain("GOSSIP_CONSENSUS")
        target = np.array([40.0, 10.0, 25.0], dtype=float)
        for d in sim.drones:
            sim.brain.peer_states[d.id].active_target = np.copy(target)
            d.acknowledged_target = np.copy(target)

        # Disperse drones around target with safe physical clearance (> 3.5m)
        offsets = [
            np.array([2.0, 0.0, 0.0], dtype=float),
            np.array([-2.0, 0.0, 0.0], dtype=float),
            np.array([0.0, 2.0, 0.0], dtype=float),
            np.array([0.0, -2.0, 0.0], dtype=float),
        ]
        for d, off in zip(sim.drones, offsets):
            d.pos = target + off

        self.assertTrue(sim.is_target_reached())

        # Move drones far away
        for d in sim.drones:
            d.pos = np.array([0.0, 0.0, 15.0], dtype=float)
        self.assertFalse(sim.is_target_reached())

    def test_generate_safe_rng_target_geofence_and_variance(self):
        """Randomized targets must respect operational geofence, variance, and tether clearance."""
        sim = SwarmSimulator(drone_count=4)
        initial_target = np.copy(sim.get_acknowledged_target())

        for _ in range(50):
            target = sim.generate_safe_rng_target()

            # Geofence bounds
            self.assertGreaterEqual(target[0], sim.rng_geofence_min[0])
            self.assertLessEqual(target[0], sim.rng_geofence_max[0])
            self.assertGreaterEqual(target[1], sim.rng_geofence_min[1])
            self.assertLessEqual(target[1], sim.rng_geofence_max[1])
            self.assertGreaterEqual(target[2], sim.rng_geofence_min[2])
            self.assertLessEqual(target[2], sim.rng_geofence_max[2])

            # Variance: >= 15m jump distance
            dist = float(np.linalg.norm(target - initial_target))
            self.assertGreaterEqual(dist, sim.rng_min_variance)

            # Tether clearance: horizontal distance >= 3.0m from anchor
            dist_anchor_xy = float(np.hypot(target[0] - sim.tether_anchor[0], target[1] - sim.tether_anchor[1]))
            self.assertGreaterEqual(dist_anchor_xy, sim.rng_tether_clearance)

    def test_rng_recon_dwell_countdown_and_station_advance(self):
        """Swarm arrives at target -> 2.0s dwell countdown -> uplinks new safe high-variance target."""
        sim = SwarmSimulator(drone_count=4)
        leader = sim.leader

        # Position leader directly at target
        target = np.copy(sim.get_acknowledged_target())
        leader.pos = np.copy(target)

        # Engage RNG recon
        sim.toggle_rng_recon(True)
        self.assertTrue(sim.rng_recon_active)
        self.assertTrue(sim.rng_dwelling)
        self.assertEqual(sim.rng_station_count, 1)
        self.assertEqual(sim.rng_dwell_timer, 2.0)

        # Step 30 frames @ 60 FPS (0.5s)
        for _ in range(30):
            sim.step()

        self.assertTrue(sim.rng_dwelling)
        self.assertAlmostEqual(sim.rng_dwell_timer, 1.5, delta=0.1)
        np.testing.assert_allclose(sim.get_acknowledged_target(), target)

        # Step remaining dwell (another 100 frames > 1.5s)
        for _ in range(100):
            sim.step()

        # Dwell complete: new target generated and uplinked to leader
        new_target = sim.get_acknowledged_target()
        dist_jump = float(np.linalg.norm(new_target - target))
        self.assertGreaterEqual(dist_jump, 14.5)
        self.assertFalse(sim.rng_dwelling)
        self.assertEqual(sim.rng_station_count, 1)

    def test_rng_recon_tether_severance_prevents_new_target(self):
        """In TETHERED_RELAY, severing the umbilical prevents new target ingestion on dwell complete."""
        sim = SwarmSimulator(drone_count=4)
        leader = sim.leader
        target = np.copy(sim.get_acknowledged_target())
        leader.pos = np.copy(target)

        sim.toggle_rng_recon(True)
        self.assertTrue(sim.rng_dwelling)

        # Sever tether
        sim.cut_tether()
        self.assertTrue(sim.tether_severed)

        # Step through dwell
        for _ in range(130):
            sim.step()

        # Leader acknowledged target must NOT adopt new target through severed wire
        np.testing.assert_allclose(leader.acknowledged_target, target)

    def test_rng_recon_mode_exclusive_cancellation(self):
        """RNG Recon mode cancels and is cancelled by other targeting modes."""
        sim = SwarmSimulator(drone_count=4)

        # Auto-patrol active -> RNG recon cancels auto-patrol
        sim.toggle_target_patrol(True)
        self.assertTrue(sim.patrol_active)

        sim.toggle_rng_recon(True)
        self.assertTrue(sim.rng_recon_active)
        self.assertFalse(sim.patrol_active)

        # Manual target dispatch cancels RNG recon
        sim.set_target([45.0, 45.0, 25.0])
        self.assertFalse(sim.rng_recon_active)

        # Re-engage RNG recon
        sim.toggle_rng_recon(True)
        self.assertTrue(sim.rng_recon_active)

        # Route deployment cancels RNG recon
        sim.start_route_preset("BOX_PATROL")
        self.assertTrue(sim.route_active)
        self.assertFalse(sim.rng_recon_active)

    def test_mavlink_bridge_udp_commands(self):
        """MavlinkBridge processes RNG recon commands over UDP."""
        sim = SwarmSimulator(drone_count=4)
        bridge = MavlinkBridge(sim, tx_port=9120, rx_port=9121, mavlink_port=14620, enable_mavlink=False)

        res1 = bridge.execute_command({"cmd": "START_RNG_RECON"})
        self.assertEqual(res1.get("status"), "OK")
        self.assertTrue(sim.rng_recon_active)

        res2 = bridge.execute_command({"cmd": "TOGGLE_RNG_RECON"})
        self.assertEqual(res2.get("status"), "OK")
        self.assertFalse(sim.rng_recon_active)

        res3 = bridge.execute_command({"cmd": "STOP_RNG_RECON"})
        self.assertEqual(res3.get("status"), "OK")
        self.assertFalse(sim.rng_recon_active)

        bridge.close()

    def test_telemetry_snapshot_serialization(self):
        """State snapshot correctly serializes target_reached and rng_recon telemetry."""
        sim = SwarmSimulator(drone_count=4)
        sim.toggle_rng_recon(True)

        snapshot = sim.get_state_snapshot()
        self.assertIn("target_reached", snapshot)
        self.assertIn("rng_recon", snapshot)
        rng_info = snapshot["rng_recon"]
        self.assertTrue(rng_info["active"])
        self.assertIn("dwelling", rng_info)
        self.assertIn("dwell_timer", rng_info)
        self.assertIn("station_count", rng_info)
        self.assertEqual(rng_info["dwell_time"], 2.0)


if __name__ == "__main__":
    unittest.main()
