"""
AlitheiaHive SITL: Tactical Formation Orientations & Perimeter In/Out Headings Tests (TKT-55)
Validates:
1. Bounded yaw slew rate (<= 45 deg/s in standard flight).
2. PERIMETER_IN / PERIMETER_PATROL inward sentry facing toward protected center.
3. PERIMETER_OUT outward radar sweep facing away from protected center.
4. Smooth 180-degree yaw transition without angular teleportation.
5. ECHELON_ESCORT uniform formation corridor heading across wingmen.
6. Decentralized GOSSIP_CONSENSUS peer nodes inward/outward headings.
7. Failsafe (BACKTRACK_HOLD / LOST_LINK) velocity-aligned heading override.
"""

import unittest
import math
import numpy as np

from SIM.backend.sitl_engine import SwarmSimulator, DT
from tests.quarantine import quarantine_until


class TestTacticalFormationHeadings(unittest.TestCase):
    def setUp(self):
        self.sim = SwarmSimulator(drone_count=5, enable_packet_loss=False, seed=42)
        self.sim.set_comms_mode("WIRELESS_STAR")

    def test_bounded_yaw_slew_rate(self):
        """Verify SITL engine clamps drone yaw rate to <= 45 deg/s (0.7854 rad/s) in standard flight."""
        drone = self.sim.drones[0]
        drone.flight_state = "CRUISE"
        drone.yaw = 0.0
        drone.target_yaw = math.pi  # 180 degree target jump

        # Step one frame (DT = 1/60s)
        self.sim.step()

        max_expected_step = math.radians(45.0) * DT + 1e-5
        actual_step = abs((drone.yaw + math.pi) % (2.0 * math.pi) - math.pi)

        self.assertLessEqual(actual_step, max_expected_step,
                             f"Yaw step {actual_step:.5f} exceeded max rate {max_expected_step:.5f}")

    def test_perimeter_in_followers_face_inward(self):
        """Verify PERIMETER_IN sentry nodes face inward toward the protected target/VIP."""
        target_pos = [0.0, 0.0, 20.0]
        self.sim.set_target(target_pos)
        self.sim.leader.pos = np.array(target_pos, dtype=float)
        self.sim.leader.vel = np.zeros(3, dtype=float)

        res = self.sim.set_formation("PERIMETER_IN")
        self.assertEqual(res["status"], "OK")

        # Pre-position followers at their perimeter stations
        for d in self.sim.drones:
            if not d.is_leader:
                d.pos = np.copy(self.sim.brain._get_formation_slot(d, self.sim.leader))
                d.vel = np.zeros(3, dtype=float)

        # Step 260 frames (~4.3s) to allow 180 deg yaw slew to settle
        for _ in range(260):
            self.sim.step()

        followers = [d for d in self.sim.drones if not d.is_leader and not d.crashed]
        for f in followers:
            to_center = np.array(target_pos[:2]) - f.pos[:2]
            dist_xy = math.hypot(to_center[0], to_center[1])
            self.assertGreater(dist_xy, 10.0, "Follower should be in outer sentry ring")

            expected_yaw = math.atan2(to_center[1], to_center[0])
            yaw_err = abs((f.yaw - expected_yaw + math.pi) % (2.0 * math.pi) - math.pi)

            self.assertLess(yaw_err, 0.05,
                            f"Drone #{f.id} yaw {f.yaw:.2f} did not face inward toward center {expected_yaw:.2f} (err: {yaw_err:.2f})")

    def test_perimeter_out_followers_face_outward(self):
        """Verify PERIMETER_OUT sentry nodes face outward away from the protected target/VIP."""
        target_pos = [0.0, 0.0, 20.0]
        self.sim.set_target(target_pos)
        self.sim.leader.pos = np.array(target_pos, dtype=float)
        self.sim.leader.vel = np.zeros(3, dtype=float)

        res = self.sim.set_formation("PERIMETER_OUT")
        self.assertEqual(res["status"], "OK")

        # Pre-position followers at their perimeter stations
        for d in self.sim.drones:
            if not d.is_leader:
                d.pos = np.copy(self.sim.brain._get_formation_slot(d, self.sim.leader))
                d.vel = np.zeros(3, dtype=float)

        # Step 260 frames (~4.3s) to allow 180 deg yaw slew to settle
        for _ in range(260):
            self.sim.step()

        followers = [d for d in self.sim.drones if not d.is_leader and not d.crashed]
        for f in followers:
            from_center = f.pos[:2] - np.array(target_pos[:2])
            dist_xy = math.hypot(from_center[0], from_center[1])
            self.assertGreater(dist_xy, 10.0, "Follower should be in outer sentry ring")

            expected_yaw = math.atan2(from_center[1], from_center[0])
            yaw_err = abs((f.yaw - expected_yaw + math.pi) % (2.0 * math.pi) - math.pi)

            self.assertLess(yaw_err, 0.05,
                            f"Drone #{f.id} yaw {f.yaw:.2f} did not face outward from center {expected_yaw:.2f} (err: {yaw_err:.2f})")

    def test_perimeter_in_to_out_transition_smoothness(self):
        """Verify switching between PERIMETER_IN and PERIMETER_OUT rotates airframes 180 deg without snaps."""
        target_pos = [0.0, 0.0, 20.0]
        self.sim.set_target(target_pos)
        self.sim.leader.pos = np.array(target_pos, dtype=float)
        self.sim.leader.vel = np.zeros(3, dtype=float)
        self.sim.set_formation("PERIMETER_IN")

        for d in self.sim.drones:
            if not d.is_leader:
                d.pos = np.copy(self.sim.brain._get_formation_slot(d, self.sim.leader))
                d.vel = np.zeros(3, dtype=float)

        # Settle in PERIMETER_IN (260 frames)
        for _ in range(260):
            self.sim.step()

        follower = [d for d in self.sim.drones if not d.is_leader][0]
        yaw_in = follower.yaw

        # Switch to PERIMETER_OUT
        self.sim.set_formation("PERIMETER_OUT")

        # Step 30 frames (0.5s): yaw must move toward opposite direction but NOT snap 180 deg immediately
        for _ in range(30):
            self.sim.step()

        yaw_mid = follower.yaw
        delta_0_5s = abs((yaw_mid - yaw_in + math.pi) % (2.0 * math.pi) - math.pi)

        # In 0.5s at 45 deg/s, max delta is ~22.5 deg (0.39 rad)
        self.assertLessEqual(delta_0_5s, math.radians(45.0) * 0.5 + 0.05,
                             f"Yaw snapped too fast: {delta_0_5s:.3f} rad in 0.5s")
        self.assertGreater(delta_0_5s, 0.05, "Yaw did not start rotating")

        # Step remaining 240 frames (4.5s total): should finish 180 deg rotation
        for _ in range(240):
            self.sim.step()

        yaw_final = follower.yaw
        total_delta = abs((yaw_final - yaw_in + math.pi) % (2.0 * math.pi) - math.pi)
        self.assertAlmostEqual(total_delta, math.pi, delta=0.05,
                               msg=f"Follower failed to reverse 180 deg (actual delta: {total_delta:.2f} rad)")

    def test_echelon_escort_uniform_corridor_heading(self):
        """Verify wingmen in ECHELON_ESCORT match the formation corridor heading."""
        self.sim.set_formation("ECHELON_ESCORT")
        self.sim.set_target([50.0, 50.0, 25.0])

        for _ in range(180):  # 3.0s simulated
            self.sim.step()

        form_heading = self.sim.brain.formation_heading
        followers = [d for d in self.sim.drones if not d.is_leader and not d.crashed]
        for f in followers:
            self.assertEqual(f.target_yaw, form_heading,
                             f"Drone #{f.id} target_yaw {f.target_yaw} != formation_heading {form_heading}")

    @quarantine_until(heartbeat=3, reason="Requires Heartbeat 3 P2P gossip formation headings")
    def test_gossip_consensus_in_and_out_headings(self):
        """Verify decentralized Gossip Consensus nodes compute inward and outward headings in perimeter modes."""
        self.sim.set_comms_mode("P2P_MESH")
        target_pos = [0.0, 0.0, 20.0]
        self.sim.set_target(target_pos)

        # Pre-position peers around target
        for i, d in enumerate(self.sim.drones):
            d.pos = np.copy(self.sim.brain._get_formation_slot(d, i, len(self.sim.drones), target_pos))
            d.vel = np.zeros(3, dtype=float)

        # 1. PERIMETER_IN (allow ~6s for P2P gossip consensus convergence + 180 deg yaw slew)
        self.sim.set_formation("PERIMETER_IN")
        for _ in range(380):
            self.sim.step()

        for d in self.sim.drones:
            to_center = np.array(target_pos[:2]) - d.pos[:2]
            dist_xy = math.hypot(to_center[0], to_center[1])
            self.assertGreater(dist_xy, 10.0)
            expected_in = math.atan2(to_center[1], to_center[0])
            yaw_err = abs((d.yaw - expected_in + math.pi) % (2.0 * math.pi) - math.pi)
            self.assertLess(yaw_err, 0.05,
                            f"Gossip node #{d.id} failed inward heading (err: {yaw_err:.2f})")

        # 2. PERIMETER_OUT (allow 180 deg reversal slew)
        self.sim.set_formation("PERIMETER_OUT")
        for _ in range(380):
            self.sim.step()

        for d in self.sim.drones:
            from_center = d.pos[:2] - np.array(target_pos[:2])
            dist_xy = math.hypot(from_center[0], from_center[1])
            self.assertGreater(dist_xy, 10.0)
            expected_out = math.atan2(from_center[1], from_center[0])
            yaw_err = abs((d.yaw - expected_out + math.pi) % (2.0 * math.pi) - math.pi)
            self.assertLess(yaw_err, 0.05,
                            f"Gossip node #{d.id} failed outward heading (err: {yaw_err:.2f})")

    def test_failsafe_heading_override(self):
        """Verify lost link and backtrack override perimeter heading with flight velocity heading."""
        self.sim.set_formation("PERIMETER_IN")
        drone = self.sim.drones[2]
        drone.flight_state = "BACKTRACK_HOLD"
        drone.vel = np.array([6.0, 0.0, 0.0])  # Moving East (+X)

        self.sim.brain.compute_guidance(DT)

        self.assertAlmostEqual(drone.target_yaw, 0.0, delta=0.01,
                               msg="Backtrack hold should face velocity direction (+X = 0 rad)")


if __name__ == "__main__":
    unittest.main()
