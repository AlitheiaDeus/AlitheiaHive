# ==============================================================================
# SwarmTech3_UpRising: Dynamic In-Flight Swarm Enrollment Test Suite
# Tests for TKT-50:
#   1. Swarm geometry unaffected while scout is on pad / transiting (> 60m).
#   2. P2P handshake triggers at <= 60m, increments swarm N -> N+1.
#   3. Smooth slot assimilation with zero Zone 4 prop clashes (d > 1.5m).
#   4. Dynamic enrollment in Gossip Consensus decentralized mesh.
#   5. Distant lost swarm isolation: pad scout stays at Base muster without telepathic cheating.
# ==============================================================================

import unittest
import numpy as np
import math

from SIM.backend.sitl_engine import (
    SwarmSimulator,
    RANGE_P2P,
    RANGE_HOWL,
    TIER_ESCORT,
)
from tests.quarantine import quarantine_until


class TestInFlightEnrollment(unittest.TestCase):
    """Test suite for TKT-50 Dynamic In-Flight Swarm Enrollment."""

    @quarantine_until(heartbeat=2, reason="Requires Heartbeat 2 multi-drone RF formation geometry")
    def test_swarm_geometry_preserved_during_scout_transit(self):
        """Verify that a distant 4-drone swarm does not resize or twitch slots
        while a newly launched scout drone is on the pad or transiting."""
        sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=42)
        sim.set_comms_mode("LEADER_FOLLOWER")
        sim.set_formation("ECHELON_ESCORT")

        # Move whole swarm to ~80m cruising east
        offset = np.array([80.0, 0.0, 5.0], dtype=float)
        for d in sim.drones:
            d.pos += offset
        sim.target_pos = np.array([120.0, 0.0, 25.0], dtype=float)
        sim.set_target(sim.target_pos)

        # Settle the 4-drone formation for 120 ticks (2.0s)
        for _ in range(120):
            sim.step()

        # Capture relative escort slot offsets from leader before launch
        leader_pos_before = np.copy(sim.leader.pos)
        relative_offsets_before = [
            np.copy(sim.drones[i].pos - leader_pos_before) for i in range(1, 4)
        ]

        # Scramble a 5th drone from the pad
        scout = sim.launch_reinforcement()
        self.assertFalse(scout.enrolled)
        self.assertEqual(scout.enrollment_status, "SCOUT")

        # Step sim for 60 ticks while scout ascends from pad (Z < 15m, dist > 70m)
        for _ in range(60):
            sim.step()
            # Verify distance between scout and leader is > 60m
            dist = float(np.linalg.norm(scout.pos - sim.leader.pos))
            self.assertGreater(dist, 60.0)
            self.assertFalse(scout.enrolled, "Scout must not enroll while outside 60m P2P range")

        # Check that existing 3 followers did not twitch or alter relative slot geometry
        leader_pos_after = np.copy(sim.leader.pos)
        for i in range(1, 4):
            rel_after = sim.drones[i].pos - leader_pos_after
            diff = float(np.linalg.norm(rel_after - relative_offsets_before[i - 1]))
            self.assertLess(
                diff,
                0.5,
                f"Drone {i} experienced slot distortion of {diff:.2f}m while scout was on pad/transiting!",
            )

    def test_p2p_handshake_triggers_at_60m_and_increments_n(self):
        """Verify that P2P handshake occurs automatically when scout closes to within 60m of peer."""
        sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=42)
        sim.set_comms_mode("LEADER_FOLLOWER")
        sim.set_formation("ECHELON_ESCORT")

        # Position swarm within 50m of base so scout meets fleet quickly
        for d in sim.drones:
            d.pos = np.array([45.0, (d.id - 1.5) * 5.0, 20.0], dtype=float)
            d.vel = np.zeros(3, dtype=float)
        sim.target_pos = np.array([55.0, 0.0, 20.0], dtype=float)
        sim.set_target(sim.target_pos)

        # Launch reinforcement
        scout = sim.launch_reinforcement()
        self.assertFalse(scout.enrolled)

        # Simulate until handshake occurs (should take ~2-4s)
        handshake_triggered = False
        for step_i in range(300):
            sim.step()
            if scout.enrolled:
                handshake_triggered = True
                self.assertEqual(scout.enrollment_status, "ENROLLED")
                self.assertGreater(scout.enrollment_timer, 0.0)
                # Verify scout is at altitude
                self.assertGreaterEqual(scout.pos[2], 18.0)
                # Verify nearest peer is within 60m
                min_peer_dist = min(
                    float(np.linalg.norm(scout.pos - d.pos))
                    for d in sim.drones
                    if d.id != scout.id and not d.crashed
                )
                self.assertLessEqual(min_peer_dist, 60.0)
                break

        self.assertTrue(handshake_triggered, "P2P handshake failed to enroll scout within 300 steps")

    @quarantine_until(heartbeat=2, reason="Requires Heartbeat 2 multi-drone RF formation slot assimilation")
    def test_smooth_slot_assimilation_zero_clash(self):
        """Verify that a newly enrolled drone assimilates into outer echelon slot with zero Zone 4 prop clashes."""
        sim = SwarmSimulator(drone_count=3, enable_packet_loss=False, seed=42)
        sim.set_comms_mode("LEADER_FOLLOWER")
        sim.set_formation("ECHELON_ESCORT")

        # Cruising formation near [40, 0, 20]
        sim.leader.pos = np.array([40.0, 0.0, 20.0], dtype=float)
        sim.target_pos = np.array([50.0, 0.0, 20.0], dtype=float)
        sim.set_target(sim.target_pos)

        for _ in range(60):
            sim.step()

        scout = sim.launch_reinforcement()

        min_separation = float("inf")
        # Run 600 steps (10 seconds) to observe full launch, transit, handshake, and settling
        for step_i in range(600):
            sim.step()
            # Monitor pairwise distances across all active drones
            active = [d for d in sim.drones if not d.crashed and d.flight_state != "LANDED"]
            for i in range(len(active)):
                for j in range(i + 1, len(active)):
                    d = float(np.linalg.norm(active[i].pos - active[j].pos))
                    if d < min_separation:
                        min_separation = d

        # Must maintain Zone 3/4 clearance (no prop clash at d <= 1.5m)
        self.assertGreater(
            min_separation,
            1.5,
            f"Prop clash occurred during in-flight enrollment! Min separation was {min_separation:.2f}m",
        )
        self.assertTrue(scout.enrolled)
        self.assertEqual(scout.flight_state, "CRUISE")

    def test_gossip_consensus_in_flight_enrollment(self):
        """Verify dynamic in-flight enrollment in Gossip Consensus decentralized mesh."""
        sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=42)
        sim.set_comms_mode("GOSSIP_CONSENSUS")
        sim.set_formation("ECHELON_ESCORT")

        for d in sim.drones:
            d.pos = np.array([40.0, (d.id - 1.5) * 6.0, 20.0], dtype=float)
        sim.target_pos = np.array([50.0, 0.0, 20.0], dtype=float)
        sim.set_target(sim.target_pos)

        for _ in range(60):
            sim.step()

        scout = sim.launch_reinforcement()
        self.assertFalse(scout.enrolled)

        enrolled = False
        for _ in range(350):
            sim.step()
            if scout.enrolled:
                enrolled = True
                break

        self.assertTrue(enrolled, "Scout failed to enroll in Gossip Consensus mode")
        # Verify sequence number matches consensus target
        peer_state = sim.brain.peer_states.get(scout.id)
        self.assertIsNotNone(peer_state)
        self.assertTrue(np.allclose(peer_state.active_target, sim.get_acknowledged_target(), atol=1.0))

    def test_distant_lost_swarm_isolation_no_telepathic_cheat(self):
        """Verify that when the fleet is > 140m away outside GCS Howl range,
        a newly scrambled scout on the pad does NOT cheat telepathically and holds at Base."""
        sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=42)
        sim.set_comms_mode("LEADER_FOLLOWER")
        sim.set_formation("ECHELON_ESCORT")

        # Swarm is far out at 200m (> 140m RANGE_HOWL)
        for d in sim.drones:
            d.pos = np.array([200.0, (d.id - 1.5) * 5.0, 30.0], dtype=float)
        sim.target_pos = np.array([210.0, 0.0, 30.0], dtype=float)
        sim.set_target(sim.target_pos)

        scout = sim.launch_reinforcement()
        self.assertFalse(scout.enrolled)
        self.assertTrue(scout.lost_link)
        self.assertTrue(np.allclose(scout.lkp, [0.0, 0.0, 20.0]))

        # Step sim for 600 ticks (10.0s) to allow cold-start pad climb-out to Z >= 18m
        for _ in range(600):
            sim.step()

        # Scout must remain at Base muster hold (XY < 10m, Z >= 18m)
        dist_base_xy = float(math.hypot(scout.pos[0], scout.pos[1]))
        self.assertLess(
            dist_base_xy,
            10.0,
            f"Pad scout cheated telepathically and flew out to distant swarm! dist: {dist_base_xy:.2f}m",
        )
        self.assertGreaterEqual(scout.pos[2], 18.0)
        self.assertFalse(scout.enrolled)


if __name__ == "__main__":
    unittest.main()
