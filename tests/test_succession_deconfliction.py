"""
SwarmTech3_UpRising: Succession Deconfliction & Anti-Jousting Test Suite
(tests/test_succession_deconfliction.py)

Verifies TKT-15:
- Lateral peel-off corridor prevents head-on jousting during autonomic succession.
- Wing slot continuity ensures existing followers maintain side affinity without criss-crossing.
- Zero Zone 4 prop-clashes (d <= 1.5m) across Echelon, Natural Flock, Orbit Vortex, and Perimeter Patrol.
"""

import math
import unittest
import numpy as np

from SIM.backend.sitl_engine import SwarmSimulator, ZONE4_CLASH


class TestSuccessionDeconfliction(unittest.TestCase):
    """Test suite validating lateral peel-off corridor and anti-jousting dynamics."""

    def test_echelon_cut_tether_zero_clash(self):
        """Demoted leader peels laterally away from successor during tether severance with zero clashes."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_formation("ECHELON_ESCORT")

        # Cruise forward for 2.0s (120 steps) to stabilize formation
        for _ in range(120):
            sim.step()

        d0 = sim.drones[0]
        d1 = sim.drones[1]
        self.assertTrue(d0.is_leader)
        self.assertFalse(d1.is_leader)

        # Sever tether: demotes Drone 0 alive and promotes Drone 1
        sim.cut_tether()

        self.assertFalse(d0.is_leader)
        self.assertTrue(d1.is_leader)
        self.assertEqual(d1.status, "PROMOTED_LEADER")
        self.assertGreater(d0.peel_off_timer, 0.0)

        min_pairwise_dist = 999.0
        # Simulate 4.0 seconds (240 ticks) of rank transition and flight
        for _ in range(240):
            sim.step()
            living = [d for d in sim.drones if not d.crashed]
            for i in range(len(living)):
                for j in range(i + 1, len(living)):
                    dist = float(np.linalg.norm(living[i].pos - living[j].pos))
                    if dist < min_pairwise_dist:
                        min_pairwise_dist = dist

            # Ensure neither d0 nor d1 crashed
            self.assertFalse(d0.crashed, "Demoted leader crashed during succession transition!")
            self.assertFalse(d1.crashed, "Promoted leader crashed during succession transition!")

        # Minimum distance between any living drones must stay strictly above Zone 4 clash envelope (1.5m)
        self.assertGreater(min_pairwise_dist, ZONE4_CLASH)

    def test_echelon_manual_demotion_anti_jousting(self):
        """Manual succession trigger with living demoted leader applies anti-jousting corridor."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_formation("ECHELON_ESCORT")

        for _ in range(120):
            sim.step()

        old_leader = sim.leader
        new_leader = sim._trigger_autonomic_succession(demote_old_leader=True)

        self.assertIsNotNone(new_leader)
        self.assertNotEqual(old_leader.id, new_leader.id)
        self.assertGreater(old_leader.peel_off_timer, 0.0)
        self.assertGreater(new_leader.promotion_timer, 0.0)

        # Vector peel_off_dir must have lateral component perpendicular to forward flight
        self.assertGreater(float(np.linalg.norm(old_leader.peel_off_dir)), 0.9)

        min_d0_d1 = 999.0
        for _ in range(240):
            sim.step()
            d = float(np.linalg.norm(old_leader.pos - new_leader.pos))
            if d < min_d0_d1:
                min_d0_d1 = d
            self.assertFalse(old_leader.crashed)
            self.assertFalse(new_leader.crashed)

        self.assertGreater(min_d0_d1, ZONE4_CLASH)

    def test_wing_continuity_and_slot_hysteresis(self):
        """Followers maintain wing affinity: right-wing followers stay right, left stay left."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_formation("ECHELON_ESCORT")

        for _ in range(60):
            sim.step()

        # Check initial slot allocations in LeaderFollowerBrain
        brain = sim.brain
        # Follower IDs are 1..5. In Echelon:
        # Slot 1: left wing (odd)
        # Slot 2: right wing (even)
        # Slot 3: left wing (odd)
        # Slot 4: right wing (even)
        # Slot 5: left wing (odd)
        self.assertEqual(brain.assigned_slots.get(1), 1)
        self.assertEqual(brain.assigned_slots.get(2), 2)
        self.assertEqual(brain.assigned_slots.get(3), 3)

        # Promote Drone 1 to Leader, demoting Drone 0
        sim._trigger_autonomic_succession(demote_old_leader=True)

        # Drone 0 should take the vacated Slot 1
        self.assertEqual(brain.assigned_slots.get(0), 1)
        # Drone 2 MUST stay in Slot 2 (right wing)
        self.assertEqual(brain.assigned_slots.get(2), 2)
        # Drone 3 MUST stay in Slot 3 (left wing)
        self.assertEqual(brain.assigned_slots.get(3), 3)
        # Drone 4 MUST stay in Slot 4 (right wing)
        self.assertEqual(brain.assigned_slots.get(4), 4)

    def test_multi_formation_succession_zero_crashes(self):
        """Succession causes zero crashes across Natural Flock, Orbit Vortex, and Perimeter Patrol."""
        formations = ["NATURAL_FLOCK", "ORBIT_VORTEX", "PERIMETER_PATROL"]

        for formation in formations:
            with self.subTest(formation=formation):
                sim = SwarmSimulator(drone_count=6)
                sim.set_formation(formation)

                for _ in range(120):
                    sim.step()

                # Trigger succession with demoted living leader
                sim.cut_tether()

                for _ in range(180):
                    sim.step()
                    # No living drones should crash
                    crashed = [d.id for d in sim.drones if d.crashed]
                    self.assertEqual(len(crashed), 0, f"Collateral crash in {formation}: {crashed}")

    def test_leader_casualty_falling_separation(self):
        """Killing active leader does not cause followers to collide with falling wreckage."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_formation("ECHELON_ESCORT")

        for _ in range(120):
            sim.step()

        # Kill Drone 0
        sim.kill_leader()
        self.assertTrue(sim.drones[0].crashed)

        # Surviving drones (1..5) should not collide with falling drone 0 or each other
        for _ in range(180):
            sim.step()
            survivor_crashes = [d.id for d in sim.drones if d.crashed and d.id != 0]
            self.assertEqual(len(survivor_crashes), 0, f"Collateral crash after leader kill: {survivor_crashes}")


if __name__ == "__main__":
    unittest.main()
