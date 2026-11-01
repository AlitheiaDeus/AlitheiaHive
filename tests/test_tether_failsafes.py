"""
Unit tests for TKT-16: Tether Failsafes: Breakaway Dive, Cable Jettison, Escort Rejoin, & Doctrine A.
"""

import unittest
import numpy as np

from SIM.backend.sitl_engine import SwarmSimulator, ZONE4_CLASH, V_CRUISE


class TestTetherFailsafes(unittest.TestCase):
    """Test suite validating emergency breakaway dive, umbilical jettison,
    dynamic 2IC succession, and Doctrine A lost-link loiter & automated RTB."""

    def test_tether_severance_no_crash_and_jettison_timer(self):
        """Severing tether must not crash Drone 0; it enters BREAKAWAY_JETTISON and promotes 2IC."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_formation("ECHELON_ESCORT")

        # Cruise forward for 1.0s to establish flight
        for _ in range(60):
            sim.step()

        d0 = sim.drones[0]
        d1 = sim.drones[1]
        self.assertTrue(d0.is_leader)
        self.assertFalse(d0.crashed)
        self.assertEqual(sim.get_2ic().id, 1)

        # Sever tether
        sim.cut_tether()

        self.assertTrue(sim.tether_severed)
        self.assertFalse(d0.is_leader)
        self.assertFalse(d0.crashed, "Drone 0 must not crash upon tether severance!")
        self.assertEqual(d0.status, "BREAKAWAY_JETTISON")
        self.assertEqual(d0.jettison_watchdog, 6.0)
        self.assertFalse(d0.tether_jettisoned)

        # Dynamic 2IC (Drone 1) promoted to leader
        self.assertTrue(d1.is_leader)
        self.assertEqual(d1.status, "PROMOTED_LEADER")

        # Doctrine A 30s loiter initiated
        self.assertEqual(sim.lost_link_loiter_timer, 30.0)
        self.assertFalse(sim.swarm_rtb_active)

    def test_dynamic_2ic_selection(self):
        """Dynamic 2IC election correctly skips dead or salvaged drones."""
        sim = SwarmSimulator(drone_count=6)
        # Kill Drone 1 prior to cut
        sim.kill_drone(1)
        self.assertTrue(sim.drones[1].crashed)

        # 2IC should now be Drone 2
        two_ic = sim.get_2ic()
        self.assertIsNotNone(two_ic)
        self.assertEqual(two_ic.id, 2)

        # Cut tether -> Drone 2 should be promoted
        sim.cut_tether()
        self.assertTrue(sim.drones[2].is_leader)
        self.assertEqual(sim.drones[2].status, "PROMOTED_LEADER")

    def test_breakaway_dive_and_cable_purge(self):
        """Drone 0 dives vertically and purges cable after physical clearance gates are satisfied."""
        sim = SwarmSimulator(drone_count=6)
        for _ in range(60):
            sim.step()

        d0 = sim.drones[0]
        d1 = sim.drones[1]

        sim.cut_tether()

        # Step 50 ticks (~0.83s) into breakaway dive
        for _ in range(50):
            sim.step()

        # Drone 0 should be actively diving in BREAKAWAY_JETTISON below leader
        self.assertLess(d0.pos[2], d1.pos[2] - 1.0, "Drone 0 should dive below leader during jettison!")
        self.assertEqual(d0.status, "BREAKAWAY_JETTISON")
        self.assertFalse(d0.tether_jettisoned)

        # Step remaining ticks until physical clearance is achieved (within 40 more ticks)
        for _ in range(40):
            sim.step()

        self.assertTrue(d0.tether_jettisoned, "Drone 0 must purge cable once clearance envelope is met!")
        self.assertEqual(d0.status, "ACTIVE")
        self.assertFalse(d0.crashed)

    def test_physical_clearance_envelope_gates(self):
        """Umbilical jettison triggers strictly when altitude gate (Delta Z >= 4.0m) and lateral gate (Delta R_perp >= 3.0m) are met."""
        sim = SwarmSimulator(drone_count=6)
        for _ in range(60):
            sim.step()

        d0 = sim.drones[0]
        d1 = sim.drones[1]

        sim.cut_tether()
        self.assertEqual(d0.status, "BREAKAWAY_JETTISON")
        self.assertFalse(d0.tether_jettisoned)

        # Step tick-by-tick and verify transition invariant
        cleared = False
        for tick in range(180):  # 3.0 seconds max
            sim.step()
            if d0.tether_jettisoned:
                cleared = True
                lead_z = d1.pos[2]
                z_floor = max(3.0, lead_z - 4.5)
                alt_cleared = (d0.pos[2] <= lead_z - 4.0) or (d0.pos[2] <= z_floor + 0.25)
                peel_dir = getattr(d0, "peel_off_dir", np.zeros(3, dtype=float))
                if float(np.linalg.norm(peel_dir[:2])) > 0.01:
                    lat_offset = abs(float(np.dot(d0.pos[:2] - d1.pos[:2], peel_dir[:2])))
                else:
                    lat_offset = float(np.linalg.norm(d0.pos[:2] - d1.pos[:2]))
                lat_cleared = lat_offset >= 3.0
                self.assertTrue(
                    (alt_cleared and lat_cleared) or (d0.jettison_watchdog <= 0.0),
                    f"Cleared prematurely without satisfying gates: alt_cleared={alt_cleared}, lat_cleared={lat_cleared}"
                )
                break

        self.assertTrue(cleared, "Drone 0 must achieve physical clearance envelope!")
        self.assertEqual(d0.status, "ACTIVE")

    def test_safety_floor_clamp_prevents_terrain_impact(self):
        """Even at low initial formation altitude, breakaway dive respects safety floor max(3.0m, Z_lead - 4.5m)."""
        sim = SwarmSimulator(drone_count=4)
        # Position drones at low altitude (Z = 5.0m)
        for d in sim.drones:
            d.pos[2] = 5.0
            d.vel = np.zeros(3, dtype=float)

        d0 = sim.drones[0]
        sim.cut_tether()

        min_altitude = 999.0
        for _ in range(120):
            sim.step()
            min_altitude = min(min_altitude, float(d0.pos[2]))

        # Safety floor is max(3.0m, 5.0 - 4.5m) = 3.0m
        self.assertGreaterEqual(
            min_altitude,
            2.9,
            f"Drone 0 penetrated safety floor! min_altitude={min_altitude}m < 3.0m",
        )
        self.assertFalse(d0.crashed)

    def test_jettison_watchdog_timeout(self):
        """If clearance gates cannot be met (e.g. pinned), 6.0s watchdog triggers jettison fallback."""
        sim = SwarmSimulator(drone_count=4)
        sim.cut_tether()

        d0 = sim.drones[0]
        d1 = sim.drones[1]

        # Artificially keep Drone 0 near leader (2.0m away, outside 1.5m clash but within 3.0m/4.0m gates)
        for _ in range(365):  # > 6.0 seconds (360 ticks)
            d0.pos = d1.pos + np.array([2.0, 0.0, 0.0], dtype=float)
            d0.vel = np.zeros(3, dtype=float)
            sim.step()

        self.assertTrue(d0.tether_jettisoned, "Watchdog must force cable jettison after 6.0s timeout!")
        self.assertEqual(d0.status, "ACTIVE")

    def test_escort_rejoin_after_purge(self):
        """Unencumbered Drone 0 climbs back to cruise altitude and rejoins formation with zero clashes."""
        sim = SwarmSimulator(drone_count=6)
        for _ in range(60):
            sim.step()

        d0 = sim.drones[0]
        sim.cut_tether()

        min_dist = 999.0
        # Step through 4.0s (240 ticks)
        for _ in range(240):
            sim.step()
            living = [d for d in sim.drones if not d.crashed]
            for i in range(len(living)):
                for j in range(i + 1, len(living)):
                    dist = float(np.linalg.norm(living[i].pos - living[j].pos))
                    if dist < min_dist:
                        min_dist = dist

        # Drone 0 should have climbed back to formation cruise altitude (>= 20m)
        self.assertGreaterEqual(d0.pos[2], 20.0, "Drone 0 should climb back to cruise altitude upon rejoin!")
        self.assertEqual(d0.status, "ACTIVE")
        # Ensure zero prop-clashes
        self.assertGreater(min_dist, ZONE4_CLASH, "Pairwise distance must exceed Zone 4 clash envelope!")

    def test_doctrine_a_loiter_grace_period_and_automated_rtb(self):
        """Fleet holds target for 30s grace period; if no reconnection, triggers automated SWARM_RTB."""
        sim = SwarmSimulator(drone_count=6)
        target = [70.0, 50.0, 30.0]
        sim.set_target(target)

        for _ in range(60):
            sim.step()

        sim.cut_tether()
        self.assertEqual(sim.lost_link_loiter_timer, 30.0)
        self.assertFalse(sim.swarm_rtb_active)

        # Step 10.0 seconds (600 ticks)
        for _ in range(600):
            sim.step()

        self.assertAlmostEqual(sim.lost_link_loiter_timer, 20.0, delta=0.2)
        self.assertFalse(sim.swarm_rtb_active)
        np.testing.assert_allclose(sim.target_pos, target)

        # Step remaining 21.0 seconds (1260 ticks) to exceed 30.0s grace period
        for _ in range(1260):
            sim.step()

        self.assertEqual(sim.lost_link_loiter_timer, 0.0)
        self.assertTrue(sim.swarm_rtb_active)
        # Target must automatically be redirected to Base [0, 0, 25.0]
        np.testing.assert_allclose(sim.target_pos, [0.0, 0.0, 25.0])

    def test_tether_failsafe_policy_rtb_override(self):
        """When drone policy is set to RTB, drone returns to base after cable purge."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_target([80.0, 80.0, 30.0])
        for _ in range(60):
            sim.step()

        d0 = sim.drones[0]
        d0.tether_failsafe_policy = "RTB"

        sim.cut_tether()

        # Step 3.5s (210 ticks): past 1.5s purge and onto RTB transit
        for _ in range(210):
            sim.step()

        self.assertTrue(d0.tether_jettisoned)
        # Leader steers toward [80, 80, 30] while Drone 0 steers back toward [0, 0, 25]
        d1 = sim.drones[1]
        self.assertGreater(d1.pos[0], 10.0)
        # Drone 0's X position should be behind leader (flying back to base)
        self.assertLess(d0.pos[0], d1.pos[0])

    def test_cancel_rtb_normalizes_untethered_cruise(self):
        """Cancelling RTB post-severance normalizes comms_mode to LEADER_FOLLOWER and flight_state to CRUISE."""
        sim = SwarmSimulator(drone_count=6)
        sim.cut_tether()
        self.assertTrue(sim.tether_severed)
        self.assertEqual(sim.comms_mode, "TETHERED_RELAY")

        # Step until 30s loiter grace period expires and RTB engages
        for _ in range(1850):
            sim.step()

        self.assertTrue(sim.swarm_rtb_active)

        # Operator cancels RTB
        sim.cancel_rtb()
        self.assertFalse(sim.swarm_rtb_active)
        self.assertEqual(sim.recovery_stage, "IDLE")
        self.assertEqual(sim.lost_link_loiter_timer, 0.0)
        # Comms mode must now be normalized to untethered LEADER_FOLLOWER
        self.assertEqual(sim.comms_mode, "LEADER_FOLLOWER")
        # Living drones must be returned to CRUISE
        for d in sim.drones:
            if not d.crashed:
                self.assertEqual(d.flight_state, "CRUISE")

    def test_comms_mode_switch_clears_loiter_timer_and_checks_drone0(self):
        """Switching to wireless or mesh modes clears tether severance loiter timer,
        and switching back to TETHERED_RELAY is rejected if primary Drone 0 is dead."""
        sim = SwarmSimulator(drone_count=6)
        sim.cut_tether()
        self.assertTrue(sim.tether_severed)
        self.assertEqual(sim.lost_link_loiter_timer, 30.0)

        # Switch to P2P MESH
        ok_p2p = sim.set_comms_mode("GOSSIP_CONSENSUS")
        self.assertTrue(ok_p2p)
        self.assertEqual(sim.comms_mode, "GOSSIP_CONSENSUS")
        self.assertEqual(sim.lost_link_loiter_timer, 0.0)

        # Telemetry snapshot must report severed=False in wireless modes
        snap = sim.get_state_snapshot()
        self.assertFalse(snap["tether"]["severed"])
        self.assertFalse(snap["tether"]["deployed"])

        # Stepping in P2P mode must not pause or re-engage the loiter timer
        sim.step()
        self.assertEqual(sim.lost_link_loiter_timer, 0.0)

        # Kill Drone 0
        sim.kill_drone(0)
        self.assertTrue(sim.drones[0].crashed)

        # Attempt to re-engage TETHERED_RELAY must fail cleanly because Drone 0 is destroyed
        ok_tether = sim.set_comms_mode("TETHERED_RELAY")
        self.assertFalse(ok_tether)
        self.assertNotEqual(sim.comms_mode, "TETHERED_RELAY")

    def test_boids_tether_avoidance_clears_segment(self):
        """Drones running boids in Tethered Relay mode calculate 3D clearance from the cable segment."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_formation("NATURAL_FLOCK")
        self.assertEqual(sim.formation_mode, "NATURAL_FLOCK")

        # Position Drone 1 right next to the tether cable (cable stretches from [0,0,0] to Drone 0 at [0,0,20])
        sim.drones[0].pos = np.array([0.0, 0.0, 20.0], dtype=float)
        sim.drones[1].pos = np.array([1.5, 0.0, 10.0], dtype=float)  # 1.5m perpendicular to cable
        sim.drones[1].vel = np.array([-1.0, 0.0, 0.0], dtype=float)  # flying towards cable

        brain = sim.brain
        acc = brain._compute_boid_acceleration(sim.drones[1], sim.drones[0])

        # Acceleration must strongly push Drone 1 outward (positive X) away from cable
        self.assertGreater(acc[0], 0.0)
        self.assertGreater(acc[0], 5.0, "Tether avoidance force must be strong enough to prevent collision")


if __name__ == "__main__":
    unittest.main()
