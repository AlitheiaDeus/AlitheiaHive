# ==============================================================================
# SwarmTech3_UpRising: Autonomous Swarm RTB & Coordinated Soft-Landing Test Suite
# Tests:
#   1. Pad geometry (>= 2.5m spacing, > 3.0m tether cylinder clearance, Z=0.5m)
#   2. Stage 1: Commanded RTB (Hold) / Overhead Marshal sentry ring at Z=25m
#   3. Cancel RTB & resume mission pursuit
#   4. Stage 2: Commanded LAND_SWARM with ATC pad reservation & 2.0s stagger
#   5. 2-stage vertical deceleration sink (-1.5 m/s to 1.0m, -0.5 m/s to 0.5m)
#   6. Touchdown disarm (motors cut, rotors 0 RPM, immune to formation pull)
#   7. Individual drone landing and relaunch
#   8. Doctrine A automated RTB transition upon loiter expiration
# ==============================================================================

import unittest
import numpy as np
import math

from SIM.backend.sitl_engine import SwarmSimulator, DT, TIER_LEADER, TIER_ESCORT
from OBC.backend.drone_firmware import Drone


class TestAutonomousSwarmRTBLanding(unittest.TestCase):
    """Test suite for TKT-19 2-Stage Tactical Recovery & Soft-Landing."""

    def test_pad_geometry_and_clearance(self):
        """Verify pad layout: >= 2.5m separation, > 3.0m tether clearance, Z=0.5m."""
        pads = [SwarmSimulator.get_pad_position(i) for i in range(8)]

        # 1. Verify Z coordinate is ground contact height 0.5m
        for i, pad in enumerate(pads):
            self.assertAlmostEqual(pad[2], 0.5, places=2, msg=f"Pad {i} Z should be 0.5m")

        # 2. Verify all pads are outside the 3.0m tether clearance envelope
        anchor = np.array([0.0, 0.0, 0.0])
        for i, pad in enumerate(pads):
            dist_to_anchor = float(np.linalg.norm(pad - anchor))
            self.assertGreater(
                dist_to_anchor,
                3.0,
                msg=f"Pad {i} at {pad} must be outside 3.0m tether cylinder (was {dist_to_anchor:.2f}m)"
            )

        # 3. Verify pairwise pad separation is at least 2.5m (well above 1.5m Zone 4 clash)
        for i in range(len(pads)):
            for j in range(i + 1, len(pads)):
                dist = float(np.linalg.norm(pads[i] - pads[j]))
                self.assertGreaterEqual(
                    dist,
                    2.49,
                    msg=f"Pads {i} and {j} must have >= 2.5m separation (was {dist:.2f}m)"
                )

    def test_stage1_overhead_marshal(self):
        """Test Stage 1: Commanded RTB (Hold) puts fleet into overhead marshal holding pattern."""
        sim = SwarmSimulator(drone_count=4)
        sim.command_rtb_hold()

        self.assertTrue(sim.overhead_marshal_active)
        self.assertTrue(sim.swarm_rtb_active)
        self.assertEqual(sim.recovery_stage, "OVERHEAD_MARSHAL")
        self.assertFalse(sim.patrol_active)
        self.assertTrue(np.allclose(sim.target_pos, [0.0, 0.0, 25.0]))

        # Advance sim for 150 steps (2.5 seconds)
        for _ in range(150):
            sim.step(DT)

        # All drones must remain airborne, armed, altitude >= 20m, zero crashes
        for drone in sim.drones:
            self.assertFalse(drone.crashed, f"Drone {drone.id} should not crash in marshal")
            self.assertGreater(drone.rotor_rpm, 4000.0, "Rotors must remain armed in marshal")
            self.assertGreater(drone.pos[2], 18.0, "Drones must maintain high holding altitude")

        # Snapshot should report overhead_marshal_active and recovery_stage
        snap = sim.get_state_snapshot()
        self.assertTrue(snap["overhead_marshal_active"])
        self.assertEqual(snap["recovery_stage"], "OVERHEAD_MARSHAL")

    def test_cancel_rtb(self):
        """Test canceling RTB returns living drones to active mission cruise."""
        sim = SwarmSimulator(drone_count=3)
        sim.command_rtb_hold()
        self.assertTrue(sim.overhead_marshal_active)

        sim.cancel_rtb()
        self.assertFalse(sim.overhead_marshal_active)
        self.assertFalse(sim.swarm_rtb_active)
        self.assertEqual(sim.recovery_stage, "IDLE")

        for drone in sim.drones:
            self.assertEqual(drone.flight_state, "CRUISE")

    def test_target_dispatch_auto_cancels_marshal(self):
        """Dispatching a new mission target waypoint away from base anchor automatically disengages marshal."""
        sim = SwarmSimulator(drone_count=3)
        sim.command_rtb_hold()
        self.assertTrue(sim.overhead_marshal_active)

        # Dispatch custom target
        sim.set_target([80.0, 50.0, 35.0])
        self.assertFalse(sim.overhead_marshal_active)
        self.assertFalse(sim.swarm_rtb_active)

    def test_stage2_land_swarm_atc_stagger(self):
        """Test Stage 2: Commanded LAND_SWARM allocates pads and staggers descent by 2.0s."""
        sim = SwarmSimulator(drone_count=4)
        queued = sim.command_land_swarm()

        self.assertEqual(queued, [0, 1, 2, 3])
        self.assertEqual(sim.recovery_stage, "LANDING")
        self.assertFalse(sim.overhead_marshal_active)

        # Pad allocation must be distinct
        pad_indices = [d.landing_pad_idx for d in sim.drones]
        self.assertEqual(len(set(pad_indices)), 4, "Each drone must receive a unique recovery pad")

        # Stagger clearance interval must be 2.0s
        self.assertAlmostEqual(sim.drones[0].landing_wait_timer, 0.0)
        self.assertAlmostEqual(sim.drones[1].landing_wait_timer, 2.0)
        self.assertAlmostEqual(sim.drones[2].landing_wait_timer, 4.0)
        self.assertAlmostEqual(sim.drones[3].landing_wait_timer, 6.0)

        # Initial landing stages
        self.assertEqual(sim.drones[0].landing_stage, "APPROACH")
        self.assertEqual(sim.drones[1].landing_stage, "WAITING_CLEARANCE")

    def test_coordinated_soft_landing_full_simulation(self):
        """Full simulation of 4 drones returning to base and executing downwash-gated 2-stage soft landing."""
        sim = SwarmSimulator(drone_count=4)
        sim.command_land_swarm()

        # Step sim until all drones touch down and disarm (downwash-gated sequential landing)
        for _ in range(4000):
            sim.step(DT)
            if all(d.flight_state == "LANDED" for d in sim.drones):
                break

        # Verify all drones reached LANDED state safely
        for drone in sim.drones:
            self.assertEqual(drone.flight_state, "LANDED", f"Drone {drone.id} should be LANDED")
            self.assertEqual(drone.status, "LANDED")
            self.assertFalse(drone.crashed, f"Drone {drone.id} should not clash during recovery")
            self.assertAlmostEqual(drone.pos[2], 0.5, delta=0.05, msg="Should be settled at 0.5m on pad")
            self.assertAlmostEqual(float(np.linalg.norm(drone.vel)), 0.0, delta=0.05, msg="Velocity must be 0")
            self.assertAlmostEqual(drone.rotor_rpm, 0.0, delta=0.1, msg="Rotors must be powered down to 0 RPM")

            # Pad alignment check
            expected_pad = sim.get_pad_position(drone.landing_pad_idx)
            self.assertAlmostEqual(float(np.linalg.norm(drone.pos[:2] - expected_pad[:2])), 0.0, delta=0.1)

    def test_single_drone_landing_and_relaunch(self):
        """Test commanding a single escort to land, followed by vertical relaunch."""
        sim = SwarmSimulator(drone_count=3)

        # Command drone 1 to land
        success = sim.command_land_drone(1)
        self.assertTrue(success)
        self.assertEqual(sim.drones[1].flight_state, "DESCENDING")
        self.assertEqual(sim.drones[1].landing_pad_idx, 0)

        # Step until drone 1 lands
        for _ in range(1000):
            sim.step(DT)
            if sim.drones[1].flight_state == "LANDED":
                break

        self.assertEqual(sim.drones[1].flight_state, "LANDED")
        self.assertEqual(sim.drones[1].rotor_rpm, 0.0)

        # Other drones should still be airborne in cruise
        self.assertIn(sim.drones[0].flight_state, ("CRUISE", "OVERHEAD_HOLD"))
        self.assertIn(sim.drones[2].flight_state, ("CRUISE", "OVERHEAD_HOLD"))

        # Now relaunch drone 1
        relaunch_ok = sim.command_takeoff_drone(1)
        self.assertTrue(relaunch_ok)
        self.assertEqual(sim.drones[1].flight_state, "TAKEOFF")
        self.assertEqual(sim.drones[1].rotor_rpm, 5200.0)
        self.assertNotIn(0, sim.landing_pads, "Pad 0 should be freed after takeoff")

        # Step until drone 1 climbs to safe altitude (Z >= 20m)
        for _ in range(300):
            sim.step(DT)
            if sim.drones[1].pos[2] >= 20.0:
                break

        self.assertGreaterEqual(sim.drones[1].pos[2], 20.0)
        self.assertIn(sim.drones[1].flight_state, ("EN_ROUTE", "CRUISE"))

    def test_doctrine_a_loiter_timeout_triggers_overhead_marshal(self):
        """Fleet Doctrine A: Tether cut loiters for 30s, then automatically triggers Stage 1 Overhead Marshal."""
        sim = SwarmSimulator(drone_count=3)
        sim.cut_tether()
        self.assertTrue(sim.tether_severed)
        self.assertEqual(sim.lost_link_loiter_timer, 30.0)
        self.assertFalse(sim.overhead_marshal_active)

        # Simulate 30 seconds of loiter (1800 ticks)
        for _ in range(1805):
            sim.step(DT)

        self.assertTrue(sim.overhead_marshal_active)
        self.assertTrue(sim.swarm_rtb_active)
        self.assertEqual(sim.recovery_stage, "OVERHEAD_MARSHAL")
        self.assertTrue(np.allclose(sim.target_pos, [0.0, 0.0, 25.0]))

    def test_out_of_horizon_recovery_rejection_and_reinforcement_muster(self):
        """Verify FIX-04: GCS commands are dropped beyond 120m radio horizon, and reinforcements muster locally."""
        sim = SwarmSimulator(drone_count=3)
        sim.comms_mode = "GOSSIP_CONSENSUS"

        # Relocate all drones well beyond 120m radio horizon
        for drone in sim.drones:
            drone.pos = np.array([160.0, 0.0, 30.0], dtype=float)

        self.assertFalse(sim.is_link_active(), "Link must be reported inactive when fleet is at 160m (>120m)")

        # 1. RTB command must be dropped over airgap
        res_rtb = sim.command_rtb_hold()
        self.assertIsInstance(res_rtb, dict)
        self.assertEqual(res_rtb.get("status"), "DROPPED")
        self.assertEqual(res_rtb.get("reason"), "OUT_OF_RANGE")
        self.assertFalse(sim.overhead_marshal_active, "Overhead marshal must NOT engage when command dropped")

        # 2. LAND_SWARM command must be dropped over airgap
        res_land = sim.command_land_swarm()
        self.assertIsInstance(res_land, dict)
        self.assertEqual(res_land.get("status"), "DROPPED")
        self.assertEqual(res_land.get("reason"), "OUT_OF_RANGE")

        # 3. Newly launched reinforcement drone cannot peek at distant fleet coordinates
        new_drone = sim.launch_reinforcement()
        self.assertIsNotNone(new_drone)

        # Reinforced drone must start in LOST_LINK with base muster coordinates, not leader's 160m pos
        self.assertTrue(new_drone.lost_link)
        self.assertEqual(new_drone.status, "LOST_LINK")
        self.assertTrue(
            np.allclose(new_drone.lkp, [0.0, 0.0, 20.0]),
            f"New drone must muster at local base [0, 0, 20], got {new_drone.lkp}"
        )

    def test_universal_lost_link_loiter_and_autonomous_rtb(self):
        """Verify TKT-54 / FIX-05: Untethered wireless fleet beyond 140m relies on autonomous OBC
        heartbeat watchdog (2.0s) and breadcrumb backtrack rather than global tether loiter timer."""
        sim = SwarmSimulator(drone_count=4)
        sim.set_comms_mode("GOSSIP_CONSENSUS")
        self.assertFalse(sim.tether_severed)

        # Record a known in-range breadcrumb at 110m before flying out to 160m
        for d in sim.drones:
            d.pos = np.array([110.0, float(d.id * 4.0), 25.0], dtype=float)
            d.record_breadcrumb(sim.sim_time)
            d.pos = np.array([160.0, float(d.id * 4.0), 25.0], dtype=float)

        self.assertFalse(sim.is_in_comms_range())

        # Step 1 tick: global tether timer stays 0.0 in wireless mode while OBC watchdog counts down
        sim.step(DT)
        self.assertEqual(sim.lost_link_loiter_timer, 0.0, "Global tether timer must stay 0.0 in wireless mode")
        self.assertLess(sim.drones[0].gcs_heartbeat_watchdog, 2.0)

        # Fast forward 2.5 seconds (150 ticks) so 2.0s OBC heartbeat watchdog expires
        for _ in range(150):
            sim.step(DT)

        # Autonomous OBC breadcrumb backtrack must engage across the isolated wireless fleet
        for d in sim.drones:
            self.assertTrue(d.lost_link)
            self.assertTrue(d.backtrack_hold, f"Drone #{d.id} must engage autonomous breadcrumb backtrack")
            self.assertEqual(d.flight_state, "BACKTRACK_HOLD")
            self.assertIsNotNone(d.backtrack_target)

    def test_coordinated_landing_holding_column_no_radial_scatter(self):
        """Verify FIX-08: Drones in WAITING_CLEARANCE hold in 3D pad columns without scattering to map edges."""
        sim = SwarmSimulator(drone_count=6)
        # Position all drones over Base area
        for i, d in enumerate(sim.drones):
            d.pos = np.array([float(i * 1.5), -6.0, 22.0], dtype=float)
            d.vel = np.zeros(3, dtype=float)

        queued = sim.command_land_swarm()
        self.assertEqual(len(queued), 6)
        self.assertEqual(sim.recovery_stage, "LANDING")

        # Step 180 ticks (3.0 seconds of simulation)
        for _ in range(180):
            sim.step(DT)

        # For all drones still waiting or approaching, verify horizontal distance from pad is bounded
        for d in sim.drones:
            if d.flight_state == "DESCENDING" and d.landing_stage == "WAITING_CLEARANCE":
                pad_pos = d.landing_pad if d.landing_pad is not None else sim.get_pad_position(d.landing_pad_idx or 0)
                dist_to_pad_xy = float(np.linalg.norm(d.pos[:2] - pad_pos[:2]))
                self.assertLess(
                    dist_to_pad_xy, 6.0,
                    f"Drone {d.id} drifted {dist_to_pad_xy:.1f}m away from pad while waiting clearance! Must hold column."
                )

    def test_base_horizon_high_gain_rf_dome(self):
        """Verify TKT-42: Within 50m radius of base anchor, high-gain base station radio guarantees 100% link quality."""
        sim = SwarmSimulator(drone_count=4)
        sim.set_rf_jamming(0.4)  # High EW electronic warfare jamming rate

        # Test distance <= 50m: guaranteed 100% link quality
        for test_dist in [0.0, 10.0, 25.0, 49.9, 50.0]:
            drop_prob, rssi, lq = sim.calculate_base_rf_link(test_dist)
            self.assertEqual(drop_prob, 0.0, f"Distance {test_dist}m must have 0 packet drop inside 50m dome")
            self.assertEqual(lq, 1.0, f"Distance {test_dist}m must guarantee 1.0 (100%) link quality")
            self.assertEqual(rssi, -35.0)

        # Beyond 50m with jamming, drop probability > 0 and quality < 1.0
        drop_prob_far, _, lq_far = sim.calculate_base_rf_link(55.0)
        self.assertGreater(drop_prob_far, 0.0, "Outside 50m dome, packet drop must reflect path loss/jamming")
        self.assertLess(lq_far, 1.0, "Outside 50m dome, link quality must degrade under jamming")

    def test_downwash_anti_vortex_gate(self):
        """Verify TKT-42: Downwash vortex gate holds Drone k+1 at Z >= 18m shelf until Drone k confirms TOUCHDOWN_CONFIRMED."""
        sim = SwarmSimulator(drone_count=2)
        # Position drones near base
        sim.drones[0].pos = np.array([0.0, -6.0, 20.0], dtype=float)
        sim.drones[1].pos = np.array([2.5, -6.0, 22.0], dtype=float)

        queued = sim.command_land_swarm()
        self.assertEqual(queued, [0, 1])

        # Drone 0 is cleared immediately for APPROACH
        self.assertEqual(sim.drones[0].landing_stage, "APPROACH")
        self.assertIsNone(sim.drones[0].gated_by_drone_id)

        # Drone 1 is gated by Drone 0
        self.assertEqual(sim.drones[1].landing_stage, "WAITING_CLEARANCE")
        self.assertEqual(sim.drones[1].gated_by_drone_id, 0)
        self.assertAlmostEqual(sim.drones[1].landing_wait_timer, 2.0)

        # Step 150 ticks (2.5 seconds): Drone 1 timer hits 0.0, but Drone 0 is still descending (not touched down)
        for _ in range(150):
            sim.step(DT)

        self.assertFalse(sim.drones[0].flight_state == "LANDED", "Drone 0 should still be descending at 2.5s")
        self.assertFalse(sim.drones[0].touchdown_confirmed)
        # Drone 1 MUST remain in WAITING_CLEARANCE held at altitude shelf Z >= 18m
        self.assertEqual(
            sim.drones[1].landing_stage,
            "WAITING_CLEARANCE",
            "Drone 1 must remain gated in WAITING_CLEARANCE while Drone 0 is descending"
        )
        self.assertGreaterEqual(sim.drones[1].pos[2], 18.0, "Drone 1 must hold altitude above ground effect shelf")

        # Step until Drone 0 confirms touchdown
        for _ in range(1200):
            sim.step(DT)
            if sim.drones[0].flight_state == "LANDED":
                break

        self.assertEqual(sim.drones[0].flight_state, "LANDED")
        self.assertTrue(sim.drones[0].touchdown_confirmed)
        self.assertEqual(sim.drones[0].landing_stage, "TOUCHDOWN_CONFIRMED")

        # 1 tick for Drone 1 to process touchdown clearance
        sim.step(DT)

        # Now Drone 1's downwash gate is released and it advances to APPROACH
        self.assertEqual(sim.drones[1].landing_stage, "APPROACH")

    def test_batched_atc_wave_management_and_overhead_marshal(self):
        """Verify TKT-42: Fleets exceeding 8 pads are batched: 8 pads assigned, remainder held in overhead marshal."""
        sim = SwarmSimulator(drone_count=12)
        queued = sim.command_land_swarm()

        # Exactly 8 drones receive recovery pad allocation
        self.assertEqual(len(queued), 8)
        self.assertEqual(len(sim.landing_pads), 8)
        self.assertEqual(sim.recovery_stage, "LANDING")
        self.assertTrue(sim.overhead_marshal_active)

        # Remaining 4 drones (drones 8..11) hold at overhead marshal shelves in [25, 35]m
        for did in range(8, 12):
            drone = sim.drones[did]
            self.assertEqual(drone.flight_state, "OVERHEAD_HOLD")
            self.assertEqual(drone.status, "MARSHAL_HOLD")
            self.assertIsNone(drone.landing_pad_idx)
            self.assertGreaterEqual(drone.acknowledged_target[2], 25.0)
            self.assertLessEqual(drone.acknowledged_target[2], 35.0)

        # Telemetry snapshot exposes ATC state
        snap = sim.get_state_snapshot()
        self.assertIn("atc", snap)
        self.assertEqual(snap["atc"]["capacity"], 8)
        self.assertEqual(snap["atc"]["pads_occupied"], 8)
        self.assertEqual(snap["atc"]["waiting_marshal_count"], 4)
        self.assertEqual(snap["atc"]["current_wave"], 1)

    def test_clear_pads_next_wave_release(self):
        """Verify TKT-42: CLEAR_PADS resets occupied pads and releases next wave from overhead marshal."""
        sim = SwarmSimulator(drone_count=12)
        sim.command_land_swarm()

        # Simulate first wave of 8 drones completing touchdown
        for did in range(8):
            d = sim.drones[did]
            d.flight_state = "LANDED"
            d.status = "LANDED"
            d.landing_stage = "TOUCHDOWN_CONFIRMED"
            d.touchdown_confirmed = True
            d.pos = sim.get_pad_position(d.landing_pad_idx)
            d.vel = np.zeros(3, dtype=float)
            d.rotor_rpm = 0.0

        # Operator triggers CLEAR_PADS // NEXT WAVE
        res = sim.clear_pads()
        self.assertEqual(res["status"], "OK")
        self.assertEqual(res["cleared_pads"], 8)
        self.assertEqual(res["released"], 4)
        self.assertEqual(res["current_wave"], 2)

        # The 4 waiting drones (8..11) are now allocated pads 0..3 in Wave 2!
        for did in range(8, 12):
            drone = sim.drones[did]
            self.assertEqual(drone.flight_state, "DESCENDING")
            self.assertIsNotNone(drone.landing_pad_idx)
            self.assertIn(drone.landing_pad_idx, range(4))

        # No drones left holding in marshal
        self.assertFalse(sim.overhead_marshal_active)

    def test_automated_turnaround_countdown(self):
        """Verify TKT-42: Automated 12s turnaround countdown clears pads and launches next wave when timers elapse."""
        sim = SwarmSimulator(drone_count=10)
        sim.command_land_swarm()

        # Mark first wave of 8 drones as landed
        for did in range(8):
            d = sim.drones[did]
            d.flight_state = "LANDED"
            d.status = "LANDED"
            d.touchdown_confirmed = True
            d.rotor_rpm = 0.0

        # Step 1 tick: turnaround timer should engage at 12.0s
        sim.step(DT)
        self.assertGreater(sim.turnaround_timer, 0.0)
        self.assertLessEqual(sim.turnaround_timer, 12.0)

        # Step 725 ticks (12.1 seconds)
        for _ in range(725):
            sim.step(DT)

        # Turnaround timer expired: pads cleared and wave 2 (remaining 2 drones) released!
        self.assertEqual(sim.atc_current_wave, 2)
        self.assertEqual(sim.drones[8].flight_state, "DESCENDING")
        self.assertEqual(sim.drones[9].flight_state, "DESCENDING")


if __name__ == "__main__":
    unittest.main()
