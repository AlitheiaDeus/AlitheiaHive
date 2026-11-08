# ==============================================================================
# SwarmTech3_UpRising: 1-Click Aux Mission - Patrol to BINGO RTB Test Suite
# Tests for TKT-58 / PLN-02:
#   1. Pre-flight capability authorization check (FORMATION_FLIGHT + PATROL)
#   2. Pre-flight insufficient battery rejection (battery <= threshold)
#   3. Aux mission deployment, route loop initiation, and state snapshot telemetry
#   4. Battery monitoring, BINGO trigger, and Stage 1 RTB Overhead Marshal transition
#   5. Stage 2 automated coordinated ATC landing pad allocation upon marshal arrival
#   6. Operator manual abort and set_target override
#   7. Brain Airgap: Autonomous OBC BINGO failsafe during LOST_LINK
#   8. Tactical power drain rates and tethered infinite power protection
# ==============================================================================

import unittest
import numpy as np

from SIM.backend.sitl_engine import (
    SwarmSimulator,
    DT,
    TIER_LEADER,
    TIER_ESCORT,
    TETHER_POWER_DATA,
    TETHER_NONE,
)
from OBC.backend.drone_firmware import Drone


class TestPatrolToBingoRTB(unittest.TestCase):
    """Test suite for TKT-58: 1-Click Aux Mission (Patrol to BINGO RTB)."""

    def setUp(self):
        self.sim = SwarmSimulator(drone_count=4)

    def test_preflight_capability_authorization(self):
        """Verify pre-flight capability gate: requires FORMATION_FLIGHT and PATROL."""
        # 1. Standard fleet airframes have FORMATION_FLIGHT and PATROL -> Authorized
        auth = self.sim.c2.check_mission_authorization("PATROL_TO_BINGO")
        self.assertTrue(auth.get("authorized", False), f"Expected authorized, got {auth}")
        self.assertEqual(auth.get("status"), "AUTHORIZED")

        # 2. Airframe lacking PATROL capability -> Rejected with TIER_AUTHORIZATION_FAILED
        for d in self.sim.drones:
            d.capabilities = ["FORMATION_FLIGHT"]  # Strip PATROL
        res = self.sim.deploy_patrol_to_bingo(bingo_threshold=25.0)
        self.assertEqual(res.get("status"), "ERROR")
        self.assertEqual(res.get("code"), "TIER_AUTHORIZATION_FAILED")

        # 3. Fleet with zero airborne drones -> Rejected with NO_AIRBORNE_AIRFRAMES
        for d in self.sim.drones:
            d.flight_state = "LANDED"
        res = self.sim.deploy_patrol_to_bingo(bingo_threshold=25.0)
        self.assertEqual(res.get("status"), "ERROR")
        self.assertEqual(res.get("code"), "TIER_AUTHORIZATION_FAILED")
        self.assertEqual(res.get("reason"), "NO_AIRBORNE_AIRFRAMES")

    def test_insufficient_battery_rejection(self):
        """Verify pre-flight rejection if fleet battery is already below requested threshold."""
        for d in self.sim.drones:
            d.battery = 24.0  # Already below 25.0%
        res = self.sim.deploy_patrol_to_bingo(bingo_threshold=25.0)
        self.assertEqual(res.get("status"), "ERROR")
        self.assertEqual(res.get("code"), "INSUFFICIENT_BATTERY")
        self.assertFalse(self.sim.bingo_mission_active)

    def test_deploy_patrol_to_bingo_initialization(self):
        """Verify deployment initializes state machine, looping route, and telemetry."""
        res = self.sim.deploy_patrol_to_bingo(bingo_threshold=30.0, pattern="BOX_PATROL")
        self.assertEqual(res.get("status"), "OK")
        self.assertEqual(res.get("action"), "PATROL_TO_BINGO_DEPLOYED")
        self.assertEqual(res.get("pattern"), "BOX_PATROL")
        self.assertEqual(res.get("bingo_threshold"), 30.0)

        # C2 and Sim properties
        self.assertTrue(self.sim.bingo_mission_active)
        self.assertEqual(self.sim.bingo_threshold, 30.0)
        self.assertFalse(self.sim.bingo_triggered)
        self.assertEqual(self.sim.bingo_stage, "PATROLLING")
        self.assertTrue(self.sim.route_active)
        self.assertTrue(self.sim.route_loop)

        # Telemetry snapshot validation
        snap = self.sim.get_state_snapshot()
        self.assertIn("patrol_to_bingo", snap)
        p_data = snap["patrol_to_bingo"]
        self.assertTrue(p_data["active"])
        self.assertEqual(p_data["threshold"], 30.0)
        self.assertFalse(p_data["triggered"])
        self.assertEqual(p_data["stage"], "PATROLLING")
        self.assertEqual(p_data["pattern"], "BOX_PATROL")

    def test_bingo_trigger_and_stage1_rtb_marshal(self):
        """Verify that when battery reaches BINGO threshold, Stage 1 RTB Overhead Marshal engages."""
        res = self.sim.deploy_patrol_to_bingo(bingo_threshold=35.0, pattern="BOX_PATROL")
        self.assertEqual(res.get("status"), "OK")

        # Advance 10 steps nominal
        for _ in range(10):
            self.sim.step(DT)
        self.assertFalse(self.sim.bingo_triggered)
        self.assertEqual(self.sim.bingo_stage, "PATROLLING")

        # Drop battery of an untethered follower below threshold
        self.sim.drones[1].battery = 34.5

        # Step C2 & simulation: BINGO watchdog trips
        self.sim.step(DT)

        self.assertTrue(self.sim.bingo_triggered)
        self.assertEqual(self.sim.bingo_stage, "RTB_TRANSIT")
        self.assertFalse(self.sim.route_active, "Patrol route must be cancelled on BINGO trigger")
        self.assertTrue(self.sim.overhead_marshal_active, "Overhead marshal must be active for Stage 1 RTB")
        self.assertTrue(self.sim.swarm_rtb_active)
        self.assertTrue(np.allclose(self.sim.target_pos, [0.0, 0.0, 25.0]))

    def test_two_stage_rtb_automatic_land_swarm(self):
        """Verify automated transition from Stage 1 Marshal Hold to Stage 2 ATC Landing."""
        self.sim.deploy_patrol_to_bingo(bingo_threshold=35.0, pattern="BOX_PATROL")
        self.sim.drones[1].battery = 34.5

        # Trip BINGO watchdog
        self.sim.step(DT)
        self.assertTrue(self.sim.bingo_triggered)
        self.assertEqual(self.sim.bingo_stage, "RTB_TRANSIT")

        # Position fleet inside Base overhead marshal area
        for d in self.sim.drones:
            d.pos = np.array([5.0, 5.0, 25.0], dtype=float)
            d.vel = np.zeros(3, dtype=float)

        # Step simulation: C2 detects fleet arrival at marshal and auto-triggers Stage 2 landing
        self.sim.step(DT)

        self.assertEqual(self.sim.bingo_stage, "LANDING")
        self.assertEqual(self.sim.recovery_stage, "LANDING")

        # Drones must have landing pads allocated and be in descending state
        assigned_pads = [d.landing_pad_idx for d in self.sim.drones if d.landing_pad_idx is not None]
        self.assertGreater(len(assigned_pads), 0, "Drones must be assigned recovery pads")

    def test_operator_manual_abort(self):
        """Verify operator can abort Patrol to BINGO mission at any time."""
        self.sim.deploy_patrol_to_bingo(bingo_threshold=25.0)
        self.assertTrue(self.sim.bingo_mission_active)

        # 1. Abort via stop_patrol_to_bingo
        stop_res = self.sim.stop_patrol_to_bingo()
        self.assertEqual(stop_res.get("status"), "OK")
        self.assertFalse(self.sim.bingo_mission_active)
        self.assertEqual(self.sim.bingo_stage, "IDLE")

        # 2. Deploy again and abort via manual set_target away from base
        self.sim.deploy_patrol_to_bingo(bingo_threshold=25.0)
        self.assertTrue(self.sim.bingo_mission_active)
        self.sim.set_target([50.0, 50.0, 30.0])
        self.assertFalse(self.sim.bingo_mission_active)
        self.assertEqual(self.sim.bingo_stage, "IDLE")

    def test_obc_brain_airgap_lost_link_bingo_failsafe(self):
        """Verify Brain Airgap: An isolated drone in LOST_LINK autonomically returns on low battery."""
        d = self.sim.drones[1]
        # Position beyond GCS and Leader radio horizon (160m distance)
        d.pos = np.array([160.0, 160.0, 25.0], dtype=float)
        d.corridor_stack.record([30.0, 30.0, 25.0])
        d.corridor_stack.record([100.0, 100.0, 25.0])

        # Step until 2.0s watchdog expires so drone is genuinely in LOST_LINK
        for _ in range(130):
            self.sim.step(DT)
        self.assertTrue(d.lost_link)

        # Set battery below drone's autonomous bingo threshold (20.0%)
        d.battery = 19.5
        self.assertFalse(d.bingo_failsafe_triggered)

        # Step physics & OBC firmware in simulation
        self.sim.step(DT)

        # Drone OBC must autonomously engage CORRIDOR_ERTB
        self.assertTrue(d.bingo_failsafe_triggered)
        self.assertEqual(d.flight_state, "CORRIDOR_ERTB")

        # Critical battery depletion <= 3.0% must engage emergency soft settle
        d.battery = 2.8
        self.sim.step(DT)
        self.assertTrue(d.gnss_settled)
        self.assertEqual(d.status, "EMERGENCY_SETTLE")

    def test_tethered_power_protection_and_tactical_drain(self):
        """Verify tethered airframes have infinite power and untethered drain tactically."""
        # Drone 0 in TETHERED_RELAY has TETHER_POWER_DATA
        d0 = self.sim.drones[0]
        d0.tether_type = TETHER_POWER_DATA
        d1 = self.sim.drones[1]
        d1.tether_type = TETHER_NONE

        # Advance 120 steps (2.0s) with 5x drain multiplier
        self.sim.battery_drain_multiplier = 5.0
        for _ in range(120):
            self.sim.step(DT)

        # D0 (tethered power) remains at 100%
        self.assertEqual(d0.battery, 100.0)
        # D1 (untethered) has drained
        self.assertLess(d1.battery, 99.0)


if __name__ == "__main__":
    unittest.main()
