# ==============================================================================
# SwarmTech3_UpRising: Search and Recall Courier Mule to LKP Test Suite
# Tests for TKT-51 / PLN-02:
#   1. Pre-flight capability authorization check (RECON + S&R_OPTICAL)
#   2. Pre-flight rejection when no lost or dark drones are detected
#   3. Courier allocation, dispatch, target LKP setting, and radio silence
#   4. Stationary LKP dwell and 60m P2P rescue handshake
#   5. Tandem escort return and GCS radio horizon downlink handover
#   6. MIA loiter timeout (15s) and safe solo return to Base
#   7. Operator manual mission abort (STOP_SEARCH_AND_RECALL)
#   8. Courier loss / crash failsafe handling
# ==============================================================================

import unittest
import numpy as np

from SIM.backend.sitl_engine import (
    SwarmSimulator,
    DT,
    RANGE_HOWL,
    RANGE_P2P,
)


class TestEscortCourierLKP(unittest.TestCase):
    """Test suite for TKT-51: Physical Courier / Search-and-Recall Rescue Mule to LKP."""

    def setUp(self):
        # Initialize standard 4-drone simulation
        self.sim = SwarmSimulator(drone_count=4)

    def test_preflight_capability_authorization(self):
        """Verify pre-flight capability gate: courier requires RECON and S&R_OPTICAL."""
        # 1. Standard fleet has RECON and S&R_OPTICAL -> Authorized
        auth = self.sim.c2.check_mission_authorization("SEARCH_AND_RECALL", target_drone_id=0)
        self.assertTrue(auth.get("authorized", False), f"Expected authorized, got {auth}")

        # 2. Airframe lacking S&R_OPTICAL -> Rejected with TIER_AUTHORIZATION_FAILED
        courier = self.sim.drones[0]
        courier.capabilities = ["FORMATION_FLIGHT", "PATROL"]  # Stripped S&R capabilities
        auth_bad = self.sim.c2.check_mission_authorization("SEARCH_AND_RECALL", target_drone_id=0)
        self.assertFalse(auth_bad.get("authorized", True))
        self.assertEqual(auth_bad.get("code") or auth_bad.get("reason"), "TIER_AUTHORIZATION_FAILED")

    def test_no_lost_drones_error(self):
        """Verify pre-flight rejection if all fleet airframes are healthy and linked."""
        res = self.sim.deploy_search_and_recall()
        self.assertEqual(res.get("status"), "ERROR")
        self.assertEqual(res.get("reason"), "NO_LOST_DRONES_DETECTED")
        self.assertFalse(self.sim.c2.courier_active)

    def test_courier_dispatch_and_initialization(self):
        """Verify courier allocation, dispatch to target LKP, and radio silence setting."""
        # Simulate Drone 2 going dark beyond radio horizon
        lost_drone = self.sim.drones[2]
        lost_pos = np.array([160.0, 40.0, 25.0], dtype=float)
        lost_drone.pos = np.copy(lost_pos)
        lost_drone.lkp = np.copy(lost_pos)
        lost_drone.lost_link = True
        lost_drone.downlink_active = False
        lost_drone.flight_state = "LOST_LINK"
        lost_drone.gcs_heartbeat_watchdog = 0.0

        res = self.sim.deploy_search_and_recall(target_drone_id=2)
        self.assertEqual(res.get("status"), "OK")
        self.assertEqual(res.get("action"), "SEARCH_AND_RECALL_DEPLOYED")
        self.assertEqual(res.get("target_drone_id"), 2)
        self.assertEqual(res.get("stage"), "TRANSIT_OUTBOUND")

        # Verify C2 state
        self.assertTrue(self.sim.c2.courier_active)
        self.assertEqual(self.sim.c2.courier_target_drone_id, 2)
        self.assertEqual(self.sim.c2.courier_stage, "TRANSIT_OUTBOUND")

        courier_id = res.get("courier_id")
        courier = self.sim.get_drone_by_id(courier_id)
        self.assertIsNotNone(courier)
        self.assertTrue(courier.courier_mission_active)
        self.assertTrue(courier.radio_silence_permitted)
        self.assertEqual(courier.flight_state, "COURIER_TRANSIT")
        self.assertTrue(np.allclose(courier.acknowledged_target, lost_pos))

        # Check telemetry snapshot
        snap = self.sim.get_state_snapshot()
        self.assertIn("search_and_recall", snap)
        s_data = snap["search_and_recall"]
        self.assertTrue(s_data["active"])
        self.assertEqual(s_data["stage"], "TRANSIT_OUTBOUND")
        self.assertEqual(s_data["courier_id"], courier_id)
        self.assertEqual(s_data["target_drone_id"], 2)

    def test_stationary_lkp_dwell_and_p2p_handshake(self):
        """Verify stationary dwell upon arrival at LKP and P2P rescue handshake within 60m."""
        lost_drone = self.sim.drones[2]
        lost_pos = np.array([160.0, 40.0, 25.0], dtype=float)
        lost_drone.pos = np.copy(lost_pos)
        lost_drone.lkp = np.copy(lost_pos)
        lost_drone.lost_link = True
        lost_drone.downlink_active = False
        lost_drone.flight_state = "LOST_LINK"
        lost_drone.gcs_heartbeat_watchdog = 0.0

        # Deploy courier (Drone 1)
        res = self.sim.deploy_search_and_recall(target_drone_id=2, courier_id=1)
        self.assertEqual(res.get("status"), "OK")
        courier = self.sim.drones[1]

        # 1. Fly courier directly to arrival envelope (d <= 5m of LKP)
        courier.pos = lost_pos + np.array([3.0, 0.0, 0.0], dtype=float)
        self.sim.step(DT)

        # Courier should switch to RENDEZVOUS_DWELL
        self.assertEqual(self.sim.c2.courier_stage, "RENDEZVOUS_DWELL")
        self.assertEqual(courier.flight_state, "RENDEZVOUS_DWELL")

        # 2. P2P handshake triggers tandem escort (lost_drone is within 3m <= RANGE_P2P 60m)
        self.sim.step(DT)
        self.assertEqual(self.sim.c2.courier_stage, "ESCORT_INBOUND")
        self.assertEqual(courier.flight_state, "ESCORT_LEAD")
        self.assertEqual(courier.courier_escortee_id, 2)
        self.assertTrue(np.allclose(courier.acknowledged_target, [0.0, 0.0, 25.0]))

        self.assertEqual(lost_drone.flight_state, "ESCORT_REJOIN")
        self.assertEqual(lost_drone.escort_leader_id, courier.id)

    def test_tandem_escort_return_and_horizon_handover(self):
        """Verify courier leads wingman back to Base and handoff completes when penetrating GCS horizon."""
        lost_drone = self.sim.drones[2]
        lost_pos = np.array([160.0, 40.0, 25.0], dtype=float)
        lost_drone.pos = np.copy(lost_pos)
        lost_drone.lkp = np.copy(lost_pos)
        lost_drone.lost_link = True
        lost_drone.downlink_active = False
        lost_drone.flight_state = "LOST_LINK"
        lost_drone.gcs_heartbeat_watchdog = 0.0

        self.sim.deploy_search_and_recall(target_drone_id=2, courier_id=1)
        courier = self.sim.drones[1]

        # Place at rendezvous and trigger escort
        courier.pos = lost_pos + np.array([2.0, 0.0, 0.0], dtype=float)
        self.sim.step(DT)  # transitions to RENDEZVOUS_DWELL
        self.sim.step(DT)  # transitions to ESCORT_INBOUND
        self.assertEqual(self.sim.c2.courier_stage, "ESCORT_INBOUND")

        # Advance tandem escort kinematics: verify lost_drone tracks behind courier
        courier.vel = np.array([-10.0, 0.0, 0.0], dtype=float)
        self.sim.step(DT)
        # Lost drone acknowledged_target should be ~4m behind courier
        expected_slot = courier.pos - (courier.vel / np.linalg.norm(courier.vel[:2])) * 4.0
        expected_slot[2] = courier.pos[2]
        self.assertTrue(np.allclose(lost_drone.acknowledged_target, expected_slot, atol=0.5))

        # 3. Simulate crossing GCS radio horizon (d <= 140m)
        courier.pos = np.array([80.0, 10.0, 25.0], dtype=float)
        lost_drone.pos = np.array([84.0, 10.0, 25.0], dtype=float)
        lost_drone.downlink_active = True
        self.sim.step(DT)

        # Handover complete: lost drone re-enrolled into fleet, courier mission ended
        self.assertEqual(self.sim.c2.courier_stage, "HANDOVER_COMPLETE")
        self.assertFalse(self.sim.c2.courier_active)
        self.assertEqual(courier.flight_state, "CRUISE")
        self.assertEqual(lost_drone.flight_state, "CRUISE")
        self.assertFalse(lost_drone.lost_link)
        self.assertIsNone(lost_drone.escort_leader_id)

    def test_mia_timeout_alone_return(self):
        """Verify courier dwells for 15s timeout if target is MIA, then returns alone."""
        lost_drone = self.sim.drones[2]
        lost_pos = np.array([160.0, 40.0, 25.0], dtype=float)
        # Lost drone crashed/drifted far away from its LKP before courier arrived (> RANGE_P2P = 60m)
        lost_drone.pos = np.array([250.0, 200.0, 0.0], dtype=float)
        lost_drone.lkp = np.copy(lost_pos)
        lost_drone.lost_link = True
        lost_drone.crashed = True
        lost_drone.downlink_active = False
        lost_drone.flight_state = "CRASHED"
        lost_drone.gcs_heartbeat_watchdog = 0.0

        self.sim.deploy_search_and_recall(target_drone_id=2, courier_id=1)
        courier = self.sim.drones[1]

        # Arrive at LKP
        courier.pos = np.copy(lost_pos)
        self.sim.step(DT)
        self.assertEqual(self.sim.c2.courier_stage, "RENDEZVOUS_DWELL")

        # Advance dwell timer past 15s timeout
        for _ in range(35):
            self.sim.c2.courier_dwell_timer += 0.5
            self.sim.step(DT)
            if self.sim.c2.courier_stage != "RENDEZVOUS_DWELL":
                break

        self.assertEqual(self.sim.c2.courier_stage, "MIA_RETURN")
        self.assertTrue(np.allclose(courier.acknowledged_target, [0.0, 0.0, 25.0]))

        # Courier arrives back at Base (d <= 15m)
        courier.pos = np.array([5.0, 5.0, 25.0], dtype=float)
        self.sim.step(DT)
        self.assertEqual(self.sim.c2.courier_stage, "MIA_COMPLETE")
        self.assertFalse(self.sim.c2.courier_active)

    def test_operator_manual_abort(self):
        """Verify operator can manually abort active Search and Recall mission."""
        lost_drone = self.sim.drones[2]
        lost_pos = np.array([160.0, 40.0, 25.0], dtype=float)
        lost_drone.pos = np.copy(lost_pos)
        lost_drone.lkp = np.copy(lost_pos)
        lost_drone.lost_link = True
        lost_drone.downlink_active = False

        self.sim.deploy_search_and_recall(target_drone_id=2, courier_id=1)
        self.assertTrue(self.sim.c2.courier_active)

        res = self.sim.stop_search_and_recall()
        self.assertEqual(res.get("status"), "OK")
        self.assertFalse(self.sim.c2.courier_active)
        self.assertEqual(self.sim.c2.courier_stage, "IDLE")

        courier = self.sim.drones[1]
        self.assertFalse(courier.courier_mission_active)
        self.assertFalse(courier.radio_silence_permitted)
        self.assertEqual(courier.flight_state, "CRUISE")

    def test_courier_lost_failsafe(self):
        """Verify graceful failsafe deactivation if courier drone crashes mid-mission."""
        lost_drone = self.sim.drones[2]
        lost_pos = np.array([160.0, 40.0, 25.0], dtype=float)
        lost_drone.pos = np.copy(lost_pos)
        lost_drone.lkp = np.copy(lost_pos)
        lost_drone.lost_link = True
        lost_drone.downlink_active = False

        self.sim.deploy_search_and_recall(target_drone_id=2, courier_id=1)
        courier = self.sim.drones[1]

        # Courier crashes mid-flight
        courier.crashed = True
        courier.flight_state = "CRASHED"
        self.sim.step(DT)

        self.assertEqual(self.sim.c2.courier_stage, "COURIER_LOST")
        self.assertFalse(self.sim.c2.courier_active)


if __name__ == "__main__":
    unittest.main()
