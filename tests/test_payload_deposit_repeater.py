# ==============================================================================
# SwarmTech3_UpRising: Cairn G1 Solar Repeater Payload Drop Test Suite
# Tests for TKT-59 / PLN-04:
#   1. Pre-flight capability authorization check (HEAVY_PAYLOAD + DROP_SERVO)
#   2. Pre-flight rejection if payload is already released (NO_PAYLOAD_ATTACHED)
#   3. Carrier allocation, dispatch, target coordinate setting, and transit
#   4. Precision descent to Z=3.0m and stable hover release latch actuation
#   5. Ballistic touchdown under gravity and ground repeater registration
#   6. Mesh network comms range extension across active ground repeater
#   7. GNSS drift reset anchor in denied airspace (RTK / UWB anchor function)
#   8. Carrier safe egress altitude ascension and Base RTB transition
#   9. Operator manual abort (STOP_PAYLOAD_DROP)
#   10. Carrier loss / crash failsafe handling
# ==============================================================================

import unittest
import numpy as np

from SIM.backend.sitl_engine import (
    SwarmSimulator,
    DT,
    RANGE_HOWL,
    RANGE_P2P,
)
from OBC.backend.drone_firmware import GroundRepeater


class TestPayloadDepositRepeater(unittest.TestCase):
    """Test suite for TKT-59: Cairn G1 Solar Repeater Payload Drop."""

    def setUp(self):
        # Initialize standard 4-drone simulation
        self.sim = SwarmSimulator(drone_count=4)

    def test_preflight_capability_authorization(self):
        """Verify pre-flight capability gate: requires HEAVY_PAYLOAD and DROP_SERVO."""
        # 1. Standard fleet airframes have HEAVY_PAYLOAD and DROP_SERVO -> Authorized
        auth = self.sim.c2.check_mission_authorization("PAYLOAD_DROP", target_drone_id=1)
        self.assertTrue(auth.get("authorized", False), f"Expected authorized, got {auth}")

        # 2. Airframe lacking DROP_SERVO -> Rejected with TIER_AUTHORIZATION_FAILED
        carrier = self.sim.drones[1]
        carrier.capabilities = ["FORMATION_FLIGHT", "RECON"]  # Stripped payload capabilities
        auth_bad = self.sim.c2.check_mission_authorization("PAYLOAD_DROP", target_drone_id=1)
        self.assertFalse(auth_bad.get("authorized", True))
        self.assertEqual(auth_bad.get("code") or auth_bad.get("reason"), "TIER_AUTHORIZATION_FAILED")

    def test_payload_already_deployed_rejection(self):
        """Verify pre-flight rejection if carrier has already deployed its payload."""
        carrier = self.sim.drones[1]
        carrier.payload_attached = False

        res = self.sim.deploy_payload_drop(carrier_id=1)
        self.assertEqual(res.get("status"), "ERROR")
        self.assertEqual(res.get("code"), "NO_PAYLOAD_ATTACHED")
        self.assertFalse(self.sim.c2.payload_drop_active)

    def test_payload_drop_dispatch_and_initialization(self):
        """Verify deployment initializes state machine, carrier flight state, and telemetry."""
        drop_pos = [60.0, 30.0, 0.0]
        res = self.sim.deploy_payload_drop(target_pos=drop_pos, carrier_id=1)
        self.assertEqual(res.get("status"), "OK")
        self.assertEqual(res.get("action"), "PAYLOAD_DROP_DEPLOYED")
        self.assertEqual(res.get("carrier_id"), 1)
        self.assertEqual(res.get("stage"), "TRANSIT_TO_DROP")

        # C2 Gateway state
        self.assertTrue(self.sim.c2.payload_drop_active)
        self.assertEqual(self.sim.c2.payload_drop_carrier_id, 1)
        self.assertEqual(self.sim.c2.payload_drop_stage, "TRANSIT_TO_DROP")
        self.assertTrue(np.allclose(self.sim.c2.payload_drop_target_pos[:2], [60.0, 30.0]))

        # Carrier state
        carrier = self.sim.drones[1]
        self.assertTrue(carrier.payload_drop_active)
        self.assertEqual(carrier.flight_state, "PAYLOAD_TRANSIT")
        self.assertTrue(np.allclose(carrier.acknowledged_target, [60.0, 30.0, 25.0]))

        # Telemetry snapshot
        snap = self.sim.get_state_snapshot()
        self.assertIn("payload_deposit", snap)
        p_data = snap["payload_deposit"]
        self.assertTrue(p_data["active"])
        self.assertEqual(p_data["stage"], "TRANSIT_TO_DROP")
        self.assertEqual(p_data["carrier_id"], 1)
        self.assertTrue(p_data["payload_attached"])

    def test_precision_hover_and_release(self):
        """Verify carrier flies overhead, descends to Z=3.0m, stabilizes, and drops payload."""
        drop_pos = [50.0, 20.0, 0.0]
        self.sim.deploy_payload_drop(target_pos=drop_pos, carrier_id=1)
        carrier = self.sim.drones[1]

        # 1. Fly carrier overhead drop zone (d_xy <= 4m)
        carrier.pos = np.array([51.0, 20.0, 25.0], dtype=float)
        self.sim.step(DT)

        # Should transition to PRECISION_DESCENT
        self.assertEqual(self.sim.c2.payload_drop_stage, "PRECISION_DESCENT")
        self.assertEqual(carrier.flight_state, "PRECISION_DESCENT")
        self.assertTrue(np.allclose(carrier.acknowledged_target, [50.0, 20.0, 3.0]))

        # 2. Descend into precision hover envelope (Z <= 3.5m, spd <= 1.0 m/s)
        carrier.pos[2] = 3.2
        carrier.vel = np.array([0.1, 0.0, -0.2], dtype=float)
        self.sim.step(DT)

        self.assertEqual(self.sim.c2.payload_drop_stage, "RELEASE_HOVER")

        # 3. Step past release delay (0.2s)
        self.sim.c2.payload_drop_timer = 0.25
        self.sim.step(DT)

        self.assertEqual(self.sim.c2.payload_drop_stage, "PAYLOAD_FALLING")
        self.assertFalse(carrier.payload_attached)
        self.assertEqual(len(self.sim.falling_payloads), 1)

    def test_ballistic_touchdown_and_repeater_activation(self):
        """Verify falling payload hits ground and registers active GroundRepeater."""
        drop_pos = [40.0, 40.0, 0.0]
        self.sim.deploy_payload_drop(target_pos=drop_pos, carrier_id=1)
        carrier = self.sim.drones[1]

        # Trigger release
        carrier.pos = np.array([40.0, 40.0, 3.0], dtype=float)
        carrier.vel = np.zeros(3, dtype=float)
        self.sim.step(DT)  # PRECISION_DESCENT
        self.sim.step(DT)  # RELEASE_HOVER
        self.sim.c2.payload_drop_timer = 0.25
        self.sim.step(DT)  # PAYLOAD_FALLING
        self.assertEqual(len(self.sim.falling_payloads), 1)

        # Simulate physics steps until touchdown (Z <= 0.05m)
        for _ in range(60):
            self.sim.step(0.02)
            if len(self.sim.ground_repeaters) > 0:
                break

        self.assertEqual(len(self.sim.ground_repeaters), 1)
        rep = self.sim.ground_repeaters[0]
        self.assertTrue(rep.active)
        self.assertTrue(rep.solar_powered)
        self.assertEqual(rep.carrier_id, 1)
        self.assertAlmostEqual(rep.pos[2], 0.05, delta=0.01)
        self.assertTrue(np.allclose(rep.pos[:2], [40.0, 40.0], atol=1.0))

    def test_mesh_relay_extension_beyond_horizon(self):
        """Verify ground repeater relays telemetry to a drone that would otherwise be out of range."""
        self.sim.set_comms_mode("GOSSIP_CONSENSUS")

        # Move Drone 2 far out to 155m from Base (beyond 140m Howl / 70m Bork horizon)
        d2 = self.sim.drones[2]
        d2.pos = np.array([155.0, 0.0, 25.0], dtype=float)
        self.sim.step(DT)

        # Without intermediate repeater or peers within 60m, Drone 2 loses direct downlink
        # Deploy a ground repeater at [100.0, 0.0, 0.05] (bridges between 50m and 155m via 60m mesh hops)
        # Add an intermediate drone or place repeater such that it is within GCS or peer range:
        # Base anchor is at (0, 0, 1.5). A drone at (50, 0, 20) is within direct range.
        self.sim.drones[0].pos = np.array([50.0, 0.0, 20.0], dtype=float)  # direct gateway
        rep = GroundRepeater(
            id=1,
            pos=np.array([105.0, 0.0, 0.05], dtype=float),
            active=True,
            solar_powered=True,
            radio_range=60.0,
            carrier_id=1,
        )
        self.sim.ground_repeaters.append(rep)

        # Advance comms step
        self.sim.step(DT)

        # Repeater is within 55m of Drone 0 (<= 60m) -> repeater receives downlink!
        self.assertTrue(rep.downlink_active, "Expected repeater to receive downlink via Drone 0")

        # Drone 2 at 155m is within 50m of repeater (<= 60m) -> Drone 2 receives downlink through repeater!
        self.assertTrue(d2.downlink_active, "Expected Drone 2 to receive downlink bridged by repeater")
        self.assertFalse(d2.lost_link)

    def test_gnss_drift_reset_near_repeater(self):
        """Verify a drone in GNSS denial resets its drift timer when passing near ground repeater."""
        rep = GroundRepeater(
            id=1,
            pos=np.array([50.0, 50.0, 0.05], dtype=float),
            active=True,
            solar_powered=True,
            radio_range=60.0,
        )
        self.sim.ground_repeaters.append(rep)

        test_drone = self.sim.drones[2]
        test_drone.gnss_denied = True
        test_drone.gnss_denied_timer = 14.5  # Accumulated drift

        # Position within 20m of repeater (<= 30m anchor radius)
        test_drone.pos = np.array([52.0, 50.0, 15.0], dtype=float)
        self.sim.step(DT)

        # Drift timer should be reset to 0.0 by surveyed local anchor
        self.assertEqual(test_drone.gnss_denied_timer, 0.0)

    def test_carrier_safe_egress_and_rtb(self):
        """Verify carrier ascends to corridor altitude and returns to Base upon drop completion."""
        self.sim.deploy_payload_drop(target_pos=[40.0, 40.0, 0.0], carrier_id=1)
        carrier = self.sim.drones[1]

        self.sim.c2.payload_drop_stage = "CARRIER_EGRESS"
        carrier.pos = np.array([40.0, 40.0, 22.0], dtype=float)  # Climbed past 20m
        self.sim.step(DT)

        # Acknowledged target should now be Base overhead sentry [0, 0, 25]
        self.assertTrue(np.allclose(carrier.acknowledged_target, [0.0, 0.0, 25.0]))

        # Arrive at Base (d <= 15m)
        carrier.pos = np.array([5.0, 5.0, 25.0], dtype=float)
        self.sim.step(DT)

        self.assertEqual(self.sim.c2.payload_drop_stage, "MISSION_COMPLETE")
        self.assertFalse(self.sim.c2.payload_drop_active)
        self.assertEqual(carrier.flight_state, "CRUISE")

    def test_operator_manual_abort(self):
        """Verify operator can manually abort active payload drop mission."""
        self.sim.deploy_payload_drop(target_pos=[50.0, 50.0, 0.0], carrier_id=1)
        self.assertTrue(self.sim.c2.payload_drop_active)

        res = self.sim.stop_payload_drop()
        self.assertEqual(res.get("status"), "OK")
        self.assertFalse(self.sim.c2.payload_drop_active)
        self.assertEqual(self.sim.c2.payload_drop_stage, "IDLE")

        carrier = self.sim.drones[1]
        self.assertFalse(carrier.payload_drop_active)
        self.assertEqual(carrier.flight_state, "CRUISE")

    def test_carrier_lost_failsafe(self):
        """Verify graceful failsafe deactivation if carrier drone crashes mid-mission."""
        self.sim.deploy_payload_drop(target_pos=[50.0, 50.0, 0.0], carrier_id=1)
        carrier = self.sim.drones[1]

        # Carrier crashes
        carrier.crashed = True
        carrier.flight_state = "CRASHED"
        self.sim.step(DT)

        self.assertEqual(self.sim.c2.payload_drop_stage, "CARRIER_LOST")
        self.assertFalse(self.sim.c2.payload_drop_active)


if __name__ == "__main__":
    unittest.main()
