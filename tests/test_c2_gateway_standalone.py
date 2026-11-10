"""
SwarmTech3_UpRising: Standalone GCS C2 Gateway Verification Tests (test_c2_gateway_standalone.py)
Validates that C2Gateway operates completely decoupled from the SITL world simulation:
  1. Ingests downlinked telemetry snapshots into local fleet cache.
  2. Evaluates link health and target arrival strictly from downlinked telemetry.
  3. Advances auto-patrol, RNG recon dwell, and BINGO RTB state machines with sim=None.
  4. Emits serialized C2 commands and target uplinks across Canyon UDP port 9001.
"""

from __future__ import annotations
import json
import socket
import unittest
import numpy as np

from GCS.backend.c2_gateway import C2Gateway, ROUTE_PRESETS


class TestC2GatewayStandalone(unittest.TestCase):
    """Test suite verifying pure decoupled operation of C2Gateway (Operator Airgap)."""

    def setUp(self):
        self.c2 = C2Gateway(sim=None)

    def tearDown(self):
        if self.c2.udp_sock is not None:
            try:
                self.c2.udp_sock.close()
            except OSError:
                pass

    def test_c2_initialization_standalone(self):
        """Verify initial state of standalone C2Gateway."""
        self.assertIsNone(self.c2.sim)
        self.assertEqual(len(self.c2.fleet_telemetry), 0)
        self.assertIsNone(self.c2.latest_frame)
        self.assertFalse(self.c2.patrol_active)
        self.assertFalse(self.c2.route_active)
        self.assertFalse(self.c2.rng_recon_active)
        self.assertFalse(self.c2.bingo_mission_active)
        np.testing.assert_allclose(self.c2.target_pos, [0.0, 0.0, 20.0])

    def test_c2_telemetry_frame_ingestion_and_link_evaluation(self):
        """Verify telemetry cache hydration and comms link evaluation without sim handle."""
        self.assertFalse(self.c2.is_link_active())

        # Feed healthy tethered relay frame
        frame = {
            "sim_time": 10.0,
            "comms_mode": "TETHERED_RELAY",
            "tether": {"severed": False, "leader_id": 0},
            "drones": [
                {
                    "id": 0,
                    "pos": [0.0, 0.0, 20.0],
                    "crashed": False,
                    "lost_link": False,
                    "is_leader": True,
                    "battery": 85.0,
                    "capabilities": ["FORMATION_FLIGHT", "PATROL"],
                },
                {
                    "id": 1,
                    "pos": [5.0, 0.0, 20.0],
                    "crashed": False,
                    "lost_link": False,
                    "is_leader": False,
                    "battery": 90.0,
                    "capabilities": ["FORMATION_FLIGHT", "PATROL"],
                },
            ],
        }
        self.c2.on_telemetry_frame(frame)
        self.assertEqual(len(self.c2.fleet_telemetry), 2)
        self.assertEqual(self.c2.last_telemetry_time, 10.0)
        self.assertTrue(self.c2.is_link_active())

        # Simulate tether severance via telemetry update
        severed_frame = dict(frame)
        severed_frame["tether"] = {"severed": True, "leader_id": 0}
        self.c2.on_telemetry_frame(severed_frame)
        self.assertFalse(self.c2.is_link_active())

    def test_c2_target_arrival_detection_standalone(self):
        """Verify swarm target arrival detection strictly from downlinked telemetry."""
        target = np.array([25.0, 25.0, 20.0])
        self.c2._target_pos = np.copy(target)

        # Leader far from target
        far_frame = {
            "sim_time": 5.0,
            "comms_mode": "LEADER_FOLLOWER",
            "drones": [
                {
                    "id": 0,
                    "pos": [0.0, 0.0, 20.0],
                    "crashed": False,
                    "lost_link": False,
                    "is_leader": True,
                    "acknowledged_target": [25.0, 25.0, 20.0],
                    "downlink_active": True,
                }
            ],
        }
        self.c2.on_telemetry_frame(far_frame)
        self.assertFalse(self.c2.is_target_reached(threshold=2.5))

        # Leader arrived within threshold
        near_frame = {
            "sim_time": 6.0,
            "comms_mode": "LEADER_FOLLOWER",
            "drones": [
                {
                    "id": 0,
                    "pos": [25.5, 24.8, 20.1],
                    "crashed": False,
                    "lost_link": False,
                    "is_leader": True,
                    "acknowledged_target": [25.0, 25.0, 20.0],
                    "downlink_active": True,
                }
            ],
        }
        self.c2.on_telemetry_frame(near_frame)
        self.assertTrue(self.c2.is_target_reached(threshold=2.5))

    def test_c2_auto_patrol_advancement_standalone(self):
        """Verify figure-8 continuous patrol generation advances with sim=None."""
        self.c2.toggle_target_patrol(True)
        self.assertTrue(self.c2.patrol_active)

        initial_pos = np.copy(self.c2.target_pos)
        # Advance C2 state machine by 2.0s
        self.c2.step(2.0)
        self.assertGreater(self.c2.c2_time, 0.0)
        # Target position must move along the parametric trajectory
        self.assertFalse(np.allclose(self.c2.target_pos, initial_pos))

    def test_c2_rng_recon_dwell_and_generation_standalone(self):
        """Verify tactical RNG touch-and-go recon dwell and coordinate re-roll with sim=None."""
        # Setup active link at target
        target = np.array([20.0, 20.0, 25.0])
        self.c2._target_pos = np.copy(target)
        frame = {
            "sim_time": 1.0,
            "comms_mode": "LEADER_FOLLOWER",
            "drones": [
                {
                    "id": 0,
                    "pos": [20.0, 20.0, 25.0],
                    "crashed": False,
                    "lost_link": False,
                    "is_leader": True,
                    "acknowledged_target": [20.0, 20.0, 25.0],
                    "downlink_active": True,
                }
            ],
        }
        self.c2.on_telemetry_frame(frame)
        self.c2.start_rng_recon()
        self.assertTrue(self.c2.rng_recon_active)

        # Step 0.1s: Arrival detected -> enters DWELLING
        self.c2.step(0.1)
        self.assertTrue(self.c2.rng_dwelling)
        self.assertEqual(self.c2.rng_status, "DWELLING")
        self.assertEqual(self.c2.rng_station_count, 1)

        # Step remaining dwell time (2.0s total)
        self.c2.step(2.1)
        # Dwell complete: should have rolled new target and returned to TRACKING
        self.assertFalse(self.c2.rng_dwelling)
        self.assertEqual(self.c2.rng_status, "TRACKING")
        self.assertFalse(np.allclose(self.c2.target_pos, target))

    def test_c2_patrol_to_bingo_standalone(self):
        """Verify Patrol to BINGO watchdog detects battery drop from downlinked telemetry."""
        # Hydrate fleet telemetry with healthy battery
        frame = {
            "sim_time": 2.0,
            "comms_mode": "LEADER_FOLLOWER",
            "drones": [
                {
                    "id": 0,
                    "pos": [0.0, 0.0, 20.0],
                    "crashed": False,
                    "flight_state": "CRUISE",
                    "downlink_active": True,
                    "battery": 80.0,
                    "capabilities": ["FORMATION_FLIGHT", "PATROL"],
                }
            ],
        }
        self.c2.on_telemetry_frame(frame)

        # Deploy Patrol to BINGO with 25% threshold
        res = self.c2.deploy_patrol_to_bingo(bingo_threshold=25.0, pattern="BOX_PATROL")
        self.assertEqual(res["status"], "OK")
        self.assertTrue(self.c2.bingo_mission_active)
        self.assertFalse(self.c2.bingo_triggered)

        # Downlink critical battery frame
        critical_frame = {
            "sim_time": 30.0,
            "comms_mode": "LEADER_FOLLOWER",
            "drones": [
                {
                    "id": 0,
                    "pos": [40.0, 40.0, 25.0],
                    "crashed": False,
                    "flight_state": "CRUISE",
                    "downlink_active": True,
                    "battery": 22.0,
                    "capabilities": ["FORMATION_FLIGHT", "PATROL"],
                }
            ],
        }
        self.c2.on_telemetry_frame(critical_frame)
        self.c2.step(0.1)

        # Watchdog must trigger RTB transit
        self.assertTrue(self.c2.bingo_triggered)
        self.assertEqual(self.c2.bingo_stage, "RTB_TRANSIT")
        self.assertFalse(self.c2.route_active)

    def test_c2_udp_command_dispatch_across_canyon(self):
        """Verify C2Gateway serializes and transmits UDP packets to port 9001 when decoupled."""
        # Bind ephemeral UDP socket to simulate Canyon SITL receiver
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        _, mock_port = sock.getsockname()
        sock.settimeout(1.0)

        try:
            self.c2.canyon_port = mock_port

            # 1. Target dispatch
            res = self.c2.dispatch_target_uplink([15.0, 25.0, 35.0])
            self.assertEqual(res["status"], "OK")
            self.assertEqual(res["channel"], "UDP_CANYON")

            data, _ = sock.recvfrom(2048)
            pkt = json.loads(data.decode("utf-8"))
            self.assertEqual(pkt["cmd"], "SET_TARGET")
            np.testing.assert_allclose(pkt["target"], [15.0, 25.0, 35.0])

            # 2. Field LZ designation
            self.c2.set_field_lz([50.0, 50.0, 0.0], name="FLZ-ALPHA", radius=18.0)
            data, _ = sock.recvfrom(2048)
            pkt = json.loads(data.decode("utf-8"))
            self.assertEqual(pkt["cmd"], "SET_FIELD_LZ")
            self.assertEqual(pkt["name"], "FLZ-ALPHA")
            np.testing.assert_allclose(pkt["coords"], [50.0, 50.0, 0.0])

            # 3. Emergency Land command
            self.c2.command_e_land(3, reason="LOW_BATTERY")
            data, _ = sock.recvfrom(2048)
            pkt = json.loads(data.decode("utf-8"))
            self.assertEqual(pkt["cmd"], "COMMAND_E_LAND")
            self.assertEqual(pkt["drone_id"], 3)
            self.assertEqual(pkt["reason"], "LOW_BATTERY")
        finally:
            sock.close()


if __name__ == "__main__":
    unittest.main()
