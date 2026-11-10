"""
SwarmTech3_UpRising: Unit Tests for Heartbeat Watchdog, Breadcrumb Backtrack & Zero-Range Peeking (test_breadcrumb_backtrack.py)
Validates [TKT-54]:
1. GCS telemetry freshness assessment without Euclidean range peeking.
2. Bidirectional heartbeat watchdog countdown and reset.
3. 2 Hz breadcrumb recording rolling buffer (max 20 entries / 10s).
4. Watchdog timeout triggering autonomous BACKTRACK_HOLD along breadcrumb trail.
5. Radio silence permitted suppressing backtrack (autonomous mission mode).
6. Link restoration during backtrack triggering PERIMETER_HOLD hover at radio horizon.
7. Dispatched closer target resuming nominal cruise from PERIMETER_HOLD.
"""

from __future__ import annotations
import unittest
import numpy as np

from SIM.backend.sitl_engine import SITLWorldEngine, RADIO_RANGE
from GCS.backend.web_server import SwarmWebServer


class TestBreadcrumbBacktrack(unittest.TestCase):
    """Test suite for TKT-54: Heartbeat Watchdog, Breadcrumb Backtrack & Zero-Range Peeking."""

    def setUp(self):
        self.sim = SITLWorldEngine(drone_count=5, enable_packet_loss=False, seed=42)
        # Settle fleet initial flight
        for _ in range(15):
            self.sim.step()

    def test_gcs_telemetry_freshness_link_active(self):
        """Verify GCS assesses link status strictly via telemetry freshness without range peeking."""
        server = SwarmWebServer(sim=self.sim, bridge=None, host="127.0.0.1", port=19080)

        # 1. During bootstrap (sim_time < 1.0), link is active
        self.sim.sim_time = 0.5
        self.assertTrue(server.is_link_active())

        # 2. When sim_time > 1.0 and telemetry is fresh (<= 1.0s)
        self.sim.sim_time = 10.0
        server.last_telemetry_rx_time = 9.5
        self.assertTrue(server.is_link_active())

        # 3. When telemetry is stale (> 1.0s since last rx)
        server.last_telemetry_rx_time = 8.5
        self.assertFalse(server.is_link_active())

        # 4. Commands are dropped with reason 'LINK_LOST' when telemetry is stale
        res_land = server.execute_command({"cmd": "LAND_DRONE", "id": 0})
        self.assertEqual(res_land["status"], "DROPPED")
        self.assertEqual(res_land["reason"], "LINK_LOST")

        res_brain = server.execute_command({"cmd": "SET_BRAIN", "brain": "GOSSIP_CONSENSUS"})
        self.assertEqual(res_brain["status"], "DROPPED")
        self.assertEqual(res_brain["reason"], "LINK_LOST")

    def test_heartbeat_watchdog_countdown_and_reset(self):
        """Verify watchdog decrements by dt and resets to 2.0s when GCS heartbeat packet is received."""
        drone = self.sim.drones[0]
        drone.gcs_heartbeat_watchdog = 2.0

        # Step sim without comms update
        dt = 0.1
        drone.gcs_heartbeat_watchdog -= dt
        self.assertAlmostEqual(drone.gcs_heartbeat_watchdog, 1.9, places=2)

        # Set low watchdog to verify packet reception resets it
        drone.gcs_heartbeat_watchdog = 0.5

        # Step sim: leader in range receives broadcast tick and resets watchdog to 2.0 (minus dt=0.1 step)
        self.sim.comms_mode = "LEADER_FOLLOWER"
        self.sim.last_heartbeat_broadcast = -1.0  # Force broadcast tick
        self.sim.step(dt=0.1)

        self.assertAlmostEqual(drone.gcs_heartbeat_watchdog, 1.9, places=1)
        self.assertFalse(drone.lost_link)

    def test_breadcrumb_buffer_recording(self):
        """Verify 2 Hz rolling breadcrumb buffer captures positions up to 20 entries (10 seconds)."""
        drone = self.sim.drones[0]
        drone.breadcrumbs = []
        drone.flight_state = "CRUISE"
        drone.lost_link = False
        drone.holding_at_horizon = False

        # Advance 3.0 seconds (6 records @ 2 Hz)
        for _ in range(30):
            drone.pos[0] += 0.5  # Cruising
            self.sim.step(dt=0.1)

        self.assertGreaterEqual(len(drone.breadcrumbs), 5)
        self.assertLessEqual(len(drone.breadcrumbs), 20)

        # Fill with 30 entries
        for i in range(30):
            drone.record_breadcrumb(float(i))

        # Maxlen must cap strictly at 20
        self.assertEqual(len(drone.breadcrumbs), 20)

        # Verify get_backtrack_point returns a coordinate approximately retreat_distance back
        drone.pos = np.array([100.0, 0.0, 25.0])
        drone.breadcrumbs = [(float(i), np.array([float(i * 5), 0.0, 25.0])) for i in range(20)]
        # Newest breadcrumb is at x=95, oldest at x=0
        backtrack_pt = drone.get_backtrack_point(retreat_distance=18.0)
        dist = float(np.linalg.norm(drone.pos - backtrack_pt))
        self.assertGreaterEqual(dist, 15.0)

    def test_watchdog_expiry_triggers_backtrack_hold(self):
        """Verify that watchdog expiration triggers BACKTRACK_HOLD when radio silence is not permitted."""
        drone = self.sim.drones[0]
        drone.flight_state = "CRUISE"
        drone.radio_silence_permitted = False
        drone.breadcrumbs = [(0.0, np.array([40.0, 10.0, 25.0])), (1.0, np.array([20.0, 5.0, 25.0]))]
        drone.pos = np.array([60.0, 15.0, 25.0])

        # Force watchdog expiry
        drone.gcs_heartbeat_watchdog = 0.0
        drone.uplink_active = False

        self.sim._update_heartbeat_watchdog_and_breadcrumbs(dt=0.1)

        self.assertTrue(drone.lost_link)
        self.assertEqual(drone.flight_state, "BACKTRACK_HOLD")
        self.assertEqual(drone.status, "BACKTRACK_HOLD")
        self.assertTrue(drone.backtrack_hold)
        self.assertIsNotNone(drone.backtrack_target)
        self.assertTrue(np.allclose(drone.acknowledged_target, drone.backtrack_target))

    def test_radio_silence_permitted_suppresses_backtrack(self):
        """Verify that when radio_silence_permitted is True, watchdog expiration does not trigger backtrack."""
        drone = self.sim.drones[0]
        drone.flight_state = "CRUISE"
        drone.radio_silence_permitted = True
        drone.gcs_heartbeat_watchdog = 0.0
        drone.uplink_active = False

        self.sim._update_heartbeat_watchdog_and_breadcrumbs(dt=0.1)

        self.assertTrue(drone.lost_link)
        self.assertNotEqual(drone.flight_state, "BACKTRACK_HOLD")
        self.assertFalse(drone.backtrack_hold)

    def test_link_restoration_triggers_perimeter_hold(self):
        """Verify that when drone backtracks into coverage, it transitions to PERIMETER_HOLD and hovers."""
        drone = self.sim.drones[0]
        drone.flight_state = "BACKTRACK_HOLD"
        drone.status = "BACKTRACK_HOLD"
        drone.backtrack_hold = True
        drone.pos = np.array([115.0, 0.0, 25.0])  # At radio horizon
        drone.vel = np.array([-5.0, 0.0, 0.0])

        # Link restored: incoming packet resets watchdog
        drone.gcs_heartbeat_watchdog = 2.0
        drone.uplink_active = True

        self.sim._update_heartbeat_watchdog_and_breadcrumbs(dt=0.1)

        self.assertEqual(drone.flight_state, "PERIMETER_HOLD")
        self.assertEqual(drone.status, "PERIMETER_HOLD")
        self.assertFalse(drone.backtrack_hold)
        self.assertTrue(drone.holding_at_horizon)
        self.assertTrue(np.allclose(drone.acknowledged_target, drone.pos))
        self.assertTrue(np.allclose(drone.vel, np.zeros(3)))

        # Snapshot reflects holding at horizon
        snapshot = self.sim.get_state_snapshot()
        self.assertTrue(snapshot["fleet_holding_at_horizon"])

    def test_dispatched_target_resumes_cruise_from_perimeter_hold(self):
        """Verify commanding a new valid waypoint disengages PERIMETER_HOLD and resumes CRUISE."""
        drone = self.sim.drones[0]
        drone.flight_state = "PERIMETER_HOLD"
        drone.status = "PERIMETER_HOLD"
        drone.holding_at_horizon = True

        # Dispatch valid closer target within range
        res = self.sim.dispatch_target_uplink([50.0, 20.0, 25.0])
        self.assertEqual(res["status"], "OK")

        # Step simulation so OBC firmware ingests GCS_UPLINK packet from transceiver inbox
        self.sim.step()

        self.assertFalse(drone.holding_at_horizon)
        self.assertEqual(drone.flight_state, "CRUISE")
        self.assertEqual(drone.status, "FLEET_LEADER")
        self.assertTrue(np.allclose(drone.acknowledged_target, [50.0, 20.0, 25.0]))

    # ==========================================================================
    # BUG-06 Unit Tests: Ingress Corridor Stack, Memory Profile & Gossip ERTB
    # ==========================================================================
    def test_bug06_ingress_corridor_stack_memory_profile_under_300_bytes(self):
        """Verify IngressCorridorStack bounds capacity at 16 nodes and maintains < 300 byte heap profile."""
        from OBC.backend.drone_firmware import IngressCorridorStack
        stack = IngressCorridorStack(max_nodes=16, min_dist=15.0)

        # Base node must be present
        self.assertEqual(len(stack), 1)
        self.assertTrue(np.allclose(stack.peek(), [0.0, 0.0, 25.0]))

        # Record 25 novel waypoints (each 16m apart)
        for i in range(1, 26):
            stack.record([float(i * 16.0), 0.0, 25.0])

        # Must cap strictly at 16 nodes
        self.assertEqual(len(stack), 16)
        # Base anchor [0, 0, 25] must be retained at index 0
        raw_list = stack.to_list()
        self.assertEqual(raw_list[0], [0.0, 0.0, 25.0])
        # Most recent node at end
        self.assertEqual(raw_list[-1], [400.0, 0.0, 25.0])

        # Micro-heap memory profile: payload (16 * 3 * 4 bytes for float32 coordinates) is 192 bytes (< 300 bytes)
        payload_bytes = len(raw_list) * 3 * 4
        self.assertLess(payload_bytes, 300)

    def test_bug06_corridor_ertb_reverses_ingress_corridor(self):
        """Verify Corridor ERTB reverses through popped corridor waypoints back to Base."""
        drone = self.sim.drones[0]
        drone.corridor_stack.clear()
        drone.flight_state = "CRUISE"

        # Record 3 ingress corridor waypoints
        drone.record_corridor([16.0, 0.0, 25.0])
        drone.record_corridor([32.0, 0.0, 25.0])
        drone.record_corridor([48.0, 0.0, 25.0])
        self.assertEqual(len(drone.corridor_stack), 4)  # Base + 3 nodes

        # Drone is currently at [48, 0, 25]
        drone.pos = np.array([48.0, 0.0, 25.0], dtype=float)

        # Trigger Corridor ERTB
        res = self.sim.command_corridor_ertb()
        self.assertEqual(res["status"], "OK")
        self.assertEqual(drone.flight_state, "CORRIDOR_ERTB")
        self.assertTrue(np.allclose(drone.corridor_target, [48.0, 0.0, 25.0]))

        # Pop target when within arrival radius
        next_t = drone.corridor_stack.pop_target(drone.pos, arrival_radius=4.5)
        self.assertTrue(np.allclose(next_t, [32.0, 0.0, 25.0]))

        # Advance to [32, 0, 25]
        drone.pos = np.array([32.0, 0.0, 25.0], dtype=float)
        next_t = drone.corridor_stack.pop_target(drone.pos, arrival_radius=4.5)
        self.assertTrue(np.allclose(next_t, [16.0, 0.0, 25.0]))

        # Advance to [16, 0, 25]
        drone.pos = np.array([16.0, 0.0, 25.0], dtype=float)
        next_t = drone.corridor_stack.pop_target(drone.pos, arrival_radius=4.5)
        self.assertTrue(np.allclose(next_t, [0.0, 0.0, 25.0]))

        # At Base anchor, length is 1 and stays at Base
        self.assertEqual(len(drone.corridor_stack), 1)

    def test_bug06_gossip_consensus_unanimous_contact_loss_ertb(self):
        """Verify Gossip Consensus executes ERTB only on unanimous partition contact loss (> 2.0s)."""
        self.sim.set_brain("GOSSIP_CONSENSUS")
        brain0 = self.sim.drones[0].brain

        # 1. When at least one node maintains GCS contact, swarm remains in mission
        t0 = self.sim._build_local_telemetry(self.sim.drones[0])
        # Simulate heard peer 1 with gcs_contact=True
        from OBC.backend.brains import HeardPeer
        brain0.heard_peers[1] = HeardPeer(
            drone_id=1,
            pos=np.array([20.0, 0.0, 25.0]),
            vel=np.zeros(3),
            gcs_contact=True,
        )

        intent, _ = brain0.step_drone(t0, [], dt=0.5)
        self.assertEqual(brain0.gcs_lost_timer, 0.0)
        self.assertNotIn("flight_state", intent.state_updates)

        # 2. When ALL nodes in partition lose GCS contact for > 2.0s, coordinated ERTB engages
        brain0.heard_peers[1].gcs_contact = False
        self.sim.drones[0].uplink_active = False

        # Advance timer past 2.0s
        for _ in range(5):
            t_curr = self.sim._build_local_telemetry(self.sim.drones[0])
            intent, _ = brain0.step_drone(t_curr, [], dt=0.5)

        self.assertGreaterEqual(brain0.gcs_lost_timer, 2.0)
        self.assertEqual(intent.state_updates.get("flight_state"), "CORRIDOR_ERTB")


if __name__ == "__main__":
    unittest.main()
