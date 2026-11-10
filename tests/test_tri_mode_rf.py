"""
SwarmTech3_UpRising: Tri-Mode Architecture & Contested RF Channel Tests (test_tri_mode_rf.py)
Automated verification for TKT-17 & PLN-03:
- Tri-mode switching (TETHERED_RELAY, LEADER_FOLLOWER, GOSSIP_CONSENSUS).
- Hardline umbilical vs. wireless RF star vs. decentralized P2P mesh.
- Dynamic tether visibility & avoidance bypass in wireless modes.
- Distance-dependent RF path loss and operator electronic warfare jamming.
- GCS 120m radio uplink horizon and gateway range verification.
- Telemetry RSSI and normalized link quality tracking.
"""

from __future__ import annotations
import unittest
import numpy as np
from typing import Dict, Any

from SIM.backend.sitl_engine import (
    SwarmSimulator,
    RADIO_RANGE,
    RANGE_HOWL,
    RANGE_BARK,
    RANGE_YIP,
    RANGE_P2P,
    TETHER_ANCHOR,
    TIER_LEADER,
    TIER_ESCORT,
)
from GCS.backend.web_server import SwarmWebServer
from SIM.backend.mavlink_bridge import MavlinkBridge


class TestTriModeRF(unittest.TestCase):
    """Test suite for Tri-Mode swarm architecture and contested RF channel model."""

    def setUp(self):
        self.sim = SwarmSimulator(drone_count=5, enable_packet_loss=False, seed=42)

    def test_mode_switching_and_brain_sync(self):
        """Verify seamless switching between TETHERED_RELAY, LEADER_FOLLOWER, and GOSSIP_CONSENSUS."""
        self.assertEqual(self.sim.comms_mode, "TETHERED_RELAY")
        self.assertEqual(self.sim.active_brain_name, "LEADER_FOLLOWER")

        # Switch to WIRELESS STAR (LEADER_FOLLOWER)
        ok = self.sim.set_comms_mode("LEADER_FOLLOWER")
        self.assertTrue(ok)
        self.assertEqual(self.sim.comms_mode, "LEADER_FOLLOWER")
        self.assertEqual(self.sim.active_brain_name, "LEADER_FOLLOWER")

        # Switch to P2P MESH (GOSSIP_CONSENSUS)
        ok = self.sim.set_comms_mode("GOSSIP_CONSENSUS")
        self.assertTrue(ok)
        self.assertEqual(self.sim.comms_mode, "GOSSIP_CONSENSUS")
        self.assertEqual(self.sim.active_brain_name, "GOSSIP_CONSENSUS")

        # Switch back to TETHERED_RELAY
        ok = self.sim.set_comms_mode("TETHERED_RELAY")
        self.assertTrue(ok)
        self.assertEqual(self.sim.comms_mode, "TETHERED_RELAY")
        self.assertEqual(self.sim.active_brain_name, "LEADER_FOLLOWER")

        # Invalid mode returns False
        self.assertFalse(self.sim.set_comms_mode("INVALID_MODE"))

    def test_tether_visibility_and_dynamics_bypass_in_wireless_modes(self):
        """In wireless modes (LEADER_FOLLOWER, GOSSIP_CONSENSUS), tether is invisible and
        cable clearance forces / severance are completely bypassed."""
        # 1. Tethered Relay Mode: Tether visible and active
        self.sim.set_comms_mode("TETHERED_RELAY")
        snap = self.sim.get_state_snapshot()
        self.assertTrue(snap["tether"]["visible"])

        # 2. Wireless Star Mode: Tether invisible
        self.sim.set_comms_mode("LEADER_FOLLOWER")
        snap = self.sim.get_state_snapshot()
        self.assertFalse(snap["tether"]["visible"])

        # Place Drone 1 directly coincident with the virtual cable line between base (0,0,1.5) and leader (0,0,20)
        follower = self.sim.drones[1]
        follower.pos = np.array([0.0, 0.0, 10.0], dtype=float)

        # Run physics step: should NOT sever tether and should NOT apply clearance repulsion
        self.sim.step(0.016)
        self.assertFalse(self.sim.tether_severed)
        avoidance = self.sim._compute_tether_avoidance()
        np.testing.assert_allclose(avoidance[follower.id], np.zeros(3))

        # 3. P2P Mesh Mode: Tether invisible
        self.sim.set_comms_mode("GOSSIP_CONSENSUS")
        snap = self.sim.get_state_snapshot()
        self.assertFalse(snap["tether"]["visible"])

    def test_physics_based_rf_path_loss_and_jamming(self):
        """Verify distance-dependent path loss and EW jamming formula:
        P_loss(d) = gamma + (1 - gamma) * 0.25 * (d / R_max)^2."""
        # 1. Hardline fiber: 0% loss regardless of distance or jamming
        self.sim.set_rf_jamming(0.5)
        prob_hardline = self.sim.calculate_packet_drop_prob(distance=100.0, is_hardline=True)
        self.assertEqual(prob_hardline, 0.0)

        rssi_hardline, lq_hardline = self.sim.calculate_rssi(distance=100.0, is_hardline=True)
        self.assertEqual(rssi_hardline, -35.0)
        self.assertEqual(lq_hardline, 1.0)

        # 2. Wireless without jamming (gamma = 0.0)
        self.sim.set_rf_jamming(0.0)
        self.sim.enable_packet_loss = True

        # At d = 0m: loss = 0.0
        self.assertAlmostEqual(self.sim.calculate_packet_drop_prob(0.0), 0.0, places=4)

        # At d = 70m (half of 140m Howl range): loss = 0.25 * (0.5)^2 = 0.0625 (6.25%)
        self.assertAlmostEqual(self.sim.calculate_packet_drop_prob(70.0), 0.0625, places=4)

        # At d = 140m (edge of range): loss = 0.25 * (1.0)^2 = 0.25 (25%)
        self.assertAlmostEqual(self.sim.calculate_packet_drop_prob(140.0), 0.25, places=4)

        # At d > 140m: loss = 1.0 (100% out of range)
        self.assertEqual(self.sim.calculate_packet_drop_prob(141.0), 1.0)

        # 3. Contested RF with Jamming (gamma = 0.3)
        self.sim.set_rf_jamming(0.3)
        # At d = 0m: loss = 0.3 (30%)
        self.assertAlmostEqual(self.sim.calculate_packet_drop_prob(0.0), 0.3, places=4)
        # At d = 140m: loss = 0.3 + 0.7 * 0.25 = 0.475 (47.5%)
        self.assertAlmostEqual(self.sim.calculate_packet_drop_prob(140.0), 0.475, places=4)

        # Jamming clamp to 0.5
        self.sim.set_rf_jamming(0.9)
        self.assertEqual(self.sim.rf_jamming_rate, 0.5)

        # 4. RSSI model
        self.sim.set_rf_jamming(0.0)
        rssi_1m, _ = self.sim.calculate_rssi(1.0)
        self.assertEqual(rssi_1m, -40.0)
        rssi_10m, _ = self.sim.calculate_rssi(10.0)
        self.assertEqual(rssi_10m, -60.0)
        rssi_100m, _ = self.sim.calculate_rssi(100.0)
        self.assertEqual(rssi_100m, -80.0)

    def test_wireless_leader_lost_link_gcs_horizon(self):
        """In LEADER_FOLLOWER mode, leader communicates with GCS over RF.
        Exceeding 140m GCS Howl horizon triggers lost link on leader."""
        self.sim.set_comms_mode("LEADER_FOLLOWER")
        leader = self.sim.leader
        self.assertIsNotNone(leader)

        # Run 0.2s to establish initial heartbeat
        for _ in range(12):
            self.sim.step()
        self.assertFalse(leader.lost_link)

        # Teleport leader outside GCS horizon (160m > 140m)
        leader.pos = np.array([160.0, 0.0, 25.0], dtype=float)

        # Advance 1.2s (> 1.0s LINK_TIMEOUT = 72 ticks)
        for _ in range(75):
            self.sim.step()

        self.assertTrue(leader.lost_link)
        self.assertEqual(leader.status, "LOST_LINK")

        # Teleport leader back within GCS horizon (< 140m)
        leader.pos = np.array([30.0, 0.0, 25.0], dtype=float)
        for _ in range(12):
            self.sim.step()

        self.assertFalse(leader.lost_link)

    def test_gcs_target_dispatch_horizon_checks(self):
        """GCS target dispatch verifies active drone presence within 140m GCS Howl radio horizon."""
        # 1. LEADER_FOLLOWER: Leader outside 140m horizon rejects dispatch
        self.sim.set_comms_mode("LEADER_FOLLOWER")
        leader = self.sim.leader
        leader.pos = np.array([160.0, 0.0, 25.0], dtype=float)

        res = self.sim.set_target([160.0, 50.0, 30.0])
        self.assertEqual(res["status"], "OUT_OF_RANGE")

        # Bring leader inside horizon (< 140m)
        leader.pos = np.array([50.0, 0.0, 25.0], dtype=float)
        res = self.sim.set_target([60.0, 40.0, 30.0])
        self.assertEqual(res["status"], "OK")

        # 2. GOSSIP_CONSENSUS: Mesh requires at least one node within 140m Howl range of Base
        self.sim.set_comms_mode("GOSSIP_CONSENSUS")

        # Move all drones out to 160m (> 140m)
        for d in self.sim.drones:
            d.pos = np.array([160.0, d.id * 5.0, 25.0], dtype=float)

        res = self.sim.set_target([160.0, 80.0, 30.0])
        self.assertEqual(res["status"], "OUT_OF_RANGE")

        # Bring just one drone (Drone 2) within GCS range
        self.sim.drones[2].pos = np.array([30.0, 0.0, 20.0], dtype=float)
        res = self.sim.set_target([160.0, 80.0, 30.0])
        self.assertEqual(res["status"], "OK")
        self.assertEqual(res["seed_drone_id"], 2)

    def test_telemetry_snapshot_contains_rf_and_mode_data(self):
        """Snapshot must include comms_mode, rf_jamming, gcs_radio_range, rf_link_budget, and per-drone RSSI."""
        self.sim.set_comms_mode("LEADER_FOLLOWER")
        self.sim.set_rf_jamming(0.25)

        snap = self.sim.get_state_snapshot()
        self.assertEqual(snap["comms_mode"], "LEADER_FOLLOWER")
        self.assertEqual(snap["rf_jamming"], 0.25)
        self.assertEqual(snap["gcs_radio_range"], 140.0)
        self.assertIn("rf_link_budget", snap)
        self.assertEqual(snap["rf_link_budget"]["howl_gcs_uplink"], 140.0)
        self.assertEqual(snap["rf_link_budget"]["bark_leader_downlink"], 90.0)
        self.assertEqual(snap["rf_link_budget"]["yip_escort_downlink"], 50.0)
        self.assertEqual(snap["rf_link_budget"]["p2p_mesh"], 60.0)

        # Check drone serialized RSSI
        d0 = snap["drones"][0]
        self.assertIn("rssi", d0)
        self.assertIn("link_quality", d0)
        self.assertIn("uplink_active", d0)
        self.assertIn("downlink_active", d0)
        self.assertIn("yip_lost", d0)
        self.assertIn("gateway_id", d0)
        self.assertIsInstance(d0["rssi"], (int, float))
        self.assertIsInstance(d0["link_quality"], (int, float))
        self.assertIsInstance(d0["rssi"], (int, float))
        self.assertIsInstance(d0["link_quality"], (int, float))

    def test_network_bridge_commands(self):
        """Verify web_server and mavlink_bridge handle SET_COMMS_MODE and SET_RF_JAMMING."""
        ws_server = SwarmWebServer(self.sim)
        mav_bridge = MavlinkBridge(self.sim, enable_mavlink=False)

        # 1. SET_COMMS_MODE via WebServer
        res = ws_server.execute_command({"cmd": "SET_COMMS_MODE", "mode": "WIRELESS_STAR"})
        self.assertEqual(res["status"], "OK")
        self.assertEqual(res["comms_mode"], "LEADER_FOLLOWER")

        # 2. SET_RF_JAMMING via WebServer
        res = ws_server.execute_command({"cmd": "SET_RF_JAMMING", "rate": 0.35})
        self.assertEqual(res["status"], "OK")
        self.assertAlmostEqual(res["rf_jamming"], 0.35, places=2)

        # 3. SET_COMMS_MODE via MavlinkBridge
        res = mav_bridge.execute_command({"cmd": "SET_COMMS_MODE", "mode": "P2P_MESH"})
        self.assertEqual(res["status"], "OK")
        self.assertEqual(res["comms_mode"], "GOSSIP_CONSENSUS")

        # 4. Out of range dispatch via MavlinkBridge
        for d in self.sim.drones:
            d.pos = np.array([200.0, d.id * 5.0, 25.0], dtype=float)
        res = mav_bridge.execute_command({"cmd": "SET_TARGET", "pos": [200.0, 50.0, 30.0]})
        self.assertEqual(res["status"], "OUT_OF_RANGE")
        mav_bridge.close()

    def test_canine_rf_constants(self):
        """Verify Canine Acoustic Taxonomy constants (TKT-49)."""
        self.assertEqual(RANGE_HOWL, 140.0)
        self.assertEqual(RANGE_BARK, 90.0)
        self.assertEqual(RANGE_YIP, 50.0)
        self.assertEqual(RANGE_P2P, 60.0)
        self.assertEqual(RADIO_RANGE, 140.0)

    def test_asymmetric_link_budget_yip_back_state(self):
        """Verify 'Yip-Back' state: Escort between 50m (Yip) and 140m (Howl) hears GCS
        commands (uplink_active = True), but GCS cannot hear telemetry directly (downlink_active = False, yip_lost = True)."""
        self.sim.set_comms_mode("GOSSIP_CONSENSUS")

        # Place escort Drone 1 at 75m from Base (alone, no peers within 60m)
        escort = self.sim.drones[1]
        escort.tier = TIER_ESCORT
        escort.is_leader = False
        escort.pos = np.array([75.0, 0.0, 20.0], dtype=float)

        # Place all other drones far away (> 160m)
        for d in self.sim.drones:
            if d.id != escort.id:
                d.pos = np.array([200.0, d.id * 10.0, 25.0], dtype=float)

        self.sim.step(0.016)

        # Escort at 75m: receives Howl (75m <= 140m), but cannot Yip back (75m > 50m)
        self.assertTrue(escort.uplink_active, "75m is within 140m GCS Howl uplink range")
        self.assertFalse(escort.downlink_active, "75m exceeds 50m Tier-2 Yip downlink range")
        self.assertTrue(escort.yip_lost, "Drone must be in Yip-Lost state when uplink_active and not downlink_active")
        self.assertIsNone(escort.gateway_id, "No mesh gateway proxy available")

    def test_tier1_bark_downlink(self):
        """Verify Tier-1 Flagship / Leader has 90m Bark downlink range."""
        self.sim.set_comms_mode("LEADER_FOLLOWER")
        leader = self.sim.leader
        self.assertIsNotNone(leader)
        leader.rf_power_mode = "BARK"

        # 1. At 75m: Leader hears Howl (75 <= 140) and GCS hears Bark (75 <= 90)
        leader.pos = np.array([75.0, 0.0, 20.0], dtype=float)
        self.sim.step(0.016)

        self.assertTrue(leader.uplink_active)
        self.assertTrue(leader.downlink_active)
        self.assertFalse(leader.yip_lost)

        # 2. At 110m: Leader hears Howl (110 <= 140), but GCS cannot hear Bark (110 > 90)
        leader.pos = np.array([110.0, 0.0, 20.0], dtype=float)
        self.sim.step(0.016)

        self.assertTrue(leader.uplink_active)
        self.assertFalse(leader.downlink_active)
        self.assertTrue(leader.yip_lost)

    def test_mesh_telemetry_gateway_multi_hop_relay(self):
        """Verify Mesh Telemetry Gateway in Gossip Consensus:
        - Drone 0 at 40m acts as Gateway (d <= 50m Yip).
        - Drone 1 at 85m is relayed via Drone 0 (distance = 45m <= 60m P2P).
        - Drone 2 at 130m is multi-hop relayed via Drone 1 (distance = 45m <= 60m P2P)."""
        self.sim.set_comms_mode("GOSSIP_CONSENSUS")

        d0 = self.sim.drones[0]
        d1 = self.sim.drones[1]
        d2 = self.sim.drones[2]

        # Move rest far away
        for d in self.sim.drones[3:]:
            d.pos = np.array([250.0, d.id * 10.0, 25.0], dtype=float)

        # Topology: Base (0) --- 40m ---> D0 (40m) --- 45m ---> D1 (85m) --- 45m ---> D2 (130m)
        d0.pos = np.array([40.0, 0.0, 20.0], dtype=float)
        d1.pos = np.array([85.0, 0.0, 20.0], dtype=float)
        d2.pos = np.array([130.0, 0.0, 20.0], dtype=float)

        self.sim.step(0.016)

        # D0: Direct gateway
        self.assertTrue(d0.uplink_active)
        self.assertTrue(d0.downlink_active)
        self.assertFalse(d0.yip_lost)
        self.assertEqual(d0.gateway_id, d0.id)

        # D1: Relayed through D0
        self.assertTrue(d1.uplink_active)
        self.assertTrue(d1.downlink_active, "D1 must have downlink_active via D0 gateway")
        self.assertFalse(d1.yip_lost, "D1 is not in yip_lost because telemetry is relayed")
        self.assertEqual(d1.gateway_id, d0.id)

        # D2: Multi-hop relayed through mesh
        self.assertTrue(d2.uplink_active)
        self.assertTrue(d2.downlink_active, "D2 must have downlink_active via multi-hop mesh")
        self.assertFalse(d2.yip_lost)
        self.assertEqual(d2.gateway_id, d0.id)

    def test_mesh_telemetry_gateway_severance_on_proxy_loss(self):
        """When all drones move beyond direct downlink range (Bark 90m for leader, Yip 50m for escort),
        all connected peers transition to Yip-Lost if they still hear GCS Howl (<= 140m)."""
        self.sim.set_comms_mode("GOSSIP_CONSENSUS")

        d0 = self.sim.drones[0]
        d1 = self.sim.drones[1]

        # Move rest far away
        for d in self.sim.drones[2:]:
            d.pos = np.array([250.0, d.id * 10.0, 25.0], dtype=float)

        # D0 (leader with 90m Bark) moves to 100m (> 90m Bark). D1 moves to 125m (> 50m Yip).
        # Distance between D0 and D1 is 25m (<= 60m P2P mesh).
        d0.pos = np.array([100.0, 0.0, 20.0], dtype=float)
        d1.pos = np.array([125.0, 0.0, 20.0], dtype=float)

        self.sim.step(0.016)

        # Neither has direct downlink to Base
        self.assertFalse(d0.downlink_active, "100m exceeds D0's 90m Tier-1 Bark downlink range")
        self.assertFalse(d1.downlink_active, "125m exceeds D1's 50m Tier-2 Yip downlink range")

        # Both hear GCS Howl (<= 140m), so both enter Yip-Lost!
        self.assertTrue(d0.uplink_active)
        self.assertTrue(d1.uplink_active)
        self.assertTrue(d0.yip_lost)
        self.assertTrue(d1.yip_lost)

    def test_bug02_uplink_dispatch_at_135m_and_alpha_tracking(self):
        """BUG-02 Verification:
        1. C2 uplink dispatch succeeds at 135m (within 140m GCS Howl horizon) in both WIRELESS_STAR
           and GOSSIP_CONSENSUS modes.
        2. Leader flight tracking to Waypoint Alpha [60.0, 40.0, 30.0] reaches arrival acceptance
           radius without stalling at (12, 8, Z) or premature cutoff.
        3. Tether failsafes are completely released when switching to wireless modes."""
        # 1. Test 135m uplink in LEADER_FOLLOWER mode
        self.sim.set_comms_mode("LEADER_FOLLOWER")
        leader = self.sim.leader
        self.assertIsNotNone(leader)
        # Position leader at 135m (< 140m RADIO_RANGE)
        leader.pos = np.array([135.0, 0.0, 25.0], dtype=float)
        res = self.sim.set_target([135.0, 20.0, 30.0])
        self.assertEqual(res["status"], "OK")
        self.assertEqual(res["channel"], "WIRELESS_STAR")
        self.sim.step(0.016)
        np.testing.assert_allclose(leader.acknowledged_target, [135.0, 20.0, 30.0])

        # Beyond 140m fails (e.g. 145m)
        leader.pos = np.array([145.0, 0.0, 25.0], dtype=float)
        res_out = self.sim.set_target([145.0, 20.0, 30.0])
        self.assertEqual(res_out["status"], "OUT_OF_RANGE")

        # 2. Test 135m uplink in GOSSIP_CONSENSUS mode
        self.sim.set_comms_mode("GOSSIP_CONSENSUS")
        for d in self.sim.drones:
            d.pos = np.array([180.0, d.id * 5.0, 25.0], dtype=float)
        # Place Drone 3 at 135m
        self.sim.drones[3].pos = np.array([135.0, 0.0, 20.0], dtype=float)
        res_gossip = self.sim.set_target([135.0, 30.0, 25.0])
        self.assertEqual(res_gossip["status"], "OK")
        self.assertEqual(res_gossip["seed_drone_id"], 3)

        # 3. Mode transition clears tether severed/lost state
        self.sim.set_comms_mode("TETHERED_RELAY")
        self.sim.tether_severed = True
        self.sim.drones[0].tether_lost = True
        self.sim.set_comms_mode("LEADER_FOLLOWER")
        self.assertFalse(self.sim.tether_severed)
        self.assertFalse(self.sim.drones[0].tether_lost)

        # 4. Flight tracking to Alpha [60.0, 40.0, 30.0]
        # Re-initialize fresh simulator at pad
        sim_alpha = SwarmSimulator(drone_count=5, seed=42)
        sim_alpha.set_comms_mode("LEADER_FOLLOWER")
        res_alpha = sim_alpha.set_target([60.0, 40.0, 30.0])
        self.assertEqual(res_alpha["status"], "OK")

        # Step 9 seconds (540 ticks)
        for _ in range(540):
            sim_alpha.step(1.0 / 60.0)

        dist_to_alpha = float(np.linalg.norm(sim_alpha.leader.pos - np.array([60.0, 40.0, 30.0])))
        self.assertLess(dist_to_alpha, 2.5, f"Leader should arrive at Alpha [60, 40, 30], but was at {sim_alpha.leader.pos}")
        # Verify leader is NOT stuck near (12, 8, Z)
        dist_to_stuck = float(np.linalg.norm(sim_alpha.leader.pos[:2] - np.array([12.0, 8.0])))
        self.assertGreater(dist_to_stuck, 30.0, "Leader is stuck near grid-scaled (12, 8) position!")

    def test_bug05_downlink_telemetry_masked_beyond_bork_range(self):
        """[BUG-05] Airborne airframes that exceed their downlink horizon (70m Bork / 50m Yip)
        must have their downlinked GCS telemetry masked to LKP with status LOST_LINK,
        while God-View sim_drones preserves unmasked ground truth."""
        sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=42)
        sim.set_comms_mode("LEADER_FOLLOWER")

        leader = sim.leader
        self.assertIsNotNone(leader)
        leader.rf_power_mode = "BORK"
        follower = sim.drones[1]

        # 1. Within 70m Bork horizon (at 50m)
        leader.pos = np.array([50.0, 0.0, 20.0], dtype=float)
        leader.lkp = np.array([50.0, 0.0, 20.0], dtype=float)
        follower.pos = np.array([45.0, 5.0, 20.0], dtype=float)
        follower.lkp = np.array([45.0, 5.0, 20.0], dtype=float)

        sim._update_radio_comms()
        self.assertTrue(leader.downlink_active)

        snap = sim.get_state_snapshot()
        gcs_lead = next(d for d in snap["drones"] if d["id"] == leader.id)
        sim_lead = next(d for d in snap["sim_drones"] if d["id"] == leader.id)

        # In range: GCS receives live coordinates
        np.testing.assert_allclose(gcs_lead["pos"], [50.0, 0.0, 20.0])
        np.testing.assert_allclose(sim_lead["pos"], [50.0, 0.0, 20.0])

        # 2. Exceed 70m Bork horizon (move leader to 75m)
        # Record LKP at 68m before losing downlink
        leader.pos = np.array([75.0, 0.0, 20.0], dtype=float)
        leader.lkp = np.array([68.0, 0.0, 20.0], dtype=float)

        # Move follower beyond 50m Yip range and beyond leader contact
        follower.pos = np.array([65.0, 0.0, 20.0], dtype=float)
        follower.lkp = np.array([48.0, 0.0, 20.0], dtype=float)

        sim._update_radio_comms()
        self.assertFalse(leader.downlink_active)
        self.assertFalse(follower.downlink_active)

        snap_out = sim.get_state_snapshot()
        gcs_lead_out = next(d for d in snap_out["drones"] if d["id"] == leader.id)
        sim_lead_out = next(d for d in snap_out["sim_drones"] if d["id"] == leader.id)

        gcs_fol_out = next(d for d in snap_out["drones"] if d["id"] == follower.id)
        sim_fol_out = next(d for d in snap_out["sim_drones"] if d["id"] == follower.id)

        # Operator Airgap: GCS receives [LOST_LINK] at LKP, zeroed velocity
        self.assertEqual(gcs_lead_out["status"], "LOST_LINK")
        self.assertEqual(gcs_lead_out["flight_state"], "LOST_LINK")
        self.assertTrue(gcs_lead_out["lost_link"])
        np.testing.assert_allclose(gcs_lead_out["pos"], [68.0, 0.0, 20.0])
        self.assertEqual(gcs_lead_out["vel"], [0.0, 0.0, 0.0])

        self.assertEqual(gcs_fol_out["status"], "LOST_LINK")
        self.assertEqual(gcs_fol_out["flight_state"], "LOST_LINK")
        self.assertTrue(gcs_fol_out["lost_link"])
        np.testing.assert_allclose(gcs_fol_out["pos"], [48.0, 0.0, 20.0])
        self.assertEqual(gcs_fol_out["vel"], [0.0, 0.0, 0.0])

        # God-View Ground Truth: Sim retains true coordinates
        np.testing.assert_allclose(sim_lead_out["pos"], [75.0, 0.0, 20.0])
        np.testing.assert_allclose(sim_fol_out["pos"], [65.0, 0.0, 20.0])
        self.assertFalse(sim_lead_out["crashed"])
        self.assertFalse(sim_fol_out["crashed"])

    def test_bug07_continuous_lkp_refresh_prevents_launchpad_snap(self):
        """Verify BUG-07: Continuous LKP updates prevent airframe snapping to launchpad on link drop."""
        sim = SwarmSimulator(drone_count=3, enable_packet_loss=False, seed=42)
        leader = sim.leader
        self.assertIsNotNone(leader)

        # Leader flies to a remote airborne coordinate in tethered mode
        target_pos = np.array([45.0, 35.0, 25.0], dtype=float)
        leader.pos = np.copy(target_pos)
        leader.flight_state = "CRUISE"

        # Step sim for 0.2s (12 frames) to let radio broadcast tick fire
        for _ in range(12):
            leader.pos = np.copy(target_pos)
            sim.step()

        # In tethered relay with intact tether, downlink is active
        self.assertTrue(leader.downlink_active)
        # BUG-07 Invariant: leader LKP must match its current position, not launchpad
        np.testing.assert_allclose(leader.lkp, target_pos, atol=1e-2)

        # Now simulate link loss (e.g. tether severed + beyond range)
        sim.tether_severed = True
        leader.pos = np.array([160.0, 35.0, 25.0], dtype=float) # Beyond Howl/Bork
        sim._update_radio_comms()
        self.assertFalse(leader.downlink_active)

        # Telemetry snapshot for GCS
        snap = sim.get_state_snapshot()
        gcs_leader = next(d for d in snap["drones"] if d["id"] == leader.id)

        # Must report the frozen LKP [45.0, 35.0, 25.0], NEVER pad 0 coordinates [0.0, -6.0, 0.5]
        self.assertTrue(gcs_leader["lost_link"])
        np.testing.assert_allclose(gcs_leader["pos"], target_pos, atol=0.1)
        self.assertGreater(float(np.linalg.norm(np.array(gcs_leader["pos"]) - np.array([0.0, -6.0, 0.5]))), 30.0)


if __name__ == "__main__":
    unittest.main()
