"""
Unit test suite for TKT-57: Airframe Tier Registry, Tether Prefixes & Mission Authorization Gate.
Validates:
- Airframe tier taxonomy (T0, T1.5, T3) and tether prefix modifiers (T-, t-).
- Default fleet configuration: 8x T1.5 Beta (t-T1.5 flagship when tethered).
- Dynamic RF power modes (YIP vs BORK) and autonomic succession switching.
- C2 Gateway pre-flight mission authorization checks (PAYLOAD_DROP, LIDAR_SCAN, etc.).
- Pre-flight gate enforcement inside deploy_mission_route.
"""

import unittest
import numpy as np

from OBC.backend.drone_firmware import (
    Drone,
    BASE_TIER_T0,
    BASE_TIER_T1_5,
    BASE_TIER_T3,
    TETHER_NONE,
    TETHER_DATA_ONLY,
    TETHER_POWER_DATA,
    RF_MODE_YIP,
    RF_MODE_BORK,
    RF_MODE_HOWL,
    DEFAULT_CAPABILITIES_T1_5,
)
from SIM.backend.sitl_engine import SwarmSimulator
from GCS.backend.c2_gateway import C2Gateway


class TestTierRegistry(unittest.TestCase):
    """Test suite for airframe tier and tether prefix taxonomy."""

    def test_full_tier_designation_modifiers(self):
        """Drone full_tier_designation correctly applies prefix semantics."""
        # Tethered Power + Data
        d_power = Drone(drone_id=0, base_tier=BASE_TIER_T0, tether_type=TETHER_POWER_DATA)
        self.assertEqual(d_power.full_tier_designation, "T-T0")

        # Tethered Data-Only
        d_fiber = Drone(drone_id=1, base_tier=BASE_TIER_T1_5, tether_type=TETHER_DATA_ONLY)
        self.assertEqual(d_fiber.full_tier_designation, "t-T1.5")

        # Wireless / Untethered
        d_wireless = Drone(drone_id=2, base_tier=BASE_TIER_T1_5, tether_type=TETHER_NONE)
        self.assertEqual(d_wireless.full_tier_designation, "T1.5")

        # Heavy specialist wireless
        d_heavy = Drone(drone_id=3, base_tier=BASE_TIER_T3, tether_type=TETHER_NONE)
        self.assertEqual(d_heavy.full_tier_designation, "T3")

    def test_default_fleet_tethered(self):
        """Default 8-drone fleet in tethered mode sets t-T1.5 Bork for leader and T1.5 Yip for escorts."""
        sim = SwarmSimulator(drone_count=8, is_tethered=True)
        leader = sim.drones[0]
        self.assertEqual(leader.base_tier, BASE_TIER_T1_5)
        self.assertEqual(leader.tether_type, TETHER_DATA_ONLY)
        self.assertEqual(leader.full_tier_designation, "t-T1.5")
        self.assertEqual(leader.rf_power_mode, RF_MODE_BORK)
        self.assertTrue(leader.is_leader)

        for i in range(1, 8):
            follower = sim.drones[i]
            self.assertEqual(follower.base_tier, BASE_TIER_T1_5)
            self.assertEqual(follower.tether_type, TETHER_NONE)
            self.assertEqual(follower.full_tier_designation, "T1.5")
            self.assertEqual(follower.rf_power_mode, RF_MODE_YIP)
            self.assertFalse(follower.is_leader)

    def test_default_fleet_untethered(self):
        """Default 8-drone fleet in untethered mode sets T1.5 Bork for leader and T1.5 Yip for escorts."""
        sim = SwarmSimulator(drone_count=8, is_tethered=False)
        leader = sim.drones[0]
        self.assertEqual(leader.base_tier, BASE_TIER_T1_5)
        self.assertEqual(leader.tether_type, TETHER_NONE)
        self.assertEqual(leader.full_tier_designation, "T1.5")
        self.assertEqual(leader.rf_power_mode, RF_MODE_BORK)

        for i in range(1, 8):
            follower = sim.drones[i]
            self.assertEqual(follower.full_tier_designation, "T1.5")
            self.assertEqual(follower.rf_power_mode, RF_MODE_YIP)

    def test_tether_severance_prefix_transition(self):
        """Severing tether transitions designation from t-T1.5 to T1.5 seamlessly."""
        sim = SwarmSimulator(drone_count=4, is_tethered=True)
        leader = sim.drones[0]
        self.assertEqual(leader.full_tier_designation, "t-T1.5")

        sim.sever_tether()
        self.assertFalse(sim.is_tethered)
        self.assertEqual(leader.tether_type, TETHER_NONE)
        self.assertEqual(leader.full_tier_designation, "T1.5")

    def test_dynamic_succession_rf_modes(self):
        """Autonomic succession demotes old leader to Yip and promotes successor to Bork."""
        sim = SwarmSimulator(drone_count=4)
        old_leader = sim.drones[0]
        successor = sim.drones[1]

        self.assertEqual(old_leader.rf_power_mode, RF_MODE_BORK)
        self.assertEqual(successor.rf_power_mode, RF_MODE_YIP)

        # Invalidate / kill leader
        sim.kill_drone(0)
        self.assertTrue(old_leader.crashed)

        # Trigger physics step which checks heartbeat and invokes _trigger_autonomic_succession
        sim.step(dt=0.2)

        self.assertEqual(old_leader.rf_power_mode, RF_MODE_YIP)
        self.assertEqual(successor.rf_power_mode, RF_MODE_BORK)
        self.assertTrue(successor.is_leader)

    def test_reinforcement_spawning_with_custom_tier(self):
        """Spawning reinforcement with custom tier and capabilities registers correctly."""
        sim = SwarmSimulator(drone_count=4)
        custom_caps = ["HEAVY_PAYLOAD", "DROP_SERVO", "LIDAR_SCAN"]
        sim.spawn_reinforcement(base_tier=BASE_TIER_T3, capabilities=custom_caps)

        new_drone = sim.drones[-1]
        self.assertEqual(new_drone.base_tier, BASE_TIER_T3)
        self.assertEqual(new_drone.full_tier_designation, "T3")
        self.assertEqual(new_drone.capabilities, custom_caps)

    def test_telemetry_snapshot_schema(self):
        """Telemetry snapshot exposes full tier designation, rf mode, and link budget."""
        sim = SwarmSimulator(drone_count=4, is_tethered=True)
        snap = sim.get_state_snapshot()

        # Link budget check
        self.assertIn("rf_link_budget", snap)
        self.assertEqual(snap["rf_link_budget"].get("bork_amplified_downlink"), 70.0)

        # Drone telemetry schema check
        d0 = snap["drones"][0]
        self.assertEqual(d0["base_tier"], "T1.5")
        self.assertEqual(d0["tether_type"], TETHER_DATA_ONLY)
        self.assertEqual(d0["full_tier_designation"], "t-T1.5")
        self.assertEqual(d0["rf_power_mode"], "BORK")
        self.assertIn("FORMATION_FLIGHT", d0["capabilities"])


class TestMissionAuthorizationGate(unittest.TestCase):
    """Test suite for C2 Pre-Flight Mission Authorization Gate."""

    def setUp(self):
        self.c2 = C2Gateway()

    def test_standard_mission_authorization(self):
        """Standard baseline missions (PATROL, RECON, PATROL_TO_BINGO) are authorized for T1.5 fleet."""
        sim = SwarmSimulator(drone_count=8)

        # Direct sim check
        res_patrol = sim.check_mission_authorization("PATROL")
        self.assertTrue(res_patrol["authorized"])
        self.assertEqual(res_patrol["mission_type"], "PATROL")

        res_recon = sim.check_mission_authorization("RECON")
        self.assertTrue(res_recon["authorized"])

        res_bingo = sim.check_mission_authorization("PATROL_TO_BINGO")
        self.assertTrue(res_bingo["authorized"])

    def test_specialist_mission_rejection(self):
        """Specialist missions fail pre-flight gate when fleet lacks required capabilities."""
        sim = SwarmSimulator(drone_count=8)

        # PAYLOAD_DROP requires HEAVY_PAYLOAD and DROP_SERVO
        res_drop = sim.check_mission_authorization("PAYLOAD_DROP")
        self.assertFalse(res_drop["authorized"])
        self.assertEqual(res_drop["reason"], "TIER_AUTHORIZATION_FAILED")
        self.assertIn("HEAVY_PAYLOAD", res_drop["missing_capabilities"])
        self.assertIn("DROP_SERVO", res_drop["missing_capabilities"])

        # LIDAR_SCAN requires LIDAR_SCAN
        res_lidar = sim.check_mission_authorization("LIDAR_SCAN")
        self.assertFalse(res_lidar["authorized"])
        self.assertIn("LIDAR_SCAN", res_lidar["missing_capabilities"])

        # SIGINT_TRIANGULATION requires JAMMER_TRIANGULATION
        res_sigint = sim.check_mission_authorization("SIGINT_TRIANGULATION")
        self.assertFalse(res_sigint["authorized"])
        self.assertIn("JAMMER_TRIANGULATION", res_sigint["missing_capabilities"])

    def test_specialist_mission_approval_after_reinforcement(self):
        """Adding a specialist T3 reinforcement airframe satisfies the authorization gate."""
        sim = SwarmSimulator(drone_count=4)

        # Initially rejected
        self.assertFalse(sim.check_mission_authorization("PAYLOAD_DROP")["authorized"])

        # Spawn T3 specialist
        sim.spawn_reinforcement(
            base_tier=BASE_TIER_T3,
            capabilities=["HEAVY_PAYLOAD", "DROP_SERVO", "LIDAR_SCAN", "FORMATION_FLIGHT"]
        )
        t3_drone = sim.drones[-1]

        # Now authorized
        res = sim.check_mission_authorization("PAYLOAD_DROP")
        self.assertTrue(res["authorized"])
        self.assertEqual(res["allocated_drone_id"], t3_drone.id)
        self.assertEqual(res["allocated_drone_tier"], "T3")

    def test_targeted_drone_authorization(self):
        """Targeting a specific airframe verifies that specific drone's capabilities."""
        sim = SwarmSimulator(drone_count=4)
        sim.spawn_reinforcement(
            base_tier=BASE_TIER_T3,
            capabilities=["HEAVY_PAYLOAD", "DROP_SERVO", "LIDAR_SCAN", "FORMATION_FLIGHT"]
        )
        t3_drone = sim.drones[-1]

        # Drone 0 (T1.5) targeted for PAYLOAD_DROP should be rejected
        res_d0 = sim.check_mission_authorization("PAYLOAD_DROP", target_drone_id=0)
        self.assertFalse(res_d0["authorized"])
        self.assertEqual(res_d0["target_drone_id"], 0)

        # T3 drone targeted should be approved
        res_t3 = sim.check_mission_authorization("PAYLOAD_DROP", target_drone_id=t3_drone.id)
        self.assertTrue(res_t3["authorized"])
        self.assertEqual(res_t3["allocated_drone_id"], t3_drone.id)

    def test_deploy_mission_route_gate_enforcement(self):
        """deploy_mission_route blocks dispatch if the mission fails pre-flight authorization."""
        sim = SwarmSimulator(drone_count=4)
        wps = [[10.0, 20.0, 25.0, 2.0]]

        # Attempt to deploy PAYLOAD_DROP on standard T1.5 fleet
        err_res = sim.deploy_mission_route(wps, name="BOMB_RUN", mission_type="PAYLOAD_DROP")
        self.assertEqual(err_res.get("status"), "ERROR")
        self.assertEqual(err_res.get("code"), "TIER_AUTHORIZATION_FAILED")
        self.assertFalse(sim.route_active)

        # Standard PATROL succeeds
        ok_res = sim.deploy_mission_route(wps, name="PATROL_RUN", mission_type="PATROL")
        self.assertEqual(ok_res.get("status"), "OK")
        self.assertTrue(sim.route_active)


if __name__ == "__main__":
    unittest.main()
