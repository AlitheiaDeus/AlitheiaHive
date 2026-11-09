# tests/test_field_lz_and_eland.py
"""
Test Suite for TKT-64 / PLN-05: Designated Field Landing Zones (FLZ) & Autonomous Emergency Landings (E-Land).
Verifies:
1. Base launchpad recovery remains the 100% default pattern.
2. Designated Field Landing Zone (FLZ) slot allocation and radial ring spacing (>= 3.5m).
3. Automatic pad denial detection and diversion from Base to FLZ.
4. Beyond-BINGO energy calculations and autonomous E-Landing trigger.
5. Safe flat space search avoiding 3D urban obstacles (AABB clearance >= 3.0m).
6. Flagship E-Landing triggering smooth autonomic succession.
"""

import unittest
import numpy as np
import math

from SIM.backend.sitl_engine import SITLWorldEngine, DT, RADIO_RANGE
from SIM.backend.urban_geometry import UrbanObstacle
from SIM.backend.atc_recovery import FieldLandingZone, get_pad_position
from OBC.backend.drone_firmware import FlightState, Drone, TIER_LEADER


class TestFieldLZAndELand(unittest.TestCase):
    """Unit tests for Designated Field Landing Zones and Emergency Landings."""

    def test_default_remains_base_pads(self):
        """Verify that default RTB and landing route strictly to Base pads (0..7)."""
        sim = SITLWorldEngine(drone_count=4, start_in_air=True)

        # 1. Stage 1 RTB Hold with default parameters
        res_rtb = sim.command_rtb_hold()
        self.assertEqual(res_rtb.get("status"), "OK")
        self.assertEqual(res_rtb.get("destination"), "BASE")
        self.assertEqual(sim.recovery_stage, "OVERHEAD_MARSHAL")

        # Flagship target must be Base overhead sentry [0, 0, 25]
        np.testing.assert_allclose(sim.drones[0].acknowledged_target, [0.0, 0.0, 25.0], atol=0.1)

        # 2. Stage 2 Land Swarm with default parameters
        queued = sim.command_land_swarm()
        self.assertEqual(len(queued), 4)

        for i, d in enumerate(sim.drones):
            self.assertEqual(d.landing_pad_idx, i)
            self.assertIsNone(d.field_lz_slot)
            self.assertEqual(d.flight_state, "DESCENDING")
            base_pad = get_pad_position(i)
            np.testing.assert_allclose(d.landing_pad, base_pad, atol=0.01)

    def test_designated_field_lz_recovery(self):
        """Verify FLZ designation, radial slot allocation, and >= 3.5m spacing."""
        sim = SITLWorldEngine(drone_count=4, start_in_air=True)

        flz_center = [50.0, 30.0, 0.05]
        res_flz = sim.set_field_lz(coords=flz_center, name="FLZ-BRAVO", radius=20.0, capacity=8, slot_spacing=4.0)
        self.assertEqual(res_flz["status"], "OK")
        self.assertIsNotNone(sim.field_lz)
        self.assertEqual(sim.field_lz["name"], "FLZ-BRAVO")

        # Command RTB to FIELD_LZ
        res_rtb = sim.command_rtb_hold(destination="FIELD_LZ")
        self.assertEqual(res_rtb["destination"], "FIELD_LZ")
        for d in sim.drones:
            self.assertEqual(d.flight_state, FlightState.FIELD_HOLD)
            np.testing.assert_allclose(d.acknowledged_target, [50.0, 30.0, 25.0], atol=0.1)

        # Command Land Swarm to FIELD_LZ
        queued = sim.command_land_swarm(destination="FIELD_LZ")
        self.assertEqual(len(queued), 4)

        field_slots = []
        for d in sim.drones:
            self.assertEqual(d.flight_state, FlightState.FIELD_DESCENDING)
            self.assertIsNotNone(d.field_lz_slot)
            self.assertIsNone(d.landing_pad_idx)
            self.assertIsNotNone(d.landing_pad)
            field_slots.append(d.landing_pad)

        # Verify pairwise separation between assigned field slots is >= 3.5m
        for i in range(len(field_slots)):
            for j in range(i + 1, len(field_slots)):
                dist = float(np.linalg.norm(field_slots[i][:2] - field_slots[j][:2]))
                self.assertGreaterEqual(
                    dist,
                    3.5,
                    f"Field slots {i} and {j} too close: {dist:.2f}m < 3.5m",
                )

        # Place Drone 0 close to its field slot for touchdown verification
        sim.drones[0].pos = np.copy(sim.drones[0].landing_pad)
        sim.drones[0].pos[2] = 1.0

        for _ in range(250):
            sim.step(DT)
            if sim.drones[0].flight_state == "FIELD_LANDED":
                break

        self.assertEqual(sim.drones[0].flight_state, "FIELD_LANDED")
        self.assertTrue(sim.drones[0].touchdown_confirmed)
        self.assertEqual(sim.drones[0].rotor_rpm, 0.0)

    def test_pad_denial_auto_diversion(self):
        """Verify automatic diversion to FLZ when Base pads are obstructed."""
        sim = SITLWorldEngine(drone_count=4, start_in_air=True)

        # Set up a backup Field LZ
        sim.set_field_lz(coords=[30.0, 40.0, 0.05], name="FLZ-DIVERT")

        # Place an urban obstacle directly on top of Base pads (centered around y=-6.0)
        pad_blocker = UrbanObstacle(
            id="PAD_BLOCKER",
            name="Construction Crane / Rubble",
            bounds=(-5.0, 5.0, -10.0, -4.0, 0.0, 15.0),
        )
        sim.urban_obstacles = [pad_blocker]

        # Verify pad denial detector flags the obstruction
        denied = sim.atc.check_pad_denial(sim.urban_obstacles)
        self.assertTrue(denied)

        # Operator issues standard nominal "BASE" RTB -> should automatically divert to FLZ!
        res_rtb = sim.command_rtb_hold(destination="BASE")
        self.assertEqual(res_rtb["destination"], "FIELD_LZ")
        self.assertEqual(sim.drones[0].flight_state, FlightState.FIELD_HOLD)

        # Operator issues standard nominal "BASE" Land -> should automatically divert to FLZ!
        queued = sim.command_land_swarm(destination="BASE")
        self.assertEqual(len(queued), 4)
        for d in sim.drones:
            self.assertEqual(d.flight_state, FlightState.FIELD_DESCENDING)
            self.assertIsNotNone(d.field_lz_slot)

    def test_beyond_bingo_detection_and_auto_eland(self):
        """Verify Beyond-BINGO math and autonomous E-Landing execution."""
        sim = SITLWorldEngine(drone_count=2, start_in_air=True)

        drone = sim.drones[1]
        # Position drone out in the field (100m from base), close to ground (Z=1.5m) for fast sink test
        drone.pos = np.array([80.0, 60.0, 1.5], dtype=float)
        home = np.array([0.0, 0.0, 0.0], dtype=float)

        # Calculate BINGO energy needed: (100m / 12 m/s) * 0.25%/s + 10% = ~12.08%
        e_needed = drone.calculate_bingo_energy(home, cruise_speed=12.0, drain_rate=0.25, safety_margin=10.0)
        self.assertAlmostEqual(e_needed, 12.08, delta=0.5)

        # With 100% battery, drone is well above BINGO
        self.assertFalse(drone.is_beyond_bingo(home))

        # Drop battery to 5.0% (beyond BINGO: 5.0 < 12.08 - 3.0 = 9.08)
        drone.battery = 5.0
        self.assertTrue(drone.is_beyond_bingo(home))

        # Advance simulation: Beyond-BINGO watchdog should trigger autonomous E-Landing
        sim.step(DT)
        self.assertEqual(drone.flight_state, "EMERGENCY_LANDING")
        self.assertEqual(drone.status, "E_LANDING")
        self.assertEqual(drone.e_landing_reason, "BEYOND_BINGO")
        self.assertIsNotNone(drone.e_landing_spot)

        # Advance until drone completes emergency touchdown
        for _ in range(250):
            sim.step(DT)
            if drone.flight_state == "E_LANDED":
                break

        self.assertEqual(drone.flight_state, "E_LANDED")
        self.assertEqual(drone.status, "E_LANDED")
        self.assertTrue(drone.touchdown_confirmed)
        self.assertEqual(drone.rotor_rpm, 0.0)
        self.assertLessEqual(drone.pos[2], 0.08)

    def test_safe_flat_space_obstacle_avoidance(self):
        """Verify that emergency landing scans avoid 3D urban obstacle volumes."""
        sim = SITLWorldEngine(drone_count=1, start_in_air=True)

        # Create building AABB directly beneath (x=0, y=0)
        building = UrbanObstacle(
            id="SKYSCRAPER",
            name="Tennozu Tower",
            bounds=(-10.0, 10.0, -10.0, 10.0, 0.0, 30.0),
        )
        sim.urban_obstacles = [building]

        # Drone is directly above the building center at [0, 0, 35]
        drone = sim.drones[0]
        drone.pos = np.array([0.0, 0.0, 35.0], dtype=float)

        # Search for safe flat space
        safe_spot = sim.atc.find_safe_flat_space(
            drone_pos=drone.pos,
            obstacles=sim.urban_obstacles,
            drones=sim.drones,
            search_radius=25.0,
        )

        # Ground clearance test: spot must be at ground level Z ~ 0.05m
        self.assertAlmostEqual(safe_spot[2], 0.05, places=2)

        # Spot must NOT be inside the building footprint [-10, 10]
        from SIM.backend.urban_geometry import point_to_aabb_distance
        dist_to_bldg = point_to_aabb_distance(safe_spot, building.bounds)
        self.assertGreaterEqual(
            dist_to_bldg,
            3.0,
            f"E-land spot {safe_spot} too close to building! Distance: {dist_to_bldg:.2f}m < 3.0m",
        )

    def test_flagship_e_land_succession(self):
        """Verify that an emergency landing by Drone #0 triggers autonomic succession to 2IC."""
        sim = SITLWorldEngine(drone_count=4, start_in_air=True)

        leader = sim.drones[0]
        self.assertTrue(leader.is_leader)
        leader.pos[2] = 1.5  # Set close to ground for fast sink test

        # Command Drone #0 into emergency landing
        success = sim.command_e_land(drone_id=0, reason="CRITICAL_ROTOR_FAILURE")
        self.assertTrue(success)
        self.assertEqual(leader.flight_state, "EMERGENCY_LANDING")

        # Advance sim until touchdown
        for _ in range(250):
            sim.step(DT)
            if leader.flight_state == "E_LANDED":
                break

        self.assertEqual(leader.flight_state, "E_LANDED")
        self.assertFalse(leader.is_leader)

        # Check succession promoted another drone
        new_leader = sim.leader
        self.assertIsNotNone(new_leader)
        self.assertNotEqual(new_leader.id, 0)
        self.assertTrue(new_leader.is_leader)
        self.assertEqual(new_leader.status, "FLEET_LEADER")


if __name__ == "__main__":
    unittest.main()
