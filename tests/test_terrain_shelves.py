"""
Unit tests for TKT-65: Modular 3D Terrain Blocks & Tactical Obstacle Shelves ("Obby").
Validates surface elevation, overhead ceiling, dynamic floor clamping, mesa rooftop landings,
side-wall structural clash enforcement, fly-under skybridge clearance, and scenario loading.
"""

import unittest
import numpy as np

from SIM.backend.sitl_engine import SITLWorldEngine
from SIM.backend.urban_geometry import (
    UrbanObstacle,
    get_default_urban_obstacles,
    get_cyberpunk_obby_obstacles,
    get_surface_elevation,
    get_overhead_ceiling,
    is_on_obstacle_top,
    point_to_aabb_distance,
)
from OBC.backend.drone_firmware import FlightState


class TestTerrainShelvesAndObby(unittest.TestCase):
    """Test suite for modular 3D terrain blocks and altitude shelves."""

    def setUp(self):
        self.engine = SITLWorldEngine(drone_count=4, cold_start=False)
        self.obby_obstacles = get_cyberpunk_obby_obstacles()

    def test_surface_elevation_calculations(self):
        """Verify get_surface_elevation correctly distinguishes open ground, mesas, and shelves."""
        # 1. Open ground
        elev_ground = get_surface_elevation(np.array([0.0, 0.0, 10.0]), self.obby_obstacles)
        self.assertAlmostEqual(elev_ground, 0.0)

        # 2. Directly above MESA-LOW (bounds: 15-35, 10-30, 0-6m)
        elev_mesa_low = get_surface_elevation(np.array([25.0, 20.0, 10.0]), self.obby_obstacles)
        self.assertAlmostEqual(elev_mesa_low, 6.0)

        # 3. Directly above MESA-HIGH (bounds: 45-70, -25-5, 0-14m)
        elev_mesa_high = get_surface_elevation(np.array([55.0, -10.0, 20.0]), self.obby_obstacles)
        self.assertAlmostEqual(elev_mesa_high, 14.0)

        # 4. Directly above SKYWAY-DECK (bounds: 25-55, 35-45, 8-11m) at Z=15m
        elev_skyway_above = get_surface_elevation(np.array([40.0, 40.0, 15.0]), self.obby_obstacles)
        self.assertAlmostEqual(elev_skyway_above, 11.0)

        # 5. Underneath SKYWAY-DECK at Z=4m (the bridge deck is overhead, so ground is 0.0m)
        elev_skyway_under = get_surface_elevation(np.array([40.0, 40.0, 4.0]), self.obby_obstacles)
        self.assertAlmostEqual(elev_skyway_under, 0.0)

    def test_overhead_ceiling_detection(self):
        """Verify get_overhead_ceiling detects underpasses, bridges, and clear sky."""
        # 1. Underneath SKYWAY-DECK at Z=4m (deck floor is at 8m)
        ceiling_under = get_overhead_ceiling(np.array([40.0, 40.0, 4.0]), self.obby_obstacles)
        self.assertIsNotNone(ceiling_under)
        self.assertAlmostEqual(ceiling_under, 8.0)

        # 2. Above SKYWAY-DECK at Z=15m (clear sky overhead)
        ceiling_above = get_overhead_ceiling(np.array([40.0, 40.0, 15.0]), self.obby_obstacles)
        self.assertIsNone(ceiling_above)

        # 3. Open terrain (clear sky overhead)
        ceiling_open = get_overhead_ceiling(np.array([0.0, 0.0, 5.0]), self.obby_obstacles)
        self.assertIsNone(ceiling_open)

    def test_dynamic_surface_floor_clamping(self):
        """Verify airborne drones are clamped to surf_elev + 0.5m when above elevated terrain."""
        self.engine.load_urban_scenario("cyberpunk_obby")
        d = self.engine.drones[1]
        d.flight_state = "CRUISE"
        # Position over MESA-LOW (ceiling: 6.0m) at Z=6.2m (below the 6.5m cruise safety floor)
        d.pos = np.array([25.0, 20.0, 6.2], dtype=float)
        d.vel = np.array([0.0, 0.0, -1.0], dtype=float)

        self.engine.step(1.0 / 60.0)

        # Floor clamp over 6.0m mesa in CRUISE must enforce Z >= 6.5m
        self.assertGreaterEqual(d.pos[2], 6.5 - 1e-4)

    def test_mesa_rooftop_landing(self):
        """Verify a drone can land cleanly on top of a 6.0m mesa without structural clash."""
        self.engine.load_urban_scenario("cyberpunk_obby")
        d = self.engine.drones[2]
        d.flight_state = "FIELD_DESCENDING"
        d.landing_pad = np.array([25.0, 20.0, 6.05], dtype=float)
        d.target_station = np.array([25.0, 20.0, 6.05], dtype=float)
        d.pos = np.array([25.0, 20.0, 6.12], dtype=float)  # Close to touchdown
        d.vel = np.array([0.0, 0.0, -0.2], dtype=float)

        for _ in range(60):
            self.engine.step(1.0 / 60.0)
            if d.flight_state == "FIELD_LANDED":
                break

        self.assertFalse(d.crashed, "Drone must NOT crash when landing on a landable mesa rooftop")
        self.assertEqual(d.flight_state, "FIELD_LANDED")
        self.assertAlmostEqual(d.pos[2], 6.05, delta=0.08)
        self.assertAlmostEqual(d.rotor_rpm, 0.0)

    def test_mesa_side_wall_structural_clash(self):
        """Verify flying horizontally into the side wall of a mesa triggers a structural crash."""
        self.engine.load_urban_scenario("cyberpunk_obby")
        d = self.engine.drones[3]
        d.flight_state = "CRUISE"
        # MESA-LOW x_min is 15.0m. Position drone at X=14.0m, Z=3.0m (below ceiling 6.0m) moving into wall
        d.pos = np.array([14.0, 20.0, 3.0], dtype=float)
        d.vel = np.array([4.0, 0.0, 0.0], dtype=float)

        self.engine.step(1.0 / 60.0)

        self.assertTrue(d.crashed, "Drone must crash when entering Zone 4 clash envelope of mesa wall")
        self.assertIn("MESA-LOW", d.mayday_reason)

    def test_fly_under_skybridge_clearance(self):
        """Verify a drone flying at Z=4.0m passes cleanly underneath the 8m-11m skybridge without clashing."""
        self.engine.load_urban_scenario("cyberpunk_obby")
        d = self.engine.drones[0]
        d.flight_state = "CRUISE"
        # Skybridge spans X: 25-55, Y: 35-45, Z: 8-11m.
        # Command navigation target across the bridge span along Y axis from Y=30 to Y=60 at X=40, Z=4.0m
        nav_target = np.array([40.0, 60.0, 4.0], dtype=float)
        self.engine.target_pos = nav_target
        d.target_station = np.copy(nav_target)
        d.acknowledged_target = np.copy(nav_target)
        d.pos = np.array([40.0, 30.0, 4.0], dtype=float)
        d.vel = np.array([0.0, 8.0, 0.0], dtype=float)

        for _ in range(180):  # 3.0 seconds at ~8-12 m/s traverses to Y=60
            self.engine.step(1.0 / 60.0)
            self.assertFalse(d.crashed, "Drone must not crash while flying underneath skybridge clearance")
            if d.pos[1] >= 48.0:
                break

        self.assertGreater(d.pos[1], 46.0)

    def test_urban_scenario_switching(self):
        """Verify loading different urban and terrain scenarios works dynamically."""
        # 1. Obby scenario
        obs_obby = self.engine.load_urban_scenario("cyberpunk_obby")
        self.assertEqual(len(obs_obby), 5)
        self.assertEqual(self.engine.urban_scenario_name, "cyberpunk_obby")

        # 2. Default canyon
        obs_canyon = self.engine.load_urban_scenario("default_canyon")
        self.assertEqual(len(obs_canyon), 4)
        self.assertEqual(self.engine.urban_scenario_name, "default_canyon")

        # 3. Clear/Empty
        obs_empty = self.engine.load_urban_scenario("empty")
        self.assertEqual(len(obs_empty), 0)
        self.assertEqual(self.engine.urban_scenario_name, "empty")

    def test_atc_safe_flat_space_on_mesa(self):
        """Verify find_safe_flat_space finds safe landing spot on top of mesa when elevated."""
        self.engine.load_urban_scenario("cyberpunk_obby")
        drone_pos = np.array([25.0, 20.0, 10.0], dtype=float)  # Over center of MESA-LOW

        spot = self.engine.atc.find_safe_flat_space(
            drone_pos=drone_pos,
            obstacles=self.engine.urban_obstacles,
            drones=self.engine.drones,
        )

        # Must land on MESA-LOW (Z = 6.05m) inset inside [15, 35] x [10, 30]
        self.assertAlmostEqual(spot[2], 6.05)
        self.assertGreaterEqual(spot[0], 17.0)
        self.assertLessEqual(spot[0], 33.0)
        self.assertGreaterEqual(spot[1], 12.0)
        self.assertLessEqual(spot[1], 28.0)


if __name__ == "__main__":
    unittest.main()
