# tests/test_urban_geometry_rf_shadow.py
"""
Unit tests for 3D Urban Geometry, Altitude Shelves, Structural Clash & RF LOS Shadowing.
Validates PLN-08 / TKT-60: 50/50 Architecture across 4 Explicit Airgaps.
"""

import unittest
import numpy as np

from SIM.backend.urban_geometry import (
    UrbanObstacle,
    point_to_aabb_distance,
    point_to_aabb_repulsion,
    ray_intersects_aabb,
    get_default_urban_obstacles,
)
from SIM.backend.sitl_engine import SITLWorldEngine, SwarmSimulator


class TestUrbanGeometryRFShadow(unittest.TestCase):
    """Test suite for 3D AABB mathematical invariants, altitude shelves, and RF occlusion."""

    def setUp(self):
        self.obstacles = get_default_urban_obstacles()
        self.bld_alpha = next(o for o in self.obstacles if o.id == "BLD-ALPHA")
        self.skybridge = next(o for o in self.obstacles if o.id == "BRG-SKYBRIDGE")
        self.twr_charlie = next(o for o in self.obstacles if o.id == "TWR-CHARLIE")

    def test_point_to_aabb_distance_outside_and_inside(self):
        """Verify Euclidean distance is 0.0 inside AABB and exact distance outside."""
        # Inside Building Alpha: [30, 50, 45, 75, 0, 40]
        p_inside = np.array([40.0, 60.0, 20.0])
        self.assertEqual(point_to_aabb_distance(p_inside, self.bld_alpha.bounds), 0.0)

        # On surface of Building Alpha (x = 30.0)
        p_surface = np.array([30.0, 60.0, 20.0])
        self.assertEqual(point_to_aabb_distance(p_surface, self.bld_alpha.bounds), 0.0)

        # 10m West of Building Alpha (x = 20.0, nearest x_min is 30.0)
        p_west = np.array([20.0, 60.0, 20.0])
        self.assertAlmostEqual(point_to_aabb_distance(p_west, self.bld_alpha.bounds), 10.0, places=4)

        # 15m above Building Alpha (z = 55.0, ceiling is 40.0)
        p_above = np.array([40.0, 60.0, 55.0])
        self.assertAlmostEqual(point_to_aabb_distance(p_above, self.bld_alpha.bounds), 15.0, places=4)

    def test_skybridge_altitude_shelves(self):
        """Verify explicit altitude shelves: Underflight (Z=8m), Deck Clash (Z=18m), Overflight (Z=25m)."""
        # Skybridge bounds: [36, 44, 35, 45, 14, 20]
        # 1. Underflight at Z=8m (6m clearance to deck floor at Z=14m)
        p_under = np.array([40.0, 40.0, 8.0])
        d_under = point_to_aabb_distance(p_under, self.skybridge.bounds)
        self.assertAlmostEqual(d_under, 6.0, places=4)
        self.assertGreater(d_under, 1.5, "Airframe flying at Z=8m must have safe clearance under skybridge")

        # 2. Structural clash inside bridge deck at Z=18m
        p_deck = np.array([40.0, 40.0, 18.0])
        d_deck = point_to_aabb_distance(p_deck, self.skybridge.bounds)
        self.assertEqual(d_deck, 0.0, "Airframe inside deck structure must register distance 0.0 (clash)")

        # 3. Overflight at Z=25m (5m clearance over ceiling at Z=20m)
        p_over = np.array([40.0, 40.0, 25.0])
        d_over = point_to_aabb_distance(p_over, self.skybridge.bounds)
        self.assertAlmostEqual(d_over, 5.0, places=4)
        self.assertGreater(d_over, 1.5, "Airframe flying at Z=25m must have safe clearance over skybridge")

    def test_ray_intersects_aabb(self):
        """Verify 3D slab ray-casting correctly identifies hits, misses, and altitude clearances."""
        # 1. Direct hit penetrating Skybridge deck at Z=17m
        origin = np.array([0.0, 40.0, 17.0])
        target = np.array([60.0, 40.0, 17.0])
        self.assertTrue(
            ray_intersects_aabb(origin, target, self.skybridge.bounds),
            "Horizontal ray at Z=17m must intersect skybridge deck",
        )

        # 2. Ray passing cleanly underneath Skybridge at Z=6m
        origin_under = np.array([0.0, 40.0, 6.0])
        target_under = np.array([60.0, 40.0, 6.0])
        self.assertFalse(
            ray_intersects_aabb(origin_under, target_under, self.skybridge.bounds),
            "Horizontal ray at Z=6m must pass underneath skybridge without occlusion",
        )

        # 3. Ray passing cleanly over Skybridge at Z=26m
        origin_over = np.array([0.0, 40.0, 26.0])
        target_over = np.array([60.0, 40.0, 26.0])
        self.assertFalse(
            ray_intersects_aabb(origin_over, target_over, self.skybridge.bounds),
            "Horizontal ray at Z=26m must pass over skybridge without occlusion",
        )

        # 4. Ray segment stopping short before reaching Tower Charlie
        # Tower Charlie: [70, 90, 20, 40, 0, 35]
        origin_short = np.array([0.0, 30.0, 15.0])
        target_short = np.array([50.0, 30.0, 15.0])  # Stops at x=50, tower starts at x=70
        self.assertFalse(
            ray_intersects_aabb(origin_short, target_short, self.twr_charlie.bounds),
            "Ray segment terminating before obstacle must not intersect",
        )

        # 5. Ray originating from inside obstacle
        origin_inside = np.array([80.0, 30.0, 15.0])
        target_outside = np.array([110.0, 30.0, 15.0])
        self.assertTrue(
            ray_intersects_aabb(origin_inside, target_outside, self.twr_charlie.bounds),
            "Ray starting inside obstacle must return True",
        )

    def test_point_to_aabb_repulsion_zone3(self):
        """Verify Zone 3 wall-sliding repulsion and momentum damping."""
        # Test point 2.5m west of Building Alpha (x = 27.5, x_min = 30.0)
        # In Zone 3: 1.5m < 2.5m <= 3.5m
        pos = np.array([27.5, 60.0, 20.0])
        vel_into_wall = np.array([5.0, 0.0, 0.0])  # Moving directly towards wall (+x direction)

        dist, a_evade = point_to_aabb_repulsion(
            pos, vel_into_wall, self.bld_alpha.bounds, zone3_outer=3.5, zone4_clash=1.5
        )
        self.assertAlmostEqual(dist, 2.5, places=4)
        # Normal points west (-x). Acceleration must push west (-x) to repel away from wall.
        self.assertLess(a_evade[0], 0.0, "Evasion acceleration must point away from building face")
        # Relative velocity damping must increase repulsion when heading into wall
        _, a_evade_static = point_to_aabb_repulsion(
            pos, np.zeros(3), self.bld_alpha.bounds, zone3_outer=3.5, zone4_clash=1.5
        )
        self.assertLess(a_evade[0], a_evade_static[0], "Momentum damping must add extra braking push in -x direction")

        # Outside Zone 3 (>3.5m)
        pos_far = np.array([20.0, 60.0, 20.0])  # 10m away
        dist_far, a_far = point_to_aabb_repulsion(
            pos_far, vel_into_wall, self.bld_alpha.bounds, zone3_outer=3.5, zone4_clash=1.5
        )
        self.assertAlmostEqual(dist_far, 10.0, places=4)
        np.testing.assert_array_equal(a_far, np.zeros(3))

    def test_sitl_initialization_and_snapshot_serialization(self):
        """Verify SITL engine loads default urban obstacles and serializes them in telemetry snapshots."""
        sim = SITLWorldEngine(drone_count=4, urban_scenario="default_canyon")
        self.assertEqual(len(sim.urban_obstacles), 4)

        snapshot = sim.get_state_snapshot()
        self.assertIn("urban_obstacles", snapshot)
        self.assertEqual(len(snapshot["urban_obstacles"]), 4)

        skybridge_data = next((o for o in snapshot["urban_obstacles"] if o["id"] == "BRG-SKYBRIDGE"), None)
        self.assertIsNotNone(skybridge_data)
        self.assertEqual(skybridge_data["obstacle_type"], "SKYBRIDGE")
        self.assertEqual(skybridge_data["color"], "#FFB800")
        self.assertEqual(skybridge_data["bounds"], [36.0, 44.0, 35.0, 45.0, 14.0, 20.0])

    def test_sitl_zone4_structural_clash_kills_motors(self):
        """Verify airframe penetrating Zone 4 structural clash envelope (d <= 1.5m) cuts motor power and crashes."""
        sim = SITLWorldEngine(drone_count=2, urban_scenario="default_canyon")
        drone = sim.drones[0]
        # Place drone directly inside Skybridge deck volume at Z=18m
        drone.pos = np.array([40.0, 40.0, 18.0], dtype=float)
        drone.vel = np.array([2.0, 0.0, 0.0], dtype=float)
        drone.flight_state = "CRUISE"

        sim.step(dt=1.0 / 60.0)

        self.assertTrue(drone.crashed, "Drone inside structural envelope must crash")
        self.assertEqual(drone.flight_state, "CRASHED")
        self.assertEqual(drone.status, "CRASHED")
        self.assertTrue(drone.collision_inferred)

    def test_sitl_skybridge_safe_underflight_and_overflight(self):
        """Verify altitude shelves: underflight (Z=8m) and overflight (Z=25m) proceed without clash."""
        sim = SITLWorldEngine(drone_count=2, urban_scenario="default_canyon")
        d_under = sim.drones[0]
        d_over = sim.drones[1]

        d_under.pos = np.array([40.0, 40.0, 8.0], dtype=float)
        d_under.flight_state = "CRUISE"

        d_over.pos = np.array([40.0, 40.0, 25.0], dtype=float)
        d_over.flight_state = "CRUISE"

        sim.step(dt=1.0 / 60.0)

        self.assertFalse(d_under.crashed, "Airframe flying at Z=8m must fly cleanly under Skybridge")
        self.assertFalse(d_over.crashed, "Airframe flying at Z=25m must fly cleanly over Skybridge")

    def test_sitl_rf_los_shadowing_behind_building_drops_link(self):
        """Verify ray-cast RF LOS occlusion cuts direct telemetry and drops packets behind tall structures."""
        sim = SITLWorldEngine(drone_count=2, urban_scenario="default_canyon")
        sim.comms_mode = "GOSSIP_CONSENSUS"
        sim.enable_packet_loss = False  # Isolate geometric occlusion from stochastic loss

        drone_east = sim.drones[1]
        # Move drone behind Tower Charlie [70, 90, 20, 40, 0, 35]
        # Line from tether anchor [0, 0, 1.5] to [100, 30, 15] intersects Tower Charlie
        drone_east.pos = np.array([100.0, 30.0, 15.0], dtype=float)
        drone_east.flight_state = "CRUISE"

        # Verify geometric ray occlusion check
        self.assertTrue(
            sim.is_rf_occluded(sim.tether_anchor, drone_east.pos),
            "Line of sight from GCS anchor to Drone East must be blocked by Tower Charlie",
        )

        sim._update_radio_comms()

        # Direct downlink and uplink must be blocked by Tower Charlie
        self.assertFalse(drone_east.downlink_active, "Downlink must be occluded behind Tower Charlie")
        self.assertFalse(drone_east.uplink_active, "Uplink must be occluded behind Tower Charlie")

    def test_sitl_rf_underflight_clean_los(self):
        """Verify RF ray passing cleanly under Skybridge is NOT occluded."""
        sim = SITLWorldEngine(drone_count=2, urban_scenario="default_canyon")
        # Position on far side of Skybridge, low altitude (Z=4m)
        target_pos = np.array([60.0, 40.0, 4.0], dtype=float)
        gcs_anchor = np.array([0.0, 40.0, 2.0], dtype=float)

        self.assertFalse(
            sim.is_rf_occluded(gcs_anchor, target_pos),
            "RF line passing underneath Skybridge at Z=2-4m must have clean LOS",
        )

    def test_sitl_zone3_wall_sliding_proximity_repulsion(self):
        """Verify drone approaching Building Alpha receives Zone 3 evasion push and records nearest_obstacle_dist."""
        sim = SITLWorldEngine(drone_count=2, urban_scenario="default_canyon")
        drone = sim.drones[0]
        # Place drone 2.5m from west wall of Building Alpha (x = 27.5, wall at x = 30)
        drone.pos = np.array([27.5, 60.0, 20.0], dtype=float)
        drone.vel = np.array([3.0, 0.0, 0.0], dtype=float)
        drone.flight_state = "CRUISE"

        sim.step(dt=1.0 / 60.0)

        self.assertFalse(drone.crashed, "Drone in Zone 3 must not crash immediately")
        self.assertAlmostEqual(drone.nearest_obstacle_dist, 2.5, places=1)
        self.assertEqual(drone.nearest_obstacle_id, "BLD-ALPHA")


if __name__ == "__main__":
    unittest.main()
