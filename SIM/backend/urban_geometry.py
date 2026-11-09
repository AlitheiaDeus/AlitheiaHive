# SIM/backend/urban_geometry.py
"""
Urban 3D Geometry, Altitude Shelves, Structural Colliders & Ray-Cast RF Shadowing.
Part of PLN-08 / TKT-60: 50/50 Architecture across 4 Explicit Airgaps.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Any, Optional
import numpy as np


@dataclass
class UrbanObstacle:
    """3D Axis-Aligned Bounding Box (AABB) obstacle with explicit altitude shelves.
    
    bounds: (x_min, x_max, y_min, y_max, z_floor, z_ceiling) in meters.
    Supports flying cleanly under (Z < z_floor) or over (Z > z_ceiling).
    """
    id: str
    name: str
    bounds: Tuple[float, float, float, float, float, float]
    blocks_rf: bool = True
    rf_attenuation_db: float = 40.0
    obstacle_type: str = "BUILDING"  # "BUILDING", "SKYBRIDGE", "TOWER", "TERRAIN_MESA", "PLATFORM"
    color: str = "#00F0FF"
    is_landable: Optional[bool] = None

    def __post_init__(self):
        if self.is_landable is None:
            self.is_landable = bool(self.obstacle_type in ("TERRAIN_MESA", "PLATFORM", "SKYBRIDGE"))

    @property
    def x_min(self) -> float:
        return self.bounds[0]

    @property
    def x_max(self) -> float:
        return self.bounds[1]

    @property
    def y_min(self) -> float:
        return self.bounds[2]

    @property
    def y_max(self) -> float:
        return self.bounds[3]

    @property
    def z_floor(self) -> float:
        return self.bounds[4]

    @property
    def z_ceiling(self) -> float:
        return self.bounds[5]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "bounds": list(self.bounds),
            "blocks_rf": self.blocks_rf,
            "rf_attenuation_db": self.rf_attenuation_db,
            "obstacle_type": self.obstacle_type,
            "color": self.color,
            "is_landable": self.is_landable,
        }


def point_to_aabb_distance(
    pos: np.ndarray,
    bounds: Tuple[float, float, float, float, float, float]
) -> float:
    """Calculate Euclidean distance from a 3D point to the surface of an AABB.
    
    Returns 0.0 if the point is strictly inside or on the surface of the box.
    """
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    p = np.asarray(pos, dtype=float)
    
    cx = np.clip(p[0], x_min, x_max)
    cy = np.clip(p[1], y_min, y_max)
    cz = np.clip(p[2], z_min, z_max)
    
    diff = p - np.array([cx, cy, cz], dtype=float)
    return float(np.linalg.norm(diff))


def get_surface_elevation(
    pos: np.ndarray,
    obstacles: Optional[List[UrbanObstacle]] = None,
    ground_base: float = 0.0
) -> float:
    """Find the highest solid surface directly below pos (x, y, z).
    
    Returns ground_base (default 0.0m) if over open terrain.
    For an obstacle shelf to be considered beneath the point, pos[2] must be
    at or above z_ceiling (with a slight 0.25m approach allowance) and within
    the horizontal bounds [x_min, x_max] x [y_min, y_max].
    """
    if not obstacles:
        return ground_base
    p = np.asarray(pos, dtype=float)
    x, y, z = p[0], p[1], p[2]
    highest = ground_base
    for obs in obstacles:
        if obs.is_landable and (obs.x_min <= x <= obs.x_max) and (obs.y_min <= y <= obs.y_max):
            if obs.z_floor <= 0.5:
                if obs.z_ceiling > highest:
                    highest = obs.z_ceiling
            elif z >= obs.z_ceiling - 0.25:
                if obs.z_ceiling > highest:
                    highest = obs.z_ceiling
    return float(highest)


def get_overhead_ceiling(
    pos: np.ndarray,
    obstacles: Optional[List[UrbanObstacle]] = None
) -> Optional[float]:
    """Find the lowest overhead obstacle shelf ceiling directly above pos (x, y, z).
    
    Returns None if the sky is clear above pos.
    """
    if not obstacles:
        return None
    p = np.asarray(pos, dtype=float)
    x, y, z = p[0], p[1], p[2]
    lowest_overhead: Optional[float] = None
    for obs in obstacles:
        if (obs.x_min <= x <= obs.x_max) and (obs.y_min <= y <= obs.y_max):
            if obs.z_floor > z:
                if lowest_overhead is None or obs.z_floor < lowest_overhead:
                    lowest_overhead = obs.z_floor
    return lowest_overhead


def is_on_obstacle_top(
    pos: np.ndarray,
    obs: UrbanObstacle,
    vertical_margin: float = 0.25,
    horizontal_inset: float = -0.1,
) -> bool:
    """Check if position is positioned safely on top of an obstacle's landable roof/deck."""
    if not obs.is_landable:
        return False
    p = np.asarray(pos, dtype=float)
    x, y, z = p[0], p[1], p[2]
    in_xy = (obs.x_min + horizontal_inset <= x <= obs.x_max - horizontal_inset) and \
            (obs.y_min + horizontal_inset <= y <= obs.y_max - horizontal_inset)
    at_top = (z >= obs.z_ceiling - vertical_margin)
    return bool(in_xy and at_top)


def point_to_aabb_repulsion(
    pos: np.ndarray,
    vel: np.ndarray,
    bounds: Tuple[float, float, float, float, float, float],
    zone3_outer: float = 3.5,
    zone4_clash: float = 1.5,
) -> Tuple[float, np.ndarray]:
    """Calculate distance and smooth Zone 3 wall-sliding repulsion acceleration.
    
    Includes relative-velocity damping (AGENTS.md Section 4: Zone 3 Momentum Damping)
    to brake incoming momentum before Zone 4 structural clash.
    
    Returns:
        (distance, evasion_accel_vector)
    """
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    p = np.asarray(pos, dtype=float)
    v = np.asarray(vel, dtype=float)
    
    cx = np.clip(p[0], x_min, x_max)
    cy = np.clip(p[1], y_min, y_max)
    cz = np.clip(p[2], z_min, z_max)
    
    c = np.array([cx, cy, cz], dtype=float)
    diff = p - c
    dist = float(np.linalg.norm(diff))
    
    if dist > zone3_outer:
        return dist, np.zeros(3, dtype=float)
    
    # Normal unit vector directed away from nearest obstacle face
    if dist > 1e-4:
        normal = diff / dist
    else:
        # Penetrated or right on face: default outward normal
        normal = np.array([0.0, 0.0, 1.0], dtype=float)
    
    if dist <= zone4_clash:
        # Inside Zone 4: Structural clash triggered by caller
        return dist, np.zeros(3, dtype=float)
    
    # Zone 3: Smooth repulsion with momentum damping
    norm_interp = (zone3_outer - dist) / (zone3_outer - zone4_clash)
    push_mag = 16.0 + 12.0 * norm_interp
    a_push = push_mag * normal
    
    # Momentum damping: brake velocity directed into the wall (-normal)
    v_into_wall = float(np.dot(v, -normal))
    if v_into_wall > 0.0:
        a_damp = 1.5 * v_into_wall * normal
    else:
        a_damp = np.zeros(3, dtype=float)
        
    return dist, a_push + a_damp


def ray_intersects_aabb(
    origin: np.ndarray,
    target: np.ndarray,
    bounds: Tuple[float, float, float, float, float, float]
) -> bool:
    """Kay-Kajiya / Smits 3D slab test for segment R(t) = origin + t*(target - origin), t in [0, 1].
    
    Returns True if the line-of-sight segment passes through or starts inside the AABB.
    """
    o = np.asarray(origin, dtype=float)
    t_end = np.asarray(target, dtype=float)
    d = t_end - o
    
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    mins = np.array([x_min, y_min, z_min], dtype=float)
    maxs = np.array([x_max, y_max, z_max], dtype=float)
    
    t_enter = -1e9
    t_exit = 1e9
    
    for i in range(3):
        if abs(d[i]) < 1e-9:
            # Ray is parallel to slab. Must be between min and max bounds.
            if o[i] < mins[i] or o[i] > maxs[i]:
                return False
        else:
            inv_d = 1.0 / d[i]
            t1 = (mins[i] - o[i]) * inv_d
            t2 = (maxs[i] - o[i]) * inv_d
            
            t_min = min(t1, t2)
            t_max = max(t1, t2)
            
            t_enter = max(t_enter, t_min)
            t_exit = min(t_exit, t_max)
            
            if t_enter > t_exit:
                return False
                
    # Check if interval [t_enter, t_exit] intersects segment t in [0, 1]
    return bool(t_exit >= 0.0 and t_enter <= 1.0 and t_enter <= t_exit)


def get_default_urban_obstacles() -> List[UrbanObstacle]:
    """Tactical urban canyon layout with dual high-rises, skybridge altitude shelf, and tower.
    Placed in the forward mission corridor (X >= 30m) to clear base launch and recovery zones."""
    return [
        UrbanObstacle(
            id="BLD-ALPHA",
            name="Building Alpha (North High-Rise)",
            bounds=(30.0, 50.0, 45.0, 75.0, 0.0, 40.0),
            blocks_rf=True,
            obstacle_type="BUILDING",
            color="#00F0FF",
        ),
        UrbanObstacle(
            id="BLD-BRAVO",
            name="Building Bravo (South High-Rise)",
            bounds=(30.0, 50.0, 5.0, 35.0, 0.0, 40.0),
            blocks_rf=True,
            obstacle_type="BUILDING",
            color="#00F0FF",
        ),
        UrbanObstacle(
            id="BRG-SKYBRIDGE",
            name="Skybridge Alpha-Bravo Deck",
            # Spans Y from 35m to 45m across the urban canyon street.
            # Altitude shelf: structural deck from Z=14m to Z=20m.
            # Free air clearance underneath: Z < 14m. Free air clearance overhead: Z > 20m.
            bounds=(36.0, 44.0, 35.0, 45.0, 14.0, 20.0),
            blocks_rf=True,
            obstacle_type="SKYBRIDGE",
            color="#FFB800",
        ),
        UrbanObstacle(
            id="TWR-CHARLIE",
            name="Tower Charlie (East Monolith)",
            # Monolith blocking direct GCS RF to eastern recon targets
            bounds=(70.0, 90.0, 20.0, 40.0, 0.0, 35.0),
            blocks_rf=True,
            obstacle_type="TOWER",
            color="#00FFAA",
            is_landable=False,
        ),
    ]


def get_cyberpunk_obby_obstacles() -> List[UrbanObstacle]:
    """Stepped terraces, elevated platforms, skyway deck, and obstacle towers ('Cyberpunk Obby').
    Provides diverse vertical landing surfaces, underpasses, and canyon routes."""
    return [
        UrbanObstacle(
            id="MESA-LOW",
            name="Terrace Mesa Low (+6m)",
            bounds=(15.0, 35.0, 10.0, 30.0, 0.0, 6.0),
            blocks_rf=True,
            obstacle_type="TERRAIN_MESA",
            color="#00F0FF",
            is_landable=True,
        ),
        UrbanObstacle(
            id="MESA-HIGH",
            name="Terrace Mesa High (+14m)",
            bounds=(45.0, 70.0, -25.0, 5.0, 0.0, 14.0),
            blocks_rf=True,
            obstacle_type="TERRAIN_MESA",
            color="#FFB800",
            is_landable=True,
        ),
        UrbanObstacle(
            id="SKYWAY-DECK",
            name="Elevated Skyway Bridge (Z: 8m-11m)",
            # Altitude shelf: deck from Z=8m to Z=11m.
            # Free air clearance underneath: Z < 8m. Free air clearance overhead: Z > 11m.
            bounds=(25.0, 55.0, 35.0, 45.0, 8.0, 11.0),
            blocks_rf=True,
            obstacle_type="SKYBRIDGE",
            color="#00FFAA",
            is_landable=True,
        ),
        UrbanObstacle(
            id="PILLAR-TOWER",
            name="East Radar Monolith (+25m)",
            bounds=(80.0, 92.0, 15.0, 27.0, 0.0, 25.0),
            blocks_rf=True,
            obstacle_type="TOWER",
            color="#FF0055",
            is_landable=False,
        ),
        UrbanObstacle(
            id="OUTPOST-ROOF",
            name="South Outpost Deck (+4m)",
            bounds=(0.0, 20.0, -40.0, -20.0, 0.0, 4.0),
            blocks_rf=True,
            obstacle_type="PLATFORM",
            color="#00F0FF",
            is_landable=True,
        ),
    ]
