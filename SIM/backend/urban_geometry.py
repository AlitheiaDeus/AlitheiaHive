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
    obstacle_type: str = "BUILDING"  # "BUILDING", "SKYBRIDGE", "TOWER"
    color: str = "#00F0FF"

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
        ),
    ]
