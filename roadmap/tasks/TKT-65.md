# TKT-65: Modular 3D Terrain Blocks & Obstacle Shelves ("Obby")

> **Category**: 🚀 Feature | **Priority**: High | **State**: in_progress  
> **Operational Inspiration**: PLN-08 Cyberpunk Obby Steppes, Altitude Shelves & Dynamic Surface Floor Clamping (50/50 Architecture across 4 Airgaps)

---

## 1. Operational Overview & Objectives

Establishes a solid, predictable spatial terrain foundation based on **Modular 3D AABB Obstacle Shelves and Steppes ("Cyberpunk Obby")**:

1. **Eliminates Continuous Heightmap Traps**:
   - Replaces risky continuous 2D heightmaps ($Z = f(X, Y)$) with deterministic, discrete 3D AABB blocks.
   - Prevents WebGL-Python floating/buried drone rendering desyncs and avoids continuous slope-landing tilt/slide physics.
2. **Altitude Shelves with Dynamic Surface Elevation**:
   - Drones can fly cleanly underneath elevated bridges/shelves ($Z < z_{\text{floor}}$), navigate through urban canyons, or land on elevated rooftops/mesas ($Z = z_{\text{ceiling}} + 0.05\text{m}$).
   - Dynamic floor clamping: Evaluates local surface elevation $Z_{\text{surface}}(X, Y, Z)$ below the airframe. The floor safety limit scales dynamically ($Z \ge Z_{\text{surface}} + 0.5\text{m}$ in flight, $Z_{\text{surface}} + 0.05\text{m}$ when landing/landed).
3. **Walkable / Landable Platforms**:
   - Landable top surfaces allow designated Field Landing Zones (FLZ) or autonomous E-landings on flat mesas and rooftops without false-positive Zone 4 structural clash.
   - Side walls and undersides remain strictly enforced Zone 4 clash surfaces ($d \le 1.5\text{m}$ triggers crash).
4. **Preset Tactical Environments**:
   - `default_canyon`: Dual high-rises, Alpha-Bravo skybridge altitude shelf, and East monolith.
   - `cyberpunk_obby`: Stepped terraces (Low Mesa $+6\text{m}$, High Mesa $+15\text{m}$), elevated skyway bridge ($Z=8\text{m}$ to $11\text{m}$), and central flight trench.
   - Dynamic scenario loading via C2 command `LOAD_URBAN_SCENARIO`.
5. **Three.js WebGL Tactical Visualizer**:
   - Renders semi-transparent volumetric blocks with neon edge wireframes and elevation markers (e.g., `[MESA +6.0m]`, `[DECK +11.0m]`).

---

## 2. Target Files & Architectural Scope

* `SIM/backend/urban_geometry.py`:
  - `UrbanObstacle` extensions (`is_landable`, `obstacle_type`, `surface_padding`).
  - `get_surface_elevation(pos, obstacles)`: Computes local ground/platform height directly beneath a 3D coordinate.
  - `get_overhead_ceiling(pos, obstacles)`: Computes lowest overhead ceiling for altitude shelf flight.
  - Enhanced `point_to_aabb_repulsion` and collision checks with surface landing exemption.
  - Scenario definitions (`cyberpunk_obby`, `default_canyon`).
* `SIM/backend/sitl_engine.py`:
  - Dynamic surface floor clamping replacing hardcoded 0.05m / 0.5m floor limits.
  - Structural clash detection distinguishing side wall impact from top-face touchdown.
  - `load_urban_scenario()` C2 dispatcher and telemetry hydration.
* `SIM/backend/atc_recovery.py`:
  - `find_safe_flat_space()` updated to evaluate rooftop/mesa landing opportunities.
* `GCS/backend/c2_gateway.py` & `GCS/backend/web_server.py`:
  - Route C2 `LOAD_URBAN_SCENARIO` / `SET_URBAN_SCENARIO` commands.
* `GCS/frontend/js/app.js`:
  - Three.js rendering of landable top faces, elevation labels, and scenario presets.
* `tests/test_terrain_shelves.py`:
  - Comprehensive unit test suite validating mesa landings, fly-under clearances, dynamic floor clamping, and wall crash enforcement.

---

## 3. Implementation Blueprint

### A. Dynamic Surface Elevation Model
```python
def get_surface_elevation(pos: np.ndarray, obstacles: List[UrbanObstacle]) -> float:
    """Find the highest solid surface directly below pos (x, y, z).
    Returns 0.0 (ground level) if over open terrain."""
    x, y, z = pos[0], pos[1], pos[2]
    highest_surface = 0.0
    for obs in obstacles:
        if obs.x_min <= x <= obs.x_max and obs.y_min <= y <= obs.y_max:
            # If drone is at or above ceiling (with 0.2m tolerance), this ceiling is a floor
            if z >= obs.z_ceiling - 0.2:
                if obs.z_ceiling > highest_surface:
                    highest_surface = obs.z_ceiling
    return highest_surface
```

### B. Altitude Shelf Separation
* If $z < z_{\text{floor}}$: Overhead clearance envelope is $z_{\text{floor}} - z$. Drone flies underneath.
* If $z > z_{\text{ceiling}}$: Surface floor is $z_{\text{ceiling}}$. Drone flies over or lands on top.
* If $z_{\text{floor}} \le z \le z_{\text{ceiling}}$: Drone is within obstacle altitude bracket. Approaching within $1.5\text{m}$ of side walls triggers Zone 4 structural clash.

---

## 4. Acceptance Criteria & Invariants

* [x] `get_surface_elevation()` accurately returns 0.0 on open ground, $z_{\text{ceiling}}$ over mesas/rooftops, and accounts for multi-shelf stacking.
* [x] Drones fly cleanly underneath elevated skybridges ($Z < z_{\text{floor}}$) without floor/ceiling clamping errors.
* [x] Drones can execute nominal and emergency landings on elevated mesas ($Z = z_{\text{ceiling}} + 0.05\text{m}$) without triggering structural clash.
* [x] Flying horizontally into mesa/building side walls ($d \le 1.5\text{m}$) strictly triggers `STRUCTURAL_CLASH`.
* [x] Dynamic floor clamp enforces $Z \ge Z_{\text{surface}} + 0.5\text{m}$ in flight and $Z_{\text{surface}} + 0.05\text{m}$ when landed.
* [x] `LOAD_URBAN_SCENARIO` loads `default_canyon`, `cyberpunk_obby`, and `empty` dynamically over C2/telemetry.
* [x] Three.js viewport renders landable platforms with elevation tags.
* [x] Targeted unit tests in `tests/test_terrain_shelves.py` pass 100%.
