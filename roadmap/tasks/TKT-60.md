# TKT-60: Urban 3D Obstacle Shelves, Structural Clash & RF LOS Shadowing

> **Category**: 🚀 Feature | **Priority**: High | **State**: in_progress  
> **Operational Inspiration**: PLN-08 Urban Canyon Geometry, Static 3D Obstacles & Safe Flight Corridors

---

## 1. Operational Overview & Objectives

Replaces flat-earth simulation with realistic 3D urban geometry (skyscrapers, elevated skybridges, and communications towers). Avoids voxel memory bloat by modeling static geometry as 3D Axis-Aligned Bounding Boxes (AABBs) with explicit floor and ceiling brackets `[x_min, x_max, y_min, y_max, z_floor, z_ceiling]`.

Key operational capabilities:
1. **3D Altitude Shelves (Flying Under and Over)**: Drones can fly underneath an elevated skybridge at $Z=8\text{m}$, fly over it at $Z=25\text{m}$, or clash into its structural deck between $Z=15\text{m}$ and $Z=22\text{m}$.
2. **Zone 4 Structural Clash ($d \le 1.5\text{m}$)**: Penetrating the $1.5\text{m}$ obstacle envelope cuts motor power (`crashed = True`, `rpm = 0.0`, ballistic fall) to enforce real collision consequences.
3. **Ray-Cast RF Line-of-Sight (LOS) Shadowing**: When the direct line-of-sight between GCS and drone (or drone and drone) intersects a building volume, the RF link is occluded (`rf_blocked = True`), creating authentic urban dead zones that trigger autonomous lost-link breadcrumb backtracking (`[TKT-54]`) or Search-and-Recall mules (`[TKT-51]`).
4. **Zone 3 Proximity Repulsion ($1.5 < d \le 3.5\text{m}$)**: Proximity bumper vectors allow autonomous guidance to slide smoothly past building corners rather than flying head-on into walls.
5. **Cyberpunk Three.js Viewport**: Glowing neon wireframe buildings and skybridges rendered natively on GPU with altitude labels and LOS occlusion indicators.

---

## 2. Target Files & Architectural Scope

* `SIM/backend/urban_geometry.py`: 3D AABB dataclass, exact point-to-box distance, normal repulsion vector, and fast Liang-Barsky / Kay-Kajiya ray-AABB slab intersection test.
* `SIM/backend/sitl_engine.py`: Obstacle registry, 3D structural clash detection loop in `step()`, ray-cast RF LOS shadowing in `_update_radio_comms()`, and scenario hydration.
* `OBC/backend/drone_firmware.py`: Proximity bumper integration & Zone 3 wall-sliding repulsion acceleration.
* `GCS/backend/c2_gateway.py` & `GCS/backend/web_server.py`: Telemetry serialization of active urban obstacles and command dispatch.
* `GCS/frontend/js/app.js`: Three.js rendering of holographic wireframe buildings, altitude shelves, and LOS occlusion visual feedback.
* `tests/test_urban_geometry_rf_shadow.py`: Comprehensive test module covering distance calculations, ray-AABB intersections, altitude shelves, structural clash, and RF shadowing.

---

## 3. Implementation Blueprint

### A. 3D AABB Math (`SIM/backend/urban_geometry.py`)
- `UrbanObstacle`:
  - `id: str`, `name: str`
  - `bounds: tuple[float, float, float, float, float, float]` -> `(x_min, x_max, y_min, y_max, z_floor, z_ceiling)`
  - `blocks_rf: bool = True`
  - `obstacle_type: str = "BUILDING"` (`BUILDING`, `SKYBRIDGE`, `TOWER`)
  - `color: str = "#00F0FF"`
- `point_to_aabb_distance(p: np.ndarray, b: tuple) -> float`:
  - Clamps point to box bounds: $c_i = \text{clamp}(p_i, \min_i, \max_i)$.
  - Outside box: $d = \|\mathbf{p} - \mathbf{c}\|_2$.
  - Inside box: $d = 0.0$ (penetration).
- `ray_intersects_aabb(origin: np.ndarray, target: np.ndarray, b: tuple) -> bool`:
  - Kay-Kajiya slab test for segment $\mathbf{R}(t) = \mathbf{o} + t(\mathbf{d} - \mathbf{o})$, $t \in [0, 1]$.
  - Fast axis-by-axis interval intersection $[t_{\min}, t_{\max}]$. Returns True if $t_{\max} \ge \max(t_{\min}, 0.0)$ and $t_{\min} \le 1.0$.

### B. SITL Collision & RF Shadowing (`SIM/backend/sitl_engine.py`)
- Add `self.urban_obstacles: list[UrbanObstacle] = []` and load `default_canyon` layout:
  - Building Alpha: `[-30, -10, 20, 50, 0, 40]`
  - Building Bravo: `[-30, -10, -50, -20, 0, 40]`
  - Skybridge: `[-25, -15, -20, 20, 15, 22]` (Clearance underneath $0 \le Z < 15\text{m}$, deck $15 \le Z \le 22\text{m}$, clearance above $Z > 22\text{m}$)
  - Tower Charlie: `[25, 45, 10, 30, 0, 35]`
- Collision check in `step()`:
  - If drone pos within $1.5\text{m}$ of any obstacle volume: `drone.crashed = True`, `drone.rpm = 0.0`, `drone.status = "CRASHED_OBSTACLE"`.
- RF occlusion check in `_update_radio_comms()`:
  - Test ray from transmitter to receiver against all active obstacles with `blocks_rf=True`. If intersection detected, sever RF link.

### C. GCS Viewport Rendering (`GCS/frontend/js/app.js`)
- Holographic Three.js meshes with wireframe edges and translucent faces.
- Skybridge rendered in warm amber `#FFB800` with visible clearance corridor indicators.

---

## 4. Acceptance Criteria & Verification Invariants

* [ ] `point_to_aabb_distance` returns $0.0$ inside box and exact Euclidean distance outside.
* [ ] `ray_intersects_aabb` returns True for penetrating rays and False for clear line-of-sight paths.
* [ ] Altitude shelf test passes: Drone flying at $Z=8\text{m}$ passes cleanly under Skybridge without clash; drone at $Z=18\text{m}$ clashes with deck structure ($d \le 1.5\text{m}$); drone at $Z=25\text{m}$ flies cleanly over Skybridge.
* [ ] Zone 4 structural clash cuts motor RPM to 0 and flags `crashed = True`.
* [ ] Ray-cast RF shadow severs telemetry/mesh packets when transmitter and receiver are occluded by Building Alpha or Tower Charlie.
* [ ] GCS hydration and live telemetry frames serialize active urban obstacle geometries.
* [ ] Targeted test module passes: `python -m unittest tests/test_urban_geometry_rf_shadow.py`.
