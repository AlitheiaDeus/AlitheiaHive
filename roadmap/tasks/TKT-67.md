# TKT-67: Render 3D Urban Obstacles & Elevation Shelves in God-View

> **Category**: TKT | **Priority**: High | **State**: completed  
> **Operational Inspiration**: God-View /sim missing 3D obstacle meshes causing drones to clash and slide against invisible buildings

---

## 1. Operational Overview & Objectives
In the God-View omniscient simulation lab (`/sim`), drones appeared to slide along or clash into invisible structures because while `GCS/frontend/js/app.js` rendered the 3D urban obstacles and altitude shelves from `urban_geometry.py`, `SIM/frontend/js/sim_app.js` lacked the obstacle rendering logic. Even though `frame.urban_obstacles` was transmitted in every telemetry broadcast, `/sim` discarded it.

This ticket implements full 3D rendering of urban obstacles in God-View:
* Translucent shaded building hulls with cyber aesthetic colors.
* Holographic wireframe edge geometry for crisp structural definition.
* Landing deck roof highlights for landable rooftop blocks.
* Tactical elevation billboard labels (e.g. `Alpha Tower [+45.0m]` or `Skybridge [Z:18.0-22.0m]`).
* Memory-leak-free dynamic lifecycle management and WebGL resource disposal.

## 2. Target Files & Architectural Scope
* `SIM/frontend/js/sim_app.js`: Implement `urbanObstacleMeshes` map, `updateUrbanObstacles(obstacles)` rendering routine, texture/geometry disposal, and wire into `onTelemetryFrame`.

## 3. Implementation Blueprint
* Defined `urbanObstacleMeshes = new Map()` in `SIM/frontend/js/sim_app.js`.
* Created `updateUrbanObstacles(obstacles)` matching the visual language of the GCS 3D viewport:
  - 3D Box geometry positioned at the AABB centroid `[cx, cy, cz]` with width, depth, and height from `bounds`.
  - Translucent `MeshStandardMaterial` body with emissive glow.
  - `LineSegments` edges via `EdgesGeometry`.
  - Landable rooftop plane highlighting with opacity and side rendering.
  - Dynamic 2D canvas sprite for elevation billboard labeling (`name [Z:...]`).
  - Active ID tracking and WebGL cleanup/disposal for purged obstacles.
* Connected `updateUrbanObstacles(frame.urban_obstacles)` into `onTelemetryFrame`.

## 4. Acceptance Criteria & Verification Invariants
* [x] God-View `/sim` renders all active 3D urban obstacles broadcasted in `frame.urban_obstacles`.
* [x] Buildings render translucent bodies and holographic edges matching architectural dimensions.
* [x] Rooftop landing decks render for `is_landable: true` blocks.
* [x] Elevation billboard sprites accurately show obstacle names and max altitudes.
* [x] Dynamic additions and removals properly dispose of Three.js geometries, textures, and materials.
