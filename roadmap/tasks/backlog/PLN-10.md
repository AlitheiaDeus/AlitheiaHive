# [PLN-10] Adversarial Targets, Emitter Triangulation & Maze Exploration

* **Grade**: 🏗️ Epic | **Type**: Tactical Intercept, Electronic Warfare & Autonomous Maze Exploration (`Backlog / Epic`) | **Priority**: Medium
* **Target Files**: `SIM/backend/sitl_engine.py`, `SIM/backend/adversary_engine.py`, `OBC/backend/brains/`, `GCS/frontend/js/app.js`
* **Operational Inspiration**:
  - Implements dynamic adversarial targets (rogue drones, mobile radio jammers, evasive surface probes) for the swarm to detect, track, and intercept.
  - Gives practical execution to `K1-EW Bloodhound` (3-antenna patch array) for passive RF Direction Finding (DF) and radio emitter triangulation.
  - Introduces autonomous maze/frontier exploration: when communication links drop inside complex subterranean or urban corridors, the swarm autonomously maps branches and coordinates searches.
* **Architecture & Airgap Contracts**:
  1. **SIM (World Engine / Bubble 3 - Ground Truth)**:
     - Spawns mobile adversarial entities running evasive waypoint kinematics and active RF jamming emitters.
     - Computes physical electromagnetic signal propagation, AoA angles, and Doppler shifts relative to drone antenna arrays.
  2. **The Canyon Link (Airgap 2)**:
     - GCS receives zero omniscient ground truth coordinates for adversaries; only downlinked telemetry detections from fleet sensors cross this boundary.
  3. **OBC (Brain Airgap / Bubble 2 - Autonomous Compute)**:
     - `K1-EW Bloodhound` firmware calculates phase-interferometric AoA vectors across its 3 antennas, fusing multi-sample bearings to triangulate emitter coordinates.
     - Frontier exploration state machine: upon entering unmapped or link-denied maze corridors, lead airframe transitions to autonomous branch exploration, caching discovered route breadcrumbs locally.
  4. **GCS (Operator Airgap / Bubble 1 - Viewport)**:
     - Displays dynamic red adversary blips with bearing uncertainty cones, updating strictly when fleet telemetry downlinks confirm sensor triangulation.
