# [PLN-08] Urban Canyon Geometry, Static 3D Obstacles & Safe Flight Corridors

* **Grade**: 🏗️ Epic | **Type**: Tactical Environment, 3D Colliders & Flight Corridors (`Backlog / Epic`) | **Priority**: Medium
* **Target Files**: `SIM/backend/sitl_engine.py`, `SIM/backend/urban_geometry.py`, `OBC/backend/drone_firmware.py`, `GCS/frontend/js/app.js`, `GCS/frontend/js/viewport.js`
* **Operational Inspiration**:
  - Replaces barren flat-earth testing with realistic 3D urban canyons, bridges, tunnels, communication towers, and rooftops.
  - **3D Altitude Shelves (Flying Under and Over)**: Avoids voxel bloat by defining obstacles as 3D Axis-Aligned Bounding Boxes (AABBs) with explicit floor and ceiling brackets `[x_min, x_max, y_min, y_max, z_floor, z_ceiling]`. Drones can fly cleanly underneath a bridge at $Z=6\text{m}$, over it at $Z=22\text{m}$, or land on a rooftop.
  - **Corridor Navigation vs. Drone-on-Drone Sensor Conflict**:
    - *The Problem*: If every drone runs naive 360° laser ranging, wingmen crossing paths in a tight formation trigger false-positive obstacle alarms, fighting swarm formation keeping.
    - *The Architecture Solution*: GCS pre-screens the terrain and uplinks **Safe 3D Corridor Tubes** (`[Waypoint A, Waypoint B, Tube Radius R]`) along with local obstacle bounding edges. Drones steer smoothly through the corridor.
    - Inter-drone separation remains handled by the battle-tested **Heartbeat 1–3 gossip mesh**, so drones never confuse a fellow airframe with a concrete wall.
  - Physical Line-of-Sight (LOS) RF shadowing: flying behind steel or concrete structures creates authentic RF Black Zones, prompting autonomous breadcrumb backtracks (`[TKT-54]`) or Search-and-Recall mules (`[PLN-02]`).

* **Architecture & Airgap Contracts**:
  1. **SIM (World Engine / Bubble 3 - Ground Truth)**:
     - Maintains static 3D AABB collision geometry `[min_x, max_x, min_y, max_y, z_floor, z_ceiling]`.
     - Zone 4 structural clash ($d \le 1.5\text{m}$ to obstacle surface within altitude brackets): cuts motor power (`crashed = True`), zeroes RPM, and initiates ballistic descent.
     - Ray-cast RF occlusion: packets traversing building volumes suffer 100% loss (RF Black Zone) independent of free-space distance attenuation.
     - Lightweight local proximity bumper: passes relative distance vector `[dx, dy, dz]` to OBC *strictly for static environmental obstacles* when within $3.5\text{m}$.
  2. **The Canyon Link (Airgap 2)**:
     - Downlinks scenario obstacle bounds during session hydration; live radio packets are dropped in transit if LOS ray intersects obstacle bounds.
  3. **OBC (Brain Airgap / Bubble 2 - Autonomous Compute)**:
     - Follows GCS-commanded corridor tube and formation offsets.
     - If local proximity bumper reports an environmental wall within $3.0\text{m}$, applies smooth Zone 3 repulsive evasion vector $\mathbf{a}_{\text{evade}} \propto -\mathbf{d}_{\text{wall}} / \|\mathbf{d}_{\text{wall}}\|^3$ to slide clear of corners without disrupting mesh consensus.
     - Followers dynamically squeeze formation width (e.g., Vee $\to$ Column Astern) when corridor width pinches below formation wingspan.
  4. **GCS (Operator Airgap / Bubble 1 - Viewport)**:
     - Three.js renders semi-transparent glowing wireframe buildings, altitude shelves, and red hatched "RF Blackout / Jammer" threat zones.
