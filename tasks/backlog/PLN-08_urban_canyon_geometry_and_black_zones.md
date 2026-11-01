# [PLN-08] Urban Canyon Geometry, Static Obstacles & RF Black Zones

* **Grade**: 🏗️ Epic | **Type**: Tactical Environment, 3D Colliders & RF Shadowing (`Backlog / Epic`) | **Priority**: Medium
* **Target Files**: `SIM/backend/sitl_engine.py`, `SIM/backend/urban_geometry.py`, `OBC/backend/brains/`, `GCS/frontend/js/app.js`, `GCS/frontend/js/viewport.js`
* **Operational Inspiration**:
  - Replaces barren flat-earth testing with realistic urban canyon corridors (skyscrapers, communication towers, industrial structures).
  - Enforces physical line-of-sight (LOS) RF shadowing: flying behind concrete or steel structures breaks Howl/Bark/Yip radio propagation, creating authentic RF Black Zones.
  - Gives practical tactical purpose to `[TKT-54]` Breadcrumb Backtracking: drones trapped in building shadows must autonomously retrace breadcrumbs out of the darkness.
* **Architecture & Airgap Contracts**:
  1. **SIM (World Engine / Bubble 3 - Ground Truth)**:
     - Implements 3D Axis-Aligned Bounding Box (AABB) and oriented prism colliders for structural obstacles `[min_x, max_x, min_y, max_y, 0, height]`.
     - Zone 4 structural clash ($d \le 1.5\text{m}$ to obstacle boundary): cuts motor power (`crashed = True`), zeroes RPM, initiates ballistic gravity descent.
     - Line-of-sight (LOS) raycast attenuation: building hulls cast ray-occlusion shadows. Packets crossing building volumes suffer 100% loss (Black Zone) independent of distance path loss.
  2. **The Canyon Link (Airgap 2)**:
     - Downlinks obstacle footprint definitions during session hydration; live radio packets are dropped in transit if LOS ray intersects obstacle bounds.
  3. **OBC (Brain Airgap / Bubble 2 - Autonomous Compute)**:
     - Proximity repulsion is derived strictly from simulated onboard sensor horizons (sonar / optical flow / LiDAR), never reading SIM geometry directly.
     - Autonomous corridor navigation: followers dynamically compress formation slot spacing ($R \to R / 2$) when entering narrow urban channels.
  4. **GCS (Operator Airgap / Bubble 1 - Viewport)**:
     - Three.js renders building wireframes/solids and hatched red volumetric "Black Zone / EW Denial" prisms.
