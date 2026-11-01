# [PLN-09] T3 Kodiak Simulated LiDAR Stream & Point-Cloud HUD Overlay

* **Grade**: 🏗️ Epic | **Type**: Sensor Simulation, Edge Compute & WebGL Point Cloud (`Backlog / Epic`) | **Priority**: Medium
* **Target Files**: `SIM/backend/sitl_engine.py`, `SIM/backend/lidar_sim.py`, `OBC/backend/drone_firmware.py`, `GCS/frontend/js/app.js`, `GCS/frontend/js/lidar_overlay.js`
* **Operational Inspiration**:
  - Delivers the signature sensor payload for `T3 Kodiak` (Heavy Edge Compute Hub) defined in `AIRFRAME_HARDWARE_TIERS.md`.
  - Simulates planar and multi-beam discrete range returns from on-airframe spinning LiDAR (Velodyne/Livox style).
  - Streams compact range arrays to the WebGL GCS, projecting a real-time neon cyberpunk point cloud directly in the Three.js viewport.
* **Architecture & Airgap Contracts**:
  1. **SIM (World Engine / Bubble 3 - Ground Truth)**:
     - Casts 16 to 32 discrete synthetic range rays centered on active `T3 Kodiak` airframes against ground truth obstacle geometry and terrain.
     - Injects Gaussian measurement noise ($\sigma = 0.05\text{m}$) and maximum range cutoff ($R_{\text{max}} = 30.0\text{m}$).
  2. **The Canyon Link (Airgap 2)**:
     - Packs polar range-bearing tuples `[(range_m, azimuth_deg, elevation_deg), ...]` into compact binary/JSON downlink buffers over UDP 9000.
     - Strictly rate-limited to 10 Hz to prevent network congestion across the airgap.
  3. **OBC (Brain Airgap / Bubble 2 - Autonomous Compute)**:
     - `T3 Kodiak` onboard processor (Jetson Orin twin) consumes raw distance returns to calculate local obstacle avoidance gradients.
     - Broadcasts perimeter hazard alert packets to peer escorts over the `Bark` swarm mesh ($80\text{m}$), allowing dumb escorts to evade obstacles without their own LiDAR.
  4. **GCS (Operator Airgap / Bubble 1 - Viewport)**:
     - Consumes downlinked range arrays and renders a real-time Three.js neon point cloud using `THREE.BufferGeometry` with intensity-based elevation color ramps.
