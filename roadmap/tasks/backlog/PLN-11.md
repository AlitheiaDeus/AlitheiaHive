# [PLN-11] 3D Tactical Corridor Pathfinding & Operator Preview ("Plan -> Verify -> Send")

* **Grade**: 🏗️ Epic | **Type**: Autonomous 3D Path Planning & Tactical C2 (`Backlog / Epic`) | **Priority**: Medium
* **Target Files**: `GCS/backend/c2_gateway.py`, `GCS/backend/pathfinder_3d.py`, `GCS/frontend/js/app.js`, `GCS/frontend/js/viewport.js`, `OBC/backend/drone_firmware.py`
* **Operational Inspiration**:
  - Operators guiding individual airframes or small recon elements through dense 3D urban canyons currently rely on direct line-of-sight vector commands or manually placed waypoints.
  - In complex terrain with towering skyscrapers, skybridges, and altitude shelves, manual single-point steering risks colliding with walls or dead-ends unless the operator manually plots a multi-turn route around every building.
  - **The "Plan -> Verify -> Send" Operator Workflow**:
    1. **Plan**: Operator selects a target destination on the map (or right-clicks a location) and selects "PLAN ROUTE" for a selected drone (e.g. Drone 0 or an aux mule).
    2. **Algorithmic Pathfinding**: The GCS pathfinding engine computes a collision-free 3D trajectory (using 3D A*, Jump Point Search, or Visibility Graph over `UrbanObstacle` AABBs) maintaining safe wall clearance ($\ge 4.0\text{m}$) and optimal altitude transitions (e.g. climbing over a low roof or threading through a street canyon).
    3. **Verify (Interactive Visual Preview)**: Before any flight command is uplinked to the airframe, the 3D viewport renders a preview corridor:
       - Neon dashed trajectory line with turn-point nodes.
       - Transparent cylindrical corridor boundary envelope (safe tube radius $R=3.0\text{m}$).
       - Clearance telemetry readout in HUD (Estimated Flight Time, Total Distance, Minimum Clearance Margin, Battery Cost Estimate).
    4. **Send (Operator Execution)**: If the operator approves the route, clicking "SEND ROUTE" uploads the discrete waypoint queue and corridor parameters to the drone's OBC buffer, which then begins autonomous execution. The operator retains a "CANCEL" / "ABORT" button at all times.

* **Architecture & Airgap Contracts**:
  1. **GCS (Operator Airgap / Bubble 1 - Path Planning Engine)**:
     - Pure computational path planning runs on GCS backend or frontend without touching SITL ground truth. It queries the known public urban geometry database.
     - Generates piecewise-linear 3D waypoints with altitude brackets and velocity profiles.
     - Three.js viewport renders the candidate path as an editable neon preview spline prior to execution.
  2. **The Canyon Link (Airgap 2 - Serialized Uplink)**:
     - Uplinks path as standard MAVLink `MISSION_ITEM_INT` sequence or serialized C2 packet `COMMAND_UPLOAD_CORRIDOR_WAYPOINTS`.
  3. **OBC (Brain Airgap / Bubble 2 - Autonomous Tracking)**:
     - Drones receive the corridor waypoints into their local `waypoint_queue` buffer.
     - Onboard guidance tracks waypoints using existing discrete waypoint sequencing (`TKT-56`) with lateral corridor bounds enforcement.
  4. **SIM (World Engine / Bubble 3 - Verification)**:
     - Kinematic engine evaluates actual trajectory against physics limits, acceleration constraints, and static obstacle clashes.
