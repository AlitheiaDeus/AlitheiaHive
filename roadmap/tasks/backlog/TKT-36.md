# [TKT-36] Interactive 3D Route Planner & Geofence Corridor Canvas

* **Grade**: ðŸš€ Feature | **Type**: Feature / Mission Planning (`Backlog`) | **Priority**: Low
* **Target Files**: `frontend/js/app.js`, `frontend/js/hud.js`
* **Technical Blueprint**:
  - Settles PLN-13.
  - Interactive 3D polyline corridor drawing with altitude shelves, Dubins path turn smoothing, and automated survey grids.
  - 3D cylindrical and polygonal keep-out threat volumes (No-Fly Zones) with red holographic visual fencing.
  - Export custom drawn routes directly to discrete waypoint queue format (`UPLOAD_MISSION_ROUTE`).
* **Targeted Test**: Visual WebGL inspection.
