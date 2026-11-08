# [TKT-36] Interactive 3D Corridor Canvas & Scenario Studio (Plan Mode)

* **Grade**: 🚀 Feature | **Type**: Mission Planning & Scenario Studio (`Backlog`) | **Priority**: Low
* **Target Files**: `GCS/frontend/studio.html`, `GCS/frontend/js/studio.js`, `GCS/frontend/js/app.js`, `GCS/frontend/js/hud.js`
* **Technical Blueprint**:
  - **The Studio Mode (Plan View)**: Dedicated mission designer canvas toggled via top bar or `/studio` route, sharing Three.js assets without launching a new backend daemon.
  - **3D Flight Corridors & Altitude Shelves**:
    - Interactive 3D polyline corridor drawing with altitude shelves ($Z_{\text{floor}}, Z_{\text{ceiling}}$), Dubins path turn smoothing, and tube width boundaries.
    - Resolves drone-on-drone sensor clashing by uplinking pre-cleared corridor pipes.
  - **Mapspace Stamping Palette**:
    - Click-and-stamp placement of 3D building AABBs, cylindrical EW Jammer bubbles, rooftop recon vantage points, and solar repeater dropzones.
  - **Export & Execution**:
    - One-click `[SAVE SCENARIO]` exporting to declarative `scenarios/*.json` schema (`[PLN-03]`).
    - One-click `[LAUNCH MISSION]` switching directly to Tactical Operator mode (`FLY`) with the active scenario hydrated in SITL.
* **Targeted Test**: Visual WebGL inspection & scenario JSON export validation.
