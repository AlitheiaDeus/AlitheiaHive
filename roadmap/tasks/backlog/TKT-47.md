# [TKT-47] Declarative Field Deployment Scenarios & Decoupled Subsystem Layouts (SIM/scenarios/)

* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / Field Architecture (`Backlog`) | **Priority**: Medium
* **Target Files**: `SIM/backend/sitl_engine.py`, `SIM/scenarios/`, `GCS/frontend/js/app.js`, `GCS/backend/c2_gateway.py`
* **Technical Blueprint**:
  - **The Mobile Field Reality**: Decouple the monolithic origin $(0, 0, 0)$ into independent physical subsystem locations defined in declarative JSON scenarios under `SIM/scenarios/`:
    1. **Geographic Home Datum**: `datum: { "lat": 35.625971, "lon": 139.734759, "alt": 20.0 }` (Tokyo Bay default).
    2. **GCS Radio Mast**: `gcs_mast: [x, y, z]` (operator station / high-gain telemetry antenna).
    3. **RTK Base Tripod**: `rtk_tripod: [x, y, z]` (independent surveyed antenna offset broadcasting RTCM corrections).
    4. **Tether Winch / Spool**: `tether_winch: [x, y, z]` (independent umbilical winch anchor, freeing recovery pads from cable snag hazard).
    5. **Relocatable Landing Zone (LZ)**: `landing_zone: { "center": [x, y, z], "heading_deg": 45.0, "pad_spacing_m": 3.0, "pad_count": 8 }`. Pads generate dynamically relative to LZ center and orientation (e.g. roll-out mat 25m East of Base or vehicle trailer).
  - **Declarative Presets**:
    - `SIM/scenarios/tokyo_waterfront.json`: Open coastal wharf setting.
    - `SIM/scenarios/mobile_van_deployment.json`: Pads clustered on mobile vehicle trailer, tether off rear bumper, GCS mast on roof rack.
    - `SIM/scenarios/rooftop_urban.json`: Tight landing grid on elevated building roof.
  - **Telemetry Downlink Discovery**:
    - SIM parses scenario layout on boot and exposes `layout` in `get_state_snapshot()`.
    - Three.js tactical viewport (`app.js`) automatically draws pads, winch, tripod, and GCS mast at their true scenario positionsâ€”zero manual UI dragging needed.
    - RTB recovery logic targets scenario-defined pad locations.
  - **Stage 2 Field Auto-Detection**:
    - When `TKT-41` initializes drones cold on the ground, grounded drone initial coordinates dynamically verify and auto-populate the pad layout.
* **Targeted Test**: `python -m unittest tests/test_sim_core.py`
