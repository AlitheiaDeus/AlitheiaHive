# [TKT-57] Airframe Tier Registry, Tether Prefixes & Mission Authorization Gate

* **Grade**: 🚀 Feature | **Type**: Fleet Governance & Mission Authorization (`Completed / Settled`) | **Priority**: Medium
* **Target Files**: `OBC/backend/drone_firmware.py`, `SIM/backend/sitl_engine.py`, `GCS/backend/c2_gateway.py`, `GCS/frontend/js/hud.js`, `tests/test_tier_registry_auth.py`
* **Operational Inspiration**:
  - Homogeneous default fleet: all 8 drones share identical baseline hardware (`T1.5 Beta`), eliminating fragile non-promotable drone bottlenecks.
  - Tether composition: decouples airframe tier from umbilical type using explicit prefixes (`T-` for Power+Data umbilical, `t-` for Data-only micro-fiber, none for wireless).
  - Dynamic RF modes: `Yip` (50m low-power escort whisper) vs `Bork` (65m amplified PA leader boost).
  - C2 Pre-Flight Authorization Gate: ensures missions requiring specialist gear (e.g. `HEAVY_PAYLOAD`, `LIDAR_SCAN`) verify capability tags before dispatch, returning deterministic error payloads.
* **Technical Blueprint**:
  - **Part 1: Airframe Tier & Tether Prefix Taxonomy (OBC Firmware & SIM)**:
    - Base tiers: `T0` (Anchor), `T1.5` (Standard Tactical Multi-Role), `T3` (Heavy Specialist).
    - Tether modifiers: `t-` (Data-only fiber, internal LiPo), `T-` (Power + Data, ground umbilical), or None.
    - Telemetry tier designation: `full_tier_designation` (e.g. `t-T1.5` for tethered flagship, `T1.5` for wireless).
    - Comms power state: `rf_power_mode` in `["YIP", "BORK", "HOWL"]`. Drone switches to `BORK` when promoted/leading.
    - Capabilities tags: `capabilities: List[str]` on each `Drone` (e.g. `["FORMATION_FLIGHT", "PATROL", "RECON", "S&R_OPTICAL", "BORK_AMPLIFY"]`).
  - **Part 2: C2 Gateway Mission Authorization Pre-Flight Gate**:
    - Add `check_mission_authorization(mission_type, required_capabilities=None) -> Dict[str, Any]` to `c2_gateway.py`.
    - If fleet lacks requested capability, reject with `TIER_AUTHORIZATION_FAILED` and list missing capabilities.
    - If authorized, select eligible airframe and approve dispatch.
  - **Part 3: GCS Tactical Viewport & Card Badges**:
    - Display tier badge (`[T1.5]`, `[t-T1.5]`, `[T3]`) and comms state (`YIP`/`BORK`) on drone HUD cards.
* **Verification Gates**:
  - Dedicated unit test suite in `tests/test_tier_registry_auth.py`.
  - Zero regression across existing test suites (`test_sim_core.py`, `test_waypoint_engine.py`) and smoke test.
