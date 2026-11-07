# [TKT-56] Discrete 4D Waypoint Queue & Loiter Progress Engine

* **Grade**: 🚀 Feature | **Type**: Mission Guidance & Canyon Protocol (`Completed`) | **Priority**: High
* **Target Files**: `GCS/backend/c2_gateway.py`, `OBC/backend/drone_firmware.py`, `OBC/backend/brains/leader_follower.py`, `OBC/backend/brains/gossip_consensus.py`, `SIM/backend/sitl_engine.py`, `GCS/frontend/index.html`, `GCS/frontend/js/hud.js`, `GCS/frontend/js/app.js`, `tests/test_waypoint_engine.py`
* **Verification Status**:
  - `tests/test_waypoint_engine.py`: 8/8 tests passed.
  - `tests/test_mission_routes.py`: 12/12 tests passed (zero regression).
  - `tests/test_rng_recon.py`: 8/8 tests passed.
  - `tests/test_formation_headings.py`: 7/7 tests passed.
  - Headless smoke test: 120/120 steps nominal (0 NaNs, 0 explosions, 0.294s).
* **Operational Inspiration**:
  - Waypoints are the operational bedrock for all Phase 3 auxiliary missions (`S&R`, `DEPOSIT`, `RECON`, `PATROL_TO_BINGO`).
  - Drones require deterministic 4D queue execution `[X, Y, Z, Loiter_Seconds]` with arrival acceptance spheres ($R \le 4.0\text{m}$), escort stabilization gates, loiter dwell timers, and discrete telemetry event streaming.
  - Operators need explicit mission controls: Pause, Resume, Skip Waypoint, and Abort Route.
* **Technical Blueprint**:
  - **Part 1: 4D Waypoint Queue & State Machine (OBC Firmware & Brains)**:
    - Formalize 4D queue entries `[x, y, z, loiter_s]` across `Drone` firmware and both brains (`LeaderFollowerBrain` and `GossipConsensusBrain`).
    - Formation Stabilization Gate: when reaching waypoint radius ($d \le 4.0\text{m}$), count down loiter dwell timer. Only advance to next waypoint when `wp_loiter_timer <= 0` AND all active living followers are settled near their formation slots ($\le 4.5\text{m}$).
    - Commands: `PAUSE_ROUTE`, `RESUME_ROUTE`, `SKIP_WAYPOINT`, `ABORT_ROUTE`.
  - **Part 2: Telemetry Snapshot & Discrete Canyon Event Stream**:
    - Telemetry frame includes `mission_route` with `active`, `paused`, `name`, `current_wp_idx`, `total_wps`, `wp_loitering`, `wp_loiter_timer`, `waypoints`.
    - Event log notices generated on state transitions: `WAYPOINT_REACHED`, `LOITER_COUNTDOWN`, `MISSION_COMPLETED`.
  - **Part 3: GCS Tactical Viewport & Controls**:
    - Add `PAUSE / RESUME` and `SKIP WP` buttons to the Tactical Route Missions panel in `index.html`.
    - Render active target waypoint sphere with a gold pulsing highlight in Three.js, distinct from upcoming cyan route vertices.
    - Display live loiter countdown badge in the HUD: `HOLDING: 3.2s`.
* **Verification Gates**:
  - Dedicated unit test suite in `tests/test_waypoint_engine.py`.
  - Passes 100% with zero regressions across `test_formation_flight_suite.py` and smoke test.
