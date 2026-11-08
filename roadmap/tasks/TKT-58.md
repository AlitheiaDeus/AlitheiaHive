# [TKT-58] 1-Click Aux Mission: Patrol to BINGO RTB

* **Grade**: 🚀 Feature | **Type**: Tactical Mission & Fuel Failsafe (`Completed / Fresh Review`) | **Priority**: Medium
* **Target Files**: `GCS/backend/c2_gateway.py`, `SIM/backend/sitl_engine.py`, `OBC/backend/drone_firmware.py`, `GCS/backend/web_server.py`, `GCS/frontend/index.html`, `GCS/frontend/css/style.css`, `GCS/frontend/js/hud.js`, `tests/test_patrol_bingo_rtb.py`
* **Operational Inspiration**:
  - Autonomous tactical patrol mission that continuously executes patrol sweeps until battery hits critical "BINGO" threshold (e.g., 25% or commanded threshold).
  - Upon BINGO trigger: automatically breaks station, cancels patrol loop, transitions swarm to RTB overhead marshal sentry hold (Stage 1), and initiates coordinated ATC landing pad allocation (Stage 2).
  - 1-Click UI operator trigger in GCS Tactical Missions panel (`PATROL -> BINGO`).
  - C2 Pre-Flight Authorization Gate: verifies `FORMATION_FLIGHT` and `PATROL` capabilities before dispatch.
* **Technical Blueprint**:
  - **Part 1: C2 Gateway Patrol to BINGO State Machine**:
    - `deploy_patrol_to_bingo(bingo_threshold: float = 25.0, pattern: str = "BOX_PATROL") -> Dict[str, Any]` in `c2_gateway.py`.
    - Capability pre-flight authorization check verifies `FORMATION_FLIGHT` and `PATROL`.
    - Battery pre-flight check rejects if battery is already `<= bingo_threshold` (`INSUFFICIENT_BATTERY`).
    - Monitors fleet battery levels via downlinked telemetry at 60 FPS in `c2_gateway.step(dt)` (Operator Airgap).
    - When min living airborne battery hits `bingo_threshold`, auto-triggers `RTB_HOLD` -> overhead marshal arrival -> `LAND_SWARM`.
    - `stop_patrol_to_bingo() -> Dict[str, Any]` clean operator abort.
  - **Part 2: GCS Tactical UI & Viewport Controls**:
    - Added `#btn-patrol-bingo` button with amber cyberpunk neon styling (`.btn-bingo`, `.active-bingo`, `@keyframes pulse-bingo`).
    - Visual badge / status indicator on HUD (`#route-active-badge`) and button update in-place (`PATROL [BINGO 25%]` -> `BINGO RTB [RTB_TRANSIT]` -> `BINGO RTB [LANDING]`).
  - **Part 3: Tactical Power Drain & Dual Airgap Defense (Resolved Open Concerns)**:
    - **Tactical Power Drain**: State-based battery drain in `sitl_engine.py` (0.15 %/s cruise, 0.25 %/s sprint, 0.001 %/s ground idle). Tethered drones with `TETHER_POWER_DATA` maintain 100% infinite power. Configurable `sim.battery_drain_multiplier` allows accelerated bench testing.
    - **Brain Airgap Failsafe**: In `OBC/backend/drone_firmware.py`, each drone maintains local `bingo_threshold = 20.0`. If a drone in `LOST_LINK` hits BINGO without GCS comms, its OBC autonomically triggers `CORRIDOR_ERTB` along its `corridor_stack` back to Base. At `<= 3.0%`, emergency blind soft-settle (`gnss_settled = True`) engages.
* **Verification Gates**:
  - [x] Unit tests in `tests/test_patrol_bingo_rtb.py`: 8/8 tests pass in 0.19s.
  - [x] Zero regression across existing test suites (`test_tier_registry_auth.py`, `test_waypoint_engine.py`, `test_airgap_integrity.py`, `test_rtb_landing.py`, `test_gnss_denial.py`): 50/50 tests pass.
  - [x] Full regression discovery suite: 259/259 tests pass cleanly in 37.9s (17 skipped).
  - [x] Headless smoke test passes in 0.17s (`run.py --smoke-test`).
