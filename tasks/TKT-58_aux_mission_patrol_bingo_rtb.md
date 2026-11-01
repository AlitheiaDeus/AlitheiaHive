# [TKT-58] 1-Click Aux Mission: Patrol to BINGO RTB

* **Grade**: 🚀 Feature | **Type**: Tactical Mission & Fuel Failsafe (`In Progress / WIP = 1`) | **Priority**: Medium
* **Target Files**: `GCS/backend/c2_gateway.py`, `SIM/backend/sitl_engine.py`, `GCS/frontend/index.html`, `GCS/frontend/js/hud.js`, `tests/test_patrol_bingo_rtb.py`
* **Operational Inspiration**:
  - Autonomous tactical patrol mission that continuously executes patrol sweeps until battery hits critical "BINGO" threshold (e.g., 25% or commanded threshold).
  - Upon BINGO trigger: automatically breaks station, cancels patrol loop, transitions swarm to RTB overhead marshal sentry hold (Stage 1), and initiates coordinated ATC landing pad allocation (Stage 2).
  - 1-Click UI operator trigger in GCS Tactical Missions panel (`PATROL -> BINGO`).
  - C2 Pre-Flight Authorization Gate: verifies `FORMATION_FLIGHT` and `PATROL` capabilities before dispatch.
* **Technical Blueprint**:
  - **Part 1: C2 Gateway Patrol to BINGO State Machine**:
    - Add `deploy_patrol_to_bingo(bingo_threshold: float = 25.0, pattern: str = "BOX_PATROL") -> Dict[str, Any]` to `c2_gateway.py`.
    - Monitors fleet battery levels at 60 FPS in SITL simulation loop.
    - When any critical airframe or average fleet battery hits `bingo_threshold`, auto-triggers `RTB_HOLD` -> `LAND_SWARM`.
  - **Part 2: GCS Tactical UI & Viewport Controls**:
    - Add `PATROL -> BINGO` button to the tactical mission presets in `#left-panel`.
    - Provide visual badge / status indicator on HUD when BINGO watchdog is armed.
  - **Part 3: Verification & Safeguards**:
    - Unit test suite verifying BINGO threshold triggers, RTB handoff, and pre-flight authorization.
* **Verification Gates**:
  - Unit tests in `tests/test_patrol_bingo_rtb.py`.
  - Zero regression across existing test suites (`test_tier_registry_auth.py`, `test_waypoint_engine.py`) and smoke test.
* **⚠️ Open Concerns (Raised During Phase 3 Overview, 2026-11-01)**:
  - **BINGO is unreachable at current drain rate**: `SIM/backend/sitl_engine.py` (~L1168) drains `drone.battery` at a flat `0.005 %/s`. Reaching a 25% threshold from 100% takes ~15,000 s (~4.2 h sim time), so the watchdog never fires in a real session. Options include a faster/load-scaled drain, a scoped slice of frozen `TKT-21` (LiPo curves), or a test-only drain override. Not decided.
  - **Airgap placement of the BINGO watchdog**: Part 1 puts the monitor in `c2_gateway.py` "in SITL simulation loop". The GCS can only act on downlinked battery telemetry, and a drone in `LOST_LINK` stops reporting. It's an open question whether BINGO should also (or instead) be an onboard OBC failsafe per the Brain Airgap.
  - *P.S. From Claude: consult your user before resolving either point.*
