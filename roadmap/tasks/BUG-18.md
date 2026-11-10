# BUG-18: Fix Telemetry Input Clobbering on SEND & Preserve Stage-to-Send Safety

> **Category**: BUG | **Priority**: High | **State**: completed  
> **Operational Inspiration**: Fix telemetry synchronization clobbering staged coordinates before operator can click SEND, preserve intentional TKT-68 Stage-to-Send flight safety guardrails, and shorten button labels to eliminate 2-line wraps in the tactical command deck.

---

## 1. Operational Overview & Objectives
Operators reported two critical ergonomics defects in the GCS operator viewport:
1. **Drones Stuck at Muster `[0, 0, 20]` on Scramble**: After ordering a cold pad scramble, the swarm climbs and holds at Base Overhead Muster `[0, 0, 20]`. When operators staged preset waypoints (`ALPHA`, `BRAVO`, `CHARLIE`, `RTB`), clicked on the 3D ground grid, or manually typed coordinates and clicked `SEND`, the drones did not maneuver away from `[0, 0, 20]`.
   - **Root Cause**: In `GCS/frontend/js/hud.js`, the 60 FPS telemetry loop checked `!inputX.matches(':focus')`. Whenever an operator clicked a preset button, clicked the ground grid, or clicked the `SEND` button, `inputX` was not focused. Within 16ms, the incoming telemetry frame silently overwrote `inputX/Y/Z` back to `cmdTarget` (`[0, 0, 20]`). When the `SEND` button fired, it read the clobbered `[0, 0, 20]` values and repeatedly re-dispatched the swarm to muster hold.
   - **Flight Safety Guardrail**: Stage-to-Send (TKT-68) is strictly intentional and required for physical flight safety (preventing drones from accidentally beelining into off-limits airspace or clashing into buildings before the operator tunes coordinates/altitude).
2. **Button Label Wrapping**: In `GCS/frontend/js/hud.js`, line 575 rewrote `#btn-patrol-bingo` to `&#x26A1; PATROL &rarr; BINGO RTB` upon receiving telemetry, exceeding button width and wrapping across two lines in the 2-column grid.

This ticket implements:
* **Guarded Telemetry Input Synchronization**: Maintain `stagedTarget` in `hud.js`. Guard input value updates so incoming 60 FPS telemetry never clobbers inputs while an input is focused or while a target is staged.
* **Stage-to-Send Flight Safety Preservation**: Preset buttons (`ALPHA`, `BRAVO`, `CHARLIE`, `RTB BASE`) and 3D ground grid clicks stage coordinates into inputs and display the amber 3D preview beacon. Clicking `SEND` or pressing `Enter` dispatches `SET_TARGET` and clears `stagedTarget`.
* **Single-Line Button Typography**: Maintain `#btn-patrol-bingo` label as `&#x26A1; PATROL &rarr; BINGO` across idle and telemetry reset cycles, eliminating 2-line wrapping.

## 2. Target Files & Architectural Scope
* `GCS/frontend/js/hud.js`: Guard telemetry input synchronization against `stagedTarget !== null`. Wire preset buttons and manual inputs to `stageTarget(...)`. Clear `stagedTarget` on `dispatchTarget(...)`. Ensure single-line typography for `btn-patrol-bingo`.
* `GCS/frontend/js/app.js`: Wire 3D ground raycast click handler to `window.tacticalHud.stageTarget(...)`.
* `GCS/frontend/index.html`: Update ground click label to `CLICK GROUND GRID TO STAGE TARGET`. Ensure single-line typography for `btn-patrol-bingo`.
* `tests/test_pad_scramble.py`: Verify scramble muster hold and ensure subsequent target dispatch successfully breaks muster.

## 3. Implementation Blueprint
* **Telemetry Guard (`hud.js`)**:
  - `const anyInputFocused = (inputX && inputX.matches(':focus')) || (inputY && inputY.matches(':focus')) || (inputZ && inputZ.matches(':focus'));`
  - `if (cmdTarget && inputX && !anyInputFocused && stagedTarget === null) { ... }`
* **Preset Buttons Event Wiring (`hud.js`)**:
  - `btn-wp-alpha` -> `stageTarget([60.0, 40.0, 30.0], 'ALPHA')`
  - `btn-wp-bravo` -> `stageTarget([-40.0, 70.0, 45.0], 'BRAVO')`
  - `btn-wp-charlie` -> `stageTarget([50.0, -50.0, 25.0], 'CHARLIE')`
  - `btn-wp-rtb` -> `stageTarget([0.0, 0.0, 20.0], 'RTB_BASE')`
* **Ground Click (`app.js`)**:
  - When operator clicks ground grid with `chk-click-target` checked, execute `window.tacticalHud.stageTarget([targetX, targetY, targetZ], 'GROUND_CLICK')`.
* **Send Button Commit (`hud.js`)**:
  - Clicking `SEND` or pressing `Enter` reads `stagedTarget` / input coordinates, calls `dispatchTarget(...)`, sends `SET_TARGET`, and clears `stagedTarget`.
* **Bingo Button Reset (`hud.js`)**:
  - In `updateMissionBadges`: when `bingoData` is inactive, set `btnBingo.innerHTML = '<span class="btn-icon">&#x26A1;</span> PATROL &rarr; BINGO';`

## 4. Acceptance Criteria & Verification Invariants
* [x] Preset waypoint buttons (`ALPHA`, `BRAVO`, `CHARLIE`, `RTB`) stage target coordinates without premature dispatch.
* [x] Ground grid click stages target coordinates and displays 3D preview beacon.
* [x] Incoming 60 FPS telemetry frames never overwrite or clobber staged input values before `SEND` is clicked.
* [x] Clicking `SEND` or pressing `Enter` dispatches staged coordinates to swarm, successfully breaking Base Overhead Muster `[0, 0, 20]`.
* [x] The `#btn-patrol-bingo` button text never exceeds one line, even after telemetry frames arrive.
* [x] Targeted tests in `tests/test_pad_scramble.py` pass cleanly.
