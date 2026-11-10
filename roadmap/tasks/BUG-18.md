# BUG-18: GCS Target Waypoint Direct Dispatch & Single-Line Typography

> **Category**: BUG | **Priority**: High | **State**: todo  
> **Operational Inspiration**: Restore intuitive 1-click waypoint dispatch from GCS HUD, eliminate "drones stuck at muster [0, 0, 20]" confusion caused by TKT-68 stage-to-send change, and shorten button labels to prevent 2-line wraps in the tactical command deck.

---

## 1. Operational Overview & Objectives
Operators reported two critical ergonomics defects in the GCS operator viewport:
1. **Drones Stuck at Muster `[0, 0, 20]` on Scramble**: After ordering a cold pad scramble, the swarm climbs and holds at Base Overhead Muster `[0, 0, 20]`. When operators clicked preset waypoints (`ALPHA`, `BRAVO`, `CHARLIE`, `RTB`) or clicked on the 3D ground grid, no target commands were transmitted because TKT-68 replaced direct dispatch with `stageTarget(...)`, requiring an unheralded secondary click on `SEND`. Operators perceived the scramble sequence as completely broken.
2. **Button Label Wrapping**: In `GCS/frontend/js/hud.js`, line 575 rewrites `#btn-patrol-bingo` to `&#x26A1; PATROL &rarr; BINGO RTB` upon receiving telemetry, exceeding button width and wrapping across two lines in the 2-column grid.

This ticket implements:
* **Direct 1-Click Waypoint Dispatch**: Preset buttons (`ALPHA`, `BRAVO`, `CHARLIE`, `RTB BASE`) directly execute `dispatchTarget(...)`, sending `SET_TARGET` immediately and maneuvering the fleet out of muster hold.
* **Instant Ground Click Dispatch**: Update ground grid click handler to directly dispatch `SET_TARGET` to the clicked coordinate, updating `chk-click-target` label to `CLICK GROUND GRID TO DISPATCH TARGET`.
* **Single-Line Button Typography**: Maintain `#btn-patrol-bingo` label as `&#x26A1; PATROL &rarr; BINGO` across idle and telemetry reset cycles, eliminating 2-line wrapping.

## 2. Target Files & Architectural Scope
* `GCS/frontend/js/hud.js`: Restore direct `dispatchTarget(...)` calls on `btn-wp-alpha`, `btn-wp-bravo`, `btn-wp-charlie`, and `btn-wp-rtb`. Update idle button reset in `updateMissionBadges` to preserve single-line `&#x26A1; PATROL &rarr; BINGO`.
* `GCS/frontend/js/app.js`: Update 3D ground raycast click handler to directly dispatch target when `chk-click-target` is enabled.
* `GCS/frontend/index.html`: Update ground click label to `CLICK GROUND GRID TO DISPATCH TARGET`. Ensure single-line typography for `btn-patrol-bingo`.
* `tests/test_pad_scramble.py`: Verify scramble muster hold and ensure subsequent target dispatch successfully breaks muster.

## 3. Implementation Blueprint
* **Preset Buttons Event Wiring (`hud.js`)**:
  - `btn-wp-alpha` -> `dispatchTarget([60.0, 40.0, 30.0], 'ALPHA')`
  - `btn-wp-bravo` -> `dispatchTarget([-40.0, 70.0, 45.0], 'BRAVO')`
  - `btn-wp-charlie` -> `dispatchTarget([50.0, -50.0, 25.0], 'CHARLIE')`
  - `btn-wp-rtb` -> `dispatchTarget([0.0, 0.0, 20.0], 'RTB_BASE')`
* **Custom Coordinate Staging**:
  - Manual numeric typing in `tgt-input-x/y/z` continues to stage coordinates and pulse the SEND button until operator clicks SEND or presses Enter.
* **Ground Click (`app.js`)**:
  - When operator clicks ground grid with `chk-click-target` checked, immediately execute `window.tacticalHud.dispatchTarget([targetX, targetY, targetZ], 'GROUND_CLICK')`.
* **Bingo Button Reset (`hud.js`)**:
  - In `updateMissionBadges`: when `bingoData` is inactive, set `btnBingo.innerHTML = '<span class="btn-icon">&#x26A1;</span> PATROL &rarr; BINGO';`

## 4. Acceptance Criteria & Verification Invariants
* [ ] Preset waypoint buttons (`ALPHA`, `BRAVO`, `CHARLIE`, `RTB`) directly dispatch `SET_TARGET` over WebSocket on single click.
* [ ] Swarm immediately breaks Base Overhead Muster `[0, 0, 20]` upon clicking any preset waypoint.
* [ ] Ground grid click directly commands target waypoint.
* [ ] The `#btn-patrol-bingo` button text never exceeds one line, even after telemetry frames arrive.
* [ ] Targeted tests in `tests/test_pad_scramble.py` pass cleanly.
