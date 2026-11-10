# TKT-68: Scramble Overhead Muster Hold & Target Stage-to-Send Confirmation

> **Category**: TKT | **Priority**: High | **State**: completed  
> **Operational Inspiration**: Prevent leader from immediately sprinting away from staggered followers on scramble, and eliminate accidental drone flight during target configuration by requiring explicit operator "SEND" confirmation.

---

## 1. Operational Overview & Objectives
During cold scramble and target selection, two operational issues caused disorderly swarm motion:
1. **Scramble Separation String**: Flagship #0 scrambled at $T=0$ and immediately sprinted toward `ALPHA [60, 40, 30]` at $12\text{ m/s}$ while followers staggered by $1.2\text{s}$ per airframe were still grounded. By the time follower 5 launched at $T=6.0\text{s}$, the flagship was 60 meters away, stringing the swarm out in single file across the sky.
2. **Premature Flight During Configuration**: Clicking preset waypoints (`ALPHA`, `BRAVO`, `CHARLIE`, `RTB`) or clicking the 3D ground map immediately dispatched `SET_TARGET` over WebSocket, flying the swarm before the operator was done tuning coordinates.

This ticket implements:
* **Overhead Muster Hold (`[0.0, 0.0, 20.0]`)**: On scramble and cold-start, the swarm forms up and holds in an overhead muster assembly directly above the launch pads ($Z=20\text{m}$). The leader does not cruise away until an explicit mission waypoint is committed.
* **Stage -> Send Confirmation Workflow**:
  - Clicking presets (`ALPHA`, `BRAVO`, `CHARLIE`, `RTB`) or clicking the 3D ground stages coordinates into `X/Y/Z` inputs.
  - Spawns a high-visibility amber/gold 3D Staged Target Beacon (`stagedBeacon`) in Three.js with an octahedron wireframe, rotating reticle, ground pulse ring, and billboard label `[CLICK SEND]`.
  - Pulses the "SEND" button with high-tech amber glow (`.btn-send.pulse-send`).
  - Typing in coordinate inputs updates the staged cursor in real-time, and pressing `Enter` submits.
  - Only when the operator explicitly clicks **SEND** (`btn-dispatch-custom`) is `SET_TARGET` transmitted over WebSocket to command fleet flight.

## 2. Target Files & Architectural Scope
* `GCS/backend/c2_gateway.py`: Set default GCS commanded target `_target_pos` to base overhead muster hold `[0.0, 0.0, 20.0]`.
* `SIM/backend/atc_recovery.py`: Set scramble target to Base Overhead Muster Hold `[0.0, 0.0, 20.0]` for all launching airframes.
* `SIM/backend/sitl_engine.py`: Reset `self.target_pos` to `[0.0, 0.0, 20.0]` on cold scramble and cold reset.
* `GCS/frontend/index.html`: Update default input values and telemetry indicators to `[0, 0, 20]`, update ground click label to `CLICK GROUND GRID TO STAGE TARGET`.
* `GCS/frontend/css/style.css`: Add amber pulse animation `.btn-send.pulse-send` and `@keyframes pulse-send-glow`.
* `GCS/frontend/js/hud.js`: Implement `stageTarget(pos, label)`, wire preset buttons to stage target, update coordinate input typing listeners and Enter key dispatch, clear pulse and staged cursor on `dispatchTarget`.
* `GCS/frontend/js/app.js`: Implement `createStagedBeacon()`, animate staged cursor, update ground click to `stageTarget`, expose `window.tacticalApp.setStagedTarget` / `clearStagedTarget`, default beacon positions to `[0, 0, 20]`.
* `SIM/frontend/js/sim_app.js`: Default initial target beacon positions to `[0, 0, 20]`.
* `tests/test_pad_scramble.py`: Add `test_scramble_muster_hold_overhead` validating overhead muster hold and station-keeping on scramble.

## 3. Implementation Blueprint
* **Scramble Station-Keeping Invariant**:
  When `command_scramble_swarm` executes:
  - All launched drones initialize with `acknowledged_target = [0.0, 0.0, 20.0]`.
  - When the leader ascends through $Z \ge 20.0\text{m}$, it holds station in overhead hover above launch pad 0 ($d_{xy} < 2.0\text{m}$).
  - Followers join their formation slots around the leader in the muster column rather than racing across the map.
* **Stage -> Send State Machine**:
  1. `EVENT: Preset Click / Ground Click / Coordinate Input` -> `ACTION: stageTarget(pos)`:
     - Sets `inputX, inputY, inputZ`.
     - Calls `window.tacticalApp.setStagedTarget(pos)`.
     - Activates `.pulse-send` on `btn-dispatch-custom`.
     - Logs `STAGED: Target staged at [x, y, z]. Press SEND to commit.`
     - Fleet remains on current target.
  2. `EVENT: SEND Button Click / Enter Key` -> `ACTION: dispatchTarget(pos)`:
     - Reads staged `[x, y, z]`.
     - Transmits `window.swarmClient.send({ cmd: 'SET_TARGET', pos })`.
     - Removes `.pulse-send`.
     - Calls `window.tacticalApp.clearStagedTarget()`.
     - Logs `DISPATCH: Commanded target uplink dispatched -> [x, y, z]`.
     - Swarm breaks muster and cruises to new target.

## 4. Acceptance Criteria & Verification Invariants
* [x] On cold start and scramble, the swarm assembles at `[0.0, 0.0, 20.0]` directly above the base pad column.
* [x] Leader remains within $2\text{m}$ of $(0, 0)$ overhead until operator dispatches a waypoint.
* [x] Clicking presets (`ALPHA`, `BRAVO`, `CHARLIE`, `RTB`) stages coordinates and does not command fleet flight.
* [x] Clicking on the 3D ground grid stages coordinates and does not command fleet flight.
* [x] Staged coordinates render an amber 3D preview beacon with targeting reticle and `[CLICK SEND]` billboard label.
* [x] The "SEND" button pulses with high-tech amber illumination when a target is staged.
* [x] Clicking "SEND" or pressing Enter commits the staged target, transmitting `SET_TARGET` and clearing the staged cursor.
* [x] Targeted test passes: `python -m unittest tests/test_pad_scramble.py` (11 tests pass in 0.380s).
