# [BUG-01] Fleet Telemetry Window Empty

* **Grade**: 🐞 Bug / Defect | **Type**: Telemetry Serialization & GCS Frontend (`Completed / Fresh Review`) | **Priority**: High
* **Target Files**: `GCS/frontend/js/hud.js`, `SIM/backend/sitl_engine.py`, `GCS/backend/web_server.py`, `tests/test_web_server.py`
* **Operational Context**:
  - The fleet telemetry window container is rendered on the GCS HUD interface, but individual drone telemetry cards/rows were completely missing or empty.
  - Occurred following the `LocalTelemetry` / `FlightIntent` refactor and telemetry card button refactoring.
* **Technical Assessment & Root Cause Confirmed**:
  - **Fatal Frontend Runtime Crash (`GCS/frontend/js/hud.js:1039`)**: In `createCardElement(drone)`, the returned object included `elActionBtn: actionBtn`. However, `actionBtn` was never declared or selected in the function scope (only `const landBtn = el.querySelector('.btn-land-action')` existed).
  - This threw `Uncaught ReferenceError: actionBtn is not defined` immediately upon processing Drone #0 inside `frame.drones.forEach(d => updateOrCreateDroneCard(d))`.
  - The uncaught exception terminated the loop before `cardsContainer.appendChild(card.el)` was called, preventing all cards from mounting into the DOM.
* **Remediation Implemented**:
  - **Part 1: Fix DOM Element Binding**: Declared `const actionBtn = el.querySelector('.btn-kill') || el.querySelector('.card-btn-action:not(.btn-land-action)') || null;` in `hud.js:createCardElement`, eliminating `ReferenceError`.
  - **Part 2: Defensive Card Generation**: Wrapped card creation in `updateOrCreateDroneCard` inside `try/catch` with fallback error badges, sanitized `pos`/`vel`/`battery` null checks, and wrapped `forEach` iteration inside `try/catch`.
  - **Part 3: Snapshot Schema Verification**: Verified backend `get_state_snapshot()` exports all required telemetry properties as JSON-compliant primitives.
  - **Part 4: Regression Test**: Added `test_bug01_telemetry_schema_and_hud_contract` to `tests/test_web_server.py`.
* **Verification Gates**:
  - Frontend runs without JavaScript `ReferenceError` (verified in Node execution sandbox).
  - Telemetry cards mount cleanly into DOM.
  - Targeted unit tests in `tests/test_web_server.py` pass (8/8 tests, 0 failures).
