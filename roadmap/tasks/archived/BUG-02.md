# [BUG-02] Async RF Range Calculation / C2 Uplink Disconnect

* **Grade**: 🐞 Bug / Defect | **Type**: RF Link Budget, Range Scaling & C2 Dispatch (`Completed / Fresh Review`) | **Priority**: Very High
* **Target Files**: `SIM/backend/sitl_engine.py`, `GCS/backend/c2_gateway.py`, `GCS/frontend/js/app.js`, `tests/test_tri_mode_rf.py`
* **Operational Context**:
  - Drones abruptly stop responding to operator commands outside a small bounding range.
  - The fleet leader at coordinates ~(12, 8, Z) fails to track toward waypoint Alpha at (60, 40, 30).
  - Confirmed via god-mode at `http://127.0.0.1:8080/sim` that drones are neither receiving nor executing C2 uplink intents.
* **Technical Assessment & Root Cause Confirmed**:
  - **Grid Scale vs World Metric Mismatch**: In `GCS/frontend/js/app.js`, the ground grid is initialized with `size = 300, divisions = 60` (5.0m per division). Audited `app.js` raycasting and click dispatchers; confirmed click coordinates and presets are emitted in pure SI meters without dividing by grid spacing.
  - **Comms Mode & Tether Invariant**: On startup, `comms_mode` defaults to `TETHERED_RELAY`. When switching to wireless modes (`LEADER_FOLLOWER` or `GOSSIP_CONSENSUS`), `set_comms_mode` did not reset residual `self.tether_severed` and `drone.tether_lost` flags, leaving drones trapped in tether RTB failsafe or rejected uplinks.
  - **Asymmetric Uplink vs Downlink Gating**: Verified `RADIO_RANGE = RANGE_HOWL` (140.0m) governs GCS-to-fleet uplinks across all modes. Added `GOSSIP_CONSENSUS` target injection to `c2_gateway.py:target_pos.setter` and updated `is_link_active` to recognize `comms_mode in ("GOSSIP_CONSENSUS", "P2P_GOSSIP_MESH", "P2P_MESH", "MESH")`.
* **Remediation Implemented**:
  - **Part 1: Audit Coordinate Mapping**: Verified `GCS/frontend/js/app.js` raycaster and click handlers dispatch world metric coordinates without dividing by grid spacing.
  - **Part 2: Verify C2 Uplink Gating**: Ensured `c2_gateway.py` and `sitl_engine.py` evaluate link viability using `RANGE_HOWL` (140.0m) for GCS-to-fleet uplinks. Added `GOSSIP_CONSENSUS` target propagation in `c2_gateway.py:target_pos.setter`.
  - **Part 3: Mode Transition Alignment**: Updated `sitl_engine.py:set_comms_mode()` so switching to wireless modes resets `self.tether_severed = False` and `drone.tether_lost = False`, completely releasing tether physical constraints and failsafe RTBs.
  - **Part 4: C2 Target State Synchronization**: Synchronized `self.c2._target_pos` directly within `sitl_engine.py:dispatch_target_uplink()`.
* **Verification Gates**:
  - Unit test `test_bug02_uplink_dispatch_at_135m_and_alpha_tracking` added to `tests/test_tri_mode_rf.py`:
    - 135m uplink dispatch verified in `LEADER_FOLLOWER` and `GOSSIP_CONSENSUS` modes.
    - Out-of-range rejection verified at 145m (> 140m `RADIO_RANGE`).
    - Clean mode transition verified clearing tether severed/lost flags.
    - Continuous 9-second flight tracking to Waypoint Alpha `[60.0, 40.0, 30.0]` verified reaching arrival acceptance radius ($d < 2.5\text{m}$, final distance $0.04\text{m}$) without stalling at `(12, 8, Z)`.
  - Full suite in `tests/test_tri_mode_rf.py`: 13/13 tests passed in 0.838s.
  - Headless smoke test: 120/120 steps passed in 0.423s (exited code 0).
