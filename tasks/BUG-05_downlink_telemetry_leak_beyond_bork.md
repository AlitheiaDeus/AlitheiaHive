# [BUG-05] Downlink Telemetry Leak Beyond Bork Range

* **Grade**: 🐞 Bug / Defect | **Type**: Operator Airgap & Downlink RF Horizon Gating (`Completed / Fresh Review`) | **Priority**: Medium
* **Target Files**: `SIM/backend/sitl_engine.py`, `GCS/backend/c2_gateway.py`, `tests/test_tri_mode_rf.py`
* **Operational Context**:
  - The Ground Control Station continued receiving active downlinked telemetry frames from airframes that had flown well past their designated Bork radio horizon ($70\text{m}$) or escort Yip budget ($50\text{m}$).
  - Violated the Canine Acoustic Taxonomy and the Operator Airgap, where telemetry must transition to `LOST_LINK` at the Last Known Position (LKP) once beyond downlink range.
* **Technical Assessment & Root Causes Identified**:
  - **Selective LKP Masking in SITL Snapshot (`sitl_engine.py:3098-3105`)**:
    In `get_state_snapshot()`, coordinate masking to LKP was gated strictly behind:
    `if d.crashed and not getattr(d, "crashed_while_linked", False) and not d.downlink_active:`
    It ONLY activated for drones that had already **crashed**!
  - Surviving, airborne airframes that lost downlink contact (`d.downlink_active == False` beyond 70m BORK or 50m YIP) had their live, omniscient ground-truth coordinates serialized straight into `d_dict["pos"]` and dispatched to the GCS over WebSocket.
  - In `GCS/backend/c2_gateway.py:is_link_active()`, the Star downlink check evaluated against `RANGE_BARK` ($90\text{m}$) instead of dynamically honoring the leader's RF power mode (`RANGE_BORK = 70.0m`).
* **Remediation Completed**:
  - **Part 1: Strict Downlink Telemetry Gating (`sitl_engine.py:get_state_snapshot`)**:
    - For living airborne drones (`not d.crashed and not d.downlink_active and d.flight_state not in ("LANDED", "GROUND_IDLE")`):
      - Mask `d_dict["pos"]` to `[round(float(v), 2) for v in d.lkp]`.
      - Zero out velocity: `d_dict["vel"] = [0.0, 0.0, 0.0]`.
      - Set `d_dict["flight_state"] = "LOST_LINK"`, `d_dict["status"] = "LOST_LINK"`, and `d_dict["lost_link"] = True`.
  - **Part 2: Preserved God-Mode Omniscience (`sitl_engine.py`)**:
    - `sim_drones_list` is populated prior to masking, retaining unmasked physical coordinates and true status for the SITL god-view inspector (`/sim`).
  - **Part 3: Dynamic Bork Range in C2 Gateway (`c2_gateway.py`)**:
    - In `c2_gateway.py:is_link_active()`, evaluate `r_down = RANGE_BORK if getattr(leader, "rf_power_mode", None) == RF_MODE_BORK or getattr(leader, "tier", None) == 1.5 else RANGE_BARK`.
    - Both range cutoff and packet drop evaluations correctly use `r_down` ($70\text{m}$ for Tier 1.5 / BORK).
* **Verification Gates**:
  - `test_bug05_downlink_telemetry_masked_beyond_bork_range`: PASS in `tests/test_tri_mode_rf.py`. Verifies that an airborne drone at 75m with `downlink_active = False` has its `drones` entry masked to LKP with status `LOST_LINK`, while `sim_drones` contains true coordinates.
  - Full suite in `tests/test_tri_mode_rf.py`: 14/14 tests PASS in 0.55s.
  - Full suite in `tests/test_brains.py`: 25/25 tests PASS in 1.96s.
  - Full suite in `tests/test_sim_core.py`: 21/21 tests PASS in 0.90s.
  - Headless smoke test `.\run.ps1 -SmokeTest`: PASS in 0.334s (exit code 0).
