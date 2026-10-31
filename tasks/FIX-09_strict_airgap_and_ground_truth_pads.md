# [FIX-09] Strict WE-GCS Airgap Enforcement, Default Cold Start & Physical Pad Ground Truth

* **Grade**: 🛠️ Improvement | **Type**: Architectural Integrity / Airgap Enforcement (`Todo`) | **Priority**: High
* **Target Files**: `SIM/backend/sitl_engine.py`, `run_with_sim.py`, `GCS/frontend/index.html`, `GCS/frontend/js/hud.js`, `GCS/frontend/css/style.css`, `SIM/frontend/index.html`, `SIM/frontend/js/sim_hud.js`, `tests/test_pad_scramble.py`
* **Operational Inspiration**:
  - GCS has no business deciding if drones start in the air or on the ground, or clicking buttons to jam GPS.
  - If an operator clears a pad on GCS without ground crew physically retrieving the drone in WE, an incoming landing drone must collide with the unretrieved drone on the pad due to operator incompetence.
* **Technical Blueprint**:
  - **Part 1: Default Cold-Start in World Engine (WE)**:
    - In `sitl_engine.py:126`, default `cold_start = True`. Drones spawn cold at $Z = 0.05\text{m}$ on numbered pads.
    - Add `--start-air` CLI flag to `run_with_sim.py` and `SITLWorldEngine(start_in_air=True)` for backward-compatible test execution where airborne initialization is desired.
    - Remove any GCS controls for ground vs air spawn. GCS simply renders live state (`GROUND_IDLE` vs `ACTIVE`).
  - **Part 2: GCS UI Cleanup (Strip EW Jamming from Operator Terminal)**:
    - Remove `btn-jam-gnss` (`TOGGLE GNSS DENIAL`) from `GCS/frontend/index.html` and `GCS/frontend/js/hud.js`. Fault injection belongs strictly in `/sim`.
    - Fix `btn-rng` styling in `GCS/frontend/css/style.css`: change `#FFAA00` to cyan `#00F0FF`, remove height/margin asymmetry with `btn-patrol`.
  - **Part 3: Physical Pad Collision on Operator Incompetence**:
    - In GCS, operator `[CLEAR PAD RECORD]` clears operator pad registry only.
    - In WE, physical pad occupancy is determined by physical airframe distance to pad center ($d_{\text{xy}} \le 1.5\text{m}, Z \le 0.52\text{m}$).
    - If GCS commands a drone to land on a pad that physically contains an unretrieved airframe, descending drone triggers Zone 4 clash at touchdown $\to$ `_trigger_crash(reason="UNRETRIEVED_PAD_COLLISION")`.
    - In `/sim` (God-View), add `[RETRIEVE DRONE FROM PAD]` button for ground crew physical retrieval.
* **Targeted Test**: `python -m unittest tests/test_pad_scramble.py tests/test_rtb_landing.py`
* **Verification Status**: Todo.
