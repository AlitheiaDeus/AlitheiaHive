# [FIX-09] Strict WE-GCS Airgap Enforcement, Default Cold Start & Comms Integrity

* **Grade**: 🛠️ Improvement | **Type**: Architectural Integrity / Airgap & Comms Enforcement (`Completed`) | **Priority**: High
* **Target Files**: `SIM/backend/sitl_engine.py`, `OBC/backend/brains/leader_follower.py`, `OBC/backend/brains/gossip_consensus.py`, `run_with_sim.py`, `GCS/frontend/index.html`, `GCS/frontend/js/hud.js`, `GCS/frontend/css/style.css`, `SIM/frontend/index.html`, `SIM/frontend/js/sim_hud.js`, `tests/test_pad_scramble.py`, `tests/test_sim_core.py`
* **Operational Inspiration**:
  - Resolves remaining issues from `potato notes.txt` (Q2, Q3, Q4, Q5, Q7):
    - GCS has no business deciding if drones start in air or on ground (Q7), nor clicking EW GPS jamming (Q3).
    - If operator clears a pad in GCS without physical retrieval in WE, incoming landing drone crashes due to operator incompetence.
    - GCS RNG RECON button styling and emoji sizing fix (Q4).
    - Toggling comms modes (Wireless Star to P2P Mesh) causing formation reset and clashes (Q5).
    - GCS omniscience leak: GCS receiving `crashed: true` for drones destroyed beyond radio horizon (Q2).
* **Technical Blueprint**:
  - **Part 1: Default Cold-Start in World Engine (Q7)**:
    - In `sitl_engine.py:126`, default `cold_start = True`. Drones spawn cold at $Z = 0.05\text{m}$ on numbered pads.
    - Add `--start-air` CLI flag to `run_with_sim.py` and `SITLWorldEngine(start_in_air=True)` for backward-compatible test execution where airborne initialization is desired.
    - Remove any GCS controls for ground vs air spawn. GCS simply renders live state (`GROUND_IDLE` vs `ACTIVE`).
  - **Part 2: GCS UI Cleanup (Q3 & Q4)**:
    - Remove `btn-jam-gnss` (`TOGGLE GNSS DENIAL`) from `GCS/frontend/index.html` and `GCS/frontend/js/hud.js`. Fault injection belongs strictly in `/sim`.
    - Fix `btn-rng` styling in `GCS/frontend/css/style.css`: change `#FFAA00` to standard neon cyan `#00F0FF`, normalize grid margins with `.btn-patrol`, and constrain emoji height.
  - **Part 3: Physical Pad Collision on Operator Incompetence**:
    - In GCS, operator `[CLEAR PAD RECORD]` clears operator pad registry only.
    - In WE, physical pad occupancy is determined by physical airframe distance to pad center ($d_{\text{xy}} \le 1.5\text{m}, Z \le 0.52\text{m}$).
    - If GCS commands a drone to land on a pad that physically contains an unretrieved airframe, descending drone triggers Zone 4 clash at touchdown $\to$ `_trigger_crash(reason="UNRETRIEVED_PAD_COLLISION")`.
    - In `/sim` (God-View), add `[RETRIEVE DRONE FROM PAD]` button for ground crew physical retrieval.
  - **Part 4: Comms Mode Handover & Anti-Crash Stabilization (Q5)**:
    - Disable/lock `TETHER RELAY` in GCS Command Deck once airborne (hardware umbilical cannot be plugged in mid-flight; it is a pre-flight mission config in WE).
    - Align `GossipConsensusBrain` formation slots to be relative to the swarm center of mass / virtual leader rather than distant waypoint `target_pos`, preventing 50m instantaneous slot jumps.
    - Seamless brain handover: preserve `assigned_slots` and `formation_heading` when switching between Wireless Star and P2P Mesh.
  - **Part 5: Comms Horizon Crash Masking (Q2)**:
    - In `sitl_engine.py` snapshot serialization: if `not drone.downlink_active`:
      - Do NOT leak `"crashed": True` to GCS if the crash occurred while out of radio contact.
      - GCS renders `[LOST_LINK]` at Last Known Position (LKP).
      - Omniscient ground truth (`crashed: True`) remains strictly in `/sim` God-View.
* **Targeted Test**: `powershell -File .\run.ps1 -UnitTest -Target tests.test_pad_scramble` & `tests.test_brains`
* **Verification Status**: Verified. Ran targeted tests `test_pad_scramble` (10/10 PASS), `test_brains` (20/20 PASS), `test_rtb_landing` (16/16 PASS), `test_sim_core` (21/21 PASS), full 196-test regression suite (196/196 PASS), formation flight suite (18/18 PASS), and smoke test (PASS in 0.122s).
