# [BUG-04] Wireless Star Coordinate Collapse / Singularity at Range

* **Grade**: 🐞 Bug / Defect | **Type**: Formation Geometry & Star Topology Failsafe (`Completed / Fresh Review`) | **Priority**: High
* **Target Files**: `SIM/backend/sitl_engine.py`, `GCS/frontend/js/app.js`, `OBC/backend/brains/leader_follower.py`, `tests/test_brains.py`, `tests/test_sim_core.py`
* **Operational Context**:
  - Under Wireless Star architecture (`LEADER_FOLLOWER`), when outside the downlink radius (~12 grid units / ~50–60m), all drones (including cold-start drones sitting on the pad) temporarily converged and collapsed into a single spatial coordinate.
  - Caused severe spatial crowding, Zone 4 prop-clash risk, and visually erratic collapse to a central singularity.
* **Technical Assessment & Root Causes Identified**:
  - **Follower LKP Overwritten with Leader Position (`sitl_engine.py:1616, 1674`, `leader_follower.py:388, 392`)**: In `_update_radio_comms()` and `LeaderFollowerBrain._ingest_packets()`, whenever a follower heard the leader, `drone.lkp` and `updates["lkp"]` were overwritten with the airborne leader's position `current_leader.pos` rather than the drone's own coordinates.
  - **Grounded / Pad Airframes Corrupted (`sitl_engine.py:1620, 1682`)**: `_update_radio_comms()` did not exclude `GROUND_IDLE` or `LANDED` drones when updating status and LKP. Pad drones had their status flipped to `LOST_LINK` and their `lkp` overwritten with airborne coordinates.
  - **Frontend Avatar Teleportation (`app.js:994`)**:
    `const renderPos = ((d.lost_link || !d.downlink_active) && Array.isArray(d.lkp)) ? d.lkp : d.pos;`
    When range exceeded the 50m escort budget (`RANGE_YIP`), `d.downlink_active` became `False`. The frontend switched rendering position to `d.lkp`. Because all drones (including pad drones) shared `d.lkp == current_leader.pos`, all 6 drone avatars snapped to the exact same 3D coordinate.
  - **Onboard Physical Convergence (`leader_follower.py:151`)**: Followers in `lost_link` steered directly toward `telemetry.lkp` (the un-offset leader position), physically drawing the airborne airframes together.
* **Remediation Completed**:
  - **Part 1: Fix Follower & Pad LKP Invariant (`sitl_engine.py`)**:
    - Changed follower LKP updates in `TETHERED_RELAY` and `LEADER_FOLLOWER` from `drone.lkp = np.copy(current_leader.pos)` to `drone.lkp = np.copy(drone.pos)`.
    - Added guards preventing `GROUND_IDLE` and `LANDED` drones from having their status changed to `LOST_LINK` or `ACTIVE` while disarmed on pads.
  - **Part 2: Onboard Follower LKP Register Tracking (`leader_follower.py`)**:
    - In `_ingest_packets()`, changed `updates["lkp"] = np.copy(l_pos)` to `updates["lkp"] = np.copy(telemetry.pos)`. Follower LKP records the follower's own position where link was active.
    - When link is lost, each follower steers to its own LKP, maintaining formation slot spacing in the sky without converging.
  - **Part 3: Frontend Grounded Airframe Guard (`app.js`)**:
    - Updated `renderPos` computation to check `d.flight_state !== 'GROUND_IDLE' && d.flight_state !== 'LANDED'`. Pad drones stay anchored to their launch pads regardless of downlink activity.
  - **Part 4: Test Alignment & Regression Coverage (`tests/test_sim_core.py`, `tests/test_brains.py`)**:
    - Updated `test_sim_core.py:test_radio_baud_and_lkp_recovery` to assert `follower.lkp == follower.pos`.
    - Added `test_bug04_follower_and_pad_lkp_isolation_in_wireless_star` and `test_bug04_airborne_followers_preserve_slot_separation_on_lost_link`.
* **Verification Gates**:
  - `test_bug04_follower_and_pad_lkp_isolation_in_wireless_star`: PASS. Confirms follower and pad drones maintain independent LKP coordinates and pad drones never corrupt to `LOST_LINK`.
  - `test_bug04_airborne_followers_preserve_slot_separation_on_lost_link`: PASS. Confirms airborne followers maintain pairwise slot separation (> 1.5m) and zero collisions when losing link.
  - `test_radio_baud_and_lkp_recovery`: PASS (21/21 in `test_sim_core.py`).
  - Full suite in `tests/test_brains.py`: 25/25 PASS in 2.49s.
  - Headless smoke test `.\run.ps1 -SmokeTest`: PASS in 0.451s (exit code 0).
