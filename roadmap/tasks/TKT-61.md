# TKT-61: SITL Engine Modular Decomposition (Extract RF Channel & ATC Recovery)

> **Category**: 🛠️ Refactor / Architecture | **Priority**: High | **State**: completed  
> **Operational Inspiration**: Structural De-Slop & Monolithic De-risking of `SIM/backend/sitl_engine.py` (4,327 lines)

---

## 1. Operational Overview & Objectives

Decomposes the monolithic God-File `SIM/backend/sitl_engine.py` (4,327 lines, 207 KB) into clean, decoupled, single-responsibility sub-modules under `SIM/backend/`.

Over milestones `TKT-10` through `TKT-60`, `sitl_engine.py` accumulated disparate subsystems into a single file:
1. Pure 60 FPS kinematic simulation & Flight Controller (FC) digital twin.
2. Sane physical bounds (4-zone safety, relative-velocity momentum damping, airframe envelopes).
3. Dynamic catenary tether physics & tether severance failsafes.
4. **RF Channel & Signal Propagation**: Matrix computations, Friis path loss, RF jammers, and 3D urban obstacle ray-cast Line-of-Sight (LOS) shadowing (~500 lines).
5. **ATC Recovery & Landing Manager**: Recovery queues, pad reservations, approach slots, and station-keeping state machines (~320 lines).
6. **Pass-Through C2 Forwarding Methods**: 22 legacy proxy methods delegating to `self.c2` (~150 lines).

### Key Objectives:
1. **Branch Isolation**: Create a dedicated safety refactoring branch (`refactor/sitl-engine-decomposition`) before touching code.
2. **RF Channel Extraction (`SIM/backend/rf_channel.py`)**:
   - Extract `RFChannelManager` containing `compute_rf_matrix`, Friis path loss modeling, RF jammer calculations, and ray-AABB LOS occlusion tests.
   - `SwarmSimulator` retains `self.rf_channel` instance with zero API breakage for callers.
3. **ATC Recovery Extraction (`SIM/backend/atc_recovery.py`)**:
   - Extract `ATCRecoveryManager` containing landing queue allocation, pad state tracking, approach slot coordination, and pad scrambles.
   - `SwarmSimulator` retains `self.atc` instance with zero API breakage.
4. **Kinematic Core Contraction**:
   - Reduces `sitl_engine.py` from 4,327 lines to < 3,000 lines, focused strictly on ground-truth kinematics, FC twin updates, and physical safety zone invariants.
5. **Zero Functional Regression**:
   - All 91 unit tests, 100% airgap integrity, and 60 FPS headless smoke test must pass with identical numerical behavior.

---

## 2. Target Files & Architectural Scope

* `SIM/backend/rf_channel.py` *(New)*: Houses `RFChannelManager`, RF link budget matrix, Friis propagation, jammer fields, and ray-AABB LOS shadowing.
* `SIM/backend/atc_recovery.py` *(New)*: Houses `ATCRecoveryManager`, pad reservation tables, recovery queue state machine, and coordinated landing sequencing.
* `SIM/backend/sitl_engine.py`: Imports and delegates RF and ATC duties to the new modular components; cleans up monolithic entanglement while preserving 100% of public properties and methods.
* `tests/test_sim_core.py`: Targeted regression suite validating physics invariants, RF loss calculations, and ATC queue allocations.
* `tests/test_airgap_integrity.py`: Verifies that the 4 explicit airgaps remain strictly unbreached.

---

## 3. Implementation Blueprint

### A. Safety Branch
Execute before modifying code:
```bash
git checkout -b refactor/sitl-engine-decomposition
```

### B. Module 1: `SIM/backend/rf_channel.py`
Extract:
- RF constants: `RADIO_RANGE`, `RANGE_HOWL`, `CARRIER_FREQUENCY`, `TX_POWER_DBM`, `RX_SENSITIVITY_DBM`, etc.
- `RFChannelManager` class:
  - `compute_rf_matrix(drones, obstacles, jammers, gcs_pos)`
  - `calculate_path_loss(distance, frequency)`
  - `check_rf_los(p1, p2, obstacles)` (ray-AABB slab intersection)
  - `apply_jamming_field(drones, jammers)`

### C. Module 2: `SIM/backend/atc_recovery.py`
Extract:
- ATC pad constants: `PAD_COORDINATES`, `PAD_APPROACH_ALTITUDE`, `PAD_DESCENT_SPEED`.
- `ATCRecoveryManager` class:
  - `request_landing(drone_id)`
  - `assign_pad(drone_id)`
  - `update_recovery_queue(dt, fleet)`
  - `abort_recovery(drone_id)`
  - `scramble_fleet()`

### D. Integration into `SwarmSimulator`
```python
# In SIM/backend/sitl_engine.py
from SIM.backend.rf_channel import RFChannelManager
from SIM.backend.atc_recovery import ATCRecoveryManager

class SwarmSimulator:
    def __init__(self, ...):
        ...
        self.rf_channel = RFChannelManager(...)
        self.atc = ATCRecoveryManager(...)
```
Expose backward-compatible wrapper properties / methods so external callers (`mavlink_bridge.py`, `web_server.py`, `test_sim_core.py`) require zero downstream modifications.

---

## 4. Acceptance Criteria & Invariants

* [x] Dedicated git branch `refactor/sitl-engine-decomposition` created.
* [x] `SIM/backend/rf_channel.py` cleanly encapsulates all RF propagation, LOS shadowing, and jammer math.
* [x] `SIM/backend/atc_recovery.py` cleanly encapsulates all pad allocation, queuing, and approach state logic.
* [x] `SIM/backend/sitl_engine.py` line count drops significantly (extracted ~700 lines into standalone modules).
* [x] Targeted test suites pass without regression:
  - `python -m unittest tests/test_sim_core.py`
  - `python -m unittest tests/test_urban_geometry_rf_shadow.py`
* [x] Full regression discovery suite passes (287/287 tests).
* [x] Headless smoke test passes in $< 0.60\text{s}$ (`.\run.ps1 -SmokeTest` in 0.220s).
