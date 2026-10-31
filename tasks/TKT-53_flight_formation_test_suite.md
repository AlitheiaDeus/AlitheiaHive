# [TKT-53] Comprehensive Flight Formation Test Suite & Telemetry Playback Recorder

* **Grade**: 🚀 Feature | **Type**: Automated Verification & Playback Harness (`In Progress`) | **Priority**: High
* **Target Files**: `tests/test_formation_flight_suite.py`, `SIM/backend/telemetry_recorder.py`, `kanban.md`
* **Operational Inspiration**:
  - Validates dynamic swarm stability, inter-drone safety envelopes, and zero Zone 4 clashes ($d \le 1.5\text{m}$) under active maneuvering before deploying live code.
  - Generates replayable telemetry recordings to inspect flight transitions visually or headlessly.
* **Technical Blueprint**:
  - **Part 1: North-Oriented Initial Formation & Settle Engine**:
    - Drones initialize in standard formation (e.g. Echelon Escort, $N=6$ or $N=8$) at target station $[0.0, 0.0, 20.0\text{m}]$, aligned North ($\psi = 90^\circ$ in ENU, pointing $+Y$).
    - `wait_for_settle(sim, timeout_sec=10.0, max_speed=0.3)` advances simulation until all living drones reach their designated formation slots within speed threshold $\le 0.3\text{ m/s}$.
  - **Part 2: Target Altitude Maneuver Testing (Up & Down)**:
    - Target altitude step Up: $[0, 0, 20\text{m}] \to [0, 0, 35\text{m}]$. Wait for settle.
    - Target altitude step Down: $[0, 0, 35\text{m}] \to [0, 0, 12\text{m}]$. Wait for settle.
    - Verify: Min inter-drone distance $> 1.5\text{m}$, zero Zone 4 clashes, vertical rate bounded by FC limits ($|\dot{Z}| \le 8\text{ m/s}$), no floor breach ($Z \ge 0.5\text{m}$).
  - **Part 3: Target Bearing & Heading Maneuver Testing (Left, Right & Reversal)**:
    - Distance steps ($R = 40\text{m}$) at various angles:
      - North-East ($+45^\circ$ / Right turn): $[28.3, 28.3, 20.0]$. Settle.
      - North-West ($-45^\circ$ / Left turn): $[-28.3, 28.3, 20.0]$. Settle.
      - Pure East ($+90^\circ$): $[40.0, 0.0, 20.0]$. Settle.
      - Pure West ($-90^\circ$): $[-40.0, 0.0, 20.0]$. Settle.
      - Reciprocal U-Turn ($180^\circ$ reversal): Target stepped behind swarm. Verifies Hungarian slot inversion deconfliction without crossing drag-races.
  - **Part 4: All-to-All Full Formation Transition Matrix ($4 \times 3 = 12$ Transitions)**:
    - Supported modes: `ECHELON_ESCORT`, `NATURAL_FLOCK` (Boids), `ORBIT_VORTEX`, `PERIMETER_PATROL`.
    - Cycles through every pairwise permutation:
      - `ECHELON` $\to$ `NATURAL_FLOCK`
      - `NATURAL_FLOCK` $\to$ `ORBIT_VORTEX`
      - `ORBIT_VORTEX` $\to$ `PERIMETER_PATROL`
      - `PERIMETER_PATROL` $\to$ `ECHELON`
      - (and reverse/cross pairs: `ECHELON` $\to$ `VORTEX`, `VORTEX` $\to$ `FLOCK`, etc.)
    - Between transitions: wait for settle, assert zero Zone 4 clashes, assert all drone states non-NaN.
  - **Part 5: Telemetry Playback Recorder & CLI Replay**:
    - `TelemetryRecorder` captures 60 FPS trajectory frames: `{timestamp, target, formation, drones: [{id, pos, vel, yaw}], min_dist, zone4_clashes}`.
    - Exports to `tests/recordings/formation_flight_run.json`.
    - CLI runner `--replay <file>` steps through recording with formatted ASCII telemetry HUD.
* **Targeted Test**: `python -m unittest tests/test_formation_flight_suite.py`
* **Verification Status**: ✅ Verified (4/4 targeted tests passing in 4.13s; 18-phase battery recording 4,530 frames in 75.5s with zero Zone 4 clashes; full discovery 191/191 tests passing in 18.06s; smoke test in 0.12s).
