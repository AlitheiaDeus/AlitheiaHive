# [TKT-20] Airframe Inertia, Actuator Dynamics & Tuning Presets

* **Grade**: ðŸŽ­ Gimmick (Parked / Frozen) | **Type**: Physics Modeling (`Frozen`) | **Priority**: Very Low
* **Status**: **FROZEN** (Parked in Frozen column to eliminate CFD/physics bloat and focus on C2 / Swarm coordination).
* **Target Files**: `backend/sitl_engine.py`, `frontend/js/app.js`, `frontend/js/hud.js`
* **Technical Blueprint**:
  - Replace point-mass instantaneous kinematics with 1st-order motor response lag: $\dot{\mathbf{a}} = \frac{\mathbf{a}_{\text{target}} - \mathbf{a}}{\tau}$ ($\tau \in [0.04\text{s}, 0.16\text{s}]$).
  - Attitude tilt-thrust acceleration coupling with angular slew rate limit ($|\dot{\theta}|, |\dot{\phi}| \le 250^\circ/\text{s}$).
  - Vectorized quadratic aerodynamic drag: $\mathbf{a}_{\text{drag}} = -k_d \|\mathbf{v}\|\mathbf{v}$.
  - 3 Presets: `LIGHTWEIGHT_RACER`, `TACTICAL_RECON`, `HEAVY_PAYLOAD`.
* **Targeted Test**: `python -m unittest tests/test_sim_core.py`
