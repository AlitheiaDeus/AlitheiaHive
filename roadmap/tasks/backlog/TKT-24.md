# [TKT-24] Dynamic Wind Gust Vector Field & Aerodynamic Disturbance

* **Grade**: ðŸŽ­ Gimmick (Parked / Frozen) | **Type**: Atmospheric Disturbance (`Frozen`) | **Priority**: Very Low
* **Status**: **FROZEN** (Parked in Frozen column to eliminate atmospheric disturbance bloat and focus on C2 / Swarm coordination).
* **Target Files**: `backend/sitl_engine.py`, `frontend/js/app.js`, `frontend/js/hud.js`
* **Technical Blueprint**:
  - 3D Dryden/Perlin wind vector field, aerodynamic crosswind drag, and autopilot crab angles.
  - Compass rose HUD widget with live wind vector arrow, velocity readout, and gust intensity.
* **Targeted Test**: `python -m unittest tests/test_sim_core.py`
