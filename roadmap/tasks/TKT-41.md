# [TKT-41] Cold-Start Pad Scramble & Sequenced Climb-Out Corridor

* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / Startup Kinematics (`Completed`) | **Priority**: High
* **Target Files**: `SIM/backend/sitl_engine.py`, `OBC/backend/drone_firmware.py`, `GCS/frontend/js/app.js`, `GCS/frontend/js/hud.js`, `tests/test_pad_scramble.py`
* **Technical Blueprint**:
  - Drones initialize cold on numbered ground pads ($Z = 0.05\text{m}$, props stationary, `flight_state = "GROUND_IDLE"`).
  - Umbilical spool retracted at base anchor when leader is grounded.
  - Sequenced Scramble sequence:
    1. Flagship #0 spins up and ascends vertically to $Z = 20\text{m}$ muster station, deploying umbilical tether.
    2. Escorts spool up staggered every 1.2s, climb along safe departure corridors, and converge into formation slots around flagship at RTB station.
  - Standby Reinforcement Staging: Drones stage cold on pad in `GROUND_IDLE` with pre-briefing text ("FLEET_ACTIVE: Gang in contact at [pos]" vs "FLEET_LOST: Gang lost at LKP"). Subsequent operator "launch ok" (`TAKEOFF_DRONE`) spools motors to 5200 RPM and launches drone along pad corridor.
  - GCS command `SCRAMBLE_SWARM`, `COLD_RESET`, and `SPAWN_REINFORCEMENT` with tactical HUD buttons.
* **Targeted Test**: `python -m unittest tests/test_pad_scramble.py`
