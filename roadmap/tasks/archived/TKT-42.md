# [TKT-42] Batched ATC Pad Recovery & Downwash Clearance Cycles

* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / Recovery & Air Traffic Control (`Completed`) | **Priority**: High
* **Target Files**: `SIM/backend/sitl_engine.py`, `GCS/backend/c2_gateway.py`, `GCS/frontend/js/hud.js`
* **Technical Blueprint**:
  - Air Traffic Control (ATC) state machine with configurable pad capacity ($N_{\text{pads}} = 8$).
  - Downwash anti-vortex gate: Drone $k+1$ is altitude-held above ground effect until Drone $k$ confirms `TOUCHDOWN_CONFIRMED` ($Z \le 0.1\text{m}$, speed $\le 0.05\text{ m/s}$, motors disarmed). Zero downwash collisions.
  - Wave management: Once 8 pads are filled, remaining drones hold in circular orbit at overhead marshal shelves ($Z \in [25, 35]\text{m}$).
  - Base Horizon High-Gain RF Dome: Within $50\text{m}$ radius of base, high-gain base station radio guarantees 100% link quality.
  - Operator `[CLEAR PADS // NEXT WAVE]` button (or 12s countdown timer) clears pads and releases the next wave of 8 drones for landing.
* **Targeted Test**: `python -m unittest tests/test_rtb_landing.py`
