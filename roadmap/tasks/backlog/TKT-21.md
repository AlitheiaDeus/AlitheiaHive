# [TKT-21] Dynamic Power Envelope: Battery Drain Curves & Low-Power Failsafes

* **Grade**: ðŸŽ­ Gimmick (Parked / Frozen) | **Type**: Chemistry & Energy Modeling (`Frozen`) | **Priority**: Very Low
* **Status**: **FROZEN** (Parked in Frozen column to eliminate chemistry bloat and focus on C2 / Swarm coordination).
* **Target Files**: `backend/sitl_engine.py`, `backend/drone_obc.py`, `frontend/js/hud.js`
* **Technical Blueprint**:
  - Non-linear LiPo discharge model: $P(t) = P_{\text{avionics}} + P_{\text{hover}} \left(\frac{\|\mathbf{T}\|}{mg}\right)^{1.5} + P_{\text{aero}}(\mathbf{v})$.
  - Tether power continuous ($100\%$ locked) vs internal battery failover upon umbilical cut.
  - 2-Stage Low-Power Failsafe: Advisory Warning ($\le 25\%$), Critical Emergency Soft-Landing ($\le 10\%$).
* **Targeted Test**: `python -m unittest tests/test_rtb_landing.py`
