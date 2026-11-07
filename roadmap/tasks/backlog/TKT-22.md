# [TKT-22] Manual RC Joystick & Gamepad Override (RadioMaster / EdgeTX)

* **Grade**: ðŸš€ Feature | **Type**: Feature (`For Doing`) | **Priority**: Medium
* **Target Files**: `frontend/js/hud.js`, `backend/c2_gateway.py`, `backend/drone_obc.py`
* **Technical Blueprint**:
  - Ingest USB HID controllers (RadioMaster Pocket CC2500, EdgeTX, Xbox/PlayStation) via HTML5 Gamepad API.
  - 4-axis flight stick mapping with deadband ($0.08$) and exponential curve ($0.3$).
  - `MANUAL_OVERRIDE` mode streaming 60 Hz normalized axis packets to flagship while escorts track dynamically.
* **Targeted Test**: `python -m unittest tests/test_web_server.py`
