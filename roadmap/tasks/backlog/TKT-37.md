# [TKT-37] SITL Execution Target Provider: ArduPilot / PX4 MAVLink Cluster

* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / SITL Integration (`Backlog`) | **Priority**: Low
* **Target Files**: `backend/providers/sitl_provider.py`, `backend/c2_gateway.py`
* **Technical Blueprint**:
  - Settles PLN-11.
  - Multi-instance SITL bridge via MAVLink UDP (port 14550), swapping internal kinematics for real autopilot firmware running in simulated GPS environments.
  - Supports `MISSION_ITEM_INT`, `COMMAND_LONG` (takeoff, land, RTL, loiter), and `PARAM_SET`.
* **Targeted Test**: Headless SITL unit test.
