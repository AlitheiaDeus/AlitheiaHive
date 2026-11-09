# [FIX-06] MAVLink Multi-Vehicle Params & Home Responder, True North Heading, and Echelon Agreement

* **Grade**: 🛠️ Improvement | **Priority**: Very High | **Status**: Completed
* **Target Subsystems**: `SIM/backend/mavlink_bridge.py`, `OBC/backend/brains/gossip_consensus.py`, `SIM/backend/sitl_engine.py`, `OBC/backend/brains/leader_follower.py`, `GCS/frontend/js/hud.js`
* **Targeted Test**: `python -m unittest tests/test_mavlink_bridge.py tests/test_tether_failsafes.py`

## 1. Context & Operational Inspiration
Mission Planner hung on vehicle 2 params because `PARAM_REQUEST_LIST` was unhandled. Home updates failed because `COMMAND_LONG` returned unsupported for cmd 179 and ACK `srcSystem` did not match commanded sysid. Infinite yellow stick was Mission Planner's "Direct to current WP" line defaulting to bearing 0 due to missing `NAV_CONTROLLER_OUTPUT`. Loiter timer lingered in P2P because severance flag wasn't cleared on mode switch. Natural boids clashed with tether due to 2D anchor check and goal migration powering repulsion.

## 2. Technical Blueprint & Acceptance Criteria
- [x] **MAVLink Multi-Vehicle Params**: Responds to `PARAM_REQUEST_LIST` (#21) and `PARAM_REQUEST_READ` (#20) with valid `PARAM_VALUE` packets for all system IDs (`sysid = 1, 2, ...`), closing Mission Planner "Getting Params..." dialog instantly.
- [x] **MAVLink Home Location**: Accepts `SET_HOME_POSITION` (#243), `MAV_CMD_DO_SET_HOME` (#179) and `MAV_CMD_GET_HOME_POSITION` (#410) with `COMMAND_ACK` (#77) sent with `srcSystem == target_sys`, setting dynamic home coordinates and broadcasting `HOME_POSITION`, eliminating "Failed to update home location (2)" popups.
- [x] **Waypoint Vectors & Yellow Sticks**: Emits `NAV_CONTROLLER_OUTPUT` (#62) at 10 Hz with calculated `target_bearing` and finite `wp_dist` to active waypoint, replacing infinite North yellow sticks in Mission Planner with true waypoint lines; maps NED velocities and clockwise heading from True North [0..35999 cdeg].
- [x] **Tether Failsafe & Loiter Reset**: Switching comms mode to `GOSSIP_CONSENSUS` or `LEADER_FOLLOWER` clears the tether severance loiter timer and hides severed tether state on GCS HUD; re-attaching tether is rejected if Drone #0 is destroyed.
- [x] **Boids 3D Tether Clearance**: Computes 3D segment distance from tether umbilical and inhibits inward goal forces, preventing follower collisions with the cable during natural flocking.
