# [FIX-07] MAVLink Multi-Vehicle Telemetry: GPS Track COG, Escort Slot Waypoints & Multi-Home Broadcast

* **Grade**: ðŸ› ï¸ Improvement | **Type**: Bug Remediation / External MAVLink C2 (`Completed`) | **Priority**: Very High
* **Target Subsystems**: `SIM/backend/mavlink_bridge.py`, `tests/test_mavlink_bridge.py`
* **Technical Blueprint**:
  - **Symptom 1: Infinite North Black Line (`GPS Track (Black)`)**:
    - Mission Planner reads Course Over Ground (`cog` / `gpshdg`) from `GPS_RAW_INT` (#24). Because packet #24 was missing, Mission Planner defaulted `cog = 0` ($0^\circ$ True North).
    - **Solution**: Implement `gps_raw_int_encode` at 5/10 Hz with true Course Over Ground calculated from velocity:
      $$\text{cog} = \left(90^\circ - \text{atan2}(v_y, v_x) \times \frac{180}{\pi}\right) \pmod{360} \times 100$$
      with `fix_type = 3` (3D GPS Fix) and `satellites_visible = 16`.
  - **Symptom 2: Escort Orange Lines Converging on Base ("Point Alpha")**:
    - In `sitl_engine.py`, only Drone #0 ingests global waypoint coordinates into `acknowledged_target`. Escorts follow formation slot offsets relative to the leader, so their `acknowledged_target` buffer remained frozen at spawn Base `[0, 0, 20]`.
    - **Solution**: In `mavlink_bridge.py`, calculate escort targets dynamically using their formation station:
      $$\mathbf{p}_{\text{target, escort}} = \mathbf{p}_{\text{leader}} + \mathbf{R}(\psi_{\text{formation}}) \cdot \mathbf{s}_{\text{offset}}$$
      giving each escort an accurate `target_bearing` and finite `wp_dist` pointing directly to its formation slot.
  - **Symptom 3: "Failed to update home location (2)"**:
    - Mission Planner writes home coordinates to every discovered vehicle `sysid` individually upon connection. When Vehicle 2 was queried, home broadcasts were only sent under sysid 1.
    - **Solution**: Broadcast `HOME_POSITION` (#242) for each active vehicle `sys_id` ($1 \dots N$) and respond to home commands with matching `src_sys = target_sys`.
  - **Symptom 4: Screen Clutter / Super Long Vectors**:
    - In `NAV_CONTROLLER_OUTPUT`, clamp `wp_dist` to a max tactical threshold (e.g. $\min(d_{\text{wp}}, 50\text{m})$).
* **Targeted Test**: `python -m unittest tests/test_mavlink_bridge.py`
