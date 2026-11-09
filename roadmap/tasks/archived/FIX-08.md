# [FIX-08] Coordinated ATC Landing Station-Keeping: Prevent Map-Edge Radial Scatter via 3D Pad Columns

* **Grade**: ðŸ› ï¸ Improvement | **Type**: Bug Remediation / Kinematic Recovery (`Completed`) | **Priority**: High
* **Target Subsystems**: `SIM/backend/sitl_engine.py`, `tests/test_rtb_landing.py`
* **Technical Blueprint**:
  - **Symptom: Drones Scatter to Map Edges During Coordinated Landing**:
    - When `command_land_swarm()` is called, all drones enter `flight_state = "DESCENDING"`.
    - Both `LeaderFollowerBrain` and `GossipConsensusBrain` skip drones in `DESCENDING`, dropping horizontal guidance forces to `[0, 0, 0]`.
    - In `sitl_engine.py:659-664`, during `drone.landing_wait_timer > 0.0` (`WAITING_CLEARANCE` queue), the engine sets **only** vertical acceleration `guidance_accs[drone.id][2] = az_hold`.
    - **Horizontal guidance is zero**. No position restoration or velocity braking exists.
    - Pairwise `_compute_4zone_avoidance()` detects the clustered waiting drones ($d < 3.5\text{m}$ / $d < 8\text{m}$) overhead and pushes them radially outward.
    - With zero horizontal braking, the evasion force accelerates them up to $12\text{ m/s}$ unopposed, scattering them across the geofence perimeter during the 2s to 14s clearance wait.
  - **Architectural Solution**:
    1. **Active Overhead Column Holding**: In `step()` under `if drone.landing_wait_timer > 0.0:`, provide horizontal holding guidance directly above the drone's assigned recovery pad:
       $$\mathbf{a}_{\text{hold, xy}} = \text{clip}\left(2.5 \cdot (\mathbf{p}_{\text{pad, xy}} - \mathbf{p}_{\text{drone, xy}}) - 3.0 \cdot \mathbf{v}_{\text{drone, xy}}, -6.0, 6.0\right)$$
    2. **Staggered Vertical Shelves**: Separate holding altitudes by $2.0\text{m}$ per pad ($Z = 22.0 + \text{pad\_idx} \times 2.0\text{m}$), ensuring hovering drones stay outside each other's Zone 3 danger envelope ($> 3.5\text{m}$) during clearance waits.
    3. **Clean Touchdown Alignment**: When `landing_wait_timer` reaches 0, the drone is already perfectly positioned above its pad for a vertical descent.
* **Targeted Test**: `python -m unittest tests/test_rtb_landing.py`
