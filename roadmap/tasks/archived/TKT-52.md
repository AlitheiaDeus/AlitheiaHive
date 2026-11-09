# [TKT-52] Tactical GNSS Denial & Dead-Reckoning RTB Failsafe (Reciprocal Heading Emergency Return)

* **Grade**: 🚀 Feature | **Type**: Autonomous Failsafe / Electronic Warfare Defense (`Completed`) | **Priority**: High
* **Target Files**: `SIM/backend/sitl_engine.py`, `SIM/backend/mavlink_bridge.py`, `OBC/backend/drone_firmware.py`, `OBC/backend/brains/base.py`, `OBC/backend/brains/leader_follower.py`, `OBC/backend/brains/gossip_consensus.py`, `GCS/frontend/js/hud.js`, `GCS/frontend/css/style.css`, `tests/test_gnss_denial.py`
* **Technical Blueprint**:
  - **Part 1: The Tactical Threat & Failure Mechanics**:
    - When GNSS/RTK is jammed or spoofed, satellite fixes drop (`fix_type = 0`, satellites = 0) and position error explodes.
    - If drones in close formation attempt to hold tight geometric slots on corrupted GPS, pairwise drift causes catastrophic mid-air prop collisions.
    - God-Mode / EW Jamming trigger: SIM Lab button `[JAM GNSS // KILL GPS]` (or API command `TOGGLE_GNSS_DENIAL`) flags `drone.gnss_denied = True`.
  - **Part 2: Formation Breakout & Separation Corridor**:
    - The instant GNSS lock is lost, the affected drone aborts formation slot tracking.
    - It immediately steps down $\Delta Z = -2.0\text{m}$ (with a minimum safety altitude floor $Z \ge 5.0\text{m}$) and applies a lateral breakout bias away from neighboring airframes to prevent collisions during drift.
  - **Part 3: Reciprocal Heading Dead-Reckoning Guidance**:
    - The drone locks its last known stable compass heading $\psi$.
    - It computes the reciprocal escape vector $\psi_{\text{reciprocal}} = (\psi + 180^\circ) \pmod{360^\circ}$ back along its entry track.
    - Autopilot drives constant velocity $v_{\text{dead_reckoning}} = 8.0\text{ m/s}$ along this reciprocal heading vector.
  - **Part 3b: RF Beacon Gradient Homing & "Cheater No Cheating" Invariant**:
    - **Strict Invariant: Cheater No Cheating**: During GNSS denial, the drone's OBC is strictly forbidden from reading true spatial coordinates (`drone.pos` or `sim.target_pos`) from simulation memory. It has NO GPS!
    - **Inverse-Square Law Hot/Cold Gradient**: The drone tracks previous scalar RSSI: $\Delta \text{RSSI} = \text{RSSI}(t) - \text{RSSI}(t - \Delta t)$.
    - If $\Delta \text{RSSI} > 0$ (warmer / signal strengthening), current dead-reckoning heading is converging toward the GCS mast; maintain track.
    - If $\Delta \text{RSSI} < 0$ (colder / signal fading), inverse-square law indicates moving away; drone applies a gentle yaw bias toward the RF signal peak.
    - Near-field threshold: Once near-field beacon strength is reached ($\text{RSSI} \ge -45\text{ dBm}$, roughly $d \le 15\text{m}$), the drone halts horizontal transit and enters the blind soft settle phase.
  - **Part 4: MAVLink Glitch Injection & GCS Alarm**:
    - `mavlink_bridge.py` drops `GPS_RAW_INT` (#24) to `fix_type = 0`, `satellites_visible = 0`, and broadcasts `STATUSTEXT(MAV_SEVERITY_CRITICAL, "GNSS DENIED: DEAD-RECKONING RTB")`.
    - GCS telemetry card flashes pulsing amber alert `[GNSS DENIED]`, badge shows `DEAD-RECKONING RTB`.
  - **Part 5: Timeout & Emergency Blind Settle**:
    - If GNSS lock is not restored within $25.0\text{s}$ and the drone has navigated back within $15\text{m}$ of Base (or timeout elapses), it zeros horizontal velocity and executes an automated $-0.8\text{ m/s}$ blind vertical descent to safely ground the airframe before drift causes a flyaway.
* **Targeted Test**: `python -m unittest tests/test_gnss_denial.py`
* **Verification Status**: ✅ Verified (8/8 targeted unit tests passing in 0.60s; 184 full test suite discovery passing in 12.36s; smoke test passing in 0.10s)
