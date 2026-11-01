# [TKT-54] Heartbeat Watchdog, Breadcrumb Backtrack & Zero-Range Peeking

* **Grade**: 🚀 Feature | **Type**: Comms Realism, OBC Failsafe & Airgap Hardening (`Completed / Review`) | **Priority**: Very High
* **Target Files**: `OBC/backend/drone_firmware.py`, `OBC/backend/brains/leader_follower.py`, `OBC/backend/brains/gossip_consensus.py`, `SIM/backend/sitl_engine.py`, `GCS/backend/web_server.py`, `GCS/frontend/js/hud.js`, `tests/test_breadcrumb_backtrack.py`
* **Operational Inspiration**:
  - Eliminates omniscient Euclidean RF range peeking (`is_in_comms_range()`) from GCS and SITL autonomy loops.
  - Fixes swarm panic on slight out-of-range commands: instead of abandoning missions or freezing, drones step back into coverage and report.
  - Establishes true packet-driven watchdog architecture as prerequisite for the unified Mission Engine and Courier Mule (`[TKT-51]`).
* **Technical Blueprint**:
  - **Part 1: Purge Omniscient Range Peeking from GCS**:
    - Remove direct `self.sim.is_in_comms_range()` memory inspection from `GCS/backend/web_server.py`.
    - GCS assesses link status strictly via telemetry freshness: `link_active = (t_now - last_telemetry_rx_time) <= 1.0s`.
    - SITL world engine stops overriding swarm guidance with external RTB based on Euclidean distance; failsafe decisions live strictly inside airframe OBCs.
  - **Part 2: Bidirectional Heartbeat Watchdogs**:
    - GCS broadcasts 10 Hz `HEARTBEAT` frames across the simulated RF channel (UDP 9001).
    - Each drone OBC decrements `gcs_heartbeat_watchdog` by `dt`. Valid packet arrival resets watchdog to `2.0s`.
    - Watchdog expiry marks link lost strictly through missed packets, not coordinate distance checks.
  - **Part 3: Breadcrumb Ring Buffer & Backtrack State Machine**:
    - During cruise, OBC maintains a 10-second rolling ring buffer of recent position breadcrumbs (recorded @ 2 Hz).
    - When watchdog trips during operator C2 cruise (`RADIO_SILENCE_PERMITTED = False`):
      - Airframe brakes, locks Last Known Position (LKP), and transitions to `BACKTRACK_HOLD`.
      - Guidance steers back along the breadcrumb trail (retreating 15–20m toward last-known good signal).
      - As soon as GCS heartbeat packets resume, drone stabilizes in `PERIMETER_HOLD` hover at the coverage boundary.
      - Airframe downlinks telemetry event: `[LINK RESTORED: Holding station at radio horizon. Commanded target was out of range.]`
  - **Part 4: GCS HUD Perimeter Warning**:
    - Displays amber HUD banner: `FLEET HOLDING AT RADIO HORIZON (OUT-OF-RANGE TARGET)`.
    - Allows operator to issue a closer waypoint or command `CONTINUE_AUTONOMOUS_SILENCE`.
* **Targeted Tests**: `python -m unittest tests/test_breadcrumb_backtrack.py`
