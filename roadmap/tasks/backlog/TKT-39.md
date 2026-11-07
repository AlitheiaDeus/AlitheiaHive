# [TKT-39] Standalone Drone OBC Network Daemon & HITL Socket Runner for Raspberry Pi Zero 2W

* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / Hardware-in-the-Loop & Embedded Decoupling (`Backlog`) | **Priority**: Medium
* **Target Files**: `OBC/runner.py`, `OBC/backend/drone_firmware.py`, `SIM/backend/sitl_engine.py`
* **Technical Blueprint**:
  - Standalone entrypoint for running an autonomous OBC bubble on a physical Raspberry Pi Zero 2W or independent machine:
    `python -m OBC.runner --drone-id <ID> --sim-ip <IP> --gcs-ip <IP> --mode [hitl|sitl|flight]`
  - Hardware-in-the-Loop (HITL) socket architecture:
    1. Ingests synthetic sensor UDP frames from `SIM` (IMU, baro, GPS, RF RSSI).
    2. Executes onboard state estimation and pluggable brains (`leader_follower.py`, `gossip_consensus.py`) directly on the Pi's ARM CPU.
    3. Emits motor velocity demand UDP packets back to `SIM` (or physical ESCs / serial autopilot in flight mode).
    4. Streams standard MAVLink v2 / JSON telemetry frames to GCS over UDP (ports 9000/14550).
  - Proves 100% plug-and-play modularity across the 3 bubbles:
    - Swap `GCS` with external ground stations (Mission Planner / QGroundControl).
    - Swap `OBC` with real Raspberry Pi Zero 2W hardware.
    - Swap `SIM` with external physics simulators (Gazebo / AirSim).
* **Targeted Test**: `python -m unittest tests/test_brains.py`
