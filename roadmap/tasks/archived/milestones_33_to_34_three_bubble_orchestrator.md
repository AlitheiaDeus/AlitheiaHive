# Settled Milestones: 3-Bubble Architecture & Process Orchestrator (TKT-33, TKT-34)

> **Status**: Settled & Verified | **Milestone**: Bubble Decoupling

### `[TKT-33]` The 3-Bubble Decoupling Architecture: Folder Restructuring (GCS/, OBC/, SIM/) & Local Port Protocol
* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / Master Architecture (Settled) | **Priority**: High
* **Target Directory Topology**:
  ```
  SwarmTech3_UpRising/
  â”œâ”€â”€ GCS/                            # Bubble 1: Ground Control Station (The Operator Airgap)
  â”‚   â”œâ”€â”€ backend/                    # C2 mission gateway, routes, RNG recon, web server
  â”‚   â””â”€â”€ frontend/                   # Operator WebGL tactical 3D viewport (Three.js / Cyberpunk HUD)
  â”œâ”€â”€ OBC/                            # Bubble 2: Onboard Computer Fleet (The Brain Airgap)
  â”‚   â”œâ”€â”€ backend/                    # Drone OBC firmware, pluggable brains, local estimators
  â”‚   â””â”€â”€ frontend/                   # (Staged / Late-game: OBC diagnostics & telemetry log viewer)
  â”œâ”€â”€ SIM/                            # Bubble 3: SITL World Simulator (The God-View / Sim Airgap)
  â”‚   â”œâ”€â”€ backend/                    # Kinematic FC twin, 4-zone safety, tether dynamics, RF channel, casualty lab
  â”‚   â””â”€â”€ frontend/                   # (God-view omniscient truth viewport & debug overlay)
  â”œâ”€â”€ run_with_sim.py                 # Top-layer multi-process orchestrator
  â””â”€â”€ tests/                          # Automated unit test suite
  ```
* **Technical Blueprint**:
  - Restructure project folders into the 3 explicit bubbles:
    1. **Bubble 1: Ground Control Station (`GCS/`)**:
       - `GCS/backend/`: Pure operator C2 layer. Ingests downlinked telemetry; emits operator flight intents (`UPLOAD_ROUTE`, `SET_FORMATION`, `DISPATCH_TARGET`). Owns mission queues, RNG recon generator, and auto-patrol trajectory math. Serves WebSocket `/ws` on port 8080.
       - `GCS/frontend/`: The operator's terminal! Three.js 3D tactical viewport and cyberpunk HUD.
    2. **Bubble 2: Swarm Fleet OBC (`OBC/`)**:
       - `OBC/backend/`: Autonomous drone firmware running in isolated OBC bubbles. Holds only local sensor inputs (estimated GPS/IMU from FC twin) and local `acknowledged_target` buffer. Executes local autonomous failsafes (Doctrine A loiter, breakaway dive, RTB soft landing) when comms drop. Inter-drone communication occurs strictly over simulated local P2P gossip mesh packets.
       - `OBC/frontend/`: Staged for late-game flight log extraction and OBC diagnostics.
    3. **Bubble 3: SITL World Simulator (`SIM/`)**:
       - `SIM/backend/`: Owns ground truth physics: 60 FPS kinematics (ideal FC twin), 4-zone safety, Zone 4 prop clashes ($d \le 1.5\text{m}$), physical tether catenary & tensile snap, and quadratic RF path loss with EW jamming. Contains God-Mode casualty test harness (`snip_tether`, `kill_drone`, `jam_rf`). Listens for commands on UDP 9001; broadcasts truth telemetry on UDP 9000 and MAVLink UDP 14550.
       - `SIM/frontend/`: God's omniscient perspective! Renders unfiltered true physical positions, ghost collision spheres, and RF dead-zone boundaries (settles PLN-14 / TKT-35).
  - **Local Port Mapping**:
    - `UDP 9000`: Telemetry Downlink (TX from SIM / Fleet)
    - `UDP 9001`: Command Uplink (RX into SIM / Fleet)
    - `UDP 14550`: MAVLink v2 binary stream
    - `TCP 8080`: HTTP GCS assets & WebSocket `/ws`
* **Targeted Test**: `python -m unittest discover tests`

---

### `[TKT-34]` Top-Layer Multi-Process Orchestrator (`run_with_sim.py`, `run.ps1`)
* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / Process Orchestration (Settled) | **Priority**: High
* **Target Files**: `run_with_sim.py`, `run.ps1`, `run.py`
* **Technical Blueprint**:
  - Independent bubbles do NOT spawn each other. A top-layer orchestrator launches and monitors the processes in order:
    1. Boot SITL World Engine daemon (`python -m backend.sitl_engine --port 9000 --listen 9001`).
    2. Boot Swarm Fleet OBC worker daemon (`python -m backend.drone_obc`).
    3. Boot Tactical GCS C2 Gateway & Web Server (`python -m backend.c2_gateway --port 8080`).
    4. Auto-launch default browser to `http://127.0.0.1:8080`.
  - **Process Lifecycle & Signal Forwarding**:
    - Orchestrator traps `SIGINT` (Ctrl+C) and terminates child processes cleanly without leaving orphaned background ports open on Windows.
  - **Modular Execution Modes**:
    - `python run_with_sim.py`: Standard full-stack launch.
    - `python run_with_sim.py --gcs-only`: Launches only GCS C2 Gateway (for connecting to external SITL or hardware).
    - `python run_with_sim.py --sim-only`: Launches only SITL World Engine in headless simulation mode.
    - `python run_with_sim.py --smoke-test`: Headless verification smoke test (exits code 0 in ~0.2s).
* **Targeted Test**: `python run_with_sim.py --smoke-test`
