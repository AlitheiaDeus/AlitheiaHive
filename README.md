# AlitheiaHive: v3-Uprising

<p align="center">
  <strong>Autonomous Drone Swarm Simulator & Hardware-Accelerated 3D WebGL Tactical GCS</strong><br>
  <em>Built on a high-performance 50/50 Architecture across 4 Strict Airgaps: Pure Python Kinematics & GPU-Native Three.js Rendering.</em>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.12-blue?logo=python&logoColor=white" alt="Python 3.12">
  <img src="https://img.shields.io/badge/WebGL-Three.js_r128-00F0FF?logo=three.js&logoColor=white" alt="Three.js">
  <img src="https://img.shields.io/badge/Architecture-4--Airgap_Decoupled-00FFAA" alt="4-Airgap Decoupled">
  <img src="https://img.shields.io/badge/Physics-60_FPS_Kinematic_Twin-FFB800" alt="60 FPS">
  <img src="https://img.shields.io/badge/Tests-117%2F117_Passing-brightgreen" alt="Tests 117/117 Passing">
  <img src="https://img.shields.io/badge/License-MIT-gray" alt="MIT License">
</p>

<p align="center">
  <img src="docs/images/alitheia_hive_operator_gcs.jpg" alt="AlitheiaHive Tactical Ground Control Station (GCS) Viewport" width="100%">
</p>

<p align="center">
  <img src="docs/images/alitheia_sitl_lab.jpg" alt="Alitheia SITL Simulation Lab & Fault Injection Deck" width="100%">
</p>

> [!TIP]
> **Deploying to Physical Hardware?**  
> Check out the **[Physical Hardware Deployment Blueprint](docs/HARDWARE_DEPLOYMENT_BLUEPRINT.md)** for a zero-fluff guide on connecting real micro-controllers (Seeed XIAO C5 / ESP32 / Pi Zero) to real hobby GPS drones over Wi-Fi, RTK, and MAVLink.

---

## 1. System Mission & The 4 Airgaps

Previous desktop swarm simulators suffered from frame drops, sluggish UI freezing, and cross-contamination between user interface inputs and physical simulation memory.

**`AlitheiaHive_v3-Uprising`** solves this by strictly enforcing a **50/50 Architecture** across **4 Explicit Airgaps**:
1. **The Operator Airgap (Tactical GCS Terminal)**: Renders only downlinked telemetry packets (`TELEMETRY_FRAME`). Emits flight intents. Never alters ground truth physics directly.
2. **The Canyon Airgap (Packet Network Link)**: The physical and electromagnetic void between ground station and fleet. Only serialized packets (UDP 9000/9001, MAVLink UDP 14550, or WebSocket JSON) cross this boundary.
3. **The Brain Airgap (Isolated Drone OBC Fleet)**: Each airframe runs an isolated Onboard Computer (OBC) bubble. Drones steer only to authenticated local target buffers, maintaining autonomous failsafes (Doctrine A loiter, automated breakaway dive, RTB soft landing) on comms loss.
4. **The God-View / Sim Airgap (SITL World Simulator & Test Harness)**: Ground truth physics engine running an **ideal Flight Controller (FC) digital twin** (60 FPS kinematics, velocity clamps, yaw slew rate limiting, tilt angle coupling). Strictly separates operator flight commands from simulation fault injections.

---

## 2. Key Capabilities & Technical Features

### Sane 4-Zone Spatial Safety (No "Truck-Sized" Bubbles)
True-to-scale physics with metric dimensions:
* **Airframe Scale**: $R_{\text{body}} = 0.75\text{m}$ (Wingspan $= 1.5\text{m}$).
* **Zone 1 ($d > 8.0\text{m}$)**: Nominal flight toward formation slots or waypoints. Zero repulsion.
* **Zone 2 ($3.5\text{m} < d \le 8.0\text{m}$)**: Smooth linear separation steering effort.
* **Zone 3 ($1.5\text{m} < d \le 3.5\text{m}$)**: **Active Relative-Velocity Damping** brakes incoming momentum and prevents discrete pinball penetration:
  $$\mathbf{a}_{\text{evade}} = \left[ 12.0 + 8.0 \cdot \left(\frac{3.5 - d_{ij}}{3.5 - 1.5}\right) \right] \hat{\mathbf{n}}_{ij} - 1.5 (\mathbf{v}_i - \mathbf{v}_j)_{\parallel}$$
* **Zone 4 ($d \le 1.5\text{m}$)**: Prop clash threshold. Motors cut (`crashed = True`), airframe enters ballistic gravity drop ($g = -9.81\text{m/s}^2$).

### Ideal Flight Controller (FC) Kinematic Digital Twin
Bypasses the CPU bloat and numerical instability of non-linear blade element aerodynamics in favor of an ideal, deterministic FC twin:
* **Velocity & Acceleration Clamps**: $v_{\text{cruise}} = 12.0\text{ m/s}$, $v_{\text{max}} = 20.0\text{ m/s}$, $a_{\text{max}} = 15.0\text{ m/s}^2$.
* **Attitude Tilt Coupling**: $\tan\theta = a_x/g$, $\tan\phi = a_y/g$, with angular slew rate limit $|\dot{\theta}|, |\dot{\phi}| \le 250^\circ/\text{s}$.
* **Yaw Slew Rate Limiting**: Maximum turning rate $\le 45^\circ/\text{s}$ with arrival deadbands ($0.25\text{m}$).

### Pluggable Guidance Brains (Centralized & Decentralized)
* **`LeaderFollowerBrain`**: Star topology where flagship transmits heartbeats and followers hold staggered formation slots.
* **`GossipConsensusBrain`**: Fully decentralized leaderless P2P mesh network. Target waypoints propagate via multi-hop gossip echo waves until a $75\%$ quorum consensus barrier is achieved, prompting simultaneous swarm maneuver.
* Hot-swappable at runtime with zero physics hitching.

### Discrete Waypoint Mission Queue (TKT-28)
* Lightweight discrete 3D/4D waypoint lists (`[x, y, z, loiter_s]`) deployed via atomic `UPLOAD_MISSION_ROUTE` packets.
* Presets: `BOX_PATROL`, `DIAMOND_SURVEY`, `HEXAGON_SWEEP`.
* Fleet OBC stores waypoint queues and advances upon reaching acceptance radius $R_{\text{accept}} = 2.5\text{m}$.
* Three.js renders dynamic connected 3D neon polyline ribbons.

### Tactical RNG Touch-and-Go Reconnaissance (TKT-29)
* **Deterministic Handshake**: Fleet emits `target_reached = True` (MAVLink `MISSION_ITEM_REACHED`) ping upon arrival.
* **Telemetry-Gated Dwell**: GCS executes 2.0s tactical sensor dwell, then rolls high-variance SAFE coordinates ($\Delta d \ge 15.0\text{m}$) outside the 3m tether cylinder.
* **Uplink Failure Hold**: GCS freezes beacon and halts re-rolls when link drops.

### Tri-Mode Comms Architecture & Contested RF Simulation (TKT-17)
* **`TETHERED_RELAY`**: Hardline umbilical to Drone #0; immune to GCS uplink jamming; 3m clearance cylinder enforced.
* **`LEADER_FOLLOWER`**: Untethered wireless star; subject to $120\text{m}$ GCS radio range and EW jamming.
* **`GOSSIP_CONSENSUS`**: Untethered decentralized P2P mesh; multi-hop gossip wave with quorum consensus.
* **RF Physics**: Quadratic path loss $P_{\text{loss}}(d) = \gamma + (1-\gamma) \cdot 0.25 \cdot (d/R_{\text{max}})^2$ with operator EW jamming slider ($\gamma \in [0.0, 0.5]$).

### Catenary Tether Dynamics, Breakaway Dive & Succession (TKT-16 / TKT-27)
* Real-time 16-point catenary curve calculating gravitational sag from anchor $(0, 0, 1.5\text{m})$.
* **Altitude-Gated Breakaway**: On cable snip, Drone #0 performs an altitude-gated breakaway dive ($\Delta Z \ge 4.0\text{m}$, lateral peel $6.0\text{ m/s}$), jettisons umbilical over 1.5s GPU dissolve, and climbs smoothly to rejoin wing escort slot with zero prop clashes.
* **Fleet Doctrine A**: Promoted 2IC holds 30s defensive loiter before executing automated `SWARM_RTB`.

### 2-Stage Soft Landing & Coordinated ATC Pad Allocation (TKT-19)
* **Stage 1 (`OVERHEAD_MARSHAL`)**: Sentry ring at $Z = 25\text{m}$, $R = 20\text{m}$ outside the 3m tether cylinder.
* **Stage 2 (`AUTHORIZED_LAND`)**: ATC pad allocation with 2.0s clearance stagger intervals, 2-stage vertical deceleration ($-1.5\text{ m/s} \rightarrow -0.5\text{ m/s}$), and touchdown disarm (0 RPM, ground-locked).

### GPU-Native WebGL Tactical GCS & Dual-Target C2 (TKT-26)
* **Commanded Target (Hollow Cyan Diamond)**: Tracks operator mouse clicks, waypoint presets, and continuous Fig-8 patrol route in real time.
* **Swarm Acknowledged Target (Solid Emerald Diamond)**: Tracks actual waypoint confirmed in active OBC memory.
* **Visual Divergence**: When uplink is severed or jammed, Commanded marker moves away while Swarm Acknowledged marker remains frozen at the Doctrine A loiter point.
* **HUD Features**: 3D billboard tactical ID badges, dynamic rotor spin and color changes, click-to-focus chase cam, and HTML5 Gamepad API override (RadioMaster / EdgeTX).

### UDP Network Bridge & MAVLink Loopback
* **Port 9000 (TX)**: Non-blocking UDP broadcast of swarm telemetry JSON array.
* **Port 9001 (RX)**: External UDP command listener.
* **Port 14550 (MAVLink v2 Loopback)**: Emits binary MAVLink `HEARTBEAT` and `GLOBAL_POSITION_INT` messages with GPS projection for integration with QGroundControl and SITL tools.

---

## 3. Quick Start & Execution

### Prerequisites
* Windows 10/11
* Python 3.12 (Anaconda / Miniconda environment recommended)

### One-Click Startup (PowerShell)
```powershell
# Run the automated headless smoke test (exits code 0 in ~0.15s)
.\run.ps1 -SmokeTest

# Launch simulation backend and auto-open WebGL GCS in default browser
.\run.ps1
```

### Direct Python Invocation
```powershell
# Interactive mode (opens browser to http://127.0.0.1:8080)
python run.py

# Custom swarm size or port
python run.py --count 12 --port 8080

# Headless verification mode
python run.py --smoke-test
```

---

## 4. Automated Test Suite

Run the full automated unit test suite covering kinematics, 4-zone safety, tether dynamics, pluggable brains, mission routes, RNG recon, and network bridges:

```powershell
python -m unittest discover tests
# or via launcher:
.\run.ps1 -UnitTest
```

```text
Ran 117 tests in 5.394s

OK
```

---

## 5. Repository Topology

<!-- TREE_START -->
```text
v3-Uprising/
├── docs/
│   ├── images/
│   │   ├── alitheia_hive_operator_gcs.jpg
│   │   ├── alitheia_sitl_lab.jpg
│   │   └── tactical_gcs_hero.png
│   └── HARDWARE_DEPLOYMENT_BLUEPRINT.md
├── GCS/
│   ├── backend/
│   │   ├── __init__.py
│   │   ├── c2_gateway.py
│   │   ├── session_manager.py
│   │   └── web_server.py
│   ├── frontend/
│   │   ├── css/
│   │   ├── js/
│   │   └── index.html
│   └── __init__.py
├── OBC/
│   ├── backend/
│   │   ├── brains/
│   │   ├── __init__.py
│   │   └── drone_firmware.py
│   ├── frontend/
│   └── __init__.py
├── roadmap/
│   ├── tasks/
│   │   ├── archived/
│   │   ├── backlog/
│   │   ├── BUG-01.md
│   │   ├── BUG-02.md
│   │   ├── BUG-03.md
│   │   ├── BUG-04.md
│   │   ├── BUG-05.md
│   │   ├── BUG-06.md
│   │   ├── FIX-05.md
│   │   ├── FIX-06.md
│   │   ├── FIX-07.md
│   │   ├── FIX-08.md
│   │   ├── FIX-09.md
│   │   ├── FIX-10.md
│   │   ├── PLN-02.md
│   │   ├── TKT-23.md
│   │   ├── TKT-41.md
│   │   ├── TKT-42.md
│   │   ├── TKT-43.md
│   │   ├── TKT-48.md
│   │   ├── TKT-49.md
│   │   ├── TKT-50.md
│   │   ├── TKT-52.md
│   │   ├── TKT-53.md
│   │   ├── TKT-54.md
│   │   ├── TKT-55.md
│   │   ├── TKT-56.md
│   │   ├── TKT-57.md
│   │   ├── TKT-58.md
│   │   └── TLK-01.md
│   └── kanban.md
├── SIM/
│   ├── backend/
│   │   ├── __init__.py
│   │   ├── fleet_registry.py
│   │   ├── mavlink_bridge.py
│   │   ├── sitl_engine.py
│   │   └── telemetry_recorder.py
│   ├── frontend/
│   │   ├── css/
│   │   ├── js/
│   │   └── index.html
│   └── __init__.py
├── tests/
│   ├── recordings/
│   │   └── formation_flight_suite.json
│   ├── __init__.py
│   ├── quarantine.py
│   ├── test_airgap_integrity.py
│   ├── test_brains.py
│   ├── test_breadcrumb_backtrack.py
│   ├── test_bubble_decoupling.py
│   ├── test_c2_downlink_gate.py
│   ├── test_fleet_registry.py
│   ├── test_formation_flight_suite.py
│   ├── test_formation_headings.py
│   ├── test_gnss_denial.py
│   ├── test_gossip_formations.py
│   ├── test_in_flight_enrollment.py
│   ├── test_mavlink_bridge.py
│   ├── test_mission_routes.py
│   ├── test_orchestrator.py
│   ├── test_pad_scramble.py
│   ├── test_rng_recon.py
│   ├── test_rtb_landing.py
│   ├── test_session_hydration.py
│   ├── test_sim_core.py
│   ├── test_succession_deconfliction.py
│   ├── test_tether_failsafes.py
│   ├── test_tier_registry_auth.py
│   ├── test_tri_mode_rf.py
│   ├── test_waypoint_engine.py
│   └── test_web_server.py
├── .gitignore
├── LICENSE
├── README.md
├── run.py
└── run_with_sim.py
```
<!-- TREE_END -->

---

## 6. Controls Reference

| Control | Action |
| :--- | :--- |
| **Left-Click + Drag** | Orbit camera around tactical space |
| **Right-Click + Drag** | Pan tactical camera |
| **Scroll Wheel** | Zoom in / out |
| **Left-Click on Ground** | Dispatch target waypoint to swarm |
| **`[C]` Key** | Toggle Chase Cam mode |
| **`[R]` Key** | Reset camera to tactical overview |

---

## 7. License

Distributed under the MIT License. Built for autonomous multi-agent robotics.
