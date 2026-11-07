# [TKT-38] Hardware Execution Target Provider: Telemetry Modem & Radio Bridge

* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / Hardware Integration (`Backlog`) | **Priority**: Very Low
* **Target Files**: `backend/providers/hardware_provider.py`, `backend/c2_gateway.py`
* **Technical Blueprint**:
  - Settles PLN-12.
  - Pure lategame hardware adapter.
  - Ingests live telemetry from USB serial radios (RFD900x / Holybro SiK / ExpressLRS) or 4G LTE cellular VPN bridges, routing physical MAVLink packets directly into the 3D WebGL tactical viewport.
* **Targeted Test**: Hardware telemetry bridge mock test.

---


## 5. Settled & Completed Deliverables (Reference Ledger)

* `[FIX-01]` Target Drop-Line Ground Plane Penetration (`app.js`)
* `[FIX-02]` GCS Failsafe Banner & Post-Severance Flight State Normalization (`hud.js`, `sim_core.py`)
* `[FIX-03]` Collapsible Panel Edge Toggles & Off-Screen Restoration Handles (`index.html`, `style.css`, `hud.js`)
* `[TKT-25]` Ground Truth vs Autopilot Decoupling: Onboard Target & Uplink Mailbox (`sim_core.py`, `brains/`, `web_server.py`)
* `[TKT-26]` Dual Target C2 Visuals: Commanded Waypoint vs Swarm Acknowledged (`app.js`, `hud.js`)
* `[TKT-27]` Altitude-Gated Breakaway Dive & Umbilical Jettison State Machine (`sim_core.py`, `leader_follower.py`, `app.js`)
* `[TKT-28]` Discrete Waypoint Mission Queue & Custom Route Deployment (`sim_core.py`, `brains/`, `app.js`, `hud.js`)
* `[TKT-29]` Tactical RNG Touch-and-Go Reconnaissance Generator (`sim_core.py`, `hud.js`, `index.html`)
* `[TKT-30]` Per-Waypoint Loiter Timers & Coordinated Formation Arrival (`drone_firmware.py`, `c2_gateway.py`, `hud.js`)
* `[TKT-31]` Telemetry-Gated C2 Downlink Handshake & Uplink Failure Hold (`c2_gateway.py`, `hud.js`)
* `[TKT-32]` Persistent Telemetry Session & Reconnect State Hydration (`c2_gateway.py`, `hud.js`, `app.js`)
* `[TKT-33]` The 3-Bubble Decoupling Architecture: Folder Restructuring (GCS/, OBC/, SIM/) & Local Port Protocol
* `[TKT-34]` Top-Layer Multi-Process Orchestrator (`run_with_sim.py`, `run.ps1`)
* `[TKT-40]` Multi-Vehicle MAVLink Telemetry Broadcast & Swarm System ID Demuxing (`mavlink_bridge.py`)
* `[TKT-46]` Two-Way MAVLink C2 Command Ingestion & Mission Planner Control Bridge (`mavlink_bridge.py`, `sitl_engine.py`)
