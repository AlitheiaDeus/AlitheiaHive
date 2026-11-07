# [TKT-49] Asymmetric RF Link Budget ("Yip-Back") & Mesh Telemetry Gateway

* **Grade**: ðŸš€ Feature | **Type**: Feature / RF Propagation & C2 Gateway (`Completed`) | **Priority**: High
* **Target Files**: `SIM/backend/sitl_engine.py`, `GCS/backend/c2_gateway.py`, `GCS/backend/web_server.py`, `GCS/frontend/js/hud.js`, `SIM/frontend/js/sim_app.js`, `tests/test_tri_mode_rf.py`
* **Technical Blueprint**:
  - **Part 1: Asymmetric Link Budget ($R_{\text{GCS\_TX}} = 140\text{m}$, $R_{\text{DRONE\_TX}} = 50\text{m}$, $R_{\text{P2P}} = 60\text{m}$)**:
    - High-power GCS base station mast transmits uplink flight intents out to $140\text{m}$.
    - Micro-airframe low-power transceivers transmit return downlink telemetry out to $50\text{m}$.
    - Inter-drone P2P mesh radio propagation operates at $60\text{m}$.
  - **Part 2: The "Yip-Back" State (One-Way Telemetry Silence)**:
    - When airframes operate in the annulus between $50\text{m}$ and $140\text{m}$ from Base without a relay proxy, they receive and execute GCS commands, but cannot "yip back" telemetry to Base.
    - GCS HUD top bar and telemetry cards reflect split link status:
      `UPLINK: ACTIVE | DOWNLINK: SILENT (YIP LOST)`.
  - **Part 3: Mesh Telemetry Gateway (The Relay Proxy)**:
    - In Gossip Mesh mode, any airframe within $50\text{m}$ of Base acts as a **Telemetry Gateway**.
    - The gateway node aggregates its local gossip peer table and downlinks state vectors for all connected mesh nodes to Base.
    - If at least one drone in the mesh is within $50\text{m}$ of Base, the GCS receives telemetry for the entire connected cluster.
  - **Part 4: God-View SIM Lab Overlay**:
    - SITL God-View (`/sim`) renders two concentric ground-plane domes: Outer Cyan Dome ($140\text{m}$ GCS Uplink) and Inner Amber/Blue Dome ($50\text{m}$ Airframe Downlink).
* **Targeted Test**: `python -m unittest tests/test_tri_mode_rf.py`
