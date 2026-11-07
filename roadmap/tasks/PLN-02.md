# [PLN-02] Auxiliary Mission Concepts & Waypoint Engine (Registered Drones Only)

* **Grade**: 🏗️ Epic | **Type**: Modular Mission Architecture (`Todo / Active Pipeline`) | **Priority**: High
* **Target Files**: `GCS/backend/c2_gateway.py`, `OBC/backend/drone_firmware.py`, `OBC/backend/mission_engine.py`, `SIM/backend/sitl_engine.py`, `GCS/frontend/js/app.js`
* **Operational Inspiration**:
  - Replaces bespoke, one-off state flags with a unified, airframe-agnostic **Mission Execution Engine**.
  - Operates as an auxiliary workflow at the click of a button in GCS for **registered airframes only** (`T1`, `T1.5`, `K1`, `T3`).
  - Phased Implementation: **Waypoints First**, followed by specialized auxiliary payloads.
* **Architecture: The 5 Standard Auxiliary Mission Workflows**:
  1. **WAYPOINT_ROUTE (The Foundation)**:
     - Discrete coordinate queue `[ [x, y, z, loiter_sec], ... ]` with arrival acceptance spheres and wrap-around looping.
  2. **SEARCH_AND_RESCUE (S&R Courier / Mule)**:
     - Autonomous blind transit to LKP, expanding spiral loiter with AoA direction finding (`Bloodhound`), P2P rescue beacon handshake, and escort home.
  3. **PAYLOAD_DEPOSIT (Cairn G1 Repeater Drop)**:
     - Transit to drop coordinates, precision hover ($Z=3.0\text{m}$), payload release confirmation, spawn stationary G1 repeater in simulation, and RTB.
  4. **TACTICAL_RECON (RNG Touch-and-Go)**:
     - Bounded random station hopping within geofenced box with dwell countdowns and coordinated formation stabilization.
  5. **PATROL_TO_BINGO (Sentry Loiter to Automatic Recovery)**:
     - Fleet holds perimeter patrol until low battery or operator BINGO trigger, transitioning autonomously into Stage 1 Overhead Marshal and Stage 2 ATC recovery.
* **Airgap Integrity**:
  - GCS defines and uploads mission payload via UDP 9001.
  - Drone OBC executes mission steps locally. GCS tracks progress purely via downlinked telemetry pings (`MISSION_ITEM_REACHED`).
