# [PLN-02] Auxiliary Mission Concepts & Waypoint Engine (Tactical Sub-Routines)

* **Grade**: 🏗️ Epic | **Type**: Modular Mission Sub-Routines (`Todo / Active Pipeline`) | **Priority**: High
* **Target Files**: `GCS/backend/c2_gateway.py`, `OBC/backend/drone_firmware.py`, `OBC/backend/mission_engine.py`, `SIM/backend/sitl_engine.py`, `GCS/frontend/js/app.js`
* **Operational Inspiration**:
  - **Aux Missions are Tactical Sub-Routines / Tool Macros, NOT Macro Operations/Quests.**
  - An **Operation (Quest/Scenario)** is the overarching mission contract (e.g., *Operation Neon Alley* managed by `[PLN-03]`).
  - An **Aux Mission** is an autonomous tactical *ability / sub-routine* the operator triggers on-demand from the GCS action bar to solve tactical challenges during an operation.
  - Replaces bespoke, one-off state flags with a unified, airframe-agnostic **Mission Execution Engine** available to registered airframes (`T1`, `T1.5`, `K1`, `T3`).

* **Architecture: The Standard Auxiliary Mission Workflows (Operator Toolset)**:
  1. **`WAYPOINT_ROUTE` (The Corridor Foundation)**:
     - Discrete coordinate queue `[ [x, y, z, loiter_sec], ... ]` with arrival acceptance spheres and wrap-around looping.
  2. **`SEARCH_AND_RECALL` (S&R Courier / Lost Wingman Retrieval)**:
     - Autonomous blind transit to Last Known Position (LKP) of a drone that drifted out of comms or got trapped in an RF black zone.
     - Expanding spiral loiter with AoA direction finding (`Bloodhound`), P2P rescue beacon handshake, and escort back into GCS radio range.
  3. **`PAYLOAD_DEPOSIT` (Cairn G1 Repeater Drop)**:
     - Transit to drop coordinates, precision hover ($Z=3.0\text{m}$), payload release confirmation, spawn stationary G1 repeater in simulation to bridge comms, and RTB.
  4. **`TOUCH_AND_GO_PROBE` (Tactical RNG Recon)**:
     - Bounded random station hopping within geofenced box with dwell countdowns and coordinated formation stabilization to probe if an area is a viable POI.
  5. **`PATROL_TO_BINGO` (Sentry Loiter to Automatic Recovery - `[TKT-58]`)**:
     - Extended perimeter sentry surveillance looping hands-off until low battery or commanded BINGO trigger, transitioning autonomously into Stage 1 Overhead Marshal and Stage 2 ATC pad recovery.

* **Airgap Integrity & Execution Model**:
  - **GCS Role**: Translates operator's Aux Mission selection into flight primitives and uplinks via UDP 9001.
  - **OBC Role**: Executes discrete navigation and payload pins locally without needing to understand the macro quest context.
  - **Telemetry Feedback**: Drone OBC downlinks simple milestone pings (`MISSION_ITEM_REACHED`), allowing GCS to update UI state and advance mission sequences.
