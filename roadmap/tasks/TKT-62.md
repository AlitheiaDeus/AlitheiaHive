# TKT-62: GCS-SIM Canyon Decoupling & Pure Network Port Isolation

> **Category**: 🛠️ Refactor / Architecture | **Priority**: High | **State**: completed  
> **Operational Inspiration**: Strict Enforcement of the Canyon Airgap — GCS has ZERO direct `sim` handles and operates strictly over UDP / MAVLink ports (9000, 9001, 14550).

---

## 1. Operational Overview & Objectives

Eliminates the legacy in-memory tightrope between the Ground Control Station (GCS) and the SITL World Simulator (SIM).

Previously, `GCS/backend/web_server.py` took a direct reference to `self.sim`, called `self.sim.step()` in an asyncio task, and routed operator commands by calling proxy methods directly on in-memory objects. This in-memory coupling prevented GCS from operating as a standalone operator terminal that can connect to real 915MHz telemetry radios or external SITL engines (ArduPilot/PX4/Gazebo).

### Key Objectives Accomplished:
1. **Branch Isolation**: Created safety branch `refactor/gcs-sim-canyon-decoupling`.
2. **Standalone GCS Web Server (`GCS/backend/web_server.py`)**:
   - `SwarmWebServer` supports `sim=None` as a first-class mode.
   - When running standalone (`sim=None`), GCS listens for telemetry frames on UDP port 9000 (`tx_port`) via `TelemetryProtocol` and broadcasts them to Three.js WebGL clients over WebSocket.
   - When operator issues commands (`POST /api/command` or WebSocket messages), GCS serializes commands into JSON and sends them via UDP socket to port 9001 (`rx_port`).
   - Retains optional `sim` fallback for backward compatibility in legacy in-memory tests, but standard execution relies purely on network sockets.
3. **Decoupled Orchestration (`run_with_sim.py`)**:
   - `--gcs-only` executes standalone GCS with `sim=None` communicating strictly over UDP 9000/9001.
   - `--multi-process` launches decoupled child processes passing `--tx-port` and `--rx-port`.
4. **Zero Functional Regression**:
   - All 288 unit tests and 60 FPS headless smoke test pass cleanly.

---

## 2. Target Files & Architectural Scope

* `GCS/backend/web_server.py`: Added UDP telemetry receiver loop (`TelemetryProtocol`) and UDP command sender; enabled `sim=None` standalone operation.
* `GCS/backend/session_manager.py`: Updated `get_hydration_bundle()` to accept dictionary state snapshots when `sim` is absent.
* `SIM/backend/mavlink_bridge.py`: Handlers for all tactical commands (`DEPLOY_PATROL_TO_BINGO`, `DEPLOY_SEARCH_AND_RECALL`, `DEPLOY_PAYLOAD_DROP`, `TOGGLE_URBAN_OBSTACLES`, `TOGGLE_GNSS_DENIAL`) over UDP 9001.
* `run_with_sim.py`: Updated `--gcs-only` and master launcher to instantiate `SwarmWebServer(sim=None)` for pure port separation.
* `tests/test_bubble_decoupling.py`: Verified `TestGCSCanyonAirgap` with standalone GCS ingesting UDP telemetry on 19000 and emitting commands to 19001.

---

## 3. Acceptance Criteria & Invariants

* [x] Dedicated git branch `refactor/gcs-sim-canyon-decoupling` created.
* [x] `SwarmWebServer` can start, receive telemetry via UDP port 9000, and broadcast to WebSockets with `sim=None`.
* [x] Operator commands from UI/API are serialized and sent over UDP port 9001.
* [x] `SIM/backend/sitl_engine.py` runs cleanly without tight in-memory coupling to `web_server.py`.
* [x] Full regression discovery suite passes (288/288 tests).
* [x] Headless smoke test passes in $< 0.50\text{s}$.
