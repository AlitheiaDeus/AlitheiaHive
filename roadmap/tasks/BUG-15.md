# BUG-15: Decouple GCS C2 Gateway to Standalone Downlink Telemetry

> **Category**: 🛡️ Architecture / 🐛 Bug | **Priority**: High | **State**: completed  
> **Operational Inspiration**: Complete Canyon Airgap decoupling: C2 Gateway operates standalone with `sim=None` (Red 6 Remediation)

---

## 1. Operational Overview & Objectives

In accordance with the **Operator Airgap** and **Canyon Airgap**, the Ground Control Station (GCS) is an operator terminal running on external hardware (e.g. laptop or tablet). It must render *only* downlinked telemetry frames and push operator flight intents over serialized network packets (UDP 9000/9001, MAVLink). It must never peek into or alter physical simulation memory, and must be fully capable of operating standalone with `sim=None`.

Currently, although [`C2Gateway`](../../GCS/backend/c2_gateway.py) resides in `GCS/backend/` (Bubble 1), it is tightly coupled to `self.sim`. When `self.sim is None`:
- `step()` immediately returns (disabling auto-patrol, RNG recon, courier escort, and payload drops).
- `dispatch_target_uplink()` fails with `NO_SIM_ATTACHED`.
- `is_target_reached()` peeks directly into `self.sim.drones` to compute swarm arrival from ground truth positions.

This ticket refactors `C2Gateway` to operate completely decoupled from `self.sim`, advancing C2 workflows via incoming downlinked JSON telemetry frames and emitting uplink commands across UDP port 9001.

---

## 2. Target Files & Architectural Scope

* [`GCS/backend/c2_gateway.py`](../../GCS/backend/c2_gateway.py): Remove mandatory `self.sim` dependencies. Maintain a local downlinked telemetry cache; calculate auto-patrol generation, RNG dwell, and target arrival strictly from downlinked telemetry. Route uplinks to UDP socket port 9001 when `sim=None`.
* [`GCS/backend/web_server.py`](../../GCS/backend/web_server.py): Connect incoming WebSocket/UDP telemetry streams to update `C2Gateway.on_telemetry_frame()`.
* [`tests/test_c2_gateway_standalone.py`](../../tests/test_c2_gateway_standalone.py): New unit test validating `C2Gateway` advancing C2 state machines with `sim=None`.

---

## 3. Implementation Blueprint

### A. Telemetry Ingestion Cache in `C2Gateway` (`c2_gateway.py`)
- Add `self.fleet_telemetry: Dict[int, Dict[str, Any]] = {}` and `on_telemetry_frame(frame: Dict[str, Any]) -> None`.
- In `step(dt)`:
  - Do not exit early when `self.sim is None`.
  - Maintain `c2_time += dt`.
  - Advance auto-patrol curves, RNG recon timers, and waypoint queue evaluations using `self.fleet_telemetry`.
- In `is_target_reached()`:
  - Calculate active centroid or leader position from `self.fleet_telemetry.values()` instead of `[d.pos for d in self.sim.drones]`.

### B. Network Uplink Dispatch (`c2_gateway.py`)
- In `dispatch_target_uplink(target_pos)`:
  - If `self.sim is not None`: retain optional in-process delegation for backward compatibility.
  - If `self.sim is None`: serialize JSON packet `{"cmd": "SET_TARGET", "target": list(target_pos)}` and send via UDP to `127.0.0.1:9001` (SIM command port).
  - Return `{"status": "OK", "channel": "UDP_CANYON", "target": list(target_pos)}`.

---

## 4. Acceptance Criteria & Verification Invariants

* [x] `C2Gateway` advances all C2 state machines (patrol, recon, aux missions) when `self.sim is None`.
* [x] Swarm centroid and target arrival are computed exclusively from downlinked telemetry frames.
* [x] Target uplinks are emitted over UDP port 9001 when operating standalone.
* [x] Targeted test module passes: `python -m unittest tests/test_c2_gateway_standalone.py`.
* [x] Full regression test suite passes: `.\run.ps1 -UnitTest` (317 tests pass).
