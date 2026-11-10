# BUG-11: Migrate Tandem Escort Stationing (ESCORT_REJOIN) to OBC Brain Firmware

> **Category**: 🛡️ Architecture / 🐛 Bug | **Priority**: High | **State**: todo  
> **Operational Inspiration**: Strict Brain Airgap enforcement for Search-and-Recall / Courier tandem missions (Red 2 Remediation)

---

## 1. Operational Overview & Objectives

In accordance with the **4 Explicit Airgaps** of the 50/50 Architecture, the SITL World Engine (Bubble 3) must never reach into drone autonomy state machines or calculate drone flight targets.

During Search-and-Recall / Courier tandem missions (TKT-51), an airframe in `ESCORT_REJOIN` trails behind the rescue courier. Currently, [`_execute_obc_step`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/SIM/backend/sitl_engine.py#L432-L444) peeks across drone objects in simulation memory, looks up the courier by ID, computes the 4.0m trailing station from the courier's ground-truth `pos` and `vel`, and directly mutates `drone.acknowledged_target` and `drone.target_station`. On physical hardware where `SITLWorldEngine` does not exist, an escortee drone in `ESCORT_REJOIN` will never calculate its trailing station and will freeze or drift indefinitely.

This ticket migrates all tandem trailing station calculations into the OBC brain firmware, operating strictly from received `RadioPacket`s (`ESCORT_YIP` / beacon telemetry).

---

## 2. Target Files & Architectural Scope

* [`OBC/backend/brains/leader_follower.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/leader_follower.py): Implement onboard tandem trailing station calculation within `step_drone` for `telemetry.flight_state == "ESCORT_REJOIN"`, consuming courier state from `self.heard_peers[escort_leader_id]`.
* [`OBC/backend/brains/base.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/base.py): Ensure `LocalTelemetry` carries `escort_leader_id: Optional[int]` (or courier peer ID) and `FlightIntent` returns `target_station` and `state_updates["acknowledged_target"]`.
* [`SIM/backend/sitl_engine.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/SIM/backend/sitl_engine.py): Purge lines 431–444 from `_execute_obc_step`. SIM must not compute or mutate target stations for escortee drones.
* [`tests/test_courier_aux_mission.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/tests/test_courier_aux_mission.py): Verify tandem escort trailing operates purely in `step_drone` from `heard_peers`.

---

## 3. Implementation Blueprint

### A. OBC Brain Guidance for `ESCORT_REJOIN` (`leader_follower.py`)
- Inside `LeaderFollowerBrain.step_drone`, when `telemetry.flight_state == "ESCORT_REJOIN"`:
  - Inspect `escort_leader_id = telemetry.escort_leader_id`.
  - Look up `courier = self.heard_peers.get(escort_leader_id)`.
  - If courier exists and is not crashed:
    - Compute tangent vector: $\mathbf{u} = \mathbf{v}_{\text{courier}} / \|\mathbf{v}_{\text{courier}}\|$ (defaulting to $[-1, 0, 0]$ if speed $\le 0.5\text{ m/s}$).
    - Trailing station: $\mathbf{p}_{\text{trail}} = \mathbf{p}_{\text{courier}} - 4.0 \cdot \mathbf{u}$, clamping $Z$ to courier altitude.
    - Return `FlightIntent(target_acc=acc_steer, target_station=p_trail, state_updates={"acknowledged_target": p_trail})`.
  - If courier is missing or timed out from `heard_peers`, hold current position (LKP fallback).

### B. Purge SITL Cheating Logic (`sitl_engine.py`)
- Remove the block in `_execute_obc_step`:
  ```python
  if drone.flight_state == "ESCORT_REJOIN" and getattr(drone, "escort_leader_id", None) is not None:
      ...
  ```
- Ensure `_build_local_telemetry` passes `escort_leader_id=getattr(drone, "escort_leader_id", None)`.

---

## 4. Acceptance Criteria & Verification Invariants

* [ ] SITL engine contains zero target mutations or trailing station math for `ESCORT_REJOIN`.
* [ ] `LeaderFollowerBrain` calculates trailing station purely from `self.heard_peers`.
* [ ] When courier packets are lost, the escortee falls back to holding its last known trailing station.
* [ ] Targeted test module passes: `python -m unittest tests/test_courier_aux_mission.py`.
