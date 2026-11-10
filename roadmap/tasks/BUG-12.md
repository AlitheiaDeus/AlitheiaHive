# BUG-12: Migrate Ingress Corridor Stack (CORRIDOR_ERTB) Popping to OBC Brain Firmware

> **Category**: 🛡️ Architecture / 🐛 Bug | **Priority**: High | **State**: todo  
> **Operational Inspiration**: Autonomic onboard reverse backtrack corridor execution (Red 3 Remediation)

---

## 1. Operational Overview & Objectives

In accordance with the **Brain Airgap**, each drone airframe is an autonomous compute bubble (OBC). Drones steer strictly to local `acknowledged_target` buffers and execute onboard failsafes when comms drop.

Currently, during Emergency Return to Base via reversible ingress corridor (`CORRIDOR_ERTB`), [`_execute_obc_step`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/SIM/backend/sitl_engine.py#L460-L470) in the SITL world engine pops nodes from `drone.corridor_stack`, evaluates arrival distance ($d \le 4.5\text{m}$), updates `drone.acknowledged_target` and `drone.target_station`, and directly switches the drone's flight state to `OVERHEAD_HOLD`. Neither [`LeaderFollowerBrain`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/leader_follower.py) nor [`GossipConsensusBrain`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/gossip_consensus.py) drives this sequence. On real hardware without SITL, an airframe executing `CORRIDOR_ERTB` will remain permanently frozen targeting the first corridor waypoint.

This ticket moves corridor node popping and arrival detection directly into OBC brain firmware.

---

## 2. Target Files & Architectural Scope

* [`OBC/backend/brains/leader_follower.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/leader_follower.py): Implement `CORRIDOR_ERTB` waypoint progression and arrival evaluation in `step_drone`.
* [`OBC/backend/brains/gossip_consensus.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/gossip_consensus.py): Implement matching `CORRIDOR_ERTB` waypoint progression.
* [`OBC/backend/brains/base.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/base.py): Ensure corridor stack state and targets are communicated cleanly via `LocalTelemetry` and `FlightIntent.state_updates`.
* [`SIM/backend/sitl_engine.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/SIM/backend/sitl_engine.py): Remove lines 459–470 from `_execute_obc_step`. SIM must not pop corridor nodes or trigger `OVERHEAD_HOLD`.
* [`tests/test_emergency_landing.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/tests/test_emergency_landing.py): Verify that corridor backtracking progresses autonomously in standalone OBC step execution.

---

## 3. Implementation Blueprint

### A. Autonomic Corridor Ingress in OBC Firmware (`leader_follower.py` & `gossip_consensus.py`)
- In `step_drone`, when `telemetry.flight_state == "CORRIDOR_ERTB"`:
  - Read active corridor target from `telemetry.corridor_target`.
  - Check distance to current target: if $d \le 4.5\text{m}$ (or if target is `None`):
    - Request the next node from the onboard corridor stack via `intent.state_updates["corridor_pop"] = True` (or pop directly if stack is maintained on brain).
    - Update `intent.target_station` and `intent.state_updates["acknowledged_target"]`.
  - If corridor stack is exhausted (or length $\le 1$) and distance to Base $(0,0,25) \le 4.5\text{m}$:
    - Transition flight state: `intent.state_updates["flight_state"] = "OVERHEAD_HOLD"`.
    - Update status: `intent.state_updates["status"] = "OVERHEAD_HOLD"`.

### B. Purge SITL Cheating Logic (`sitl_engine.py`)
- Remove corridor popping and state mutation from `_execute_obc_step`:
  ```python
  if drone.flight_state == "CORRIDOR_ERTB" and hasattr(drone, "corridor_stack"):
      ...
  ```
- If `intent.state_updates.get("corridor_pop")`, pop node from `drone.corridor_stack` within physical airframe register updates.

---

## 4. Acceptance Criteria & Verification Invariants

* [ ] SITL engine contains zero corridor node popping logic or `OVERHEAD_HOLD` flight state transitions.
* [ ] OBC brain autonomously pops corridor nodes upon arriving within $4.5\text{m}$.
* [ ] Drones reaching the final corridor node within $4.5\text{m}$ of Base transition cleanly to `OVERHEAD_HOLD`.
* [ ] Targeted test module passes: `python -m unittest tests/test_emergency_landing.py`.
