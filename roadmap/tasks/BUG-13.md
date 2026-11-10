# BUG-13: Route C2 Target Uplinks Through Transceiver Inboxes (GCS_UPLINK)

> **Category**: 🛡️ Architecture / 🐛 Bug | **Priority**: High | **State**: todo  
> **Operational Inspiration**: Enforce Canyon Airgap: GCS and tether links deliver packets to inbox, never mutating drone RAM (Red 4 Remediation)

---

## 1. Operational Overview & Objectives

In accordance with the **Canyon Airgap**, communication between the Ground Control Station and the drone fleet occurs strictly across serialized network channels (UDP 9000/9001, MAVLink, and RF transceiver packets). An external operator or world engine must never mutate drone flight buffers in Python memory directly.

Currently, [`dispatch_target_uplink`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/SIM/backend/sitl_engine.py#L2863-L2875) directly overwrites `d.acknowledged_target` and mutates `d.flight_state` in memory for leader and follower drones. This completely bypasses the transceiver buffer. Although both [`LeaderFollowerBrain`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/leader_follower.py) and [`GossipConsensusBrain`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/gossip_consensus.py) possess unit-tested logic to ingest `GCS_UPLINK` packets, SITL's uplink dispatcher cheats by writing to RAM.

This ticket refactors `dispatch_target_uplink` to queue authentic `RadioPacket(msg_type="GCS_UPLINK", ...)` instances into drone inboxes.

---

## 2. Target Files & Architectural Scope

* [`SIM/backend/sitl_engine.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/SIM/backend/sitl_engine.py): Refactor `dispatch_target_uplink` to queue `RadioPacket(sender_id=-1, msg_type="GCS_UPLINK", payload={"cmd": "SET_TARGET", "target": target_arr.tolist()})` into target drone `inbox`es instead of mutating `d.acknowledged_target`.
* [`OBC/backend/brains/leader_follower.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/leader_follower.py): Ensure `GCS_UPLINK` packet handling in `step_drone` unfreezes `holding_at_horizon`, transitions `PERIMETER_HOLD` $\rightarrow$ `CRUISE`, and updates `acknowledged_target`.
* [`OBC/backend/brains/gossip_consensus.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/gossip_consensus.py): Ensure `GCS_UPLINK` packets seed consensus sequence in `peer_state`.
* [`tests/test_airgap_integrity.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/tests/test_airgap_integrity.py): Assert zero direct memory mutation during target dispatch; verify targets update only after `step_drone` processes incoming inbox packets.

---

## 3. Implementation Blueprint

### A. Radio Packet Queueing in `dispatch_target_uplink` (`sitl_engine.py`)
- For `TETHERED_RELAY`:
  - Verify tether umbilical integrity (`not self.tether_severed`).
  - Queue `RadioPacket(sender_id=-1, msg_type="GCS_UPLINK", timestamp=self.sim_time, payload={"cmd": "SET_TARGET", "target": target_arr.tolist()})` into `drone.inbox` for Drone 0 (or tethered leader).
- For `LEADER_FOLLOWER`:
  - Check distance to leader $\le \text{RADIO\_RANGE}$ ($120\text{m}$).
  - Queue `RadioPacket` into `current_leader.inbox`.
- For `GOSSIP_CONSENSUS`:
  - Check nearest drone distance $\le \text{RANGE\_HOWL}$ ($70\text{m}$).
  - Queue `RadioPacket` into `seed_drone.inbox`.
- Remove all lines setting `d.acknowledged_target = np.copy(...)` directly in `dispatch_target_uplink`.

### B. Brain Ingestion Verification (`leader_follower.py` & `gossip_consensus.py`)
- Verify that `step_drone` processes `GCS_UPLINK` packets:
  - If `cmd == "SET_TARGET"`: sets `intent.state_updates["acknowledged_target"] = np.array(payload["target"])`.
  - Resets `holding_at_horizon = False`.
  - If in `PERIMETER_HOLD`, transitions `flight_state = "CRUISE"` and `status = "FLEET_LEADER"`.

---

## 4. Acceptance Criteria & Verification Invariants

* [ ] `dispatch_target_uplink` contains zero direct mutations of `acknowledged_target`, `flight_state`, or `status`.
* [ ] Target waypoints are delivered exclusively as `RadioPacket(msg_type="GCS_UPLINK")` into drone transceiver `inbox` buffers.
* [ ] Brain ingests uplink packet during `step_drone` and returns state updates via `FlightIntent`.
* [ ] Targeted test module passes: `python -m unittest tests/test_airgap_integrity.py`.
