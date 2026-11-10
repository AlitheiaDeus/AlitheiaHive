# BUG-13: Route C2 Target Uplinks Through Transceiver Inboxes (GCS_UPLINK)

> **Category**: 🛡️ Architecture / 🐛 Bug | **Priority**: High | **State**: completed  
> **Operational Inspiration**: Enforce Canyon Airgap: GCS and tether links deliver packets to inbox, never mutating drone RAM (Red 4 Remediation)

---

## 1. Operational Overview & Objectives

In accordance with the **Canyon Airgap**, communication between the Ground Control Station and the drone fleet occurs strictly across serialized network channels (UDP 9000/9001, MAVLink, and RF transceiver packets). An external operator or world engine must never mutate drone flight buffers in Python memory directly.

Previously, [`dispatch_target_uplink`](../../SIM/backend/sitl_engine.py#L2863-L2875) directly overwrote `d.acknowledged_target` and mutated `d.flight_state` in memory for leader and follower drones. This completely bypassed the transceiver buffer. Although both [`LeaderFollowerBrain`](../../OBC/backend/brains/leader_follower.py) and [`GossipConsensusBrain`](../../OBC/backend/brains/gossip_consensus.py) possessed unit-tested logic to ingest `GCS_UPLINK` packets, SITL's uplink dispatcher cheated by writing to RAM.

This ticket refactored `dispatch_target_uplink` to queue authentic `RadioPacket(msg_type="GCS_UPLINK", ...)` instances into drone inboxes.

---

## 2. Target Files & Architectural Scope

* [`SIM/backend/sitl_engine.py`](../../SIM/backend/sitl_engine.py): Refactored `dispatch_target_uplink` to queue `RadioPacket(sender_id=-1, msg_type="GCS_UPLINK", payload={"cmd": "SET_TARGET", "target": target_arr.tolist()})` into target drone `inbox`es instead of mutating `d.acknowledged_target`.
* [`OBC/backend/brains/leader_follower.py`](../../OBC/backend/brains/leader_follower.py): Ensured `GCS_UPLINK` packet handling in `step_drone` unfreezes `holding_at_horizon`, transitions `PERIMETER_HOLD` $\rightarrow$ `CRUISE`, and updates `acknowledged_target`.
* [`OBC/backend/brains/gossip_consensus.py`](../../OBC/backend/brains/gossip_consensus.py): Ensured `GCS_UPLINK` packets seed consensus sequence in `peer_state`.
* [`tests/test_airgap_integrity.py`](../../tests/test_airgap_integrity.py): Asserted zero direct memory mutation during target dispatch; verified targets update only after `step_drone` processes incoming inbox packets.

---

## 3. Implementation Blueprint

### A. Radio Packet Queueing in `dispatch_target_uplink` (`sitl_engine.py`)
- For `TETHERED_RELAY`:
  - Verified tether umbilical integrity (`not self.tether_severed`).
  - Queued `RadioPacket(sender_id=-1, msg_type="GCS_UPLINK", timestamp=self.sim_time, payload={"cmd": "SET_TARGET", "target": target_arr.tolist()})` into `drone.inbox` for Drone 0 (or tethered leader).
- For `LEADER_FOLLOWER`:
  - Checked distance to leader $\le \text{RADIO\_RANGE}$ ($120\text{m}$).
  - Queued `RadioPacket` into `current_leader.inbox`.
- For `GOSSIP_CONSENSUS`:
  - Checked nearest drone distance $\le \text{RANGE\_HOWL}$ ($70\text{m}$).
  - Queued `RadioPacket` into `seed_drone.inbox`.
- Removed all lines setting `d.acknowledged_target = np.copy(...)` directly in `dispatch_target_uplink`.

### B. Brain Ingestion Verification (`leader_follower.py` & `gossip_consensus.py`)
- Verified that `step_drone` processes `GCS_UPLINK` packets:
  - If `cmd == "SET_TARGET"`: sets `intent.state_updates["acknowledged_target"] = np.array(payload["target"])`.
  - Resets `holding_at_horizon = False`.
  - If in `PERIMETER_HOLD`, transitions `flight_state = "CRUISE"` and `status = "FLEET_LEADER"`.

---

## 4. Acceptance Criteria & Verification Invariants

* [x] `dispatch_target_uplink` contains zero direct mutations of `acknowledged_target`, `flight_state`, or `status`.
* [x] Target waypoints are delivered exclusively as `RadioPacket(msg_type="GCS_UPLINK")` into drone transceiver `inbox` buffers.
* [x] Brain ingests uplink packet during `step_drone` and returns state updates via `FlightIntent`.
* [x] Targeted test module passes: `python -m unittest tests/test_airgap_integrity.py` and `tests/test_breadcrumb_backtrack.py`.
