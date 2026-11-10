# BUG-14: Decouple SITL Sim Proxies to Broadcast Command Packets

> **Category**: 🛡️ Architecture / 🐛 Bug | **Priority**: Normal | **State**: todo  
> **Operational Inspiration**: Replace in-memory brain proxy iteration with RF broadcast command packets (Red 5 Remediation)

---

## 1. Operational Overview & Objectives

In accordance with the **Brain Airgap**, each drone owns an isolated [`BaseBrain`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/base.py) instance. A swarm of separate physical microcontrollers cannot be updated via an in-memory Python `for` loop iterating over drone instances.

Currently, the compatibility proxies [`_LeaderFollowerSimProxy`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/SIM/backend/sitl_engine.py#L3167-L3225) and [`_GossipSimProxy`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/SIM/backend/sitl_engine.py#L3356-L3450) mutate private fields (`brain.formation_mode`, `brain.peer_state.pending_seq`, `seed_drone.acknowledged_target`) by looping through `self.drones` directly in Python memory. Furthermore, `inject_target` writes consensus registers directly into `seed_drone.brain.peer_state`.

This ticket refactors the proxy classes to simulate authentic over-the-air C2 command broadcasts, queuing `RadioPacket`s into transceivers rather than reaching into private brain memory.

---

## 2. Target Files & Architectural Scope

* [`SIM/backend/sitl_engine.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/SIM/backend/sitl_engine.py): Refactor `_LeaderFollowerSimProxy.formation_mode.setter` and `_GossipSimProxy.inject_target` to transmit `C2_COMMAND` or `GOSSIP_SEED` packets across the simulated RF channel.
* [`OBC/backend/brains/leader_follower.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/leader_follower.py): Process `C2_COMMAND` (`{"cmd": "SET_FORMATION", "mode": ...}`) in `step_drone` to switch `self.formation_mode`.
* [`OBC/backend/brains/gossip_consensus.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/OBC/backend/brains/gossip_consensus.py): Handle `GOSSIP_SEED` packets to initialize `peer_state` consensus from inbox rather than external direct RAM mutation.
* [`tests/test_gossip_formations.py`](file:///c:/Users/AlitheiaDeus/DeusLocker/L2C/SwarmTech/v3-Uprising/tests/test_gossip_formations.py): Verify formation switching operates via over-the-air packet processing.

---

## 3. Implementation Blueprint

### A. Radio Packet Broadcast in Sim Proxies (`sitl_engine.py`)
- In `_LeaderFollowerSimProxy.formation_mode.setter`:
  - Instead of `for d in eng.drones: d.brain.formation_mode = val`, queue a `RadioPacket(sender_id=-1, msg_type="C2_COMMAND", payload={"cmd": "SET_FORMATION", "mode": val})` into the active leader's `inbox` (or broadcast to all in-range drones).
- In `_GossipSimProxy.inject_target`:
  - Queue `RadioPacket(sender_id=-1, msg_type="GCS_UPLINK", payload={"cmd": "GOSSIP_SEED", "target": target_arr.tolist(), "seq": seq})` into `seed_drone.inbox`.
  - Remove direct writes to `seed_drone.brain.peer_state` and `seed_drone.acknowledged_target`.

### B. Brain Ingestion (`leader_follower.py` & `gossip_consensus.py`)
- In `LeaderFollowerBrain.step_drone`:
  - When `pkt.msg_type == "C2_COMMAND"` and `pkt.payload.get("cmd") == "SET_FORMATION"`:
    - Set `self.formation_mode = pkt.payload["mode"]`.
    - Set `self._heading_initialized = False`.
- In `GossipConsensusBrain.step_drone`:
  - When packet with `GOSSIP_SEED` arrives: initialize `self.peer_state.pending_target`, `pending_seq`, and transition to `PROPAGATING`.

---

## 4. Acceptance Criteria & Verification Invariants

* [ ] `_LeaderFollowerSimProxy` and `_GossipSimProxy` do not directly mutate private brain attributes of peer drones in RAM.
* [ ] Formation switches and target injections are driven by `RadioPacket`s in transceivers.
* [ ] Targeted test module passes: `python -m unittest tests/test_gossip_formations.py`.
