# BUG-17: Autonomous Outbox Radio Packet Emission for Gossip & Beacons

> **Category**: 🛡️ Architecture / 🐛 Bug | **Priority**: Normal | **State**: completed  
> **Operational Inspiration**: Forbid SITL from forging radio packets; OBC firmware autonomously emits beacons and gossip echoes (Red 8 Remediation)

---

## 1. Operational Overview & Objectives

In accordance with the **Brain Airgap** and **God-View / Sim Airgap**, the SITL world engine governs physical channel delivery (path loss, obstacles, dropped packets, and collision). It must never manufacture or forge radio packets on behalf of airframes.

Previously, [`_deliver_radio_packets`](../../SIM/backend/sitl_engine.py#L381-L408) in the SITL world engine purged `sender.outbox` and manually appended `PEER_BEACON` and `GOSSIP_ECHO` packets into the drone's outbox before calling `step_drone`. In a physical swarm, radio packets must be emitted autonomously by the OBC brain firmware during its own control step. The physics engine should only read existing packets from `drone.outbox` and simulate their transmission over the air.

This ticket moved all periodic beacon and gossip packet generation into `GossipConsensusBrain.step_drone`, returning them cleanly in the `outbox` return value.

---

## 2. Target Files & Architectural Scope

* [`OBC/backend/brains/gossip_consensus.py`](../../OBC/backend/brains/gossip_consensus.py): Implemented autonomous periodic emission of `PEER_BEACON` and `GOSSIP_ECHO` packets within `step_drone`, returning them in `outbox`.
* [`SIM/backend/sitl_engine.py`](../../SIM/backend/sitl_engine.py): Purged forged packet appending from `_deliver_radio_packets`. SIM physics loop strictly delivers existing packets in `sender.outbox` across the RF propagation model.
* [`tests/test_airgap_integrity.py`](../../tests/test_airgap_integrity.py): Added `test_autonomous_outbox_emission_zero_sitl_forgery` verifying autonomous emission and zero SITL forgery.

---

## 3. Implementation Blueprint

### A. Autonomous Packet Generation in `GossipConsensusBrain` (`gossip_consensus.py`)
- Inside `GossipConsensusBrain.step_drone(telemetry, inbox, dt)`:
  - Maintained onboard beacon clock: `self.heartbeat_timer += dt`.
  - When `heartbeat_timer >= 0.1` (10 Hz beacon):
    - Reset `self.heartbeat_timer -= HEARTBEAT_INTERVAL`.
    - Emitted `self.emit_gossip_packet(telemetry)` if propagating, else `PEER_BEACON`.

### B. Purge SITL Forgery Logic (`sitl_engine.py`)
- In `_deliver_radio_packets`:
  - Deleted forged packet synthesis.
  - Deliver packets that the drone OBC firmware placed in `sender.outbox` during the preceding step, filtering by range, occlusion, and channel noise.

---

## 4. Acceptance Criteria & Verification Invariants

* [x] SITL engine contains zero packet manufacturing or outbox tampering code.
* [x] `GossipConsensusBrain.step_drone` autonomously emits `PEER_BEACON` and `GOSSIP_ECHO` packets in `outbox`.
* [x] Swarm gossip consensus and peer table discovery maintain 100% operational fidelity without SITL packet injection.
* [x] Targeted test module passes: `python -m unittest tests/test_airgap_integrity.py` and `tests/test_gossip_formations.py`.
