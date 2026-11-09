# [BUG-03] P2P Mesh Split-Brain Target Oscillation

* **Grade**: 🐞 Bug / Defect | **Type**: Decentralized Gossip Consensus & Quorum Arbitration (`Completed / Fresh Review`) | **Priority**: Very High
* **Target Files**: `OBC/backend/brains/gossip_consensus.py`, `SIM/backend/sitl_engine.py`, `tests/test_brains.py`, `tests/test_gossip_formations.py`
* **Operational Context**:
  - After toggling comms mode between Wireless Star and P2P Mesh, Drone #0 targeted Bravo while the remainder of the swarm oscillated continuously between Alpha and Bravo.
  - Near Alpha, drones (with the exception of Drone #1) briefly flashed yellow badges, redirected heading toward Bravo, came within comms range of Drone #0, and then immediately flipped back toward Alpha.
* **Technical Assessment & Root Causes Identified**:
  - **Sequence Desynchronization on Hot-Swap (`sitl_engine.py:set_brain`)**: When switching to `GOSSIP_CONSENSUS`, every drone initialized a new brain with `seq = 1`, but Drone #0 held target Bravo while followers held Alpha.
  - **Deadlock on Equal Sequence Numbers (`gossip_consensus.py:242`)**: In `ingest_gossip_packets()`, if `peer_echo_seq == self.peer_state.pending_seq` (`1 == 1`), the node merged acks but did NOT reconcile diverging targets. Neither cluster adopted the other's target.
  - **Multi-Hop Relay Ghost Timestamp Refresh**: Relaying `mesh_summary` without retaining the original `last_heard` observation timestamp created an infinite loop preventing distant/disconnected peers from ever being pruned from `heard_peers`.
  - **Quorum Partition Barrier Stall**: 75% quorum across 6 nodes requires 5 acks. A 2-node contact could not meet the global quorum barrier (2/6 = 33% < 75%), causing pending target thrashing.
* **Remediation Completed**:
  - **Part 1: Unified Consensus Target on Hot-Swap (`sitl_engine.py:set_brain`)**:
    - Added consensus target resolution: `consensus_target = self.leader.acknowledged_target if self.leader else self.target_pos`.
    - Every drone initializes its `GossipConsensusBrain` with this consensus target, eliminating initial target divergence on hot-swap.
  - **Part 2: Equal Sequence Number Tie-Breaking (`gossip_consensus.py`)**:
    - When `peer_echo_seq == self.peer_state.pending_seq`: if targets diverge (`norm >= 0.1m`), newer `target_timestamp` wins.
    - If timestamps match within `1e-4`, lower sender/drone ID deterministically wins (flagship / leader bias). The loser adopts the winning target, resets `propagating_timer = 0.0`, and propagates.
  - **Part 3: Multi-Hop Observation Timestamp Preservation (`gossip_consensus.py`)**:
    - `mesh_summary` carries true `last_heard` observation clocks. Receivers set `last_heard_clock = max(existing, m_last_heard)`, breaking the infinite lease renewal ghost loop and allowing disconnected peers to prune cleanly after `LINK_TIMEOUT` (1.0s).
  - **Part 4: Quorum Timeout Fallback & Per-Tick Ingestion (`gossip_consensus.py`, `sitl_engine.py`)**:
    - Added `propagating_timer` fallback: commits pending target if `ack_ratio >= 0.75` OR if `propagating_timer >= 2.0s` and all reachable living peers in `heard_peers` have acked.
    - Removed premature radio inbox clearing in `sitl_engine.py:_deliver_radio_packets` so `step_drone(telemetry, inbox, dt)` receives packets and runs `ingest_gossip_packets` unconditionally every tick.
* **Verification Gates**:
  - `test_bug03_hot_swap_unifies_consensus_target_and_prevents_oscillation`: PASS. Verifies unified initialization and zero split-brain oscillation across 30 simulation steps.
  - `test_bug03_equal_seq_divergent_target_tie_breaking`: PASS. Verifies newer timestamp wins, and identical timestamps break ties via lower drone ID without oscillation or deadlock.
  - `test_bug03_mesh_summary_does_not_ghost_loop`: PASS. Verifies disconnected peers expire after 1.0s despite multi-hop mesh summary relaying.
  - Full suite in `tests/test_brains.py`: 23/23 tests PASS in 2.43s.
  - Headless smoke test `.\run.ps1 -SmokeTest`: PASS in 0.406s (exit code 0).
