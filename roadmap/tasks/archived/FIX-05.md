# [FIX-05] Comms Horizon Telemetry Gate, Universal Lost-Link Recovery & Pad Isolation

* **Grade**: ðŸ› ï¸ Improvement | **Type**: Bugfix / Comms Horizon & Autonomy (`Completed`) | **Priority**: Very High
* **Target Files**: `GCS/backend/c2_gateway.py`, `GCS/backend/web_server.py`, `SIM/backend/sitl_engine.py`, `OBC/backend/brains/gossip_consensus.py`, `GCS/frontend/js/app.js`, `GCS/frontend/js/hud.js`, `tests/test_rtb_landing.py`, `tests/test_gossip_formations.py`
* **Technical Blueprint**:
  - **Part 1: GCS Telemetry Downlink Gate (No Spy Satellite Cheating)**:
    - `get_state_snapshot()` or `c2_gateway.py`: GCS only receives live telemetry for airframes that have an active RF path to the GCS base antenna ($d \le 120\text{m}$, or via connected mesh peers $\le 120\text{m}$).
    - Drones beyond the comms horizon stop streaming live position updates to GCS.
    - In `GCS/frontend/js/app.js`, when telemetry packets cease for a drone, it freezes at its Last Known Position (LKP), renders as an amber/ghost marker with an elapsed lost timer (`[LOST_LINK: 00:14s]`), and halts real-time tracking through the void.
    - Omniscient ground truth remains exclusively in `/sim` (God-View).
  - **Part 2: Universal Lost-Link Recovery (Unstick the Stranded Fleet)**:
    - Remove the `if self.tether_severed` restriction in `sitl_engine.py:516`.
    - In any comms mode (`LEADER_FOLLOWER` or `GOSSIP_CONSENSUS`), when the fleet loses GCS heartbeat for $> 1.0\text{s}$ (`LINK_TIMEOUT`), the 15.0s lost-link loiter countdown starts.
    - When loiter expires, the autonomous flight controller executes automated Stage 1 Overhead Marshal (`command_rtb_hold(force_autonomous=True)`), flying the fleet back to Base $(0, 0, 20\text{m})$ until it re-enters the 120m GCS dome and regains link.
  - **Part 3: Pad Launch Isolation (No Telepathic Echelon Slotting)**:
    - In `GossipConsensusBrain`, when computing guidance for a drone in `LOST_LINK` (or when local peer table has 0 peers in range), the drone does NOT compute an echelon slot against distant un-broadcast targets.
    - If no peers are within 120m, the pad drone holds at Base $(0, 0, 20\text{m})$ or flies as a single-ship scout ($N=1$, apex), rather than assuming a 9th wing slot 40m laterally off a distant target.
* **Targeted Test**: `python -m unittest tests/test_rtb_landing.py tests/test_gossip_formations.py`
