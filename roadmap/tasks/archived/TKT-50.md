# [TKT-50] Dynamic In-Flight Swarm Enrollment & P2P Rejoin Handshake

* **Grade**: ðŸš€ Feature | **Type**: Feature / Autonomous Decentralized Swarm Guidance (`Completed`) | **Priority**: Medium
* **Target Files**: `OBC/backend/brains/gossip_consensus.py`, `OBC/backend/brains/leader_follower.py`, `SIM/backend/sitl_engine.py`, `OBC/backend/drone_firmware.py`, `GCS/frontend/js/hud.js`, `tests/test_in_flight_enrollment.py`
* **Technical Blueprint**:
  - **Part 1: Solo Scout Departure & Search Corridor**:
    - When a reinforcement drone is scrambled from Base to join a distant swarm, it takes off vertically to $Z=20\text{m}$ and transits along a safe search/rejoin corridor toward the last known contact point as a solo scout ($N=1$, apex) instead of jumping to phantom slots.
  - **Part 2: Dynamic In-Flight P2P Enrollment Handshake**:
    - When the scout closes distance to within $60\text{m}$ of any active swarm peer:
      1. Peer-discovery ping triggers a dynamic enrollment handshake.
      2. Living fleet size increments $N \rightarrow N+1$.
      3. Consensus renegotiates formation slots smoothly; the scout slides into its newly assigned outer wing slot without sudden velocity spikes or prop clashes ($d > 1.5\text{m}$).
  - **Part 3: Zero-Clash Formation Insertion**:
    - Relative-velocity damping and slot easing prevent prop clash during wing slot assimilation.
* **Targeted Test**: `python -m unittest tests/test_in_flight_enrollment.py`
