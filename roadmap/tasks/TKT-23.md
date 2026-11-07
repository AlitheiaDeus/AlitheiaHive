# [TKT-23] Natural Boids Flocking Architecture: True Reynolds 3D Flocking

* **Grade**: ðŸš€ Feature | **Type**: Feature / Bio-Inspired Swarm Kinematics (`Completed`) | **Priority**: High
* **Target Files**: `OBC/backend/brains/leader_follower.py`, `OBC/backend/brains/gossip_consensus.py`, `tests/test_brains.py`
* **Technical Blueprint**:
  - Replace rigid Cartesian grid world offsets in `NATURAL_FLOCK` with true Reynolds 3D boid forces:
    1. **Separation ($\mathbf{a}_{\text{sep}}$, radius $< 4.0\text{m}$)**: Inverse-distance quadratic repulsion preventing local crowding, combined with Zone 3 relative-velocity momentum damping.
    2. **Alignment ($\mathbf{a}_{\text{align}}$, radius $< 12.0\text{m}$)**: Velocity matching with neighbors, steering drones toward the average heading/speed of nearby flockmates for sweeping synchronized turns.
    3. **Cohesion ($\mathbf{a}_{\text{coh}}$, radius $< 18.0\text{m}$)**: Attraction toward the local flock centroid, preserving flock integrity without clumping.
    4. **Goal Migration ($\mathbf{a}_{\text{goal}}$)**: Direct proportional steering toward commanded target or patrol route beacon.
  - **Constraints & Safety Invariants**:
    - Altitude floor clamp: $Z \ge 10.0\text{m}$ (anti-ground collision).
    - Tether clearance cylinder: 3.0m radial repulsion when tether is active.
    - Combined control acceleration bounded by FC limit $\|\mathbf{a}\| \le 15.0\text{ m/s}^2$ and cruise speed $v_{\text{cruise}} = 12.0\text{ m/s}$.
  - Seamlessly supported across both `LeaderFollowerBrain` and `GossipConsensusBrain`.
* **Targeted Test**: `python -m unittest tests/test_brains.py`
