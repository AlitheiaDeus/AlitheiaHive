# [TKT-43] Dynamic Hungarian Formation Inversion & U-Turn Slot Flipping

* **Grade**: 🛠️ Improvement | **Type**: Formation Kinematics & Optimization (`Completed`) | **Priority**: High
* **Target Files**: `OBC/backend/brains/hungarian.py`, `OBC/backend/brains/leader_follower.py`, `OBC/backend/brains/gossip_consensus.py`, `tests/test_brains.py`
* **Technical Blueprint**:
  - Distance-minimizing slot assignment on heading reversals ($> 120^\circ$, $\mathbf{v}_{\text{old}} \cdot \mathbf{v}_{\text{new}} < -0.5$).
  - Solves $\min \sum \|p_i - s_j\|^2$ using pure-NumPy Kuhn-Munkres/Hungarian bipartite matching (`OBC/backend/brains/hungarian.py`) with zero external dependencies.
  - Trailing wingtip drones immediately claim the new vanguard slots in the reversed direction, while forward drones ease back into the trail.
  - Eliminates 80m cross-flight drag races, formation distortion, and prop clash risks during sharp U-turns.
  - Fully compatible with Stage 1 RTB Overhead Marshal equidistant ring holding.
* **Targeted Test**: `python -m unittest tests/test_brains.py` (17/17 passed), full discovery (187/187 passed, 17 skipped), smoke test (0.129s, code 0).
