# [TKT-44] High-Scale Multi-Tier & Concentric Swarm Formations

* **Grade**: ðŸŽ­ Gimmick (Parked / Frozen) | **Type**: Large-Scale Swarm Formations (`Frozen`) | **Priority**: Very Low
* **Status**: **FROZEN** (Parked in Frozen column to eliminate premature scale bloat and focus on core swarm C2).
* **Target Files**: `OBC/backend/brains/`, `SIM/backend/sitl_engine.py`
* **Technical Blueprint**:
  - Prevent formation scale blowout and RF link degradation at $N \ge 16\text{--}48$.
  - Multi-tier stacked echelons: Stacked sub-flights (Alpha, Bravo, Charlie) stepped 10m back and 2.5m higher like a military strike package.
  - Concentric orbital shells for `ORBIT_VORTEX` and `PERIMETER`: Inner shell ($R=12\text{m}$), Mid shell ($R=24\text{m}$), Outer shell ($R=36\text{m}$).
  - Keeps all airframes within dense RF range ($< 60\text{m}$) while maintaining full 3m safety clearance envelope.
* **Targeted Test**: `python -m unittest tests/test_brains.py`
