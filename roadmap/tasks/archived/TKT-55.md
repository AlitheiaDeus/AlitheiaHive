# [TKT-55] Tactical Formation Orientations & Perimeter In/Out Headings

* **Grade**: 🚀 Feature | **Type**: Formation Guidance & Kinematics (`Completed / Fresh Review`) | **Priority**: High
* **Target Files**: `OBC/backend/brains/leader_follower.py`, `OBC/backend/brains/gossip_consensus.py`, `SIM/backend/sitl_engine.py`, `tests/test_formation_flight_suite.py`
* **Operational Inspiration**:
  - Drones currently freeze their yaw heading at whatever arbitrary arrival angle they stopped at when cruising speed drops below $0.5\text{m/s}$.
  - In tactical formations, physical sensor and weapon orientation must align with the mission:
    - `PERIMETER_PATROL_IN`: Sentry nodes face inward toward the protected target/VIP.
    - `PERIMETER_PATROL_OUT`: Defensive radar sweep facing radially outward.
    - `ECHELON_ESCORT`: Wingmen align with the formation corridor heading ($\psi_{\text{corr}}$).
* **Technical Blueprint**:
  - **Part 1: Formation Target Yaw Generation**:
    - In `leader_follower.py` & `gossip_consensus.py`:
      - For `PERIMETER_PATROL` (default inward):
        $$\psi_{\text{target}} = \text{atan2}(y_{\text{center}} - y_i, x_{\text{center}} - x_i)$$
      - For `PERIMETER_PATROL_OUT`:
        $$\psi_{\text{target}} = \text{atan2}(y_i - y_{\text{center}}, x_i - x_{\text{center}})$$
      - For `ECHELON_ESCORT` and transit modes:
        $$\psi_{\text{target}} = \psi_{\text{formation}}$$
  - **Part 2: Kinematic Slew Rate Enforcement**:
    - SITL world engine enforces bounded yaw acceleration and slew rate ($\le 45^\circ/\text{s}$), preventing sudden angular snaps.
  - **Part 3: Mode Toggles in GCS**:
    - Extend `SET_FORMATION` to support `PERIMETER_IN` and `PERIMETER_OUT`.
* **Targeted Tests**: `python -m unittest tests/test_formation_flight_suite.py`
