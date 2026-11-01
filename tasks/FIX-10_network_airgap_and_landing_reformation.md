# [FIX-10] Strict Network Decoupling, Landing Huddle Fix & 3-Way Death Doctrine

* **Grade**: 🛠️ Improvement | **Type**: Architectural Airgap, Network Sockets & Guidance (`Completed`) | **Priority**: High
* **Target Files**: `SIM/backend/sitl_engine.py`, `OBC/backend/drone_firmware.py`, `GCS/backend/web_server.py`, `GCS/backend/session_manager.py`, `GCS/frontend/js/hud.js`, `tests/test_rtb_landing.py`, `tests/test_bubble_decoupling.py`, `tests/test_web_server.py`
* **Operational Inspiration**:
  - Resolves landing pad huddle bug: Airborne followers congregating into a single pad on leader landing.
  - Implements 3 realistic casualty identification pathways (LKP timeout / operator manual declaration, proximity mid-air clash inference, mayday impact beacon).
  - Eliminates GCS direct memory peeking across the canyon boundary.
  - Enforces radio horizon gating across all operator flight intents (`SET_FORMATION`, `SET_BRAIN`, `LAND_DRONE`).
* **Technical Blueprint**:
  - **Part 1: Landing Pad "Huddle" Bug Remediation**:
    - In `sitl_engine.py`: updated `leader` property and added check so that when the current leader transitions to `DESCENDING` or `LANDED`, it abdicates leadership.
    - Trigger `_trigger_autonomic_succession(demote_old_leader=True)` to promote highest-ranking airborne escort in the overhead holding ring.
    - Hardened `sitl_engine.py` landing pad resolution: eliminated `landing_pad_idx or 0` fallback to Pad 0; defaults to safe overhead hover ($Z=25\text{m}$) if unassigned.
  - **Part 2: 3-Way Realistic Casualty & Destruction Doctrine**:
    - Path 1: `LOST_LINK` at LKP with operator "Declare Destroyed" action (`DECLARE_DESTROYED` / manual confirmation).
    - Path 2: Pre-loss proximity clash inference ($d \le 1.5\text{m}$) with `collision_inferred = True`.
    - Path 3: Surviving micro-beacon broadcast `MAYDAY: IMPACT_DETECTED` on physical collision within radio contact.
    - Removed direct omniscient memory peeking into `sim.drones` from `session_manager.py` and `web_server.py`; state derived strictly from downlinked telemetry frames.
  - **Part 3: Canyon Radio Horizon Gating**:
    - In `sitl_engine.py`: gated `set_formation()`, `command_land_drone()`, `command_takeoff_drone()`, `set_brain()` with `is_in_comms_range()` and `drone.uplink_active`.
    - In `GCS/backend/web_server.py`: dropped out-of-range commands with explicit status responses.
    - Stripped dead casualty injection listeners from `GCS/frontend/js/hud.js` and added HUD status badges for the 3 death doctrines.
* **Verification & Results**:
  - `tests/test_rtb_landing.py`: 16/16 PASSED (landing pad stagger, marshal hold, cancel rtb, pad allocations).
  - `tests/test_bubble_decoupling.py`: 4/4 PASSED (airgap isolation, socket bridges).
  - `tests/test_web_server.py`: 7/7 PASSED (REST/WebSocket API, telemetry dispatch).
  - Full automated unit test suite: **196/196 PASSED** in 17.258s.
  - Headless smoke test: **PASS** (0.108s, 120 steps @ 60 FPS, 0 NaNs).
