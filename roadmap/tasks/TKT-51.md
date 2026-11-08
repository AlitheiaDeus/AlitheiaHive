# [TKT-51] Physical Courier / Search-and-Recall Rescue Protocol (Data Mule to LKP)

* **Grade**: 🚀 Feature | **Type**: Autonomous Mission / Search & Rescue (`In Progress / WIP = 1`) | **Priority**: Medium
* **Target Files**: `GCS/backend/c2_gateway.py`, `SIM/backend/sitl_engine.py`, `OBC/backend/drone_firmware.py`, `GCS/backend/web_server.py`, `GCS/frontend/index.html`, `GCS/frontend/css/style.css`, `GCS/frontend/js/hud.js`, `tests/test_escort_courier_lkp.py`
* **Operational Inspiration**:
  - Tactical messenger pigeon / data mule dispatched to the Last Known Position (LKP) of dark, disconnected wingmen trapped beyond the radio horizon or in RF shadow zones.
  - Replaces fragile daisy-chain mesh hops and dangerous expanding search spirals with deterministic point-to-point corridor navigation, stationary P2P beacon dwell, and tandem escort return.
  - 1-Click UI operator trigger in GCS Tactical Missions panel (`SEARCH & RECALL`).
  - Pre-flight capability authorization: verifies `RECON` and `S&R_OPTICAL` capabilities before dispatch.
* **Technical Blueprint**:
  - **Part 1: C2 Gateway Courier Dispatch & Allocation**:
    - `deploy_search_and_recall(target_drone_id: Optional[int] = None, courier_id: Optional[int] = None) -> Dict[str, Any]` in `c2_gateway.py`.
    - Automatically identifies dark/lost-link target drone with known LKP (`[x, y, z]`).
    - Verifies pre-flight authorization for courier airframe (`RECON` / `S&R_OPTICAL`).
    - Loads courier with target LKP and return corridor intent.
  - **Part 2: Radio-Silent Corridor Transit to LKP**:
    - Courier navigates autonomously to target $p_{\text{LKP}}$.
    - GCS marks courier status as `COURIER_TRANSIT` (radio silence once beyond radio range).
  - **Part 3: Point-to-Point Rendezvous & P2P Rescue Handshake**:
    - Arriving at $p_{\text{LKP}}$, courier dwells stationary (no blind expanding spiral into obstacles).
    - Broadcasts P2P rescue pings across $60\text{m}$ radius (`RANGE_P2P`).
    - Upon contact with lost wingman ($d \le 60\text{m}$):
      - Handshake delivers return directive.
      - Lost drone clears lost-link loiter and locks onto courier in tandem escort formation ($d \approx 4\text{m}$, Zone 2 clearance).
  - **Part 4: Reverse Corridor Escort & Comms Horizon Handover**:
    - Courier escorts the rescued wingman back along the safe return corridor toward Base.
    - When the pair penetrates the GCS radio horizon ($d \le 140\text{m}$ Howl / $70\text{m}$ Bork), GCS downlink resumes.
    - Recovered drone clears escort state and re-enrolls into the main fleet.
  - **Part 5: MIA Timeout & Safeguards**:
    - If no peer responds within $15.0\text{s}$ stationary dwell window at LKP, courier concludes target is MIA/destroyed.
    - Courier turns around and returns to Base alone with `SWARM_MIA_AT_LKP` mission debrief.
    - `stop_search_and_recall()` provides clean operator abort.
* **Verification Gates**:
  - Unit tests in `tests/test_escort_courier_lkp.py`.
  - Zero regression across existing test suites (`test_patrol_bingo_rtb.py`, `test_tier_registry_auth.py`, `test_waypoint_engine.py`, `test_rtb_landing.py`).
  - Headless smoke test passes (`run.py --smoke-test`).
