# [TKT-57] Airframe Tier Registry & Mission Authorization Gate

* **Grade**: 🚀 Feature | **Type**: Fleet Governance & Mission Authorization (`In Progress / WIP = 1`) | **Priority**: Medium
* **Target Files**: `OBC/backend/drone_firmware.py`, `SIM/backend/sitl_engine.py`, `GCS/backend/c2_gateway.py`, `GCS/frontend/js/hud.js`, `tests/test_tier_registry_auth.py`
* **Operational Inspiration**:
  - Phase 3 auxiliary missions (`S&R`, `DEPOSIT`, `RECON`, `PATROL_TO_BINGO`, `LIDAR_SCAN`) demand specific airframe capabilities.
  - An operator must not dispatch a heavy payload drop or deep communications relay to an ultralight micro-scout that lacks the motor payload capacity or RF transceivers.
  - The C2 mission gateway and OBC fleet must evaluate airframe tier capabilities before mission authorization, rejecting inappropriate dispatches with deterministic error payloads while keeping swarm consensus intact.
* **Technical Blueprint**:
  - **Part 1: Airframe Tier Taxonomy & Capabilities Registry**:
    - Formalize airframe tiers and role mappings in `OBC/backend/drone_firmware.py` / `c2_gateway.py`:
      - `T0`: Static Repeater / Base Anchor (`Den`, `Totem`, `Cairn`)
      - `T1`: Ultralight Scout (`Threadfin`, `Pup`) -> Authorized: `RECON`, `SCOUT`, `S&R_OPTICAL`
      - `T2`: Standard Tactical Multi-Role (`Alpha`, `Beta`) -> Authorized: `STANDARD_ESCORT`, `PATROL_TO_BINGO`, `LIGHT_DEPOSIT`, `S&R`
      - `T3`: Heavy Lift / Specialist (`Kodiak`, `Mastiff`, `Bloodhound`, `Kourier`) -> Authorized: `HEAVY_DEPOSIT`, `LIDAR_SWEEP`, `RF_JAMMING_TRIANGULATION`, `COMMS_RELAY`
    - Add `tier: str` and `capabilities: List[str]` to `Drone` firmware and state dictionaries.
  - **Part 2: Mission Authorization & Pre-Flight Gate (C2 Gateway)**:
    - Gateway validates requested mission type against available, enrolled, living fleet airframes.
    - If no enrolled airframe satisfies required tier/capabilities, reject dispatch with:
      `{"status": "REJECTED", "reason": "TIER_AUTHORIZATION_FAILED", "required_tier": ["T2", "T3"], "available_tiers": [...]}`.
    - If authorized, tag designated mission airframe IDs in the telemetry mission state.
  - **Part 3: GCS Viewport HUD Capability Indicator**:
    - Display tier badge (`T1`, `T2`, `T3`) in the GCS airframe card and show mission capability status in HUD.
* **Verification Gates**:
  - Dedicated unit test suite in `tests/test_tier_registry_auth.py`.
  - Validates rejection of unauthorized dispatches, acceptance of authorized dispatches, and telemetry downlinks.
