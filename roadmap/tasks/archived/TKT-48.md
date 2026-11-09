# [TKT-48] Airframe Hardware Identity & Fleet Enrollment Registry (64-bit MCU UID, Tail Numbers, & MAVLink AUTOPILOT_VERSION)

* **Grade**: ðŸ› ï¸ Improvement | **Type**: Fleet Management & Protocol (`Completed`) | **Priority**: High
* **Target Files**: `SIM/backend/sitl_engine.py`, `SIM/backend/mavlink_bridge.py`, `GCS/frontend/js/hud.js`, `tests/test_mavlink_bridge.py`
* **Technical Blueprint**:
  - **The Airframe Identity Hierarchy**: Replace transient integer array indices (`Drone #0`, `Drone #1`) with a 3-tier realistic identity model:
    1. **Tier 1: Hardware Silicon UID (`hardware_uid`)**:
       - 64-bit / 128-bit immutable hardware serial representing the STM32 MCU UID or Pixhawk serial number (e.g. `0x3842_A10F_9B20_007A`).
       - Stored in persistent airframe metadata; never changes over the life of the physical drone.
       - Tied to physical maintenance logs, flight hour accumulators, and casualty records.
    2. **Tier 2: Tactical Tail Number / Call-Sign (`tail_number`)**:
       - Human-readable alphanumeric call-sign stenciled onto the fuselage and stored in EEPROM (e.g. `ALITHEIA-V01` through `ALITHEIA-V08`, reserves `ALITHEIA-RES-01`).
       - Displayed on GCS HUD badges and voice-synthesized alerts.
    3. **Tier 3: Dynamic Swarm Slot & Network System ID (`sysid`)**:
       - Dynamic tactical role and MAVLink system ID (e.g. Slot 0 = Flagship `sysid 1`, Slot 1 = Escort 1 `sysid 2`).
       - Assigned dynamically during pad boot / enrollment handshake (`ENROLL_AIRFRAME`).
  - **Fleet Enrollment Registry (`FleetRegistry`)**:
    - Central registry tracking:
      `{ hardware_uid: str, tail_number: str, sysid: int, pad_id: int, status: "ACTIVE" | "CASUALTY" | "RESERVE" }`
    - When an airframe crashes (Zone 4 prop clash or ground impact):
      - Status transitions to `CASUALTY`. Airframe's hardware UID and final LKP are permanently preserved in the mission casualty log.
      - Its tactical slot and pad are declared vacant.
    - When a reinforcement launches:
      - A standby reserve airframe with a fresh, distinct hardware UID (e.g. `ALITHEIA-RES-01`) is enrolled and assigned the vacant slot and `sysid`.
  - **MAVLink Protocol Compatibility**:
    - **`AUTOPILOT_VERSION` (#148)**: Responds to requests with `capabilities`, `flight_sw_version`, and `uid2` (128-bit hardware UID byte array) per airframe.
    - **`PARAM_REQUEST_LIST` (#21) & "Getting Params..." Dismissal**:
      - Mission Planner hangs for 15s on "Getting Params..." waiting for parameter enumeration.
      - Implement lightweight MAVLink parameter server responding with minimal parameter table (`SYSID_THISMAV`, `FRAME_CLASS`, `ARMING_CHECK`, `PILOT_SPEED_UP`) and emitting `PARAM_VALUE` (#22).
      - Instantly dismisses the "Getting Params..." dialog in Mission Planner within 0.02s without user intervention.
* **Targeted Test**: `python -m unittest tests/test_mavlink_bridge.py`
