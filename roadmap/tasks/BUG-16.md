# BUG-16: Command ATC Recovery via Pure C2 Uplink Packets & Parity Bridge

> **Category**: 🛡️ Architecture / 🐛 Bug | **Priority**: High | **State**: completed  
> **Operational Inspiration**: Decouple ATC recovery from God-mode actuator manipulation; complete Canyon UDP command parity (Red 7 & Yellow 4 Remediation)

---

## 1. Operational Overview & Objectives

In accordance with the **Sim/God-View Airgap** and **Canyon Airgap**, air traffic recovery coordinators and GCS command decks must interface with airframes via authentic command packets, rather than directly manipulating actuators or flight controller registers in memory.

Currently, [`atc_recovery.py`](../../SIM/backend/atc_recovery.py):
- Directly sets `drone.rotor_rpm = 5200.0`, `drone.flight_state = "TAKEOFF"`, and zeroes `drone.vel` in RAM during `command_scramble`.
- Overwrites flight states, targets, and internal gossip registers directly in RAM during `command_rtb_hold` and `command_e_land`.
- Furthermore, when running fully decoupled over UDP, several tactical commands (`E_LAND`, `SET_FIELD_LZ`, `CLEAR_FIELD_LZ`, `TAKEOFF_DRONE`, `DECLARE_LOST`) are missing from [`MavlinkBridge.execute_command`](../../SIM/backend/mavlink_bridge.py) (**Yellow 4** in `.agents/reports/AIRGAP_VIOLATIONS_REPORT.md`), causing them to drop with `UNKNOWN_COMMAND`.

This ticket refactors ATC recovery to issue standardized C2 command packets to drone inboxes, and implements the missing command handlers in `MavlinkBridge` for full Canyon UDP parity.

---

## 2. Target Files & Architectural Scope

* [`SIM/backend/atc_recovery.py`](../../SIM/backend/atc_recovery.py): Refactor `command_scramble`, `command_rtb_hold`, and `command_e_land` to deliver command packets to drone inboxes.
* [`OBC/backend/drone_firmware.py`](../../OBC/backend/drone_firmware.py): Have onboard flight controller firmware transition flight states and ramp rotor RPM in response to received command packets.
* [`SIM/backend/mavlink_bridge.py`](../../SIM/backend/mavlink_bridge.py): Add command dispatch handlers for `COMMAND_E_LAND`, `SET_FIELD_LZ`, `CLEAR_FIELD_LZ`, `TAKEOFF_DRONE`, and `DECLARE_LOST`.
* [`tests/test_rtb_landing.py`](../../tests/test_rtb_landing.py): Verify ATC procedures execute cleanly via packet interfaces.

---

## 3. Implementation Blueprint

### A. ATC Command Packet Delivery (`atc_recovery.py`)
- In `command_scramble`:
  - Instead of forcing `drone.rotor_rpm = 5200.0` and `drone.flight_state = "TAKEOFF"`, queue `RadioPacket(sender_id=-1, msg_type="C2_COMMAND", payload={"cmd": "TAKEOFF"})` into `drone.inbox`.
  - Let `Drone.step` / FC twin handle motor arming and vertical climb.
- In `command_rtb_hold`:
  - Transmit `RadioPacket(sender_id=-1, msg_type="C2_COMMAND", payload={"cmd": "HOLD_POSITION", "target": target_hold})` to drones.
- In `command_e_land`:
  - Transmit `RadioPacket(sender_id=-1, msg_type="C2_COMMAND", payload={"cmd": "E_LAND", "spot": spot.tolist(), "reason": reason})` to target drone.

### B. Canyon UDP Command Parity in `MavlinkBridge` (`mavlink_bridge.py` - Yellow 4)
- Add command handlers in `MavlinkBridge.execute_command`:
  - `"COMMAND_E_LAND"` / `"E_LAND"`: Delegate to `engine.command_e_land(cmd.get("drone_id", 0))`.
  - `"SET_FIELD_LZ"`: Delegate to `engine.set_field_lz(...)`.
  - `"CLEAR_FIELD_LZ"`: Delegate to `engine.clear_field_lz(...)`.
  - `"TAKEOFF_DRONE"`: Deliver takeoff command packet to specified drone.
  - `"DECLARE_LOST"`: Mark specified drone comms lost / dead-reckoning.

---

## 4. Acceptance Criteria & Verification Invariants

* [x] ATC recovery coordinator does not directly overwrite rotor RPM or zero velocities in RAM.
* [x] Airframes initiate takeoff and e-landing from parsed `C2_COMMAND` packets.
* [x] Decoupled UDP commands (`E_LAND`, `SET_FIELD_LZ`, `CLEAR_FIELD_LZ`, `TAKEOFF_DRONE`, `DECLARE_LOST`) execute without falling through to `UNKNOWN_COMMAND`.
* [x] Targeted test module passes: `python -m unittest tests/test_mavlink_bridge.py tests/test_field_lz_and_eland.py`.
