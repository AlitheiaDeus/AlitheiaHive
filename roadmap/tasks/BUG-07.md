# BUG-07: Comms LKP Continuous Refresh & Launchpad Snap Fix

> **Category**: 🐛 Bug | **Priority**: High | **State**: todo  
> **Operational Inspiration**: Drones snapping/teleporting to launchpad on comms drop because LKP was never refreshed during flight

---

## 1. Operational Overview & Objectives

In the tactical GCS viewport, whenever an airframe loses comms (e.g. entering an RF shadow, exceeding radio horizon, or crashing into building geometry), the GCS freezes its displayed position at its Last Known Position (`LKP`).

**The Bug**:
- Currently, `drone.lkp` is initialized at airframe spawn (`[0.0, -6.0, 0.5]`).
- During active flight with healthy downlink, `drone.lkp` was **never continuously updated** in `SIM/backend/sitl_engine.py` or `OBC/backend/drone_firmware.py`.
- Consequently, the moment comms dropped (`downlink_active = False`), the telemetry serializer transmitted the stale initial spawn coordinates (`[0.0, -6.0, 0.5]`).
- In the operator's 3D viewport, a drone flying at `[40, 30, 25]` instantly "teleported" back to the launchpad on pad 0.

**Objective**:
- Continuously refresh `drone.lkp = np.copy(drone.pos)` whenever downlink is active.
- When downlink drops, `drone.lkp` preserves the exact 3D coordinates from the last valid frame prior to packet loss.
- Ensure the GCS rendering cleanly displays the frozen ghost at the true point of signal loss, not the launchpad.

---

## 2. Target Files & Architectural Scope

* `SIM/backend/sitl_engine.py`: In the simulation tick / telemetry generation loop, update `drone.lkp` whenever `drone.downlink_active` is True.
* `OBC/backend/drone_firmware.py`: Ensure OBC maintains local LKP sync from navigation estimators.
* `tests/test_rf_channel.py` (or new test): Unit test asserting that during active downlink `drone.lkp` equals `drone.pos`, and after link loss `drone.lkp` retains the last active position while `drone.pos` diverges or halts.

---

## 3. Implementation Blueprint

1. **SITL Engine LKP Tracking**:
   - In `sitl_engine.py`, during airframe physics integration or telemetry packaging:
     ```python
     if drone.downlink_active:
         drone.lkp = np.copy(drone.pos)
     ```
2. **Telemetry Serialization Invariant**:
   - In `get_telemetry_frame()`:
     ```python
     pos = drone.pos if drone.downlink_active else drone.lkp
     ```
   - Verify that when `downlink_active == False`, `pos` matches the last position before drop, not `[0.0, -6.0, 0.5]`.

---

## 4. Acceptance Criteria & Invariants

* [x] Airframe `lkp` dynamically tracks `pos` while `downlink_active == True`.
* [x] When `downlink_active` transitions to `False`, `lkp` freezes at the exact coordinates of loss.
* [x] Telemetry frame reports the frozen coordinates, eliminating the launchpad snap artifact.
* [x] Targeted regression unit test passes (`python -m unittest tests/test_rf_channel.py`).
