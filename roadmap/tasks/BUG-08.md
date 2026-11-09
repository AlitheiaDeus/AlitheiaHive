# BUG-08: Ghost Flagship Demotion & Casualty Badge Precedence

> **Category**: 🐛 Bug | **Priority**: High | **State**: todo  
> **Operational Inspiration**: Demoted and casualty drones retaining FLAGSHIP status in telemetry and masking crash/lost-link badges

---

## 1. Operational Overview & Objectives

In the default urban obstacle scenario, followers in wide formation slots impacted building walls. Following crashes or signal loss, operators observed **multiple airframes embedded in buildings displaying as `FLAGSHIP`**.

**Root Causes**:
1. **Unenforced Demotion**: When a leader airframe crashed or lost comms, autonomic succession promoted a healthy wingman to leader (`is_leader = True`). However, the former leader was not explicitly demoted (`is_leader = False`) in its state or in frozen telemetry frames.
2. **Badge Evaluation Precedence**: In the GCS frontend (`GCS/frontend/js/app.js` and `hud.js`), status badge evaluation checked `if (d.is_leader)` with higher priority than `else if (d.lost_link)` or before casualty status, causing frozen dead leaders to permanently glow with the gold `FLAGSHIP` badge instead of red `CRASH` / amber `LOST LINK`.

**Objective**:
- Enforce strict single-flagship invariant: exactly one active airframe in the swarm carries `is_leader = True`. Demoted or casualty drones must have `is_leader = False`.
- In GCS HUD and 3D billboard sprites, enforce status precedence: `crashed` / `e_landed` / `lost_link` take precedence over or qualify flagship status so casualties are unmistakable.

---

## 2. Target Files & Architectural Scope

* `SIM/backend/sitl_engine.py`: Autonomic leader succession must demote prior leaders (`prior_leader.is_leader = False`).
* `OBC/backend/drone_firmware.py`: OBC state machine demotes airframe when succession message is received or when entering terminal casualty state.
* `GCS/frontend/js/app.js`: Ensure billboard sprite color and text badge logic prioritize `CRASHED` and `LOST_LINK` over `FLAGSHIP`.
* `GCS/frontend/js/hud.js`: Ensure telemetry card header displays casualty state prominently even if airframe was a leader.
* `tests/test_leader_succession.py` (or targeted test): Verify that only one drone in telemetry has `is_leader = True` and crashed leaders are demoted.

---

## 3. Implementation Blueprint

1. **Strict Demotion on Succession & Crash**:
   - In `sitl_engine.py`:
     ```python
     if old_leader is not None:
         old_leader.is_leader = False
     ```
   - When a drone crashes or enters `CRASHED` state, clear `drone.is_leader = False`.
2. **Frontend Badge Precedence**:
   - In `app.js` sprite generation:
     ```javascript
     if (d.crashed) {
         statusText = "CRASH";
         badgeColor = "#ff0055";
     } else if (d.lost_link) {
         statusText = "LOST";
         badgeColor = "#ffaa00";
     } else if (d.is_leader) {
         statusText = "FLAGSHIP";
         badgeColor = "#00f0ff";
     }
     ```
   - In `hud.js`, display `CRASHED` in red banner for casualties.

---

## 4. Acceptance Criteria & Invariants

* [x] At any simulation tick, at most ONE airframe has `is_leader == True`.
* [x] Crashed or demoted airframes immediately have `is_leader == False`.
* [x] GCS 3D viewport renders `CRASH` (red) or `LOST` (amber) on dead/disconnected airframes, never active `FLAGSHIP`.
* [x] Unit test verifies single-leader invariant during cascading casualties.
