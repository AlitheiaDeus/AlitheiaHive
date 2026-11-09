# TKT-64: Designated Field Landing Zones (FLZ) & Autonomous Emergency Landings (E-Land)

> **Category**: 🚀 Feature | **Priority**: High | **State**: completed  
> **Operational Inspiration**: PLN-05 Off-Nominal Landing Zones & Field E-Land (50/50 Architecture across 4 Airgaps)

---

## 1. Operational Overview & Objectives

Introduces **Designated Field Landing Zones (FLZ)** and **Autonomous Emergency Landings (E-Land)** without breaking the primary Base launchpad recovery workflow:

1. **Default Recovery Remains Base Pads**:
   - The primary recovery destination remains the Base launchpad (centered at `[0.0, -6.0, 0.5]` with 8 concrete pads `PAD 0`–`PAD 7`).
   - Standard `RTB_HOLD` and `LAND_SWARM` continue to target Base pads by default.
2. **Designated Field Landing Zones (FLZ)**:
   - Used strictly when required by mission parameters or necessity:
     - *Mission Requirement*: The mission explicitly sets a remote/forward operating field LZ (e.g. forward recon outpost, initial remote field launch without base pads).
     - *Launchpad Denial / Narrow Pad Clearance*: Base pads are blocked by an obstacle/wreckage, or declared too narrow/unsafe (due to absence of high-precision fiducial vision or RTK carrier-phase positioning on micro-drones), or pad capacity is exceeded.
   - Drones touch down with wide tactical field separation ($\ge 3.5\text{m}$ to $5.0\text{m}$ spacing in an open radial ring or grid) at ground elevation ($Z = 0.05\text{m}$), eliminating prop clash without requiring tight concrete pads.
3. **Autonomous Emergency Landings (E-Land)**:
   - Triggered when an airframe is **severely compromised** (rotor failure, structural damage, critical casualty), **stuck in link-loss without forward progress**, or **Beyond BINGO**.
   - *BINGO vs Beyond BINGO*:
     - **BINGO**: The calculated battery required to transit back home to Base or FLZ:
       $$E_{\text{needed}} = \left(\frac{d_{\text{home}}}{v_{\text{cruise}}}\right) \times \dot{B}_{\text{drain}} + B_{\text{safety\_margin}}$$
     - **Beyond BINGO**: Battery remaining has dropped below $E_{\text{needed}} - \Delta_{\text{margin}}$. Continuing toward Base will result in mid-air motor power exhaustion and a catastrophic ballistic crash.
   - *Safe Flat Space Search*:
     - The drone scans surrounding ground space ($r \le 20\text{m}$) using analytical distance queries against all active `UrbanObstacle` 3D bounding boxes (`point_to_aabb_distance`).
     - Rejects any spot within $3.0\text{m}$ of a building wall or directly under an obstacle shelf.
     - Ensures $\ge 3.0\text{m}$ separation from any other grounded airframe.
     - Executes a controlled descent ($v_z = -1.2\text{ m/s} \to -0.3\text{ m/s}$ cushion) directly onto the flat ground.
     - Cuts motor power upon contact (`rotor_rpm = 0.0`), transitions to `E_LANDED`, squawks emergency beacon coordinates in telemetry, and autonomic leader succession promotes an airborne peer if the casualty was the flagship.

---

## 2. Target Files & Architectural Scope

* `OBC/backend/drone_firmware.py`: `FlightState` members (`FIELD_HOLD`, `FIELD_DESCENDING`, `FIELD_LANDED`, `EMERGENCY_LANDING`, `E_LANDED`), beyond-BINGO calculation, and airframe state attributes.
* `SIM/backend/atc_recovery.py`: `FieldLandingZone` dataclass, FLZ slot allocation, pad denial detection, safe flat space search algorithm, and E-land dispatchers.
* `SIM/backend/sitl_engine.py`: Beyond-BINGO tick watchdog, guidance state machine overrides for `FIELD_DESCENDING` and `EMERGENCY_LANDING`, touchdown motor cutoff, and telemetry hydration.
* `GCS/backend/c2_gateway.py` & `GCS/backend/web_server.py`: C2 handlers for `SET_FIELD_LZ`, `CLEAR_FIELD_LZ`, `COMMAND_E_LAND`, `COMMAND_E_LAND_SWARM`, and destination routing for `COMMAND_RTB_HOLD` / `COMMAND_LAND_SWARM`.
* `GCS/frontend/js/app.js`: Three.js rendering of amber/olive tactical Field LZ perimeter ring and HUD status badges (`FIELD_LANDED`, `E-LANDING`, `E-LANDED`).
* `tests/test_field_lz_and_eland.py`: Comprehensive test module validating default Base preservation, FLZ geometry, pad denial auto-diversion, beyond-BINGO detection, obstacle-avoiding flat space search, and leader succession.

---

## 3. Acceptance Criteria & Invariants

* [x] Base launchpad pads (0..7) remain 100% default recovery target.
* [x] Setting an active FLZ routes RTB to field coordinates with wide slot spacing ($\ge 3.5\text{m}$).
* [x] Automatic pad denial triggers diversion to FLZ when Base pads are obstructed or declared narrow.
* [x] Beyond-BINGO watchdog triggers autonomous E-landing before battery completely exhausts.
* [x] Safe flat space search chooses clear ground $\ge 3.0\text{m}$ away from obstacles and other airframes.
* [x] Flagship E-land smoothly promotes 2IC via autonomic succession.
* [x] Targeted unit test suite passes with 100% assertions.
