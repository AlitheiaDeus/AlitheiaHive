# TKT-66: Simplified RF & GNSS Shadow Occlusion Engine

> **Category**: 🚀 Feature | **Priority**: High | **State**: todo  
> **Operational Inspiration**: PLN-08 Ray-Cast RF Shadows, Virtual 4-Satellite GNSS Constellation & Dead-Reckoning Failsafe (50/50 Architecture across 4 Airgaps)

---

## 1. Operational Overview & Objectives

Builds a mathematically grounded yet computationally lightweight **RF & GNSS Shadow Occlusion Engine** leveraging our existing 3D AABB terrain blocks and altitude shelves (`TKT-60` / `TKT-65`):

1. **RF Line-of-Sight (LOS) Occlusion**:
   * Evaluates line-of-sight rays between transmitting and receiving antennas (Leader $\leftrightarrow$ Follower, Follower $\leftrightarrow$ Follower, GCS $\leftrightarrow$ Fleet).
   * Packets traversing solid obstacles (`UrbanObstacle.blocks_rf == True`) suffer complete packet drop or severe dB attenuation ($>40\text{dB}$), inducing realistic comms blackout zones behind high-rises and mesas.
   * Naturally exercises autonomous failsafes: Heartbeat 1–3 gossip healing, Lost-Link loiter, and Breadcrumb Backtracking (`TKT-54`).

2. **Simplified Virtual 4-Satellite GNSS Constellation**:
   * Avoids heavy orbital ephemeris math by defining a **fixed tactical 4-satellite constellation** above the AO:
     - $\mathbf{S}_1$ (North-East): $\text{Az} = 45^\circ, \text{El} = 60^\circ$
     - $\mathbf{S}_2$ (South-East): $\text{Az} = 135^\circ, \text{El} = 50^\circ$
     - $\mathbf{S}_3$ (North-West): $\text{Az} = 315^\circ, \text{El} = 55^\circ$
     - $\mathbf{S}_4$ (Zenith / Overhead): $\text{Az} = 0^\circ, \text{El} = 85^\circ$
   * Each tick, for each drone, casts 4 upward rays: $\mathbf{r}_k(t) = \mathbf{p}_{\text{drone}} + t \cdot \hat{\mathbf{u}}_k, t \in [0, \infty)$.
   * An obstacle shelf (e.g. `SKYWAY-DECK` at $Z \in [8, 11]\text{m}$ or `MESA-HIGH` at $Z = 14\text{m}$) occludes any satellite ray it intersects.
   * **Fix State Machine**:
     - **4 Satellites Visible**: `3D_FIX` (Ideal precision, nominal flight).
     - **3 Satellites Visible**: `2D_FIX` (Degraded vertical accuracy, altitude hold gated by barometric twin).
     - **$< 3$ Satellites Visible**: `GNSS_DENIED` / `DEAD_RECKONING` (Flying under bridge or deep urban canyon shadow; triggers drift accumulation or blind settle failsafe).

3. **Canyon Airgap & Telemetry Hydration**:
   * Telemetry frame downlinks `gnss_sats_visible: int` and `gnss_fix: str` for each airframe.
   * GCS renders live GNSS health pills (`GNSS 4/4 FIX`, `GNSS 2/4 DEGRADED`, `GNSS 0/4 DENIED`) in the tactical HUD.

---

## 2. Target Files & Architectural Scope

* `SIM/backend/urban_geometry.py`:
  - Definition of `VIRTUAL_GNSS_CONSTELLATION` unit direction vectors.
  - `check_gnss_satellite_visibility(pos, obstacles)`: Ray-casts upward against all active AABBs, returning a boolean mask and visible count.
  - `check_rf_los(p1, p2, obstacles)`: Ray-casts between two 3D points.
* `SIM/backend/rf_channel.py`:
  - Enforce RF packet drops when `check_rf_los` reports obstacle obstruction.
* `SIM/backend/sitl_engine.py`:
  - Per-tick GNSS visibility evaluation for all drones.
  - Hydrate telemetry frames with GNSS health data.
* `OBC/backend/drone_firmware.py`:
  - Local airframe reaction to degraded GNSS (dead-reckoning drift and warning squawks).
* `GCS/frontend/js/app.js`:
  - Three.js tactical HUD badge updates reflecting GNSS constellation health.
* `tests/test_rf_gnss_shadows.py`:
  - Test suite validating RF drop-off behind mesas, under-bridge GNSS shadow ($0/4$ sats), open-air $4/4$ fix, and dead-reckoning state transitions.

---

## 3. Acceptance Criteria & Invariants

* [ ] `check_gnss_satellite_visibility` returns 4/4 visible satellites on open terrain.
* [ ] Drones flying underneath `SKYWAY-DECK` ($Z < 8\text{m}$) have zenith satellite $\mathbf{S}_4$ occluded.
* [ ] Drones positioned in the shadow of `PILLAR-TOWER` or `MESA-HIGH` correctly lose visibility to low-elevation azimuth satellites.
* [ ] Drones with $< 3$ satellites transition to degraded/denied GNSS state.
* [ ] RF packets between two drones blocked by `MESA-HIGH` are dropped or attenuated.
* [ ] Full test suite in `tests/test_rf_gnss_shadows.py` passes 100%.
