# TKT-69: Migrate Zone 2 & 3 Avoidance Reflexes to OBC Firmware (Airgap Enforcement)

> **Category**: 🚀 Feature / 🛡️ Architecture | **Priority**: High | **State**: in_progress  
> **Operational Inspiration**: Strict 4-Airgap Enforcement between OBC Autonomic Reflexes and SIM God-View Physics

---

## 1. Operational Overview & Objectives

In accordance with the **4 Explicit Airgaps** of the 50/50 Architecture, guidance intelligence, spatial awareness, and avoidance reflexes must execute within the drone's onboard compute bubble (Bubble 2: OBC Brain Airgap). Physics simulation, ground truth state, and destructive crash consequences belong strictly to the SITL World Engine (Bubble 3: God-View / Sim Airgap).

Previously, Zone 2 and Zone 3 repulsive forces (`a_avoid`) were computed inside SIM's `_compute_4zone_avoidance()` and injected directly into the kinematic integration `drone.acc = total_acc + a_avoid`. This violated the Brain Airgap by acting as an omniscient physics "bumper car" force rather than an onboard pilot reflex.

This ticket migrates Zone 2 and Zone 3 avoidance reflexes into OBC firmware (`OBC/backend/brains/`), ensuring:
1. **Zone 1 (Nominal Guidance)**, **Zone 2 (Caution Separation)**, and **Zone 3 (Emergency Thrust + Momentum Damping)** are calculated by OBC brains using strictly local telemetry, RF peer broadcasts, and onboard proximity bumpers.
2. Forward guidance intent is automatically attenuated when evasive maneuvers demand high acceleration, preventing guidance from overriding evasive reflexes.
3. **Zone 4 ($d \le 1.5\text{m}$ Prop Clash & Structural Impact)** remains exclusively in SIM as physical ground truth damage (cutting rotor RPM, triggering ballistic tumble, and destroying airframes).
4. No arcade repulsive forces are injected by SIM into drone kinematic acceleration.

---

## 2. Target Files & Architectural Scope

* `OBC/backend/brains/base.py`: Add pure geometric math helpers (`_is_on_obstacle_top`, `_point_to_aabb_repulsion`), implement `compute_inter_drone_avoidance()`, `compute_obstacle_avoidance()`, and `apply_avoidance_reflex()`.
* `OBC/backend/brains/leader_follower.py`: Route all active flight states through `_finalize_flight_intent()` wrapping `apply_avoidance_reflex()`.
* `OBC/backend/brains/gossip_consensus.py`: Route active flight modes through `_finalize_flight_intent()` and synchronize echelon freeze constants (`FREEZE_SPEED = 1.5`, `FREEZE_DIST = 8.0`).
* `SIM/backend/sitl_engine.py`: Extract `_check_zone4_physical_collisions()` for structural clash and prop strike destruction. Remove `+ a_avoid` from kinematic acceleration integration. Pass `nearby_peers` and `urban_obstacles` into immutable `LocalTelemetry`.
* `tests/test_pad_scramble.py`: Adjust simulation timing for clean vertical pad ascent to Base muster.

---

## 3. Implementation Blueprint

### A. Pure Math Geometric Helpers in OBC (`OBC/backend/brains/base.py`)
- Extracted standalone pure-math helpers `_is_on_obstacle_top` and `_point_to_aabb_repulsion` before `BaseBrain` to avoid any forbidden imports from `SIM.backend.urban_geometry` (satisfies AST airgap scan in `test_airgap_integrity.py`).
- Added `nearby_peers` and `urban_obstacles` fields to `LocalTelemetry`.

### B. Onboard Inter-Drone & Obstacle Avoidance (`BaseBrain`)
- `compute_inter_drone_avoidance(telemetry, peers=None)`:
  - Skips `LANDED`, `GROUND_IDLE`, `FIELD_LANDED`, and `E_LANDED` drones so landed pad airframes do not push airborne drones.
  - Suppresses horizontal avoidance during pad climb (`TAKEOFF` when $Z < 5.0\text{m}$) and pad docking (`DESCENDING`, `FIELD_DESCENDING`, `EMERGENCY_LANDING` when $Z < 3.0\text{m}$).
  - Evaluates Zone 2 caution buffer ($3.5 < d \le 8.0\text{m}$), Zone 3 emergency thrust ($1.5 < d \le 3.5\text{m}$), lateral/vertical corridor breakout, relative-velocity momentum damping ($\mathbf{a}_{\text{damp}} = -3.5 (\mathbf{v}_{\text{closing}} \cdot \mathbf{n}_{ij})$), and P2P RSSI emergency cushion ($d \le 2.2\text{m}$).
- `compute_obstacle_avoidance(telemetry, obstacles=None)`:
  - Repels from static urban AABBs when within $3.5\text{m}$ using local proximity bumper data.
- `apply_avoidance_reflex(telemetry, a_nominal, peers=None)`:
  - Blends nominal guidance acceleration with Zone 2 & 3 evasion reflexes.
  - Attenuates nominal guidance when evasive acceleration exceeds $8.0\text{ m/s}^2$ ($a_{\text{atten}} = \max(0.1, 1.0 - (\text{avoid\_mag} - 8.0) / 10.0)$).
  - Clamps combined acceleration to $A_{\max} = 15.0\text{ m/s}^2$.

### C. Brain Guidance Integration (`leader_follower.py` & `gossip_consensus.py`)
- `LeaderFollowerBrain` and `GossipConsensusBrain` route all active flight intent returns through `apply_avoidance_reflex()`.
- Synchronized echelon freeze thresholds (`FREEZE_SPEED = 1.5`, `FREEZE_DIST = 8.0`) between both brain implementations to eliminate off-balance heading drift and pinwheel spin.

### D. SITL Physics Airgap (`SIM/backend/sitl_engine.py`)
- Removed `+ a_avoid` from kinematic acceleration integration: `total_acc = guidance_accs[drone.id] + tether_accs[drone.id]`.
- Implemented `_check_zone4_physical_collisions()`:
  - Inter-drone prop clash ($d \le 1.5\text{m}$) and building structural clash ($d \le 1.5\text{m}$) trigger `_trigger_crash(reason=...)`, cutting RPM to $0.0$ and initiating ballistic tumble.
- Refactored `_compute_4zone_avoidance()` to delegate to `_check_zone4_physical_collisions()` and query OBC reflexes for diagnostics.

---

## 4. Acceptance Criteria & Verification Invariants

* [x] **Airgap Integrity AST Scan**: Zero cross-bubble imports from `SIM` or `GCS` into `OBC/` (`test_airgap_integrity.py` passes).
* [x] **Zero Arcade Physics**: SIM kinematic acceleration does not inject `a_avoid`; Zone 4 clash strictly destroys airframes.
* [x] **Stable Launchpads**: Landed drones (`LANDED`, `GROUND_IDLE`, `FIELD_LANDED`, `E_LANDED`) do not repel airborne drones.
* [x] **Formation Heading Agreement**: Gossip and Leader-Follower brains agree on heading and avoid asymmetric pinwheel drift (`test_gossip_formations.py` passes 11/11).
* [x] **Full Regression Pass**: All targeted tests and headless smoke test pass with zero regressions.
