# BUG-19: Cairn G1 Carrier Binding and Target Drop Coordinate Designation

> **Category**: BUG | **Priority**: High | **State**: todo  
> **Operational Inspiration**: Prevent Cairn G1 Repeater drop button from executing blind 40m forward drops, bind drop controls specifically to airframes carrying the G1 payload (`HEAVY_PAYLOAD` & `DROP_SERVO`), provide a pad staging option for T3 Kodiak G1 carriers, and designate explicit drop coordinates.

---

## 1. Operational Overview & Objectives
Operators reported several issues with the Cairn G1 Solar Repeater drop aux mission (`TKT-59`):
1. **Blind Drop Coordinates**: If no target coordinate was specified, `deploy_payload_drop` defaulted to dropping $40\text{m}$ in front of the carrier's heading without operator awareness of where the repeater would touch down.
2. **Missing Carrier in Default Set**: In standard 6- or 8-drone fleets, all airframes are initialized as T1.5 Beta (`BASE_TIER_T1_5`) lacking `HEAVY_PAYLOAD` and `DROP_SERVO`. The button was active on the HUD, but clicking it caused an authorization error.
3. **Unbound Global Button**: The button was labeled as a generic swarm action rather than indicating which specific carrier is armed with the Cairn G1 repeater.
4. **Airframe Hardware Tier Taxonomy Violation**: Any drone was considered as carrying payload, even when lacking drop hardware.

This ticket implements:
* **Specific Carrier Binding in HUD**:
  - The HUD dynamically detects which airborne drone in telemetry has `payload_attached` and `DROP_SERVO`/`HEAVY_PAYLOAD` capabilities.
  - If a carrier exists: the button activates and displays `⬢ CAIRN G1 DROP (DRONE #X)`.
  - If no carrier exists in the fleet: the button is visually dimmed and displays `⬢ CAIRN G1 [NO CARRIER]` with an informative tooltip.
* **Explicit Target Drop Coordinates**:
  - When `DEPLOY_PAYLOAD_DROP` is dispatched, it defaults to the active staged target coordinates `[x, y, 0.0]` or currently commanded mission waypoint rather than blind $40\text{m}$ forward projection.
  - The 3D viewport renders a dedicated drop footprint marker on the terrain.
* **Pad Staging Option for T3 Kodiak Carrier**:
  - Add `STAGE T3 (G1)` control on the Command Deck to allow staging a T3 Kodiak Heavy Specialist equipped with `DROP_SERVO`, `HEAVY_PAYLOAD`, and Cairn G1 payload onto an available launch pad.

## 2. Target Files & Architectural Scope
* `GCS/backend/c2_gateway.py`: Reject unequipped carriers cleanly without fallback to random drones; default drop coordinate to staged/commanded target; accept explicit `carrier_id` and `target_pos`.
* `GCS/frontend/js/hud.js`: Inspect telemetry for eligible G1 carrier airframes; update `#btn-payload-drop` label and active/disabled state accordingly; pass staged coordinates and carrier ID upon dispatch; wire `#btn-spawn-kodiak`.
* `GCS/frontend/index.html`: Update `#btn-payload-drop` layout and add `#btn-spawn-kodiak` (`⬢ STAGE T3 (G1)`).
* `GCS/backend/web_server.py`: Handle `SPAWN_REINFORCEMENT` with `tier="T3"` and Cairn G1 capabilities.
* `SIM/backend/sitl_engine.py`: Support spawning T3 Kodiak with `DEFAULT_CAPABILITIES_T3` and `payload_attached=True`.
* `OBC/backend/drone_firmware.py`: Initialize `payload_attached = True` only for airframes equipped with `DROP_SERVO` or `HEAVY_PAYLOAD`.
* `tests/test_payload_deposit_repeater.py`: Verify carrier binding, designated drop coordinate transit, and unequipped rejection.

## 3. Implementation Blueprint
* **Firmware Initialization (`drone_firmware.py`)**:
  - `self.payload_attached: bool = ("HEAVY_PAYLOAD" in self.capabilities and "DROP_SERVO" in self.capabilities)`
* **Carrier Discovery & Mission Authorization (`c2_gateway.py`)**:
  - When searching for eligible carriers, filter strictly by `payload_attached` AND `DROP_SERVO` in capabilities.
  - If none found: return `{"status": "ERROR", "code": "NO_ELIGIBLE_CARRIER", "reason": "No airframes in fleet equipped with Cairn G1 payload."}`
  - Target coordinate: if `target_pos` provided, use `[target_pos[0], target_pos[1], 0.0]`; otherwise use `self._target_pos` if not at muster `[0,0,20]`.
* **GCS HUD Monitoring (`hud.js`)**:
  - Find `carrier = frame.drones?.find(d => !d.crashed && d.payload_attached && d.capabilities?.includes('DROP_SERVO'))`
  - Update button text and state:
    - Active drop: `⬢ DROP [stage] (D#id)`
    - Armed carrier present: `⬢ CAIRN G1 DROP (#id)` (enabled)
    - No carrier: `⬢ CAIRN G1 [NO CARRIER]` (disabled)
  - On click: pass staged target coordinate `[inputX, inputY, 0]` and `carrier_id: carrier.id`.
* **Pad Staging (`web_server.py` & `index.html`)**:
  - Button `#btn-spawn-kodiak` sends `{ cmd: 'SPAWN_REINFORCEMENT', base_tier: 'T3' }`.
  - Spawns T3 Kodiak on next available pad.

## 4. Acceptance Criteria & Verification Invariants
* [ ] Standard T1.5 airframes initialize with `payload_attached = False`.
* [ ] Cairn G1 button displays `⬢ CAIRN G1 [NO CARRIER]` and disables when no airframe carries G1.
* [ ] Staging a T3 Kodiak onto launch pad and launching it activates `⬢ CAIRN G1 DROP (#ID)` with its ID.
* [ ] Commanded drop targets designated staged coordinates `[X, Y, 0]` rather than blind $40\text{m}$ forward offset.
* [ ] Targeted tests in `tests/test_payload_deposit_repeater.py` pass cleanly.
