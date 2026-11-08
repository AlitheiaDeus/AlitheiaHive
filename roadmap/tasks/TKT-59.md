# [TKT-59] 1-Click Aux Mission: Cairn G1 Solar Repeater Payload Drop

* **Grade**: 🚀 Feature | **Type**: Tactical Mission & RF Mesh Extension (`In Progress`) | **Priority**: High
* **Target Files**: `GCS/backend/c2_gateway.py`, `SIM/backend/sitl_engine.py`, `OBC/backend/drone_firmware.py`, `GCS/backend/web_server.py`, `GCS/frontend/index.html`, `GCS/frontend/css/style.css`, `GCS/frontend/js/hud.js`, `GCS/frontend/js/viewport.js`, `tests/test_payload_deposit_repeater.py`
* **Operational Inspiration**:
  - Direct delivery and deployment of a stationary **Cairn G1 Solar Repeater** pod onto the ground to bridge RF blackout zones and extend GCS telemetry range.
  - Dual tactical utility:
    1. **RF Mesh Range Extender**: Once grounded, the solar pod powers on its micro-transceiver and joins the P2P mesh (`RANGE_P2P = 60.0m`), relaying packets across the canyon airgap.
    2. **Local Surveyed Anchor**: In GNSS-denied/jammed zones (`TKT-52`), the stationary pod serves as a fixed reference anchor ($Z=0.05\text{m}$) that passing drones can ping to zero out their dead-reckoning drift timer.
  - Pre-flight capability authorization: requires `HEAVY_PAYLOAD` and `DROP_SERVO`.
  - Precision delivery sequence:
    1. Carrier transit to designated target $(X, Y)$ drop coordinates at cruise altitude ($Z=25\text{m}$).
    2. Vertical descent to precision release hover ($Z = 3.0\text{m}$, $\|\mathbf{v}\| \le 0.3\text{m/s}$).
    3. Magnetic latch actuation / servo drop (`carrier.payload_attached = False`).
    4. Ballistic touchdown under gravity & aerodynamic drag ($Z \to 0.05\text{m}$).
    5. Repeater initialization: registers into `sitl_engine.ground_repeaters`, participating in multi-hop gossip mesh routing.
    6. Carrier egress: ascends back to safe corridor altitude ($Z=25\text{m}$) and returns to base or rejoins fleet.
* **Technical Blueprint**:
  - **Part 1: Firmware & OBC State Tracking (`drone_firmware.py`)**:
    - Add `payload_attached: bool = True` (for carriers equipped with `HEAVY_PAYLOAD`).
    - Add `payload_drop_active: bool = False`, `payload_target_pos: Optional[np.ndarray] = None`.
    - Add `GroundRepeater` class / struct representing deployed stationary solar repeaters on the ground (`id`, `pos`, `active`, `solar_powered`, `downlink_active`, `uplink_active`).
  - **Part 2: C2 Gateway Mission Controller (`c2_gateway.py`)**:
    - Add `deploy_payload_drop(target_pos, carrier_id) -> Dict[str, Any]` and `stop_payload_drop() -> Dict[str, Any]`.
    - Add capability pre-flight authorization check: requires `HEAVY_PAYLOAD` and `DROP_SERVO`.
    - Implement drop stage state machine in `step(dt)`:
      - `TRANSIT_TO_DROP` -> `PRECISION_HOVER` ($Z=3.0\text{m}$) -> `RELEASE` -> `TOUCHDOWN` -> `CARRIER_EGRESS`.
    - Expose `payload_drop` telemetry in state snapshots.
  - **Part 3: SITL World Engine Mesh Integration (`sitl_engine.py`)**:
    - Maintain `self.ground_repeaters: List[GroundRepeater] = []`.
    - Update `_update_radio_comms()` to include ground repeaters in direct gateway and multi-hop flood-fill mesh routing over `RANGE_P2P`.
    - Passing drones within $30\text{m}$ of a ground repeater reset their `gnss_denied_timer = 0.0`.
  - **Part 4: Web Server & GCS HUD / Viewport (`web_server.py`, `index.html`, `style.css`, `hud.js`, `viewport.js`)**:
    - WebSocket commands `DEPLOY_PAYLOAD_DROP` and `STOP_PAYLOAD_DROP`.
    - Add `#btn-payload-drop` in GCS action bar (`📦 CAIRN G1 DROP`) with amber/cyan cyberpunk styling.
    - Viewport: Renders deployed solar repeater models on the terrain with an expanding neon beacon ring.
* **Verification Gates**:
  - [ ] Unit tests in `tests/test_payload_deposit_repeater.py`:
    - Pre-flight capability authorization check (`HEAVY_PAYLOAD`, `DROP_SERVO`).
    - Pre-flight payload presence check (cannot drop twice).
    - Precision hover arrival gate ($Z \le 3.5\text{m}$, speed $\le 0.5\text{m/s}$).
    - Ballistic release and touchdown ($Z = 0.05\text{m}$).
    - Repeater mesh integration: verifies multi-hop comms extension across ground repeater.
    - GNSS drift reset gate: verifies passing drone resets `gnss_denied_timer` near repeater.
    - Carrier egress ascension and RTB transition.
    - Operator manual abort.
  - [ ] Zero regressions across existing test suite.
  - [ ] Headless smoke test passes in <0.5s.
