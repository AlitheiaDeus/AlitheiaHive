# Settled Milestones: MAVLink Telemetry, Viewports & Airgap Guards

> **Status**: Settled & Verified | **Milestones**: M30-M46, FIX-03, FIX-04

### `[TKT-32]` Persistent Telemetry Session & Reconnect State Hydration
* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / GCS Detachment (Settled) | **Priority**: High
* **Target Subsystems**: `backend/c2_gateway.py`, `frontend/js/hud.js`, `frontend/js/app.js`
* **Technical Blueprint**:
  - Eliminates "mission amnesia" on browser page refresh.
  - **Server-Side Mission Event Journal**: `c2_gateway.py` maintains an in-memory circular event buffer (`event_history: List[Dict]`, max 100 entries) capturing timestamps and event messages (e.g. `CASUALTY: Drone #2 crashed`, `FAILSAFE: Umbilical severed`, `MISSION: BOX_PATROL deployed`).
  - **WebSocket State Hydration on Connect**:
    - Upon new WebSocket connection (such as browser page refresh), server transmits an initial hydration bundle:
      1. Complete running simulation state snapshot.
      2. Historical event journal `event_history`.
    - Frontend HUD immediately populates the event log list with true historical mission events, highlights the true active comms mode and formation buttons, synchronizes the RF jamming slider, and displays active route progressâ€”instantly eliminating the "amnesia / hallucination" effect.
* **Targeted Test**: `python -m unittest tests/test_web_server.py`

---

### `[TKT-31]` Telemetry-Gated C2 Downlink Handshake & Uplink Failure Hold
* **Grade**: ðŸ› ï¸ Improvement | **Type**: Bugfix / Architecture Decoupling (Settled) | **Priority**: High
* **Target Subsystems**: `backend/c2_gateway.py`, `frontend/js/hud.js`
* **Technical Blueprint**:
  - Resolves Trial 2: Commanded Target beacon "going ham" on tether cut.
  - **Root Cause**: GCS evaluated arrival by peeking at `drones[0].pos` directly in Python memory. When the uplink dropped, GCS failed to clear dwell timer and spammed 60 random targets/second until automated RTB moved the drone away.
  - **Architectural Solution**:
    1. **Telemetry-Gated Arrival**: GCS C2 layer determines arrival ONLY from downlinked telemetry packets (`target_reached = True`). When umbilical/RF is severed, GCS receives NO arrival packets and marks link state as `SIGNAL_LOST`.
    2. **Uplink Failure Hold**: When `dispatch_target_uplink()` returns `DROPPED` or `OUT_OF_RANGE`:
       - GCS immediately enters `rng_status = "UPLINK_OFFLINE"` / `rng_paused = True`.
       - GCS FREEZES the Commanded Target at the last attempted coordinate. It NEVER re-rolls random coordinates while the link is dead.
       - Live HUD badge reflects `RNG [LINK LOST: HOLDING BEACON]`.
    3. **Graceful Recovery**: When link is re-established, GCS unpauses and verifies telemetry arrival before rolling subsequent stations.
* **Targeted Test**: `python -m unittest tests/test_rng_recon.py`

---

### `[TKT-30]` Per-Waypoint Loiter Timers & Coordinated Formation Arrival
* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / Mission Guidance (Settled) | **Priority**: High
* **Target Subsystems**: `backend/drone_obc.py`, `backend/c2_gateway.py`, `frontend/js/hud.js`
* **Technical Blueprint**:
  - Resolves Trial 1: Leader instantly transitions waypoints without waiting for trailing escorts.
  - **4D Waypoint Protocol**: Extends waypoint schema to support per-waypoint loiter durations: `[x, y, z, loiter_s]` or `{ "x": x, "y": y, "z": z, "loiter": loiter_s }`.
  - **OBC Loiter State Machine**:
    1. Flagship arrives within acceptance sphere ($R_{\text{accept}} = 2.5\text{m}$) of active waypoint.
    2. Flagship enters `WAYPOINT_LOITER` state, zeroing horizontal velocity demand and counting down `wp_loiter_timer = wp.loiter`.
    3. **Formation Stabilization Gate**: Optional settle check verifies escorts are within formation arrival deadband ($\le 4.0\text{m}$ of assigned wing slots) before loiter timer elapses, preventing escort abandonment.
    4. Once loiter timer reaches 0.0s, flagship transitions `acknowledged_target` to `current_wp_idx + 1`.
  - **Presets**:
    - `BOX_PATROL`: 4.0s observation loiter at vertices, 0s at transit points.
    - `DIAMOND_SURVEY`: 3.0s sensor dwell at survey stations.
    - `HEXAGON_SWEEP`: 2.0s perimeter security checks.
  - **HUD Readout**: Live route badge countdown: `BOX [WP 2/4 (LOITER 2.8s)]`.
* **Targeted Test**: `python -m unittest tests/test_mission_routes.py`

---

### `[TKT-35]` Dual Viewport & SITL God-View Simulation Lab (The 2 Separate UIs)
* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / UI Architecture Decoupling (`Completed`) | **Priority**: High
* **Target Files**: `GCS/frontend/`, `SIM/frontend/`, `GCS/backend/c2_gateway.py`, `SIM/backend/sitl_engine.py`
* **Technical Blueprint**:
  - Settles PLN-14 with strict enforcement of the **Operator vs Sim Airgap**.
  - Decouple the frontend into **Two Distinct Web Interfaces**:
    1. **Operator Tactical GCS** (`GCS/frontend/` at `http://localhost:8080/`):
       - Palette: Dark Navy (`#061019`) & Neon Cyan (`#00F0FF`).
       - Pure operator terminal: Renders strictly downlinked telemetry frames received over WebSocket/MAVLink.
       - Exposes valid C2 operational controls: Waypoint dispatches, discrete mission routes, formation switching, brain swapping (Leader-Follower vs Gossip), RTB, reinforcement launch.
       - **Zero God-Mode Buttons**: Completely purge/hide simulation fault injectors (`KILL LEADER`, `CUT TETHER`, `RF JAMMING SLIDER`, `RESPAWN SWARM`). A real drone operator in the field does not have a "jam my own radio" or "sever cable" button on their mission laptop.
    2. **SITL God-View Simulation Lab** (`SIM/frontend/` at `http://localhost:8080/sim` or port 8081):
       - Palette: Deep Void (`#03070d`) & Neon Magenta (`#FF00EA`) with warning Amber (`#FFB800`).
       - Omniscient Three.js viewport: Renders unfiltered ground-truth physics state, ghost collision spheres (Zone 4 prop clash at $d \le 1.5\text{m}$), tether tension vectors, and radio propagation range spheres ($120\text{m}$ bubble).
       - Full simulation lab fault injection deck: Kill Drone #N, Cut Tether, Dynamic RF Interference / Jamming slider ($0\text{--}50\text{ dB}$), Respawn Fleet, Dynamic Wind Gust trigger, and real-time Ground-Truth vs Downlinked Telemetry deviation meters.
* **Targeted Test**: Visual WebGL inspection & aiohttp route tests.

---

### `[FIX-03]` Collapsible Panel Edge Toggles & Off-Screen Restoration Handles
* **Grade**: ðŸ› ï¸ Improvement | **Type**: Quick Patch / Bug Fix (`Completed`) | **Priority**: High
* **Target Files**: `GCS/frontend/index.html`, `GCS/frontend/css/style.css`, `GCS/frontend/js/hud.js`
* **Technical Blueprint**:
  - Address demo defect: When collapsing `FLEET TELEMETRY` or `COMMAND DECK`, panels translate `-340px` and `+360px`, sliding the collapse buttons completely off-screen with zero remaining UI elements to restore them.
  - Add viewport-docked `.edge-restore-tab` buttons pinned to the screen edges (`left: 0` / `right: 0`) at `top: 56px`:
    - Left tab: `<button id="tab-restore-left" class="edge-restore-tab left-tab hidden" title="Expand Fleet Telemetry"><span class="edge-icon">&#x25BA;</span> FLEET</button>`
    - Right tab: `<button id="tab-restore-right" class="edge-restore-tab right-tab hidden" title="Expand Command Deck">COMMAND <span class="edge-icon">&#x25C4;</span></button>`
  - Style tabs with cyberpunk glassmorphism, glowing cyan borders, hover expansion, and subtle shadows.
  - Wire toggle handlers in `hud.js`: Collapsing panel reveals restore tab; clicking restore tab smoothly expands panel and hides tab.
* **Targeted Test**: `python -m unittest tests/test_web_server.py`

---

### `[FIX-04]` P2P Echelon Heading Pinwheel Stabilization & RF Horizon Airgap Guard
* **Grade**: ðŸ› ï¸ Improvement | **Type**: Bugfix / Kinematics & Airgap Protocol (`Completed`) | **Priority**: Very High
* **Target Files**: `OBC/backend/brains/gossip_consensus.py`, `SIM/backend/sitl_engine.py`, `GCS/frontend/js/hud.js`, `tests/test_gossip_formations.py`, `tests/test_rtb_landing.py`
* **Technical Blueprint**:
  - **Part 1: P2P Echelon Heading Pinwheel Stabilization**:
    - **Problem**: When drone counts in Echelon are asymmetric (e.g., 7 drones: 1 apex + 4 right + 2 left; or when a drone on one wing crashes), the geometric centroid of all drones shifts laterally away from the center line. Because heading was updated via `target_vec = target_pos - centroid`, this lateral offset induced a persistent false yaw error, rotating the formation. Drones then chased their rotated slots, creating a positive feedback loop where the entire formation spun like a pinwheel!
    - **Solution**:
      1. Swarm cruising heading tracks the **mean consensus velocity vector**: $\bar{\mathbf{v}} = \frac{1}{N}\sum \mathbf{v}_i$. When $\|\bar{\mathbf{v}}\| \ge 0.8\text{ m/s}$, $\text{desired\_yaw} = \text{atan2}(\bar{v}_y, \bar{v}_x)$.
      2. Anti-jitter freeze: when speed $< 0.8\text{ m/s}$ or near station ($d \le 2.5\text{m}$), **freeze heading** at the last stable cruising angle.
      3. When starting toward a distant target from rest ($d \ge 2.5\text{m}$), use the **apex drone** (`sorted_ids[0]` at the tip of the V), which has zero lateral wing bias.
  - **Part 2: RF Horizon Airgap Guard for Recovery Commands**:
    - **Problem**: `command_rtb_hold()` and `command_land_swarm()` directly mutated `drone.flight_state` in Python memory, acting like telepathic cheats and bypassing the 120m RF radio horizon.
    - **Solution**:
      1. `command_rtb_hold()` and `command_land_swarm()` must query `self.is_link_active()`. If the fleet is outside the 120m GCS radio horizon (or tether is cut with star mode), the command is rejected with `{"status": "DROPPED", "reason": "OUT_OF_RANGE", "error": "Fleet outside GCS radio horizon (120m)"}`.
      2. Outside the dome, fleets only return via their autonomous **`LOST_LINK` failsafe** ($> 1.0\text{s}$ timeout &rarr; autonomous RTB).
  - **Part 3: Reinforcement Pad Comms Realism**:
    - In `launch_reinforcement()`, if the active fleet is outside the 120m radio horizon of the base, the newly spawned drone does NOT magically peek at `current_leader.pos`. Its target is the local Base station $(0, 0, 20\text{m})$, where it holds at the local muster shelf.
  - **Part 4: GCS HUD Feedback**:
    - When an uplink or recovery command fails with `OUT_OF_RANGE`, GCS HUD displays an explicit warning alert `[RF LINK LOST: FLEET OUT OF HORIZON]` instead of silent failure.
* **Targeted Test**: `python -m unittest tests/test_gossip_formations.py tests/test_rtb_landing.py`

---

### `[TKT-40]` Multi-Vehicle MAVLink Telemetry Broadcast & Swarm System ID Demuxing
* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / MAVLink Integration (`Completed`) | **Priority**: High
* **Target Files**: `SIM/backend/mavlink_bridge.py`, `tests/test_mavlink_bridge.py`
* **Technical Blueprint**:
  - Vectorize MAVLink loopback emission across all active airframes in `self.sim.drones`.
  - Assign unique MAVLink `srcSystem = drone.id + 1` (Drone #0 -> sysid 1, Drone #1 -> sysid 2, ...).
  - Emit periodic `HEARTBEAT` (1 Hz) and `GLOBAL_POSITION_INT` (10 Hz) for every living drone on port 14550.
  - Project local Cartesian NED coordinates $(x, y, z)$ to WGS-84 GPS coordinates `(lat, lon, alt)` relative to `HOME_LAT`, `HOME_LON`, and `HOME_ALT`.
  - External GCS (Mission Planner 1.3.x, QGroundControl) automatically discovers all drones on UDP port 14550 and displays the full formation simultaneously on the map with individual vehicle telemetry cards and vehicle selection dropdowns.
* **Targeted Test**: `python -m unittest tests/test_mavlink_bridge.py`

---

### `[TKT-46]` Two-Way MAVLink C2 Command Ingestion & Mission Planner Control Bridge
* **Grade**: ðŸ› ï¸ Improvement | **Type**: Feature / External GCS Control (`Completed`) | **Priority**: High
* **Target Files**: `SIM/backend/mavlink_bridge.py`, `SIM/backend/sitl_engine.py`, `tests/test_mavlink_bridge.py`
* **Technical Blueprint**:
  - Non-blocking inbound MAVLink packet listener on `self.mavlink_sock` receiving commands dispatched by Mission Planner / QGroundControl over UDP.
  - **Dynamic SIM Home Anchor & Auto-Centering**:
    - Coordinates are owned by the SIM world engine (`self.sim.home_lat`, `self.sim.home_lon`, `self.sim.home_alt`), defaulting to Tokyo Bay / Tennozu Isle (`35.625971, 139.734759, 20.0m`). Aux connectors query SIM dynamically rather than hardcoding.
    - Periodic (1 Hz) and requested broadcast of `HOME_POSITION` (#242) and `GPS_GLOBAL_ORIGIN` (#49) so Mission Planner / QGC automatically centers its satellite map directly onto our launch site.
    - Periodic (1 Hz) and requested broadcast of `HOME_POSITION` (#242) and `GPS_GLOBAL_ORIGIN` (#49) so Mission Planner / QGC automatically centers its satellite map directly onto our launch site.
  - Ingests `SET_POSITION_TARGET_GLOBAL_INT` (#86) and `SET_POSITION_TARGET_LOCAL_NED` (#84):
    - Decodes GPS `lat`, `lon`, `alt` from Mission Planner "Fly Here" clicks.
    - Inversely projects back to local simulation Cartesian coordinates `[x, y, z]` relative to `HOME_LAT`, `HOME_LON`, and `HOME_ALT`:
      - $y = (\text{lat\_deg} - \text{HOME\_LAT}) \times 111139.0$
      - $x = (\text{lon\_deg} - \text{HOME\_LON}) \times 111139.0 \times \cos(\text{radians}(\text{HOME\_LAT}))$
      - $z = \text{alt\_m} - \text{HOME\_ALT}$ (or direct relative altitude).
  - **Tactical Geofence Clamping & Rejection Gate**:
    - Protects render distance, tether envelopes, and RF horizons from out-of-bounds clicks:
    - Maximum horizontal radius: $R_{\text{horiz}} \le 120.0\text{m}$ (matching the GCS RF horizon ring; $X, Y \in [-120, 120]\text{m}$).
    - Altitude clamp: $Z \in [5.0, 45.0]\text{m}$ (prevents terrain collisions or orbital escapes).
    - Out-of-bounds clicks (e.g. Timbuktu > 120m away) are strictly rejected with `COMMAND_ACK(MAV_RESULT_DENIED)` and logged with warning `[MAVLINK] Click rejected: Out of tactical geofence`.
    - Safe clicks update swarm target position â€” the swarm in the 3D WebGL simulator immediately steers to the location clicked on Mission Planner's map!
  - Ingests `COMMAND_LONG` (#76) standard actions:
    - `MAV_CMD_NAV_TAKEOFF` (#24) &rarr; Triggers pad scramble / takeoff.
    - `MAV_CMD_NAV_RETURN_TO_LAUNCH` (#20) &rarr; Triggers `command_rtb_hold()`.
    - `MAV_CMD_NAV_LAND` (#21) &rarr; Triggers `command_land_swarm()`.
    - `MAV_CMD_COMPONENT_ARM_DISARM` (#400) &rarr; Arms/disarms airframes.
  - Ingests `SET_MODE` (#11) &rarr; Maps Guided / Auto / RTL modes to swarm autopilot states.
  - Protocol Handshake: Emits `COMMAND_ACK` (#77) with `MAV_RESULT_ACCEPTED` back to Mission Planner so it audibly confirms execution.
  - Target system ID demuxing: Broadcasts (`target_system = 0/1`) command the flagship / entire formation; specific `target_system = N` commands target individual airframe $N-1$.
* **Targeted Test**: `python -m unittest tests/test_mavlink_bridge.py`

---

