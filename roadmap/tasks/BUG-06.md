# [BUG-06] OBC Failsafe Consolidation: t-GCSH/GCSH Split, Gateway Relay & Corridor ERTB

* **Grade**: 🐞 Bug / Defect | **Type**: OBC Failsafe, Protocol Arbitration & Corridor Navigation (`Completed / Fresh Review`) | **Priority**: Very High
* **Target Files**: `OBC/backend/drone_firmware.py`, `OBC/backend/brains/leader_follower.py`, `OBC/backend/brains/gossip_consensus.py`, `SIM/backend/sitl_engine.py`, `tests/test_breadcrumb_backtrack.py`, `tests/test_tether_failsafes.py`
* **Operational Context**:
  - A timing and authority race condition exists between the legacy global tether severance loiter timer (`lost_link_loiter_timer = 30.0s` in `sitl_engine.py`) and the autonomous OBC heartbeat watchdog (`gcs_heartbeat_watchdog = 2.0s`).
  - Upon tether severance, the 2.0s watchdog expires 28 seconds before the 30s loiter timer, prematurely forcing drones into `BACKTRACK_HOLD` where 18m breadcrumb rewinds cannot restore a severed physical hardline.
  - When full link loss occurs outside comms range, drones currently risk straight-line beelines back to Base $(0,0,25\text{m})$, ignoring obstacles and urban canyon geometry.
* **Technical Assessment & Architectural Invariants**:
  - **1. Separate Physical from Wireless Heartbeat**: Differentiate `t-GCSH` (100 Hz physical wire/fiber carrier) from `GCSH` (10 Hz Howl 140m wireless broadcast).
  - **2. Tethered Breakaway & 2IC Loiter**: In `TETHERED_RELAY`, wire signal loss or cable strain ($T > 35\text{N}$) triggers immediate pause, breakaway dive ($\Delta Z \ge 4\text{m}$, lateral peel $6\text{ m/s}$), and umbilical purge. Promoted 2IC commands a 30s station-keeping hold to settle formation shuffles.
  - **3. In-Range Relay vs. Leadership Gate**: Drones lacking `FLAGSHIP` authorization (e.g., T2 Pup, K1 Kourier) cannot usurp formation leadership. If only a T2 hears `GCSH`, it acts strictly as an in-range transparent RF gateway (`gateway_id = T2`), relaying GCS packets to the 2IC flagship.
  - **4. Gossip Consensus Bitmask**: In `GOSSIP_CONSENSUS`, every node shares its `gcs_contact` flag in 10 Hz `GOSSIP_ECHO` frames. As long as any node in the connected mesh partition maintains contact, the swarm remains in mission. Only when the partition bitmask is all zeros for $> 2.0\text{s}$ does coordinated ERTB engage.
  - **5. Reversible Ingress Corridor Stack (Low-RAM Hardware Invariant)**: In place of raw high-res point-cloud buffers (which exhaust ESP32-C5 MicroPython heap), OBC maintains a sparse `IngressCorridorStack` (max 16 waypoints, $\Delta d \ge 15.0\text{m}$ novel displacement threshold). ERTB pops this stack in reverse, guaranteeing obstacle-cleared egress back to Base.
* **Remediation Plan & Acceptance Criteria**:
  - **Part 1: Protocol & Watchdog Separation (`drone_firmware.py`, `sitl_engine.py`)**:
    - Add `tether_heartbeat_watchdog` (0.5s timeout) and cable strain sensor threshold check ($T > 35\text{N}$).
    - Suppress `gcs_heartbeat_watchdog` breadcrumb preemption while tether breakaway dive and 30s Doctrine A loiter are active.
  - **Part 2: Flagship Authority Gate & Gateway Relay (`leader_follower.py`)**:
    - In 2IC succession check, verify candidate `FLAGSHIP` capability. If non-flagship hears GCSH, bind as `gateway_id` and relay packets to active 2IC.
    - If 30s loiter expires with zero GCSH and zero relay paths, 2IC triggers corridor ERTB.
  - **Part 3: Mesh Partition Quorum Vector (`gossip_consensus.py`)**:
    - Propagate `gcs_contact` bitmask across 60m P2P mesh. Atomic transition to ERTB occurs only on unanimous partition timeout ($> 2.0\text{s}$).
  - **Part 4: Reversible Corridor Navigation (`drone_firmware.py`, `BaseBrain`)**:
    - Implement `IngressCorridorStack` with 15m displacement filter. When ERTB triggers, autopilot sequentially steers through popped corridor nodes in reverse order back to Base $[0, 0, 25\text{m}]$.
* **Verification Gates**:
  - Automated unit tests in `tests/test_tether_failsafes.py` validating 30s 2IC loiter without premature watchdog preemption.
  - Unit tests in `tests/test_breadcrumb_backtrack.py` verifying reverse corridor stack execution and gossip quorum bitmask consensus.
  - Memory profile verifying corridor stack consumes $< 300$ bytes per airframe.
