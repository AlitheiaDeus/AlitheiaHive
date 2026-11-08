# [PLN-03] Gamified Tactical Operations & Scenario Studio (The Quest & Mapspace Engine)

> **Category**: 🏗️ Epic | **Priority**: Low | **State**: Backlog  
> **Operational Inspiration**: Declarative Mapspace Contracts, Procedural Scenario Director & Gamified C2 Operations

---

## 1. Operational Overview & Vision
* **The "Tool with a Game Attached"**: Transforms AlitheiaHive into a self-contained, playable tactical C2 sandbox while keeping the bridge to real-world hardware 100% open.
* **Separation of Quests vs. Aux Missions**:
  - **Macro Operation (Quest/Scenario)**: The overarching tactical mission contract (e.g., *"Operation Neon Alley"* or *"Hostile Radar Jammer Hunt"*), defining map layout, static obstacles, threat bubbles, dynamic objectives, and pass/fail scoring.
  - **Aux Missions (`[PLN-02]`)**: The operator's tactical toolkit of sub-routines (`SEARCH_AND_RECALL`, `PATROL_TO_BINGO`, `TOUCH_AND_GO_PROBE`, `PAYLOAD_DROP`) deployed to solve the operational objectives.
* **RTS Squad Grouping (StarCraft / C&C Style)**:
  - Supports pre-assigned tactical squads (e.g., **Squad Alpha** = Scouts D0, D1; **Squad Bravo** = Heavies D2, D3).
  - Operator can issue independent routes, formation spreads, or Aux Mission triggers per squad tab/hotkey (`1`, `2`).

---

## 2. The Declarative Scenario Contract (`scenarios/*.json`)
A human-readable, version-controlled JSON/YAML schema defining the mission environment:

```json
{
  "scenario_id": "operation_midnight_echo",
  "name": "Operation Midnight Echo",
  "mapspace": {
    "bounds": [-100, 100, -100, 100, 0, 50],
    "obstacles": [
      { "id": "tower_alpha", "type": "AABB", "pos": [20, 15, 0], "size": [12, 12, 35] },
      { "id": "skybridge_1", "type": "AABB", "pos": [0, 20, 12], "size": [20, 4, 3] }
    ],
    "danger_zones": [
      { "id": "ew_bubble_1", "type": "CYLINDER", "pos": [30, 25, 0], "radius": 20, "rf_jam_rate": 0.4 }
    ]
  },
  "sim_flags": {
    "adversary_spawns": [
      { "type": "JAMMER_PROBE", "pos": [45, 30, 15], "behavior": "EVASIVE_PATROL" }
    ]
  },
  "gcs_briefing": {
    "objectives": [
      { "id": "OBJ_1", "type": "RECON_DWELL", "pos": [20, 15, 37], "dwell_s": 5.0 },
      { "id": "OBJ_2", "type": "PAYLOAD_DEPOSIT", "pos": [-25, 30, 14] },
      { "id": "OBJ_3", "type": "BINGO_RTB", "threshold_pct": 25.0 }
    ],
    "scoring": {
      "target_time_s": 120.0,
      "max_clashes": 0,
      "min_battery_margin": 15.0
    }
  }
}
```

---

## 3. Core Architectural Modules

### A. The Procedural Generator Engine (`tools/scenario_generator.py`)
* Zero-dependency Python CLI tool / backend generator.
* Takes procedural generation parameters (`--seed`, `--type urban_canyon|subterranean_maze`, `--density medium`) and procedurally rolls building boxes, street corridors, and objective placements.
* Instant generation of unlimited fresh tactical maps without touching sim code.

### B. Scenario Studio ("Plan Mode" in GCS / `studio.html`)
* Interactive 3D designer canvas sharing existing Three.js assets.
* Allows operators to drag/drop building AABBs, draw red danger circles, designate dropzones, and click `[EXPORT JSON]` or `[LAUNCH SCENARIO]`.

### C. GCS Objective Tracker & Debrief Engine (`GCS/backend/mission_director.py`)
* **GCS Evaluates Mission Clear**: Monitors downlinked telemetry against the scenario objective checklist.
* Tracks milestones (e.g. dwell timer countdowns, payload drops, pad landings).
* Displays post-mission tactical score card (Mission Rank: S/A/B/F based on elapsed time, zero clashes, remaining fuel).

---

## 4. Airgap Enforcement Invariants
1. **SIM Airgap**: SIM reads `mapspace.obstacles` for physical collisions and RF occlusion, and `sim_flags` for adversary spawning. SIM never inspects GCS scoring rules.
2. **GCS Airgap**: GCS renders `mapspace.obstacles` as glowing 3D wireframe meshes in Three.js and displays `gcs_briefing` cards. GCS evaluates pass/fail from telemetry.
3. **Brain Airgap**: Drone OBC receives strictly standard 4D waypoints, corridors, or Aux Mission commands. OBC never carries map history or quest storyline memory.
