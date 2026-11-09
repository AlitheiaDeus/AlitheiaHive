# TKT-63: Core Hygiene — StrEnum Flight States, PyMAVLink Geodetics & UV Packaging

> **Category**: 🛠️ Refactor / Hygiene | **Priority**: High | **State**: completed  
> **Operational Inspiration**: Eliminates magic strings across OBC/SITL layers, introduces WGS-84 geodetic precision into MAVLink GPS mapping, adds transparent SciPy acceleration to Kuhn-Munkres matching, and bootstraps modern zero-friction `uv` packaging.

---

## 1. Operational Overview & Objectives

Following our comprehensive SDK/package audit, this ticket addresses high-impact core hygiene improvements with zero breaking changes to wire protocols or WebGL frontend:

1. **`FlightState` Enumeration (`OBC/backend/drone_firmware.py`)**:
   - Replaced loose string literals across airframe firmware and simulation engines with a standard `FlightState(str, Enum)`.
   - Preserves 100% backward compatibility with string equality (`== "CRUISE"`, `in ("LANDED", "GROUND_IDLE")`) and JSON serialization over WebSocket/UDP.
   - Eliminates silent typo bugs across nominal flight, emergency failsafes, and recovery stages.
2. **PyMAVLink WGS-84 Geodetic Offsets (`SIM/backend/mavlink_bridge.py`)**:
   - Upgraded spherical flat-Earth `111139.0` approximation with geodetic formulas accounting for Earth's ellipsoidal curvature ($R_{\text{lat}}, R_{\text{lon}}$) when converting between local Cartesian meters and GPS degrees.
   - Applies to outbound `GLOBAL_POSITION_INT` / `HOME_POSITION` and inbound `SET_POSITION_TARGET_GLOBAL_INT` / `SET_HOME_POSITION`.
3. **Kuhn-Munkres SciPy Acceleration (`OBC/backend/brains/hungarian.py`)**:
   - Conditionally imports `scipy.optimize.linear_sum_assignment` for C-accelerated bipartite matching on desktop SITL.
   - Retains pure-NumPy $O(N^3)$ solver as fallback for embedded companion microcomputers (MicroPython / XIAO C5).
4. **Modern `uv` Packaging (`pyproject.toml`)**:
   - Defined a standardized `pyproject.toml` specification for dependencies (`numpy`, `aiohttp`, `pymavlink`, and optional `scipy`, `pydantic`).
   - Enables reproducible one-command environment management (`uv run python run.py`) without disrupting Conda `run.ps1` workflows.

---

## 2. Target Files & Architectural Scope

* `roadmap/tasks/TKT-63.md`: This specification file.
* `roadmap/kanban.md`: Active task registration in Kanban board.
* `pyproject.toml`: Modern Python project specification.
* `.gitignore`: Ignore `.venv/` and `*.uv` cache files.
* `OBC/backend/drone_firmware.py`: `FlightState` enum definition and integration.
* `OBC/backend/brains/hungarian.py`: SciPy `linear_sum_assignment` transparent acceleration with NumPy fallback.
* `SIM/backend/mavlink_bridge.py`: WGS-84 geodetic conversion helpers and `FlightState` references.
* `SIM/backend/sitl_engine.py`: `FlightState` imports and usage across stage machines.
* `SIM/backend/atc_recovery.py`: `FlightState` usage in ATC recovery operations.

---

## 3. Acceptance Criteria & Invariants

* [x] `FlightState(str, Enum)` defined and adopted across OBC and SIM backend.
* [x] WGS-84 geodetic calculations active in `mavlink_bridge.py`.
* [x] Kuhn-Munkres matching in `hungarian.py` uses `scipy.optimize` when available, falling back to NumPy.
* [x] `pyproject.toml` added to repo root; `.gitignore` updated.
* [x] Full regression discovery suite passes (288/288 tests).
* [x] Headless smoke test passes in $< 0.50\text{s}$ (0.195s verified).
* [x] Kanban board validated with `kanban_ctl.py validate`.
