# [TLK-01] Codebase Sweep Findings, Test Remediation & Phase 3 Alignment

* **Grade**: 💬 Discussion / Strategic Reform | **Type**: Architectural Alignment & Remediation (`Todo / Active Pipeline`) | **Priority**: Very High
* **Source Landmark Reports**: [`../../../docs/internal/sweep_report.md`](../../../docs/internal/sweep_report.md), [`../../../docs/internal/walkthrough.md`](../../../docs/internal/walkthrough.md)
* **Target Files**: `tests/test_tri_mode_rf.py`, `tests/test_rtb_landing.py`, `tests/test_in_flight_enrollment.py`, `SIM/backend/sitl_engine.py`, `OBC/backend/brains/`, `tasks/TKT-58_aux_mission_patrol_bingo_rtb.md`, `tasks/PLN-02_auxiliary_mission_engine.md`
* **Operational Context**:
  - A full read-only project sweep executed on 2026-11-01 surfaced 10 findings across SIM, OBC, and GCS bubbles.
  - The baseline test suite ran 230 tests in 16.88s with 3 test regressions caused by recent architectural changes (commit `21cc0c0`, TKT-54 breadcrumb backtrack refactoring, and TKT-41 cold-start pad scramble).
  - Several Phase 3 dependencies are currently trapped in `Frozen` (`TKT-21` LiPo curves blocking `TKT-58` BINGO failsafe) and `Backlog` (`TKT-51` S&R Courier is Workflow #2 of `PLN-02`; `TKT-47` LZ offsets needed for `PLN-05`).

---

## Strategic Architectural Doctrine: The "Induced Coma" Refactor

> **The Core Realization**:  
> Every previous attempt to "fix the airgap while maintaining 100% working tests at each intermediate step" resulted in agents quietly hacking fallback cheats (e.g. `getattr(drone, "acknowledged_target", self.sim.target_pos)`).  
> In order to preserve the Phase 4 hardware guarantee (where physical MicroPython OBCs on real XIAO C5s cannot have a `sim` object), **we must cut out the rotting bits first, even if it temporarily puts the simulation into a coma, and patch subsystems back cleanly one by one.**

### The 3-Step Resuscitation Plan:
1. **Sever the Cord (Induce the Coma)**:
   - Strip `self.sim` completely from `BaseBrain.__init__`. Brains receive **zero** references to the simulator.
   - Enforce via automated test (`tests/test_airgap_integrity.py`): any file in `OBC/` containing `from SIM`, `import SIM`, or `self.sim` instantly fails.
   - Redefine Brain input/output strictly as a Phase 4 pure I/O black box:
     $$\text{OBC}(\text{LocalTelemetry}, [\text{InboxPackets}], \Delta t) \longrightarrow (\mathbf{u}_{\text{flight}}, [\text{OutboxPackets}])$$
2. **Accept the Flatline**:
   - Complex formations and gossip waves will temporarily break. Do NOT add backward-compatibility fallback cheats to keep tests passing.
3. **Sequential Subsystem Resuscitation**:
   - **Heartbeat 1 (Solo Navigation)**: Individual drone steers purely to its onboard `acknowledged_target` buffer.
   - **Heartbeat 2 (Leader-Follower)**: Followers compute target offset purely from received telemetry packets heard over the RF channel.
   - **Heartbeat 3 (P2P Gossip)**: Swarm consensus operates strictly over localized peer messages bounded by `RANGE_P2P` (60m).
   - When the simulation wakes up, the brain code is 100% Phase-4 ready for real hardware.

---

## Technical Agenda & Remediation Plan

### Part 1: Automated Test Baseline Remediation (Immediate Code Fixes)
1. **Fix `test_tier1_bark_downlink` (`tests/test_tri_mode_rf.py:298`)**:
   - Commit `21cc0c0` enforced `RANGE_BORK` (70m) on leader downlink calculations for T1.5 Beta airframes.
   - Test expects 90m Bark range and places the leader at 77.6m distance ($[75, 0, 20]$).
   - *Action*: Clarify whether promoted flagship leader is T1 (Bark 90m) or T1.5 (Bork 70m). If testing Bark specifically, set `leader.rf_power_mode = RF_MODE_BARK` or test Bork within 65m.
2. **Update `test_universal_lost_link_loiter_and_autonomous_rtb` (`tests/test_rtb_landing.py:112`)**:
   - `TKT-54` moved wireless lost-link recovery to autonomous OBC breadcrumbs and restricted `sim.lost_link_loiter_timer` exclusively to severed hardline tethers.
   - *Action*: Update test assertions to verify OBC breadcrumb backtracking rather than expecting the deprecated global sim timer.
3. **Calibrate `test_distant_lost_swarm_isolation_no_telepathic_cheat` (`tests/test_in_flight_enrollment.py:165`)**:
   - Under `TKT-41` cold-start pad scramble, a newly launched reinforcement drone takes longer than 300 ticks (5s) to spool up rotors and climb vertically to $Z \ge 18\text{m}$ (reaches $Z = 8.78\text{m}$).
   - *Action*: Extend test step loop from 300 ticks to 600 ticks (10s) to allow full climb-out to muster altitude.

---

### Part 2: Phase 3 Roadblock Resolution (TKT-58, TKT-51, TKT-47)
1. **TKT-21 Battery Drain Model Unfreezing (for TKT-58)**:
   - Carve out a scoped, practical **Tactical Power Drain** in `sitl_engine.py` (e.g. state-based: $0.15\%/\text{s}$ cruise, $0.25\%/\text{s}$ sprint) so BINGO failsafe triggers realistically in 5–7 minutes of simulation time.
   - Move BINGO watchdog to OBC firmware level (`OBC/backend/drone_firmware.py`) so drones in `LOST_LINK` also execute autonomous low-power RTB per the Brain Airgap.
2. **Promote `TKT-51` (Search & Rescue Courier / Mule)**:
   - Move from `tasks/backlog/` to `tasks/` as the second concrete auxiliary mission under `PLN-02`.
3. **Align `TKT-47` (Relocatable LZs & Decoupled Subsystems)**:
   - Prepare `TKT-47` as the structural foundation for `PLN-05` (Off-Nominal Landing Zones & Field E-Land).

---

### Part 3: Invariant & Airgap Hardening
1. **Attitude Tilt Rate Clamp**: Implement $|\dot{\theta}| \le 250^\circ/\text{s}$ dt-based slew rate limiter on `pitch` and `roll` in `sitl_engine.py` (matching `TKT-55` yaw slew clamp).
2. **Doc Alignment**: Update AGENTS.md Invariant 4 damping coefficient (doc: $-1.5$, code: $-3.5$) and update test suite count from 176 to 230 tests.
3. **Tether Dissolve Visual**: Add 1.5s opacity fade in `GCS/frontend/js/app.js` upon tether snap.

---

## Verification Criteria
- [x] Automated regression suite passes 236/236 tests cleanly with code 0 (`.\run.ps1 -UnitTest`).
- [x] Smoke test passes in $< 0.15\text{s}$ (`.\run.ps1 -SmokeTest`).
- [x] `tests/test_airgap_integrity.py` confirms zero SIM imports or `self.sim` usage in `OBC/`.
- [x] No regression in single-process or multi-process execution modes.
- *(Note: Parts 2 and 3 remain scoped for subsequent feature branches following completion of the 4-commit Induced Coma refactor on `refactor/airgap-induced-coma`.)*
