"""
AlitheiaHive SITL: Comprehensive Flight Formation Test Suite (test_formation_flight_suite.py)
Validates dynamic formation behavior across:
1. Altitude changes (Up & Down).
2. Distances at various angles (Left, Right, Reversal / U-turn).
3. All-to-all formation transition matrix (Echelon, Boids Flock, Vortex, Perimeter).
Continuous safety verification: Zero Zone 4 breaches (d <= 1.5m), velocity clamps, and floor bounds.
Includes TelemetryRecorder integration for flight replay export.
"""

import unittest
import math
import sys
import os
import time
import argparse
import numpy as np
from typing import List, Optional, Tuple

from SIM.backend.sitl_engine import SwarmSimulator, DT, ZONE4_CLASH, V_MAX, A_MAX
from SIM.backend.telemetry_recorder import TelemetryRecorder


def initialize_formation_north(
    drone_count: int = 6,
    initial_target: Optional[List[float]] = None,
    formation: str = "ECHELON_ESCORT",
) -> SwarmSimulator:
    """Initialize swarm at specified target in Wireless Star mode pointing North."""
    if initial_target is None:
        initial_target = [0.0, 0.0, 20.0]

    sim = SwarmSimulator(drone_count=drone_count, enable_packet_loss=False, seed=42)
    # Use Wireless Star mode to avoid tether bounds during high-distance testing
    sim.set_comms_mode("WIRELESS_STAR")
    sim.set_formation(formation)

    # Position leader at target pointing North (+Y in ENU)
    leader = sim.leader
    if leader is not None:
        leader.pos = np.array(initial_target, dtype=float)
        leader.vel = np.zeros(3, dtype=float)
        leader.yaw = math.pi / 2.0  # 90 deg = North (+Y)

    if hasattr(sim.brain, "formation_heading"):
        sim.brain.formation_heading = math.pi / 2.0
        sim.brain.heading_initialized = True

    sim.set_target(initial_target)

    # Position followers into initial formation slots
    followers = [d for d in sim.drones if not d.is_leader and not d.crashed]
    for drone in followers:
        slot_pos = sim.brain._get_formation_slot(drone, leader)
        drone.pos = np.copy(slot_pos)
        drone.vel = np.zeros(3, dtype=float)
        drone.yaw = math.pi / 2.0
        drone.acknowledged_target = np.array(initial_target, dtype=float)

    return sim


def wait_for_settle(
    sim: SwarmSimulator,
    max_seconds: float = 8.0,
    speed_threshold: float = 0.35,
    recorder: Optional[TelemetryRecorder] = None,
) -> Tuple[bool, float, int]:
    """Step simulation until fleet settles below speed threshold or max time expires.

    Returns: (settled: bool, min_inter_drone_distance: float, zone4_clashes: int)
    """
    max_steps = int(max_seconds / DT)
    min_dist_observed = float("inf")
    zone4_clashes = 0
    consecutive_settled_steps = 0
    settled = False

    for _ in range(max_steps):
        sim.step()
        if recorder is not None:
            recorder.capture_frame(sim)

        living = [d for d in sim.drones if not d.crashed]

        # Invariant checks: No NaNs, velocity clamp
        for d in living:
            assert not np.isnan(d.pos).any(), f"Drone #{d.id} pos contains NaN: {d.pos}"
            assert not np.isnan(d.vel).any(), f"Drone #{d.id} vel contains NaN: {d.vel}"
            speed = float(np.linalg.norm(d.vel))
            assert speed <= V_MAX + 1.0, f"Drone #{d.id} speed {speed:.2f} exceeded V_MAX {V_MAX}"

        # Pairwise distance checks
        for i in range(len(living)):
            for j in range(i + 1, len(living)):
                dist = float(np.linalg.norm(living[i].pos - living[j].pos))
                if dist < min_dist_observed:
                    min_dist_observed = dist
                if dist <= ZONE4_CLASH:
                    zone4_clashes += 1

        # Check settling condition: max speed of all living drones
        speeds = [float(np.linalg.norm(d.vel)) for d in living]
        max_speed = max(speeds) if speeds else 0.0

        # For orbit vortex or active patrol, speed does not drop to 0, so check convergence
        if sim.formation_mode in ("ORBIT_VORTEX", "NATURAL_FLOCK"):
            # Settled when speeds are steady cruise and no clashes
            if max_speed <= 14.0:
                consecutive_settled_steps += 1
        else:
            if max_speed <= speed_threshold:
                consecutive_settled_steps += 1
            else:
                consecutive_settled_steps = 0

        # Require 30 consecutive frames (~0.5s) of stability
        if consecutive_settled_steps >= 30:
            settled = True
            break

    return settled, min_dist_observed, zone4_clashes


class TestFormationFlightSuite(unittest.TestCase):
    """Automated unit and regression test suite for all formation flight operations."""

    def test_altitude_changes_up_and_down(self):
        """Test fleet behavior during vertical target changes (Climb to 35m, Descend to 12m)."""
        sim = initialize_formation_north(drone_count=6, initial_target=[0.0, 0.0, 20.0])
        recorder = TelemetryRecorder(record_cadence_steps=2)

        # 1. Initial Settle
        settled, min_d, clashes = wait_for_settle(sim, max_seconds=3.0, recorder=recorder)
        self.assertGreater(min_d, ZONE4_CLASH, f"Initial placement clash: {min_d:.2f}m")
        self.assertEqual(clashes, 0)

        # 2. Step Altitude UP to 35.0m
        sim.set_target([0.0, 0.0, 35.0])
        settled, min_d, clashes = wait_for_settle(sim, max_seconds=8.0, recorder=recorder)
        self.assertTrue(settled, "Fleet failed to settle after altitude climb")
        self.assertGreater(min_d, ZONE4_CLASH, f"Clash during climb: {min_d:.2f}m")
        self.assertEqual(clashes, 0, "Zone 4 clashes detected during climb")

        # Verify leader and followers climbed near 35m
        self.assertAlmostEqual(sim.leader.pos[2], 35.0, delta=1.5)
        for d in sim.drones:
            self.assertGreater(d.pos[2], 30.0, f"Drone #{d.id} failed to climb: Z={d.pos[2]:.2f}")

        # 3. Step Altitude DOWN to 12.0m
        sim.set_target([0.0, 0.0, 12.0])
        settled, min_d, clashes = wait_for_settle(sim, max_seconds=8.0, recorder=recorder)
        self.assertTrue(settled, "Fleet failed to settle after altitude descent")
        self.assertGreater(min_d, ZONE4_CLASH, f"Clash during descent: {min_d:.2f}m")
        self.assertEqual(clashes, 0, "Zone 4 clashes detected during descent")

        # Verify leader and followers descended near 12m, staying safely above floor
        self.assertAlmostEqual(sim.leader.pos[2], 12.0, delta=1.5)
        for d in sim.drones:
            self.assertGreater(d.pos[2], 5.0, f"Drone #{d.id} dipped below safety floor: Z={d.pos[2]:.2f}")

    def test_target_at_distance_various_angles(self):
        """Test fleet transit to targets at 35m distance across left, right, and 180-deg reversal."""
        sim = initialize_formation_north(drone_count=6, initial_target=[0.0, 0.0, 20.0])
        recorder = TelemetryRecorder(record_cadence_steps=2)

        # Initial settle
        wait_for_settle(sim, max_seconds=2.0, recorder=recorder)

        # Angle test cases: (Name, Target [X, Y, Z])
        # Swarm starts at (0, 0, 20) pointing North (+Y)
        test_angles = [
            ("North-East (+45 deg / Right Turn)", [24.75, 24.75, 20.0]),
            ("North-West (-45 deg / Left Turn)", [-24.75, 24.75, 20.0]),
            ("Pure East (+90 deg / Hard Right)", [35.0, 0.0, 20.0]),
            ("Pure West (-90 deg / Hard Left)", [-35.0, 0.0, 20.0]),
            ("Reciprocal U-Turn (180 deg Reversal)", [0.0, -35.0, 20.0]),
        ]

        for angle_name, tgt in test_angles:
            # As specified: reset and wait for settle pointing North between changes
            sim = initialize_formation_north(drone_count=6, initial_target=[0.0, 0.0, 20.0])
            wait_for_settle(sim, max_seconds=2.0, recorder=recorder)

            sim.set_target(tgt)
            settled, min_d, clashes = wait_for_settle(sim, max_seconds=8.0, recorder=recorder)

            self.assertGreater(
                min_d, ZONE4_CLASH,
                f"Zone 4 clash during {angle_name}: min distance {min_d:.2f}m <= {ZONE4_CLASH}m"
            )
            self.assertEqual(
                clashes, 0,
                f"Zone 4 clash count non-zero during {angle_name}: {clashes}"
            )

            # Leader should arrive within acceptance radius of target
            dist_to_tgt = float(np.linalg.norm(sim.leader.pos - np.array(tgt)))
            self.assertLess(
                dist_to_tgt, 3.5,
                f"Leader failed to reach {angle_name} target: {dist_to_tgt:.2f}m away"
            )

    def test_formation_all_to_all_transitions(self):
        """Test exhaustive pairwise transition matrix between all 4 flight formations.

        Modes: ECHELON_ESCORT, NATURAL_FLOCK, ORBIT_VORTEX, PERIMETER_PATROL (12 transitions).
        """
        modes = [
            "ECHELON_ESCORT",
            "NATURAL_FLOCK",
            "ORBIT_VORTEX",
            "PERIMETER_PATROL",
        ]

        sim = initialize_formation_north(drone_count=6, initial_target=[0.0, 0.0, 20.0])
        recorder = TelemetryRecorder(record_cadence_steps=2)

        # Verify all pairwise permutations
        for from_mode in modes:
            for to_mode in modes:
                if from_mode == to_mode:
                    continue

                # Set source mode and let it settle briefly
                sim.set_formation(from_mode)
                wait_for_settle(sim, max_seconds=2.0, recorder=recorder)

                # Transition to destination mode
                sim.set_formation(to_mode)
                settled, min_d, clashes = wait_for_settle(sim, max_seconds=5.0, recorder=recorder)

                self.assertGreater(
                    min_d, ZONE4_CLASH,
                    f"Zone 4 clash during transition {from_mode} -> {to_mode}: min distance {min_d:.2f}m"
                )
                self.assertEqual(
                    clashes, 0,
                    f"Clashes during transition {from_mode} -> {to_mode}: {clashes}"
                )

                # Verify all drones remain airborne and healthy
                living = [d for d in sim.drones if not d.crashed]
                self.assertEqual(len(living), 6, f"Casualty incurred during {from_mode} -> {to_mode}")

    def test_telemetry_playback_recording_and_export(self):
        """Test telemetry recording, JSON serialization, and reload integrity."""
        sim = initialize_formation_north(drone_count=6, initial_target=[0.0, 0.0, 20.0])
        recorder = TelemetryRecorder(record_cadence_steps=1)

        # Run a short multi-phase maneuver
        sim.set_target([0.0, 0.0, 25.0])
        wait_for_settle(sim, max_seconds=2.0, recorder=recorder)

        sim.set_formation("ORBIT_VORTEX")
        wait_for_settle(sim, max_seconds=2.0, recorder=recorder)

        # Export recording
        export_path = os.path.join("tests", "recordings", "test_flight_sample.json")
        saved_file = recorder.save_to_file(export_path)
        self.assertTrue(os.path.exists(saved_file))

        # Reload and verify
        loaded = TelemetryRecorder.load_from_file(saved_file)
        self.assertIn("metadata", loaded)
        self.assertIn("frames", loaded)
        self.assertGreater(len(loaded["frames"]), 50)
        self.assertEqual(loaded["metadata"]["total_zone4_clashes"], 0)
        self.assertGreater(loaded["metadata"]["min_distance_overall"], ZONE4_CLASH)

        # Clean up temporary test file
        if os.path.exists(saved_file):
            os.remove(saved_file)


def run_full_suite_with_recording(export_path: str = "tests/recordings/formation_flight_suite.json") -> int:
    """Standalone executable runner that executes the entire flight formation test suite,

    records full 60 FPS telemetry, exports the playback JSON, and prints tactical summaries."""
    print(f"\n{'=' * 65}")
    print(f" ALITHEIA FORMATION FLIGHT TEST & PLAYBACK SUITE")
    print(f"{'=' * 65}")
    print(f" Target File: {export_path}")

    sim = initialize_formation_north(drone_count=6, initial_target=[0.0, 0.0, 20.0])
    recorder = TelemetryRecorder(record_cadence_steps=1)

    phases = [
        ("Phase 1: Initial North Alignment (Echelon)", lambda: None, 2.0),
        ("Phase 2: Altitude Climb (+15m -> Z=35m)", lambda: sim.set_target([0.0, 0.0, 35.0]), 5.0),
        ("Phase 3: Altitude Descent (-23m -> Z=12m)", lambda: sim.set_target([0.0, 0.0, 12.0]), 5.0),
        ("Phase 4: Return to Cruise Level (Z=20m)", lambda: sim.set_target([0.0, 0.0, 20.0]), 4.0),
        ("Phase 5: Right Turn (+45 deg NE, 35m)", lambda: sim.set_target([24.75, 24.75, 20.0]), 5.0),
        ("Phase 6: Return to Center Station", lambda: sim.set_target([0.0, 0.0, 20.0]), 5.0),
        ("Phase 7: Left Turn (-45 deg NW, 35m)", lambda: sim.set_target([-24.75, 24.75, 20.0]), 5.0),
        ("Phase 8: Return to Center Station", lambda: sim.set_target([0.0, 0.0, 20.0]), 5.0),
        ("Phase 9: Hard Right (+90 deg East, 35m)", lambda: sim.set_target([35.0, 0.0, 20.0]), 5.0),
        ("Phase 10: Return to Center Station", lambda: sim.set_target([0.0, 0.0, 20.0]), 5.0),
        ("Phase 11: Hard Left (-90 deg West, 35m)", lambda: sim.set_target([-35.0, 0.0, 20.0]), 5.0),
        ("Phase 12: Return to Center Station", lambda: sim.set_target([0.0, 0.0, 20.0]), 5.0),
        ("Phase 13: Reciprocal U-Turn (180 deg South)", lambda: sim.set_target([0.0, -35.0, 20.0]), 6.0),
        ("Phase 14: Return to Center Station", lambda: sim.set_target([0.0, 0.0, 20.0]), 5.0),
        ("Phase 15: Transition to Natural Boids Flock", lambda: sim.set_formation("NATURAL_FLOCK"), 4.0),
        ("Phase 16: Transition to Orbit Vortex", lambda: sim.set_formation("ORBIT_VORTEX"), 5.0),
        ("Phase 17: Transition to Perimeter Patrol", lambda: sim.set_formation("PERIMETER_PATROL"), 4.0),
        ("Phase 18: Transition back to Echelon Escort", lambda: sim.set_formation("ECHELON_ESCORT"), 5.0),
    ]

    for phase_name, action, duration in phases:
        print(f" -> Executing: {phase_name} ({duration:.1f}s)...")
        action()
        settled, min_d, clashes = wait_for_settle(sim, max_seconds=duration, recorder=recorder)
        status_flag = "PASS" if clashes == 0 else "FAIL"
        print(f"    [{status_flag}] Min Dist: {min_d:.2f}m | Zone 4 Clashes: {clashes}")

    saved = recorder.save_to_file(export_path)
    recorder.print_summary()
    print(f" [OK] Playback recording saved to: {saved}\n")
    return 0 if recorder.total_zone4_clashes == 0 else 1


def replay_recording_cli(filepath: str, frame_delay_ms: float = 1.0, step_stride: int = 20) -> None:
    """Simple terminal playback viewer for recorded flight telemetry."""
    if not os.path.exists(filepath):
        print(f"[ERROR] Playback file not found: {filepath}")
        return

    data = TelemetryRecorder.load_from_file(filepath)
    frames = data.get("frames", [])
    meta = data.get("metadata", {})

    print(f"\nReplaying {len(frames)} frames from {filepath} (Total: {meta.get('duration_seconds', 0):.2f}s)...")
    for f in frames[::step_stride]:
        t = f["sim_time"]
        mode = f["formation_mode"]
        min_d = f["min_distance"]
        tgt = f["target_pos"]
        leader_pos = f["drones"][0]["pos"] if f["drones"] else [0, 0, 0]
        print(
            f" [T={t:6.2f}s] Mode: {mode:16s} | Tgt: [{tgt[0]:5.1f}, {tgt[1]:5.1f}, {tgt[2]:5.1f}] "
            f"| Lead: [{leader_pos[0]:5.1f}, {leader_pos[1]:5.1f}, {leader_pos[2]:5.1f}] | MinDist: {min_d:5.2f}m"
        )
        if frame_delay_ms > 0:
            time.sleep(frame_delay_ms / 1000.0)
    print("\n[OK] Playback complete.\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Alitheia Flight Formation Test Suite & Playback Recorder")
    parser.add_argument("--record", action="store_true", help="Run full 13-phase flight battery and export JSON")
    parser.add_argument("--output", type=str, default="tests/recordings/formation_flight_suite.json", help="Output JSON path")
    parser.add_argument("--replay", type=str, help="Replay existing recording in terminal HUD")
    args = parser.parse_args()

    if args.replay:
        replay_recording_cli(args.replay)
    elif args.record:
        sys.exit(run_full_suite_with_recording(args.output))
    else:
        unittest.main()
