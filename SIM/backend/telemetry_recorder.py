"""
AlitheiaHive SITL: Telemetry Playback Recorder (telemetry_recorder.py)
Bubble 3: World Ground Truth / Flight Trajectory Recording & Playback Harness
Captures high-fidelity 60 FPS flight frames, calculates continuous inter-drone safety
invariants (Zone 4 clashes, velocity bounds, altitude floors), and exports replayable logs.
"""

import json
import math
import os
import time
from typing import List, Dict, Any, Optional
import numpy as np


class TelemetryRecorder:
    """Records 60 FPS trajectory snapshots from SITL simulation for headless verification

    and interactive playback."""

    def __init__(self, record_cadence_steps: int = 1):
        self.record_cadence: int = max(1, record_cadence_steps)
        self.step_counter: int = 0
        self.frames: List[Dict[str, Any]] = []
        self.min_distance_overall: float = float("inf")
        self.total_zone4_clashes: int = 0
        self.max_speed_observed: float = 0.0

    def reset(self) -> None:
        """Clear recorded frames and reset telemetry accumulators."""
        self.step_counter = 0
        self.frames.clear()
        self.min_distance_overall = float("inf")
        self.total_zone4_clashes = 0
        self.max_speed_observed = 0.0

    def capture_frame(self, sim: Any) -> Optional[Dict[str, Any]]:
        """Capture a telemetry snapshot at the configured cadence."""
        self.step_counter += 1
        if (self.step_counter - 1) % self.record_cadence != 0:
            return None

        living_drones = [d for d in sim.drones if not getattr(d, "crashed", False)]

        # Calculate minimum pairwise distance
        min_dist_frame = float("inf")
        zone4_clashes_frame = 0

        for i in range(len(living_drones)):
            d_i = living_drones[i]
            speed = float(np.linalg.norm(d_i.vel))
            if speed > self.max_speed_observed:
                self.max_speed_observed = speed

            for j in range(i + 1, len(living_drones)):
                d_j = living_drones[j]
                dist = float(np.linalg.norm(d_i.pos - d_j.pos))
                if dist < min_dist_frame:
                    min_dist_frame = dist
                if dist <= 1.5:  # Zone 4 clash envelope
                    zone4_clashes_frame += 1

        if min_dist_frame < self.min_distance_overall:
            self.min_distance_overall = min_dist_frame
        self.total_zone4_clashes += zone4_clashes_frame

        # Extract drone telemetry
        drone_snapshots = []
        for d in sim.drones:
            drone_snapshots.append({
                "id": d.id,
                "tier": getattr(d, "tier", 2),
                "is_leader": getattr(d, "is_leader", False),
                "pos": [round(float(v), 3) for v in d.pos],
                "vel": [round(float(v), 3) for v in d.vel],
                "speed": round(float(np.linalg.norm(d.vel)), 2),
                "yaw": round(float(d.yaw), 3),
                "flight_state": getattr(d, "flight_state", "CRUISE"),
                "crashed": getattr(d, "crashed", False),
                "target_station": [round(float(v), 3) for v in d.target_station] if getattr(d, "target_station", None) is not None else None,
            })

        frame = {
            "step": self.step_counter,
            "sim_time": round(float(sim.sim_time), 3),
            "formation_mode": getattr(sim, "formation_mode", getattr(getattr(sim, "brain", None), "formation_mode", "UNKNOWN")),
            "active_brain": getattr(sim, "active_brain_name", "UNKNOWN"),
            "target_pos": [round(float(v), 3) for v in sim.target_pos],
            "drones": drone_snapshots,
            "min_distance": round(min_dist_frame, 3) if min_dist_frame != float("inf") else -1.0,
            "zone4_clashes": zone4_clashes_frame,
        }

        self.frames.append(frame)
        return frame

    def save_to_file(self, filepath: str) -> str:
        """Export recorded flight frames to JSON file."""
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        payload = {
            "metadata": {
                "total_frames": len(self.frames),
                "duration_seconds": self.frames[-1]["sim_time"] if self.frames else 0.0,
                "min_distance_overall": round(self.min_distance_overall, 3) if self.min_distance_overall != float("inf") else 0.0,
                "total_zone4_clashes": self.total_zone4_clashes,
                "max_speed_observed": round(self.max_speed_observed, 2),
            },
            "frames": self.frames,
        }
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        return filepath

    @staticmethod
    def load_from_file(filepath: str) -> Dict[str, Any]:
        """Load recorded flight frames from JSON file."""
        with open(filepath, "r", encoding="utf-8") as f:
            return json.load(f)

    def print_summary(self) -> None:
        """Print concise summary table of the recorded flight."""
        duration = self.frames[-1]["sim_time"] if self.frames else 0.0
        print(f"\n{'=' * 65}")
        print(f" FLIGHT FORMATION TELEMETRY RECORDING SUMMARY")
        print(f"{'=' * 65}")
        print(f" Captured Frames    : {len(self.frames):,} frames ({duration:.2f}s simulated)")
        print(f" Min Inter-Drone Dist: {self.min_distance_overall:.3f}m (Safety floor: > 1.500m)")
        print(f" Zone 4 Clashes     : {self.total_zone4_clashes} clashes (Strict requirement: 0)")
        print(f" Max Speed Observed : {self.max_speed_observed:.2f} m/s (Clamp: 20.0 m/s)")
        print(f"{'=' * 65}\n")
