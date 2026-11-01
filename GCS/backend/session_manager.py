"""
SwarmTech3_UpRising: Persistent Telemetry Session & Reconnect Hydration Manager (session_manager.py)
Bubble 1: Ground Control Station (Operator Airgap)
Eliminates browser refresh "mission amnesia" by maintaining a server-side circular
event history journal and generating state hydration bundles for incoming WebSocket clients.
"""

from __future__ import annotations
from collections import deque
import time
from typing import Dict, List, Optional, Any, Union


class SessionManager:
    """Manages persistent GCS operator sessions, historical mission event journals,
    and state rehydration bundles for reconnecting WebSocket operator terminals."""

    def __init__(self, max_events: int = 100):
        self.max_events: int = max_events
        self.event_history: deque = deque(maxlen=max_events)
        self.session_start_wall: float = time.time()
        self.event_counter: int = 0
        # Seed initial session creation event
        self.add_event("SYSTEM", "Tactical GCS Mission Session Initialized.")

    def add_event(
        self,
        event_type: str,
        message: str,
        sim_time: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Record an event into the circular event journal."""
        self.event_counter += 1
        wall_time = time.strftime("%H:%M:%S")
        record = {
            "id": self.event_counter,
            "type": event_type.upper(),
            "message": message,
            "wall_time": wall_time,
            "sim_time": round(float(sim_time), 2) if sim_time is not None else None,
            "timestamp": time.time(),
        }
        self.event_history.append(record)
        return record

    def get_event_history(self) -> List[Dict[str, Any]]:
        """Return the current event history as a chronological list."""
        return list(self.event_history)

    def clear(self) -> None:
        """Clear the circular event log."""
        self.event_history.clear()

    def get_hydration_bundle(self, sim: Any) -> Dict[str, Any]:
        """Generate a complete state hydration bundle for a reconnecting WebSocket client.
        Combines the complete running simulation state snapshot with the historical event journal."""
        if hasattr(sim, "get_state_snapshot"):
            bundle = dict(sim.get_state_snapshot())
        else:
            bundle = {}

        leader = getattr(sim, "leader", None)
        active_brain = getattr(sim, "active_brain_name", "LEADER_FOLLOWER")
        comms_mode = getattr(sim, "comms_mode", "TETHERED_RELAY")
        formation = getattr(sim, "formation_mode", "ECHELON_ESCORT")
        rf_jamming = getattr(sim, "rf_jamming_rate", 0.0)
        tether_severed = getattr(sim, "tether_severed", False)
        patrol_active = getattr(sim, "patrol_active", False)
        rng_recon_active = getattr(sim, "rng_recon_active", False)
        route_active = getattr(sim, "route_active", False)
        route_name = getattr(sim, "route_name", "NONE")
        route_wps = getattr(sim, "route_waypoints", [])
        route_loop = getattr(sim, "route_loop", True)

        # Operator Airgap: Derive confirmed crashed IDs strictly from downlink perspective
        drones_telemetry = bundle.get("drones", [])
        if drones_telemetry:
            crashed_ids = [d["id"] for d in drones_telemetry if d.get("crashed", False) or d.get("declared_destroyed", False)]
        else:
            drones = getattr(sim, "drones", [])
            crashed_ids = [
                d.id for d in drones
                if getattr(d, "declared_destroyed", False) or (getattr(d, "crashed", False) and getattr(d, "crashed_while_linked", False))
            ]

        target_pos = [round(float(v), 2) for v in getattr(sim, "target_pos", [60.0, 40.0, 30.0])]
        ack_target = [round(float(v), 2) for v in getattr(sim, "get_acknowledged_target", lambda: target_pos)()]

        bundle["type"] = "SESSION_HYDRATION"
        bundle["event_history"] = self.get_event_history()
        bundle["active_brain"] = active_brain
        bundle["comms_mode"] = comms_mode
        bundle["formation"] = formation
        bundle["rf_jamming"] = round(float(rf_jamming), 2)
        bundle["tether_severed"] = tether_severed
        bundle["crashed_ids"] = crashed_ids
        bundle["commanded_target"] = target_pos
        bundle["acknowledged_target"] = ack_target
        bundle["patrol_active"] = patrol_active
        bundle["rng_recon_active"] = rng_recon_active
        bundle["route"] = {
            "active": route_active,
            "name": route_name,
            "waypoints": [[round(float(c), 2) for c in wp] for wp in route_wps],
            "loop": route_loop,
            "current_wp_idx": getattr(leader, "current_wp_idx", 0) if leader else 0,
        }
        bundle["recovery_stage"] = getattr(sim, "recovery_stage", "IDLE")
        bundle["overhead_marshal_active"] = getattr(sim, "overhead_marshal_active", False)
        bundle["swarm_rtb_active"] = getattr(sim, "swarm_rtb_active", False)

        return bundle
