"""
SwarmTech3_UpRising: Web & WebSocket Telemetry Server (web_server.py)
High-performance aiohttp web server serving the WebGL Tactical GCS frontend
and streaming 60 FPS JSON simulation state frames over WebSocket (/ws).
"""

from __future__ import annotations
import asyncio
import ctypes
import json
import logging
import os
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional, Set, Any

# Ensure repository root is on sys.path even when executed directly via file path
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# Self-healing Conda Windows DLL path bootstrap:
if sys.platform == "win32":
    _prefix = Path(sys.prefix)
    _dll_candidates = [
        _prefix / "Library" / "bin",
        _prefix / "Scripts",
        _prefix.parent.parent / "Library" / "bin",
    ]
    for _d in _dll_candidates:
        if _d.exists():
            _p_str = str(_d)
            if _p_str not in os.environ.get("PATH", ""):
                os.environ["PATH"] = _p_str + os.pathsep + os.environ.get("PATH", "")
            try:
                os.add_dll_directory(_p_str)
            except (AttributeError, OSError):
                pass

import socket
from aiohttp import web, WSMsgType

from SIM.backend.sitl_engine import SwarmSimulator, DT
from SIM.backend.mavlink_bridge import MavlinkBridge
from SIM.backend.urban_geometry import get_default_urban_obstacles
from GCS.backend.session_manager import SessionManager

logger = logging.getLogger("SwarmWebServer")



class TelemetryProtocol(asyncio.DatagramProtocol):
    """Async UDP datagram protocol receiving telemetry snapshots from port 9000."""

    def __init__(self, server: "SwarmWebServer"):
        self.server = server

    def datagram_received(self, data: bytes, addr: tuple):
        try:
            snapshot = json.loads(data.decode("utf-8"))
            self.server.handle_inbound_telemetry(snapshot)
        except Exception:
            pass

class SwarmWebServer:
    """aiohttp server hosting the WebGL GCS static assets and streaming
    real-time 60 FPS simulation telemetry over WebSocket (/ws)."""

    def __init__(
        self,
        sim: Optional[SwarmSimulator] = None,
        bridge: Optional[MavlinkBridge] = None,
        host: str = "127.0.0.1",
        port: int = 8080,
        tx_port: int = 9000,
        rx_port: int = 9001,
        frontend_dir: Optional[Path] = None,
        fps: int = 60,
    ):
        self.sim: Optional[SwarmSimulator] = sim
        self.bridge: Optional[MavlinkBridge] = bridge
        self.host: str = host
        self.port: int = port
        self.tx_port: int = tx_port
        self.rx_port: int = rx_port
        self.fps: int = fps
        self.dt: float = 1.0 / float(fps)
        self.latest_snapshot: Optional[Dict[str, Any]] = None
        self.cmd_sock: Optional[socket.socket] = None
        self.telemetry_transport: Optional[asyncio.DatagramTransport] = None
        self.telemetry_protocol: Optional[asyncio.DatagramProtocol] = None

        # Persistent Telemetry Session & Reconnect Hydration Manager (TKT-32)
        self.session_manager: SessionManager = SessionManager(max_events=100)
        self._last_tether_severed: bool = False
        self._last_crashed_ids: Set[int] = set()
        self._last_leader_id: int = (
            self.sim.leader.id
            if (self.sim and getattr(self.sim, "leader", None))
            else 0
        )
        self._last_rtb_active: bool = False

        if frontend_dir is None:
            candidate_gcs = Path(__file__).resolve().parent.parent / "frontend"
            if candidate_gcs.exists() and (candidate_gcs / "index.html").exists():
                self.frontend_dir = candidate_gcs
            else:
                self.frontend_dir = Path(__file__).resolve().parent.parent.parent / "frontend"
        else:
            self.frontend_dir = Path(frontend_dir)

        # SITL God-View Simulation Lab Frontend (SIM/frontend/) (TKT-35)
        repo_root = Path(__file__).resolve().parent.parent.parent
        self.sim_frontend_dir = repo_root / "SIM" / "frontend"

        # Hydrate default urban canyon for tactical 3D viewport (PLN-08 / TKT-60)
        if self.sim is not None and not getattr(self.sim, "urban_obstacles", None):
            self.sim.load_urban_scenario("default_canyon")

        @web.middleware
        async def no_cache_middleware(request: web.Request, handler) -> web.StreamResponse:
            response = await handler(request)
            if request.path.startswith(("/js", "/css", "/static", "/sim")) or request.path in ("/", "/sim"):
                response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
                response.headers["Pragma"] = "no-cache"
                response.headers["Expires"] = "0"
            return response

        self.app: web.Application = web.Application(middlewares=[no_cache_middleware])
        self.active_websockets: Set[web.WebSocketResponse] = set()
        self.runner: Optional[web.AppRunner] = None
        self.site: Optional[web.TCPSite] = None
        self.sim_task: Optional[asyncio.Task] = None
        self.is_running: bool = False
        self.frames_broadcast: int = 0
        self.last_telemetry_rx_time: float = -100.0
        self._last_wall_rx_time: float = -100.0

        self._setup_routes()

    def is_link_active(self) -> bool:
        """Assess link status strictly via downlinked telemetry freshness without omniscient range peeking (TKT-54)."""
        if self.sim is not None:
            if getattr(self.sim, "sim_time", 0.0) < 1.0:
                return True
            return (getattr(self.sim, "sim_time", 0.0) - self.last_telemetry_rx_time) <= 1.0

        if self.latest_snapshot is None or self._last_wall_rx_time <= 0:
            return False
        return (time.monotonic() - self._last_wall_rx_time) <= 1.5

    def _setup_routes(self) -> None:
        """Configure REST, WebSocket, and static frontend file endpoints."""
        self.app.router.add_get("/ws", self.websocket_handler)
        self.app.router.add_get("/api/status", self.api_status_handler)
        self.app.router.add_post("/api/command", self.api_command_handler)

        # Static GCS frontend routes
        self.app.router.add_get("/", self.index_handler)

        # SITL God-View Simulation Lab routes (TKT-35)
        self.app.router.add_get("/sim", self.sim_index_handler)
        self.app.router.add_get("/sim/", self.sim_index_handler)

        # Ensure GCS frontend directories exist
        self.frontend_dir.mkdir(parents=True, exist_ok=True)
        css_dir = self.frontend_dir / "css"
        css_dir.mkdir(parents=True, exist_ok=True)
        js_dir = self.frontend_dir / "js"
        js_dir.mkdir(parents=True, exist_ok=True)

        self.app.router.add_static("/static", path=str(self.frontend_dir), name="static", show_index=False)
        self.app.router.add_static("/css", path=str(css_dir), name="css", show_index=False)
        self.app.router.add_static("/js", path=str(js_dir), name="js", show_index=False)

        # Ensure SIM frontend directories exist (TKT-35)
        if self.sim_frontend_dir.exists():
            sim_css_dir = self.sim_frontend_dir / "css"
            sim_css_dir.mkdir(parents=True, exist_ok=True)
            sim_js_dir = self.sim_frontend_dir / "js"
            sim_js_dir.mkdir(parents=True, exist_ok=True)

            self.app.router.add_static("/sim/css", path=str(sim_css_dir), name="sim_css", show_index=False)
            self.app.router.add_static("/sim/js", path=str(sim_js_dir), name="sim_js", show_index=False)

    async def index_handler(self, request: web.Request) -> web.Response:
        """Serve index.html from frontend directory or a fallback status page."""
        index_file = self.frontend_dir / "index.html"
        if index_file.exists():
            return web.FileResponse(str(index_file))

        fallback_html = """<!DOCTYPE html>
<html>
<head><title>SwarmTech3_UpRising Telemetry Server</title></head>
<body style="background:#061019;color:#00F0FF;font-family:monospace;padding:2rem;">
  <h2>[SwarmTech3_UpRising] Telemetry & WebGL GCS Server Active</h2>
  <p>WebSocket endpoint: <code>/ws</code></p>
  <p>Status: ONLINE | 60 FPS Vectorized Engine Active</p>
</body>
</html>"""
        return web.Response(text=fallback_html, content_type="text/html")

    async def sim_index_handler(self, request: web.Request) -> web.Response:
        """Serve SITL God-View Simulation Lab index.html (TKT-35)."""
        sim_index_file = self.sim_frontend_dir / "index.html"
        if sim_index_file.exists():
            return web.FileResponse(str(sim_index_file))

        fallback_html = """<!DOCTYPE html>
<html>
<head><title>SITL God-View Simulation Lab</title></head>
<body style="background:#03070d;color:#FF00EA;font-family:monospace;padding:2rem;">
  <h2>[SITL World Engine] Simulation Lab Viewport</h2>
  <p>Status: ONLINE | God-View Viewport Not Found</p>
</body>
</html>"""
        return web.Response(text=fallback_html, content_type="text/html")

    async def api_status_handler(self, request: web.Request) -> web.Response:
        """Return high-level simulation status JSON."""
        if self.sim is not None:
            sim_time = round(float(self.sim.sim_time), 3)
            drone_count = len(self.sim.drones)
            formation = self.sim.formation_mode
        elif self.latest_snapshot is not None:
            sim_time = round(float(self.latest_snapshot.get("sim_time", 0.0)), 3)
            drone_count = len(self.latest_snapshot.get("drones", []))
            formation = self.latest_snapshot.get("formation", "HEXAGON")
        else:
            sim_time = 0.0
            drone_count = 0
            formation = "UNKNOWN"

        return web.json_response({
            "status": "ONLINE" if self.is_link_active() else "LOST_LINK",
            "sim_time": sim_time,
            "drone_count": drone_count,
            "formation": formation,
            "connected_clients": len(self.active_websockets),
            "frames_broadcast": self.frames_broadcast,
        })

    async def api_command_handler(self, request: web.Request) -> web.Response:
        """Ingest commands over HTTP POST."""
        try:
            data = await request.json()
            result = self.execute_command(data)
            return web.json_response(result)
        except Exception as e:
            return web.json_response({"status": "ERROR", "error": str(e)}, status=400)

    def execute_command(self, cmd_dict: Dict[str, Any]) -> Dict[str, Any]:
        """Dispatch operator directive to simulation engine."""
        cmd = cmd_dict.get("cmd", "").upper()

        if self.sim is None:
            # Standalone GCS Mode: Forward operator command across Canyon Airgap via UDP
            if self.cmd_sock is not None:
                try:
                    payload = json.dumps(cmd_dict).encode("utf-8")
                    self.cmd_sock.sendto(payload, (self.host, self.rx_port))
                except OSError as e:
                    logger.warning("Failed to emit command across UDP port %d: %s", self.rx_port, e)
            sim_time = self.latest_snapshot.get("sim_time", 0.0) if self.latest_snapshot else 0.0
            self.session_manager.add_event("CMD", f"Operator Command Dispatched: {cmd}", sim_time=sim_time)
            res = {"status": "OK", "cmd": cmd, "dispatched_via": "UDP"}
            if "pos" in cmd_dict:
                res["target"] = cmd_dict["pos"]
            return res

        if cmd in ("SET_TARGET", "DISPATCH_TARGET"):
            pos = cmd_dict.get("pos", [60.0, 40.0, 30.0])
            self.sim.patrol_active = False
            res = self.sim.set_target(pos)
            self.session_manager.add_event("CMD", f"Dispatching Target -> ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})", sim_time=self.sim.sim_time)
            if isinstance(res, dict) and res.get("status") in ("OUT_OF_RANGE", "DROPPED"):
                return {
                    "status": res.get("status"),
                    "cmd": cmd,
                    "error": res.get("error"),
                    "reason": res.get("reason"),
                }
            return {"status": "OK", "cmd": "SET_TARGET", "target": pos, "patrol_active": False}

        elif cmd in ("TOGGLE_PATROL", "TOGGLE_TARGET_PATROL", "SET_PATROL"):
            active = cmd_dict.get("active", None)
            state = self.sim.toggle_target_patrol(active)
            self.session_manager.add_event("CMD", f"Target Auto-Patrol Loop {'ENGAGED' if state else 'DISENGAGED'}", sim_time=self.sim.sim_time)
            return {"status": "OK", "cmd": cmd, "patrol_active": state}

        elif cmd in ("TOGGLE_RNG_RECON", "TOGGLE_RNG", "RNG_RECON"):
            active = cmd_dict.get("active", None)
            state = self.sim.toggle_rng_recon(active)
            self.session_manager.add_event("CMD", f"Tactical RNG Reconnaissance {'ENGAGED' if state else 'DISENGAGED'}", sim_time=self.sim.sim_time)
            return {"status": "OK", "cmd": cmd, "rng_recon_active": state}

        elif cmd in ("START_RNG_RECON", "START_RNG"):
            res = self.sim.start_rng_recon()
            self.session_manager.add_event("CMD", "Tactical RNG Reconnaissance ENGAGED", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("STOP_RNG_RECON", "STOP_RNG"):
            res = self.sim.stop_rng_recon()
            self.session_manager.add_event("CMD", "Tactical RNG Reconnaissance STOPPED", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("DEPLOY_PATROL_TO_BINGO", "PATROL_TO_BINGO", "START_PATROL_BINGO", "PATROL_BINGO"):
            threshold = float(cmd_dict.get("threshold", cmd_dict.get("bingo_threshold", 25.0)))
            pattern = str(cmd_dict.get("pattern", "BOX_PATROL"))
            res = self.sim.deploy_patrol_to_bingo(bingo_threshold=threshold, pattern=pattern)
            self.session_manager.add_event("CMD", f"Deployed Aux Mission: PATROL -> BINGO (Pattern: {pattern}, Threshold: {threshold}%)", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("STOP_PATROL_TO_BINGO", "ABORT_PATROL_BINGO", "STOP_PATROL_BINGO"):
            res = self.sim.stop_patrol_to_bingo()
            self.session_manager.add_event("CMD", "Aborted Aux Mission: PATROL -> BINGO", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("DEPLOY_SEARCH_AND_RECALL", "SEARCH_AND_RECALL", "DISPATCH_COURIER", "START_SEARCH_AND_RECALL"):
            target_id = cmd_dict.get("target_drone_id", cmd_dict.get("target_id"))
            courier_id = cmd_dict.get("courier_id")
            res = self.sim.deploy_search_and_recall(target_drone_id=target_id, courier_id=courier_id)
            self.session_manager.add_event("CMD", f"Deployed Aux Mission: SEARCH & RECALL (Target: {target_id}, Courier: {courier_id})", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("STOP_SEARCH_AND_RECALL", "ABORT_SEARCH_AND_RECALL", "STOP_COURIER"):
            res = self.sim.stop_search_and_recall()
            self.session_manager.add_event("CMD", "Aborted Aux Mission: SEARCH & RECALL", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("DEPLOY_PAYLOAD_DROP", "PAYLOAD_DROP", "DEPLOY_CAIRN_G1", "DROP_REPEATER"):
            target_pos = cmd_dict.get("target_pos", cmd_dict.get("pos"))
            carrier_id = cmd_dict.get("carrier_id")
            res = self.sim.deploy_payload_drop(target_pos=target_pos, carrier_id=carrier_id)
            self.session_manager.add_event("CMD", f"Deployed Aux Mission: Cairn G1 Repeater Payload Drop (Carrier: {carrier_id})", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("STOP_PAYLOAD_DROP", "ABORT_PAYLOAD_DROP", "ABORT_DROP"):
            res = self.sim.stop_payload_drop()
            self.session_manager.add_event("CMD", "Aborted Aux Mission: Payload Drop", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("TOGGLE_URBAN_OBSTACLES", "SET_URBAN_OBSTACLES"):
            enabled = cmd_dict.get("enabled", True)
            if enabled:
                self.sim.set_urban_obstacles(get_default_urban_obstacles())
            else:
                self.sim.set_urban_obstacles([])
            self.session_manager.add_event(
                "MAP",
                f"Urban 3D Obstacle Geometry {'ENABLED (DEFAULT CANYON)' if enabled else 'CLEARED'}",
                sim_time=self.sim.sim_time,
            )
            return {"status": "OK", "cmd": cmd, "urban_obstacles_count": len(self.sim.urban_obstacles)}

        elif cmd in ("START_ROUTE_PRESET", "SET_ROUTE_PRESET"):
            preset = cmd_dict.get("preset", cmd_dict.get("name", "BOX_PATROL"))
            res = self.sim.start_route_preset(preset)
            self.session_manager.add_event("CMD", f"Deploying Mission Route Preset -> {preset}", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("UPLOAD_MISSION_ROUTE", "DEPLOY_ROUTE", "SET_MISSION_ROUTE"):
            waypoints = cmd_dict.get("waypoints", [])
            loop = bool(cmd_dict.get("loop", True))
            name = str(cmd_dict.get("name", "CUSTOM"))
            res = self.sim.deploy_mission_route(waypoints, loop=loop, name=name)
            self.session_manager.add_event("CMD", f"Uploaded Route '{name}' with {len(waypoints)} waypoints", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("STOP_MISSION_ROUTE", "STOP_ROUTE", "ABORT_ROUTE"):
            res = self.sim.stop_mission_route()
            self.session_manager.add_event("CMD", "Aborted active mission route: STOP_ROUTE", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("PAUSE_MISSION_ROUTE", "PAUSE_ROUTE"):
            res = self.sim.pause_mission_route()
            self.session_manager.add_event("CMD", "Mission Route Paused: PAUSE_ROUTE", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("RESUME_MISSION_ROUTE", "RESUME_ROUTE"):
            res = self.sim.resume_mission_route()
            self.session_manager.add_event("CMD", "Mission Route Resumed: RESUME_ROUTE", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("SKIP_MISSION_WAYPOINT", "SKIP_WAYPOINT", "SKIP_WP"):
            res = self.sim.skip_mission_waypoint()
            new_idx = res.get("current_wp_idx", "?")
            self.session_manager.add_event("CMD", f"Waypoint Skipped -> Advancing to WP #{new_idx}", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("CHECK_MISSION_AUTH", "VALIDATE_MISSION_AUTHORIZATION"):
            m_type = cmd_dict.get("mission_type", cmd_dict.get("type", "PATROL"))
            reqs = cmd_dict.get("required_capabilities")
            target_did = cmd_dict.get("target_drone_id")
            res = self.sim.check_mission_authorization(
                mission_type=m_type,
                required_capabilities=reqs,
                target_drone_id=target_did,
            )
            return res

        elif cmd in ("SET_COMMS_MODE", "SET_MODE"):
            mode = cmd_dict.get("mode", "TETHERED_RELAY")
            # Invariant: Physical tether umbilical cannot be attached mid-flight from GCS
            if mode.upper() in ("TETHERED_RELAY", "TETHER_RELAY", "TETHERED") and self.sim.comms_mode != "TETHERED_RELAY":
                airborne = any(
                    not d.crashed and d.flight_state not in ("LANDED", "GROUND_IDLE")
                    for d in self.sim.drones
                )
                if airborne:
                    self.session_manager.add_event("WARN", "Rejected Tether Relay switch: Umbilical cannot be attached mid-flight", sim_time=self.sim.sim_time)
                    return {
                        "status": "ERROR",
                        "error": "TETHER_LOCK_AIRBORNE",
                        "message": "Physical tether umbilical cannot be attached mid-flight from GCS.",
                        "comms_mode": self.sim.comms_mode,
                    }
            ok = self.sim.set_comms_mode(mode)
            self.session_manager.add_event("CMD", f"Swarm Architecture switched -> {mode}", sim_time=self.sim.sim_time)
            return {
                "status": "OK" if ok else "ERROR",
                "cmd": cmd,
                "comms_mode": self.sim.comms_mode,
                "active_brain": self.sim.active_brain_name,
            }

        elif cmd in ("SET_RF_JAMMING", "SET_JAMMING"):
            rate = float(cmd_dict.get("rate", cmd_dict.get("jamming", 0.0)))
            jamming = self.sim.set_rf_jamming(rate)
            self.session_manager.add_event("CMD", f"Electronic Warfare RF Jamming set to {int(jamming*100)}%", sim_time=self.sim.sim_time)
            return {
                "status": "OK",
                "cmd": cmd,
                "rf_jamming": jamming,
            }

        elif cmd in ("TOGGLE_GNSS_DENIAL", "JAM_GNSS", "SET_GNSS_DENIAL"):
            drone_id = cmd_dict.get("id", cmd_dict.get("drone_id", None))
            denied = cmd_dict.get("denied", None)
            state = self.sim.toggle_gnss_denial(
                drone_id=int(drone_id) if drone_id is not None else None,
                denied=bool(denied) if denied is not None else None,
            )
            target_str = f"Drone #{drone_id}" if drone_id is not None else "ENTIRE FLEET"
            self.session_manager.add_event(
                "EW",
                f"Tactical GNSS Denial {target_str} -> {'ACTIVE (DEAD-RECKONING RTB)' if state else 'RESTORED'}",
                sim_time=self.sim.sim_time,
            )
            return {
                "status": "OK",
                "cmd": cmd,
                "gnss_denied": state,
                "target": target_str,
            }

        elif cmd == "SET_FORMATION":
            mode = cmd_dict.get("mode", "ECHELON_ESCORT")
            res = self.sim.set_formation(mode)
            if isinstance(res, dict) and res.get("status") in ("DROPPED", "OUT_OF_RANGE"):
                return res
            self.session_manager.add_event("CMD", f"Changing Swarm Formation -> {mode}", sim_time=self.sim.sim_time)
            return {"status": "OK", "cmd": cmd, "formation": self.sim.formation_mode}

        elif cmd == "KILL_LEADER":
            self.sim.kill_leader()
            new_id = self.sim.leader.id if self.sim.leader else -1
            self.session_manager.add_event("CASUALTY", f"Injected casualty: KILL_LEADER. New leader: Drone #{new_id}", sim_time=self.sim.sim_time)
            return {"status": "OK", "cmd": cmd, "new_leader_id": new_id}

        elif cmd in ("CUT_TETHER", "JETTISON_TETHER", "SEVER_TETHER"):
            self.sim.cut_tether()
            self.session_manager.add_event("FAILSAFE", "Tactical umbilical manually severed: CUT_TETHER", sim_time=self.sim.sim_time)
            return {
                "status": "OK",
                "cmd": cmd,
                "tether_severed": self.sim.tether_severed,
            }

        elif cmd == "KILL_DRONE":
            drone_id = int(cmd_dict.get("id", 1))
            self.sim.kill_drone(drone_id)
            self.session_manager.add_event("CASUALTY", f"Injected escort casualty: Drone #{drone_id} terminated", sim_time=self.sim.sim_time)
            return {"status": "OK", "cmd": cmd, "killed_id": drone_id}

        elif cmd == "REBOOT_DRONE":
            drone_id = int(cmd_dict.get("id", 1))
            self.sim.reboot_drone(drone_id)
            self.session_manager.add_event("STATUS", f"Drone #{drone_id} rebooted and active", sim_time=self.sim.sim_time)
            return {"status": "OK", "cmd": cmd, "rebooted_id": drone_id}

        elif cmd in ("SPAWN_REINFORCEMENT", "SPAWN_DRONE", "STAGE_REINFORCEMENT"):
            new_drone = self.sim.spawn_reinforcement(auto_launch=False)
            self.session_manager.add_event("SPAWN", f"Standby Drone #{new_drone.id} staged on pad ({new_drone.briefing})", sim_time=self.sim.sim_time)
            return {
                "status": "OK",
                "cmd": cmd,
                "drone_id": new_drone.id,
                "briefing": new_drone.briefing,
                "total_drones": len(self.sim.drones),
            }

        elif cmd in ("LAUNCH_REINFORCEMENT", "REINFORCE_SWARM"):
            new_drone = self.sim.launch_reinforcement()
            self.session_manager.add_event("LAUNCH", f"Reinforcement Drone #{new_drone.id} launched from base pad", sim_time=self.sim.sim_time)
            return {
                "status": "OK",
                "cmd": cmd,
                "drone_id": new_drone.id,
                "total_drones": len(self.sim.drones),
            }

        elif cmd in ("SCRAMBLE_SWARM", "SCRAMBLE", "PAD_SCRAMBLE"):
            stagger = float(cmd_dict.get("stagger", 1.2))
            res = self.sim.command_scramble_swarm(stagger_s=stagger)
            self.session_manager.add_event("SCRAMBLE", f"Cold scramble ordered (stagger: {stagger}s)", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("COLD_RESET", "COLD_GROUND", "COLD_START"):
            count = cmd_dict.get("count", None)
            res = self.sim.command_cold_start_swarm(count=int(count) if count is not None else None)
            self.session_manager.add_event("CMD", "Fleet reset to cold ground launch pads", sim_time=self.sim.sim_time)
            return res

        elif cmd in ("REBOOT_ALL_DEAD", "RESPAWN_ALL_DEAD", "REBOOT_DEAD"):
            revived = self.sim.reboot_all_dead()
            self.session_manager.add_event("STATUS", f"Respawned {len(revived)} destroyed drone(s) in-place", sim_time=self.sim.sim_time)
            return {
                "status": "OK",
                "cmd": cmd,
                "revived_ids": revived,
                "count": len(revived),
            }

        elif cmd in ("CLEAR_WRECKAGE", "SALVAGE_WRECKAGE", "CLEAR_CASUALTIES"):
            purged = self.sim.clear_crashed_wreckage()
            self.session_manager.add_event("STATUS", f"Salvaged {len(purged)} destroyed wreckage from battlefield", sim_time=self.sim.sim_time)
            return {
                "status": "OK",
                "cmd": cmd,
                "purged_ids": purged,
                "count": len(purged),
                "remaining_drones": len(self.sim.drones),
            }

        elif cmd == "REBOOT_FLEET":
            self.sim.reboot_fleet()
            self.session_manager.add_event("STATUS", "Tactical swarm reboot executed across fleet", sim_time=self.sim.sim_time)
            return {"status": "OK", "cmd": cmd, "drone_count": len(self.sim.drones)}

        elif cmd in ("RTB_HOLD", "COMMAND_RTB_HOLD", "RETURN_TO_BASE_HOLD", "RTB"):
            res = self.sim.command_rtb_hold()
            if isinstance(res, dict) and res.get("status") == "DROPPED":
                return {
                    "status": "DROPPED",
                    "cmd": cmd,
                    "reason": res.get("reason", "OUT_OF_RANGE"),
                    "error": res.get("error", "Link inactive or out of range"),
                }
            self.session_manager.add_event("RTB", "Initiating Stage 1: Swarm RTB to overhead sentry hold (0,0,25)", sim_time=self.sim.sim_time)
            return {
                "status": "OK",
                "cmd": cmd,
                "recovery_stage": self.sim.recovery_stage,
                "overhead_marshal_active": self.sim.overhead_marshal_active,
            }

        elif cmd in ("CORRIDOR_ERTB", "ERTB", "EMERGENCY_RTB"):
            res = self.sim.command_corridor_ertb()
            self.session_manager.add_event("RTB", "Initiating Corridor ERTB along ingress breadcrumbs back to Base", sim_time=self.sim.sim_time)
            return {"status": "OK", "cmd": cmd, "result": res}

        elif cmd in ("CANCEL_RTB", "RESUME_MISSION"):
            self.sim.cancel_rtb()
            self.session_manager.add_event("STATUS", "Cancelling recovery: Resuming normal swarm cruise", sim_time=self.sim.sim_time)
            return {
                "status": "OK",
                "cmd": cmd,
                "recovery_stage": self.sim.recovery_stage,
                "overhead_marshal_active": self.sim.overhead_marshal_active,
            }

        elif cmd in ("LAND_SWARM", "COMMAND_LAND_SWARM", "LAND_ALL"):
            res = self.sim.command_land_swarm()
            if isinstance(res, dict) and res.get("status") == "DROPPED":
                return {
                    "status": "DROPPED",
                    "cmd": cmd,
                    "reason": res.get("reason", "OUT_OF_RANGE"),
                    "error": res.get("error", "Link inactive or out of range"),
                }
            queued = res if isinstance(res, list) else []
            self.session_manager.add_event("ATC", f"Engaging Stage 2: Swarm ATC landing pad allocation for {len(queued)} drone(s)", sim_time=self.sim.sim_time)
            return {
                "status": "OK",
                "cmd": cmd,
                "queued_ids": queued,
                "count": len(queued),
            }

        elif cmd in ("CLEAR_PADS", "CLEAR_PADS_NEXT_WAVE", "NEXT_WAVE", "CLEAR_PADS_RELEASE"):
            rearm = bool(cmd_dict.get("rearm", False))
            res = self.sim.clear_pads(turnaround_rearm=rearm)
            self.session_manager.add_event(
                "ATC",
                f"ATC Pads cleared // Next wave released: {res.get('released', 0)} drone(s)",
                sim_time=self.sim.sim_time,
            )
            return {
                "status": "OK",
                "cmd": cmd,
                "cleared_pads": res.get("cleared_pads", 0),
                "released": res.get("released", 0),
                "released_drone_ids": res.get("released_drone_ids", []),
                "current_wave": res.get("current_wave", 1),
            }

        elif cmd in ("RETRIEVE_PAD", "RETRIEVE_DRONE_FROM_PAD"):
            pad_idx = int(cmd_dict.get("pad_idx", 0))
            did = self.sim.retrieve_drone_from_pad(pad_idx)
            self.session_manager.add_event(
                "SIM",
                f"Ground crew retrieved airframe from pad {pad_idx}: Drone {did}" if did is not None else f"Pad {pad_idx} empty",
                sim_time=self.sim.sim_time,
            )
            return {"status": "OK", "cmd": cmd, "pad_idx": pad_idx, "retrieved_drone_id": did}

        elif cmd in ("RETRIEVE_ALL_PADS", "RETRIEVE_ALL_LANDED"):
            retrieved = self.sim.retrieve_all_landed_drones()
            self.session_manager.add_event(
                "SIM",
                f"Ground crew retrieved {len(retrieved)} airframe(s) from launch pads into hangar",
                sim_time=self.sim.sim_time,
            )
            return {"status": "OK", "cmd": cmd, "retrieved_drone_ids": retrieved, "count": len(retrieved)}

        elif cmd in ("LAND_DRONE", "COMMAND_LAND_DRONE"):
            drone_id = int(cmd_dict.get("id", cmd_dict.get("drone_id", 0)))
            link_ok = self.is_link_active()
            if not link_ok:
                return {
                    "status": "DROPPED",
                    "cmd": cmd,
                    "drone_id": drone_id,
                    "reason": "LINK_LOST",
                    "error": "No fresh telemetry downlinked from fleet. Command dropped.",
                }
            success = self.sim.command_land_drone(drone_id)
            if success:
                self.session_manager.add_event("ATC", f"Commanding Drone #{drone_id} to soft-land on assigned pad", sim_time=self.sim.sim_time)
            return {
                "status": "OK" if success else "ERROR",
                "cmd": cmd,
                "drone_id": drone_id,
                "reason": "UNAVAILABLE" if not success else None,
            }

        elif cmd in ("TAKEOFF_DRONE", "RELAUNCH_DRONE"):
            drone_id = int(cmd_dict.get("id", cmd_dict.get("drone_id", 0)))
            link_ok = self.is_link_active()
            if not link_ok:
                return {
                    "status": "DROPPED",
                    "cmd": cmd,
                    "drone_id": drone_id,
                    "reason": "LINK_LOST",
                    "error": "No fresh telemetry downlinked from fleet. Command dropped.",
                }
            success = self.sim.command_takeoff_drone(drone_id)
            if success:
                self.session_manager.add_event("TAKEOFF", f"Launching Drone #{drone_id} from pad: TAKEOFF_DRONE", sim_time=self.sim.sim_time)
            return {
                "status": "OK" if success else "ERROR",
                "cmd": cmd,
                "drone_id": drone_id,
                "reason": "UNAVAILABLE" if not success else None,
            }

        elif cmd in ("DECLARE_DESTROYED", "MARK_DESTROYED", "CONFIRM_CASUALTY", "DECLARE_LOST"):
            drone_id = int(cmd_dict.get("id", cmd_dict.get("drone_id", 0)))
            drone = self.sim.get_drone_by_id(drone_id)
            if drone:
                drone.declared_destroyed = True
                drone.flight_state = "DECLARED_LOST"
                drone.status = "DECLARED_LOST"
                self.session_manager.add_event("OPERATOR", f"Operator confirmed airframe destruction: Drone #{drone_id} marked DECLARED_LOST", sim_time=self.sim.sim_time)
                return {"status": "OK", "cmd": cmd, "drone_id": drone_id}
            return {"status": "ERROR", "error": "DRONE_NOT_FOUND"}

        elif cmd in ("CONTINUE_AUTONOMOUS_SILENCE", "ALLOW_RADIO_SILENCE", "SET_RADIO_SILENCE"):
            permitted = bool(cmd_dict.get("permitted", True))
            for d in getattr(self.sim, "drones", []):
                d.radio_silence_permitted = permitted
            self.session_manager.add_event("C2", f"Radio silence permitted set to {permitted}", sim_time=self.sim.sim_time)
            return {"status": "OK", "cmd": cmd, "radio_silence_permitted": permitted}

        elif cmd == "SET_BRAIN":
            name = cmd_dict.get("brain", "LEADER_FOLLOWER")
            if not self.is_link_active():
                return {
                    "status": "DROPPED",
                    "reason": "LINK_LOST",
                    "error": "Fleet is beyond radio horizon (telemetry link lost). Brain hot-swap dropped.",
                }
            ok = self.sim.set_brain(name)
            self.session_manager.add_event("CMD", f"Hot-swapping Brain Architecture -> {name}", sim_time=self.sim.sim_time)
            return {
                "status": "OK" if ok else "INVALID_BRAIN",
                "cmd": cmd,
                "active_brain": self.sim.active_brain_name,
                "comms_mode": self.sim.comms_mode,
            }

        else:
            brain_res = self.sim.brain.handle_command(cmd_dict)
            if brain_res is not None:
                return {"status": "OK", **brain_res}
            return {"status": "UNKNOWN_COMMAND", "cmd": cmd}

    async def websocket_handler(self, request: web.Request) -> web.WebSocketResponse:
        """Handle 2-way real-time WebSocket connection:
        streams 60 FPS telemetry down, ingests commands up."""
        ws = web.WebSocketResponse(heartbeat=15.0)
        await ws.prepare(request)

        self.active_websockets.add(ws)

        # 1. Immediately send session hydration bundle to eliminate page-refresh amnesia (TKT-32)
        if self.sim is not None:
            hydration_bundle = self.session_manager.get_hydration_bundle(self.sim)
            initial_snapshot = self.sim.get_state_snapshot()
        else:
            snap = self.latest_snapshot or {"sim_time": 0.0, "drones": []}
            hydration_bundle = self.session_manager.get_hydration_bundle(snap)
            initial_snapshot = dict(snap)

        await ws.send_str(json.dumps(hydration_bundle))

        # 2. Send initial telemetry state on connect
        initial_snapshot["type"] = "TELEMETRY_FRAME"
        await ws.send_str(json.dumps(initial_snapshot))

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        cmd_dict = json.loads(msg.data)
                        res = self.execute_command(cmd_dict)
                        await ws.send_str(json.dumps({"type": "COMMAND_ACK", **res}))
                    except Exception as e:
                        await ws.send_str(json.dumps({"type": "ERROR", "message": str(e)}))
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            self.active_websockets.discard(ws)

        return ws

    def handle_inbound_telemetry(self, snapshot: Dict[str, Any]) -> None:
        """Handle incoming telemetry frame received over UDP from SITL or physical radio."""
        self.latest_snapshot = snapshot
        sim_time = snapshot.get("sim_time", 0.0)
        self.last_telemetry_rx_time = sim_time
        self._last_wall_rx_time = time.monotonic()
        self._track_telemetry_events(snapshot)

        # Broadcast state snapshot over WebSocket
        if self.active_websockets:
            payload = json.dumps(snapshot)
            for ws in list(self.active_websockets):
                if not ws.closed:
                    asyncio.create_task(ws.send_str(payload))

    def _track_telemetry_events(self, snapshot: Dict[str, Any]) -> None:
        """Extract and record mission casualty/failsafe/succession events from telemetry."""
        sim_time = snapshot.get("sim_time", 0.0)
        tether_state = snapshot.get("tether", {})
        tether_severed = bool(tether_state.get("severed", False))
        if tether_severed and not self._last_tether_severed:
            self.session_manager.add_event("FAILSAFE", "Tactical Umbilical Severed! Physical cable snap detected.", sim_time=sim_time)
            self._last_tether_severed = True
        elif not tether_severed and self._last_tether_severed:
            self.session_manager.add_event("STATUS", "Tactical umbilical reconnected / swarm rebooted.", sim_time=sim_time)
            self._last_tether_severed = False

        downlink_drones = snapshot.get("drones", [])
        cur_confirmed_dead = set()
        for d_info in downlink_drones:
            did = d_info.get("id")
            is_dead = d_info.get("crashed", False)
            is_declared = d_info.get("declared_destroyed", False)
            is_mayday = d_info.get("mayday_beacon", False)
            is_collision = d_info.get("collision_inferred", False)

            if is_dead or is_declared:
                cur_confirmed_dead.add(did)
                if did not in self._last_crashed_ids:
                    if is_declared:
                        self.session_manager.add_event("CASUALTY", f"Drone #{did} declared lost/destroyed by operator.", sim_time=sim_time)
                    elif is_collision:
                        self.session_manager.add_event("CASUALTY", f"Mid-air collision confirmed for Drone #{did}! Proximity clash detected.", sim_time=sim_time)
                    elif is_mayday:
                        self.session_manager.add_event("CASUALTY", f"Mayday beacon received: Drone #{did} catastrophic impact.", sim_time=sim_time)
                    else:
                        self.session_manager.add_event("CASUALTY", f"Drone #{did} crash confirmed via telemetry downlink.", sim_time=sim_time)
        self._last_crashed_ids = cur_confirmed_dead

        cur_leader_id = tether_state.get("leader_id", -1)
        if cur_leader_id != self._last_leader_id and self._last_leader_id != -1 and cur_leader_id != -1:
            self.session_manager.add_event("SUCCESSION", f"Autonomic Succession: Drone #{cur_leader_id} promoted to Swarm Leader", sim_time=sim_time)
            self._last_leader_id = cur_leader_id

        rtb_active = bool(snapshot.get("swarm_rtb_active", False))
        if rtb_active and not self._last_rtb_active:
            self.session_manager.add_event("RTB", "Doctrine A 30s loiter grace period expired. Swarm RTB initiated to Base (0,0,25).", sim_time=sim_time)
            self._last_rtb_active = True
        elif not rtb_active and self._last_rtb_active:
            self._last_rtb_active = False

    async def _simulation_loop(self) -> None:
        """Core async loop stepping the simulation at locked 60 FPS and
        broadcasting JSON frames to all connected GCS clients (in-memory mode)."""
        interval = self.dt
        next_tick = time.perf_counter()

        while self.is_running and self.sim is not None:
            # Advance GCS C2 layer and physics simulation world
            self.sim.step_c2(interval)
            self.sim.step(interval, advance_c2=False)

            # Broadcast state snapshot over WebSocket & UDP
            snapshot = self.sim.get_state_snapshot()
            snapshot["type"] = "TELEMETRY_FRAME"
            self.latest_snapshot = snapshot

            # Telemetry Freshness Assessment (TKT-54: No range peeking)
            drones_list = snapshot.get("drones", [])
            tether_intact = (snapshot.get("comms_mode") == "TETHERED_RELAY" and not snapshot.get("tether", {}).get("severed", False))
            any_downlink = any(d.get("downlink_active", False) for d in drones_list)
            if any_downlink or tether_intact:
                self.last_telemetry_rx_time = self.sim.sim_time

            self._track_telemetry_events(snapshot)

            # Step UDP / MAVLink bridge if attached
            if self.bridge is not None:
                self.bridge.step()

            # Broadcast state snapshot over WebSocket
            if self.active_websockets:
                payload = json.dumps(snapshot)
                stale_sockets = []

                for ws in list(self.active_websockets):
                    try:
                        if not ws.closed:
                            await ws.send_str(payload)
                        else:
                            stale_sockets.append(ws)
                    except Exception:
                        stale_sockets.append(ws)

                for stale in stale_sockets:
                    self.active_websockets.discard(stale)

            self.frames_broadcast += 1

            # Precision frame pacing with zero clock drift
            next_tick += interval
            now = time.perf_counter()
            sleep_time = next_tick - now
            if sleep_time > 0.001:
                await asyncio.sleep(sleep_time)
            else:
                if sleep_time < -interval:
                    next_tick = now  # Reset deadline if running significantly behind
                await asyncio.sleep(0)  # Yield control to event loop

    async def start(self) -> None:
        """Start web server, UDP command dispatcher, and telemetry ingestion."""
        # Initialize UDP command dispatch socket (for Canyon port 9001)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Boost Windows multimedia timer resolution to 1ms
        if sys.platform == "win32":
            try:
                ctypes.windll.winmm.timeBeginPeriod(1)
            except Exception as e:
                logger.warning(f"Could not enable Windows 1ms timer resolution: {e}")

        self.is_running = True
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()

        self.site = web.TCPSite(self.runner, self.host, self.port)
        await self.site.start()

        loop = asyncio.get_running_loop()
        if self.sim is None:
            # Standalone GCS Mode: Receive 60 FPS telemetry across Canyon UDP port (9000)
            try:
                listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listen_sock.bind((self.host, self.tx_port))
                listen_sock.setblocking(False)
                transport, protocol = await loop.create_datagram_endpoint(
                    lambda: TelemetryProtocol(self),
                    sock=listen_sock,
                )
                self.telemetry_transport = transport
                self.telemetry_protocol = protocol
                logger.info("Standalone GCS UDP Telemetry listener active on %s:%d", self.host, self.tx_port)
            except OSError as err:
                logger.warning("Could not bind standalone GCS UDP listener on %s:%d: %s", self.host, self.tx_port, err)
        else:
            # In-Memory Mode: Launch simulation loop as concurrent asyncio task
            self.sim_task = asyncio.create_task(self._simulation_loop())

    async def stop(self) -> None:
        """Gracefully stop server, close client sockets, and cancel sim/telemetry loops."""
        self.is_running = False

        if self.telemetry_transport is not None:
            self.telemetry_transport.close()
            self.telemetry_transport = None

        if self.cmd_sock is not None:
            try:
                self.cmd_sock.close()
            except OSError:
                pass
            self.cmd_sock = None

        if self.sim_task is not None:
            self.sim_task.cancel()
            try:
                await self.sim_task
            except asyncio.CancelledError:
                pass
            self.sim_task = None

        # Close all active WebSockets
        for ws in list(self.active_websockets):
            try:
                await ws.close(code=1000, message=b"Server shutdown")
            except (ConnectionResetError, asyncio.CancelledError, RuntimeError, OSError):
                pass
        self.active_websockets.clear()

        # Shutdown aiohttp runner
        if self.runner is not None:
            await self.runner.cleanup()
            self.runner = None

        # Close network bridge if attached
        if self.bridge is not None:
            self.bridge.close()

        # Reset Windows multimedia timer resolution
        if sys.platform == "win32":
            try:
                ctypes.windll.winmm.timeEndPeriod(1)
            except (AttributeError, OSError):
                pass


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="SwarmTech3 Tactical GCS Web Server")
    parser.add_argument("--port", type=int, default=8080, help="Web server port")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Web server host")
    parser.add_argument("--tx-port", type=int, default=9000, help="UDP Telemetry TX port from SITL")
    parser.add_argument("--rx-port", type=int, default=9001, help="UDP Command RX port to SITL")
    parser.add_argument("--count", type=int, default=8, help="Number of drones")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    # Standalone GCS operator terminal (no in-memory sim handle)
    server = SwarmWebServer(
        sim=None,
        host=args.host,
        port=args.port,
        tx_port=args.tx_port,
        rx_port=args.rx_port,
    )

    async def main():
        await server.start()
        print(f"SwarmTech3 WebGL GCS running at http://{args.host}:{args.port}", flush=True)
        try:
            while True:
                await asyncio.sleep(1)
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            await server.stop()

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
