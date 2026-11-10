"""
SwarmTech3_UpRising: UDP & MAVLink Network Bridge (mavlink_bridge.py)
Transmits swarm telemetry JSON on UDP port 9000,
receives operator command JSON on UDP port 9001,
and emits simulated MAVLink v2 binary telemetry on UDP port 14550.
"""

from __future__ import annotations
import json
import math
import socket
import struct
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

# Ensure repository root is on sys.path
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

try:
    from pymavlink.dialects.v20 import common as mavlink2
    HAVE_PYMAVLINK = True
except ImportError:
    HAVE_PYMAVLINK = False

from SIM.backend.sitl_engine import SwarmSimulator, RADIO_RANGE, RANGE_HOWL
from OBC.backend.drone_firmware import Drone, FlightState


# ==============================================================================
# Network Port Constants
# ==============================================================================
DEFAULT_HOST: str = "127.0.0.1"
DEFAULT_TX_PORT: int = 9000        # UDP Swarm state JSON broadcast
DEFAULT_RX_PORT: int = 9001        # UDP External command listener
DEFAULT_MAVLINK_PORT: int = 14550   # UDP QGroundControl / SITL loopback

# Default Reference Geographic Anchor for MAVLink GPS mapping (Tokyo Bay / Shinagawa Canal)
DEFAULT_HOME_LAT: float = 35.625971
DEFAULT_HOME_LON: float = 139.734759
DEFAULT_HOME_ALT: float = 20.0             # Meters AMSL

# Backwards compatibility constants
HOME_LAT: float = DEFAULT_HOME_LAT
HOME_LON: float = DEFAULT_HOME_LON
HOME_ALT: float = DEFAULT_HOME_ALT

# Standard MAVLink parameters returned to Mission Planner / QGroundControl
STANDARD_PARAMS: List[Tuple[str, int, Any]] = [
    ("SYSID_THISMAV", 6, lambda s: float(s)),
    ("FRAME_CLASS", 1, lambda s: 1.0),   # 1 = Quad
    ("FRAME_TYPE", 1, lambda s: 1.0),    # 1 = X
    ("ARMING_CHECK", 6, lambda s: 0.0),  # Checks disabled
    ("PILOT_SPEED_UP", 6, lambda s: 250.0),
    ("PILOT_SPEED_DN", 6, lambda s: 150.0),
    ("STAT_RUNTIME", 5, lambda s: 120.0),
    ("BATT_CAPACITY", 6, lambda s: 5000.0),
    ("BATT_MONITOR", 1, lambda s: 4.0),
    ("FS_GCS_ENABLE", 1, lambda s: 1.0),
]


# ==============================================================================
# UDP & MAVLink Network Bridge
# ==============================================================================

# ==============================================================================
# WGS-84 Geodetic Coordinate Conversion Helpers (TKT-63)
# ==============================================================================
def cartesian_to_gps(
    home_lat: float, home_lon: float, home_alt: float, pos: np.ndarray
) -> Tuple[float, float, float]:
    """Convert local Cartesian meters [x=East, y=North, z=Up] to WGS-84 [lat_deg, lon_deg, alt_amsl].
    Accounts for WGS-84 ellipsoidal meridional and parallel curvature."""
    lat_rad = math.radians(home_lat)
    r_lat = 111132.954 - 559.822 * math.cos(2.0 * lat_rad) + 1.175 * math.cos(4.0 * lat_rad)
    r_lon = (math.pi / 180.0) * 6378137.0 * math.cos(lat_rad)
    lat_deg = home_lat + (pos[1] / r_lat)
    lon_deg = home_lon + (pos[0] / r_lon)
    alt_m = home_alt + pos[2]
    return lat_deg, lon_deg, alt_m


def gps_to_cartesian(
    home_lat: float, home_lon: float, home_alt: float, lat_deg: float, lon_deg: float, alt_m: float
) -> Tuple[float, float, float]:
    """Convert WGS-84 [lat_deg, lon_deg, alt_m] to local Cartesian meters [x=East, y=North, z=RelAlt]."""
    lat_rad = math.radians(home_lat)
    r_lat = 111132.954 - 559.822 * math.cos(2.0 * lat_rad) + 1.175 * math.cos(4.0 * lat_rad)
    r_lon = (math.pi / 180.0) * 6378137.0 * math.cos(lat_rad)
    y = (lat_deg - home_lat) * r_lat
    x = (lon_deg - home_lon) * r_lon
    z = (alt_m - home_alt) if alt_m > home_alt else alt_m
    return x, y, z

class MavlinkBridge:
    """Manages UDP JSON telemetry broadcast, inbound command ingestion,
    and simulated binary MAVLink v2 loopback messages for external GCS/SITL."""

    def __init__(
        self,
        sim: SwarmSimulator,
        host: str = DEFAULT_HOST,
        tx_port: int = DEFAULT_TX_PORT,
        rx_port: int = DEFAULT_RX_PORT,
        mavlink_port: int = DEFAULT_MAVLINK_PORT,
        enable_mavlink: bool = True,
    ):
        self.sim: SwarmSimulator = sim
        self.host: str = host
        self.tx_port: int = tx_port
        self.rx_port: int = rx_port
        self.mavlink_port: int = mavlink_port
        self.enable_mavlink: bool = enable_mavlink

        self.last_mavlink_heartbeat: float = -1.0
        self.last_mavlink_pos: float = -1.0
        self.last_reached_sent: bool = False
        self.packet_count: int = 0
        self.commands_processed: int = 0
        self.mavlink_commands_processed: int = 0

        # Setup TX Socket (Broadcast / Send JSON)
        self.tx_sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Setup RX Socket (Non-blocking Listener for JSON commands)
        self.rx_sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_sock.bind((self.host, self.rx_port))
        self.rx_sock.setblocking(False)

        # Setup MAVLink TX/RX Socket
        self.mavlink_sock: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mavlink_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.mavlink_sock.bind((self.host, 0))
        self.mavlink_sock.setblocking(False)
        self.last_client_mavlink_addr: Optional[Tuple[str, int]] = None

        # MAVLink Protocol Encoder
        if HAVE_PYMAVLINK:
            self.mav = mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)
        else:
            self.mav = None

    @property
    def home_lat(self) -> float:
        """Dynamic geographic latitude anchor queried from SIM world engine."""
        return getattr(self.sim, "home_lat", DEFAULT_HOME_LAT)

    @home_lat.setter
    def home_lat(self, val: float) -> None:
        if hasattr(self.sim, "home_lat"):
            self.sim.home_lat = float(val)

    @property
    def home_lon(self) -> float:
        """Dynamic geographic longitude anchor queried from SIM world engine."""
        return getattr(self.sim, "home_lon", DEFAULT_HOME_LON)

    @home_lon.setter
    def home_lon(self, val: float) -> None:
        if hasattr(self.sim, "home_lon"):
            self.sim.home_lon = float(val)

    @property
    def home_alt(self) -> float:
        """Dynamic geographic altitude anchor queried from SIM world engine."""
        return getattr(self.sim, "home_alt", DEFAULT_HOME_ALT)

    @home_alt.setter
    def home_alt(self, val: float) -> None:
        if hasattr(self.sim, "home_alt"):
            self.sim.home_alt = float(val)

    def broadcast_telemetry(self, snapshot: Optional[Dict[str, Any]] = None) -> int:
        """Broadcast simulation state JSON snapshot to UDP port 9000.
        Returns number of bytes sent."""
        if snapshot is None:
            snapshot = self.sim.get_state_snapshot()

        payload = json.dumps(snapshot).encode("utf-8")
        bytes_sent = self.tx_sock.sendto(payload, (self.host, self.tx_port))
        self.packet_count += 1
        return bytes_sent

    def poll_commands(self) -> List[Dict[str, Any]]:
        """Non-blocking poll of UDP port 9001 for incoming command JSON packets.
        Executes recognized commands on the simulator and returns response list."""
        responses = []
        while True:
            try:
                data, addr = self.rx_sock.recvfrom(4096)
                if not data:
                    break

                try:
                    cmd_json = json.loads(data.decode("utf-8"))
                    res = self.execute_command(cmd_json)
                    res["client_addr"] = addr
                    responses.append(res)
                    self.commands_processed += 1
                except (json.JSONDecodeError, UnicodeDecodeError) as e:
                    responses.append({"status": "ERROR", "error": f"Invalid JSON: {str(e)}"})

            except BlockingIOError:
                break
            except Exception as e:
                responses.append({"status": "ERROR", "error": str(e)})
                break

        return responses

    def execute_command(self, cmd_dict: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a structured command on the underlying simulation engine."""
        cmd = cmd_dict.get("cmd", "").upper()

        if cmd in ("SET_TARGET", "DISPATCH_TARGET"):
            pos = cmd_dict.get("pos", [60.0, 40.0, 30.0])
            self.sim.patrol_active = False
            res = self.sim.set_target(pos)
            if isinstance(res, dict) and res.get("status") == "OUT_OF_RANGE":
                return {"status": "OUT_OF_RANGE", "cmd": cmd, "error": res.get("error")}
            return {"status": "OK", "cmd": cmd, "target": pos, "patrol_active": False}

        elif cmd in ("TOGGLE_PATROL", "TOGGLE_TARGET_PATROL", "SET_PATROL"):
            active = cmd_dict.get("active", None)
            state = self.sim.toggle_target_patrol(active)
            return {"status": "OK", "cmd": cmd, "patrol_active": state}

        elif cmd in ("TOGGLE_RNG_RECON", "TOGGLE_RNG", "RNG_RECON"):
            active = cmd_dict.get("active", None)
            state = self.sim.toggle_rng_recon(active)
            return {"status": "OK", "cmd": cmd, "rng_recon_active": state}

        elif cmd in ("START_RNG_RECON", "START_RNG"):
            return self.sim.start_rng_recon()

        elif cmd in ("STOP_RNG_RECON", "STOP_RNG"):
            return self.sim.stop_rng_recon()

        elif cmd in ("SET_COMMS_MODE", "SET_MODE"):
            mode = cmd_dict.get("mode", "TETHERED_RELAY")
            ok = self.sim.set_comms_mode(mode)
            return {
                "status": "OK" if ok else "ERROR",
                "cmd": cmd,
                "comms_mode": self.sim.comms_mode,
                "active_brain": self.sim.active_brain_name,
            }

        elif cmd in ("SET_RF_JAMMING", "SET_JAMMING"):
            rate = float(cmd_dict.get("rate", cmd_dict.get("jamming", 0.0)))
            jamming = self.sim.set_rf_jamming(rate)
            return {
                "status": "OK",
                "cmd": cmd,
                "rf_jamming": jamming,
            }

        elif cmd == "SET_FORMATION":
            mode = cmd_dict.get("mode", "ECHELON_ESCORT")
            self.sim.set_formation(mode)
            return {"status": "OK", "cmd": cmd, "formation": self.sim.formation_mode}

        elif cmd == "KILL_LEADER":
            self.sim.kill_leader()
            new_leader_id = self.sim.leader.id if self.sim.leader else -1
            return {"status": "OK", "cmd": cmd, "new_leader_id": new_leader_id}

        elif cmd == "KILL_DRONE":
            drone_id = int(cmd_dict.get("id", 1))
            self.sim.kill_drone(drone_id)
            return {"status": "OK", "cmd": cmd, "killed_id": drone_id}

        elif cmd == "REBOOT_DRONE":
            drone_id = int(cmd_dict.get("id", 1))
            self.sim.reboot_drone(drone_id)
            return {"status": "OK", "cmd": cmd, "rebooted_id": drone_id}

        elif cmd in ("LAUNCH_REINFORCEMENT", "REINFORCE_SWARM"):
            new_drone = self.sim.launch_reinforcement()
            return {
                "status": "OK",
                "cmd": cmd,
                "drone_id": new_drone.id,
                "total_drones": len(self.sim.drones),
            }

        elif cmd in ("REBOOT_ALL_DEAD", "RESPAWN_ALL_DEAD", "REBOOT_DEAD"):
            revived = self.sim.reboot_all_dead()
            return {
                "status": "OK",
                "cmd": cmd,
                "revived_ids": revived,
                "count": len(revived),
            }

        elif cmd in ("CLEAR_WRECKAGE", "SALVAGE_WRECKAGE", "CLEAR_CASUALTIES"):
            purged = self.sim.clear_crashed_wreckage()
            return {
                "status": "OK",
                "cmd": cmd,
                "purged_ids": purged,
                "count": len(purged),
                "remaining_drones": len(self.sim.drones),
            }

        elif cmd in ("RETRIEVE_PAD", "RETRIEVE_DRONE_FROM_PAD"):
            pad_idx = int(cmd_dict.get("pad_idx", 0))
            did = self.sim.retrieve_drone_from_pad(pad_idx)
            return {"status": "OK", "cmd": cmd, "pad_idx": pad_idx, "retrieved_drone_id": did}

        elif cmd in ("RETRIEVE_ALL_PADS", "RETRIEVE_ALL_LANDED"):
            retrieved = self.sim.retrieve_all_landed_drones()
            return {"status": "OK", "cmd": cmd, "retrieved_drone_ids": retrieved, "count": len(retrieved)}

        elif cmd == "REBOOT_FLEET":
            self.sim.reboot_fleet()
            return {"status": "OK", "cmd": cmd, "drone_count": len(self.sim.drones)}

        elif cmd in ("CUT_TETHER", "JETTISON_TETHER", "SEVER_TETHER"):
            self.sim.cut_tether()
            two_ic = self.sim.get_2ic()
            return {
                "status": "OK",
                "cmd": cmd,
                "tether_severed": self.sim.tether_severed,
                "two_ic_id": two_ic.id if two_ic else None,
            }

        elif cmd in ("RTB_HOLD", "COMMAND_RTB_HOLD", "RETURN_TO_BASE_HOLD", "RTB"):
            self.sim.command_rtb_hold()
            return {
                "status": "OK",
                "cmd": cmd,
                "recovery_stage": self.sim.recovery_stage,
                "overhead_marshal_active": self.sim.overhead_marshal_active,
            }

        elif cmd in ("CANCEL_RTB", "RESUME_MISSION"):
            self.sim.cancel_rtb()
            return {
                "status": "OK",
                "cmd": cmd,
                "recovery_stage": self.sim.recovery_stage,
                "overhead_marshal_active": self.sim.overhead_marshal_active,
            }

        elif cmd in ("LAND_SWARM", "COMMAND_LAND_SWARM", "LAND_ALL"):
            queued = self.sim.command_land_swarm()
            return {
                "status": "OK",
                "cmd": cmd,
                "queued_ids": queued,
                "count": len(queued),
            }

        elif cmd in ("LOAD_URBAN_SCENARIO", "SET_URBAN_SCENARIO", "LOAD_SCENARIO"):
            scenario_name = str(cmd_dict.get("scenario", cmd_dict.get("name", "default_canyon")))
            obstacles = self.sim.load_urban_scenario(scenario_name)
            return {
                "status": "OK",
                "cmd": cmd,
                "scenario": scenario_name,
                "obstacle_count": len(obstacles),
            }

        elif cmd in ("LAND_DRONE", "COMMAND_LAND_DRONE"):
            drone_id = int(cmd_dict.get("id", cmd_dict.get("drone_id", 0)))
            success = self.sim.command_land_drone(drone_id)
            return {"status": "OK" if success else "ERROR", "cmd": cmd, "drone_id": drone_id}

        elif cmd in ("TAKEOFF_DRONE", "RELAUNCH_DRONE"):
            drone_id = int(cmd_dict.get("id", cmd_dict.get("drone_id", 0)))
            success = self.sim.command_takeoff_drone(drone_id)
            return {"status": "OK" if success else "ERROR", "cmd": cmd, "drone_id": drone_id}

        elif cmd in ("SPAWN_REINFORCEMENT", "SPAWN_DRONE", "STAGE_REINFORCEMENT"):
            new_drone = self.sim.spawn_reinforcement(auto_launch=False)
            return {
                "status": "OK",
                "cmd": cmd,
                "drone_id": new_drone.id,
                "briefing": new_drone.briefing,
                "total_drones": len(self.sim.drones),
            }

        elif cmd in ("SCRAMBLE_SWARM", "SCRAMBLE", "PAD_SCRAMBLE"):
            stagger = float(cmd_dict.get("stagger", 1.2))
            return self.sim.command_scramble_swarm(stagger_s=stagger)

        elif cmd in ("COLD_RESET", "COLD_GROUND", "COLD_START"):
            count = cmd_dict.get("count", None)
            return self.sim.command_cold_start_swarm(count=int(count) if count is not None else None)

        elif cmd == "SET_BRAIN":
            name = cmd_dict.get("brain", "LEADER_FOLLOWER")
            ok = self.sim.set_brain(name)
            return {
                "status": "OK" if ok else "INVALID_BRAIN",
                "cmd": cmd,
                "active_brain": self.sim.active_brain_name,
                "comms_mode": self.sim.comms_mode,
            }

        elif cmd in ("START_ROUTE_PRESET", "SET_ROUTE_PRESET"):
            preset = cmd_dict.get("preset", cmd_dict.get("name", "BOX_PATROL"))
            return self.sim.start_route_preset(preset)

        elif cmd in ("UPLOAD_MISSION_ROUTE", "DEPLOY_ROUTE", "SET_MISSION_ROUTE"):
            waypoints = cmd_dict.get("waypoints", [])
            loop = bool(cmd_dict.get("loop", True))
            name = str(cmd_dict.get("name", "CUSTOM"))
            return self.sim.deploy_mission_route(waypoints, loop=loop, name=name)

        elif cmd in ("STOP_MISSION_ROUTE", "STOP_ROUTE", "ABORT_ROUTE"):
            return self.sim.stop_mission_route()

        elif cmd in ("DEPLOY_PATROL_TO_BINGO", "PATROL_TO_BINGO", "START_PATROL_BINGO", "PATROL_BINGO"):
            threshold = float(cmd_dict.get("threshold", cmd_dict.get("bingo_threshold", 25.0)))
            pattern = str(cmd_dict.get("pattern", "BOX_PATROL"))
            return self.sim.deploy_patrol_to_bingo(bingo_threshold=threshold, pattern=pattern)

        elif cmd in ("STOP_PATROL_TO_BINGO", "ABORT_PATROL_BINGO", "STOP_PATROL_BINGO"):
            return self.sim.stop_patrol_to_bingo()

        elif cmd in ("DEPLOY_SEARCH_AND_RECALL", "SEARCH_AND_RECALL", "DISPATCH_COURIER", "START_SEARCH_AND_RECALL"):
            target_id = cmd_dict.get("target_drone_id", cmd_dict.get("target_id"))
            courier_id = cmd_dict.get("courier_id")
            return self.sim.deploy_search_and_recall(target_drone_id=target_id, courier_id=courier_id)

        elif cmd in ("STOP_SEARCH_AND_RECALL", "ABORT_SEARCH_AND_RECALL", "STOP_COURIER"):
            return self.sim.stop_search_and_recall()

        elif cmd in ("DEPLOY_PAYLOAD_DROP", "PAYLOAD_DROP", "DEPLOY_CAIRN_G1", "DROP_REPEATER"):
            target_pos = cmd_dict.get("target_pos") or cmd_dict.get("target") or cmd_dict.get("pos")
            carrier_id = int(cmd_dict["carrier_id"]) if cmd_dict.get("carrier_id") is not None else None
            return self.sim.deploy_payload_drop(target_pos=target_pos, carrier_id=carrier_id)

        elif cmd in ("STOP_PAYLOAD_DROP", "ABORT_PAYLOAD_DROP", "ABORT_DROP"):
            return self.sim.stop_payload_drop()

        elif cmd in ("TOGGLE_URBAN_OBSTACLES", "SET_URBAN_OBSTACLES"):
            from SIM.backend.urban_geometry import get_default_urban_obstacles
            enabled = cmd_dict.get("enabled", True)
            if enabled:
                self.sim.set_urban_obstacles(get_default_urban_obstacles())
            else:
                self.sim.set_urban_obstacles([])
            return {"status": "OK", "cmd": cmd, "urban_obstacles_count": len(self.sim.urban_obstacles)}

        elif cmd in ("TOGGLE_GNSS_DENIAL", "SET_GNSS_DENIAL"):
            drone_id = cmd_dict.get("drone_id", None)
            denied = cmd_dict.get("denied", None)
            res = self.sim.toggle_gnss_denial(drone_id=drone_id, denied=denied)
            return {"status": "OK", "cmd": cmd, "gnss_denied": res}

        elif cmd in ("COMMAND_E_LAND", "E_LAND"):
            drone_id = cmd_dict.get("drone_id", cmd_dict.get("id"))
            reason = str(cmd_dict.get("reason", "MANUAL_OPERATOR"))
            if drone_id is None or str(drone_id).upper() == "ALL":
                res = self.sim.command_e_land_swarm(reason=reason)
                return {"status": "OK", "cmd": cmd, "drones_landed": res}
            ok = self.sim.command_e_land(int(drone_id), reason=reason)
            return {"status": "OK" if ok else "ERROR", "cmd": cmd, "drone_id": int(drone_id)}

        elif cmd in ("SET_FIELD_LZ", "DEPLOY_FIELD_LZ"):
            coords = cmd_dict.get("coords", [0.0, 0.0, 0.0])
            name = str(cmd_dict.get("name", "FLZ-1"))
            radius = float(cmd_dict.get("radius", 15.0))
            capacity = int(cmd_dict.get("capacity", 16))
            slot_spacing = float(cmd_dict.get("slot_spacing", 4.0))
            res = self.sim.set_field_lz(coords, name=name, radius=radius, capacity=capacity, slot_spacing=slot_spacing)
            return {"status": "OK", "cmd": cmd, "field_lz": res}

        elif cmd in ("CLEAR_FIELD_LZ", "REMOVE_FIELD_LZ"):
            res = self.sim.clear_field_lz()
            return {"status": "OK", "cmd": cmd, **res}

        elif cmd in ("TAKEOFF_DRONE", "LAUNCH_DRONE"):
            drone_id = int(cmd_dict.get("drone_id", cmd_dict.get("id", 0)))
            ok = self.sim.command_takeoff_drone(drone_id)
            return {"status": "OK" if ok else "ERROR", "cmd": cmd, "drone_id": drone_id}

        elif cmd in ("DECLARE_LOST", "DECLARE_DESTROYED", "MARK_DESTROYED", "CONFIRM_CASUALTY"):
            drone_id = int(cmd_dict.get("drone_id", cmd_dict.get("id", 0)))
            drone = self.sim.get_drone_by_id(drone_id)
            if drone:
                drone.declared_destroyed = True
                drone.flight_state = "DECLARED_LOST"
                drone.status = "DECLARED_LOST"
                return {"status": "OK", "cmd": cmd, "drone_id": drone_id}
            return {"status": "ERROR", "error": "DRONE_NOT_FOUND"}

        else:
            # Check active brain hook
            brain_res = self.sim.brain.handle_command(cmd_dict)
            if brain_res is not None:
                return {"status": "OK", **brain_res}
            return {"status": "UNKNOWN_COMMAND", "cmd": cmd}

    def emit_mavlink_loopback(self) -> int:
        """Emit simulated MAVLink v2 HEARTBEAT (1 Hz) and GLOBAL_POSITION_INT (10 Hz)
        messages for all active swarm drones on port 14550.
        Each drone is assigned a unique MAVLink System ID: srcSystem = drone.id + 1."""
        if not self.enable_mavlink or not HAVE_PYMAVLINK or self.mav is None:
            return 0

        current_time = self.sim.sim_time
        bytes_sent = 0

        send_hb = (current_time - self.last_mavlink_heartbeat >= 1.0)
        send_pos = (current_time - self.last_mavlink_pos >= 0.1)

        if send_hb:
            self.last_mavlink_heartbeat = current_time
        if send_pos:
            self.last_mavlink_pos = current_time

        if send_hb or send_pos:
            for drone in self.sim.drones:
                if drone.crashed:
                    continue

                sys_id = int(drone.id) + 1
                self.mav.srcSystem = sys_id

                # 1. HEARTBEAT at 1 Hz
                if send_hb:
                    msg_hb = self.mav.heartbeat_encode(
                        type=mavlink2.MAV_TYPE_QUADROTOR,
                        autopilot=mavlink2.MAV_AUTOPILOT_GENERIC,
                        base_mode=mavlink2.MAV_MODE_FLAG_SAFETY_ARMED | mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        custom_mode=4,  # Auto / Guided
                        system_status=mavlink2.MAV_STATE_ACTIVE,
                    )
                    pkt_hb = msg_hb.pack(self.mav)
                    bytes_sent += self.mavlink_sock.sendto(pkt_hb, (self.host, self.mavlink_port))

                # 2. GLOBAL_POSITION_INT, ATTITUDE, and VFR_HUD at 10 Hz (every 0.1s)
                if send_pos:
                    # WGS-84 coordinate projection relative to dynamic SIM home anchor (TKT-63)
                    lat_deg, lon_deg, alt_m = cartesian_to_gps(
                        self.home_lat, self.home_lon, self.home_alt, drone.pos
                    )
                    rel_alt_m = drone.pos[2]

                    # True navigation heading: clockwise azimuth from True North [0..360) deg
                    # In sim, drone.yaw is counter-clockwise rad from East (+X)
                    nav_heading_deg = (90.0 - math.degrees(drone.yaw)) % 360.0
                    hdg_cdeg = int(round(nav_heading_deg * 100.0)) % 36000

                    # NED velocities: North = pos[1] vel, East = pos[0] vel, Down = -pos[2] vel
                    vx_north_cms = int(round(drone.vel[1] * 100.0))
                    vy_east_cms = int(round(drone.vel[0] * 100.0))
                    vz_down_cms = int(round(-drone.vel[2] * 100.0))

                    msg_pos = self.mav.global_position_int_encode(
                        time_boot_ms=int(current_time * 1000.0),
                        lat=int(lat_deg * 1e7),
                        lon=int(lon_deg * 1e7),
                        alt=int(alt_m * 1000.0),
                        relative_alt=int(rel_alt_m * 1000.0),
                        vx=vx_north_cms,
                        vy=vy_east_cms,
                        vz=vz_down_cms,
                        hdg=hdg_cdeg,
                    )
                    pkt_pos = msg_pos.pack(self.mav)
                    bytes_sent += self.mavlink_sock.sendto(pkt_pos, (self.host, self.mavlink_port))

                    # 2b. ATTITUDE (#30) for Mission Planner artificial horizon & compass
                    if hasattr(self.mav, "attitude_encode"):
                        yaw_nav_rad = math.radians(nav_heading_deg)
                        if yaw_nav_rad > math.pi:
                            yaw_nav_rad -= 2.0 * math.pi
                        msg_att = self.mav.attitude_encode(
                            time_boot_ms=int(current_time * 1000.0),
                            roll=float(drone.roll),
                            pitch=float(drone.pitch),
                            yaw=float(yaw_nav_rad),
                            rollspeed=0.0,
                            pitchspeed=0.0,
                            yawspeed=0.0,
                        )
                        pkt_att = msg_att.pack(self.mav)
                        bytes_sent += self.mavlink_sock.sendto(pkt_att, (self.host, self.mavlink_port))

                    # 2c. VFR_HUD (#74) for Mission Planner HUD speed & climb
                    if hasattr(self.mav, "vfr_hud_encode"):
                        speed_xy = float(math.hypot(drone.vel[0], drone.vel[1]))
                        msg_vfr = self.mav.vfr_hud_encode(
                            airspeed=speed_xy,
                            groundspeed=speed_xy,
                            heading=int(round(nav_heading_deg)) % 360,
                            throttle=50 if drone.flight_state in ("TAKEOFF", "CRUISE", "EN_ROUTE") else 0,
                            alt=float(alt_m),
                            climb=float(drone.vel[2]),
                        )
                        pkt_vfr = msg_vfr.pack(self.mav)
                        bytes_sent += self.mavlink_sock.sendto(pkt_vfr, (self.host, self.mavlink_port))

                    # 2d. NAV_CONTROLLER_OUTPUT (#62) for Mission Planner "Direct to current WP" line
                    if hasattr(self.mav, "nav_controller_output_encode"):
                        drone_tgt = getattr(drone, "target_station", None)
                        if drone_tgt is None or getattr(drone, "is_leader", False):
                            drone_tgt = getattr(drone, "acknowledged_target", self.sim.target_pos)

                        delta_xy = drone_tgt[:2] - drone.pos[:2]
                        dist_to_wp = float(np.linalg.norm(delta_xy))
                        if dist_to_wp > 0.1:
                            tb_deg = int(round((90.0 - math.degrees(math.atan2(delta_xy[1], delta_xy[0]))) % 360.0))
                        else:
                            tb_deg = int(round(nav_heading_deg)) % 360

                        # Clamp wp_dist to max tactical horizon (50m) to prevent unbounded line sprawl
                        wp_dist_m = min(50, int(round(dist_to_wp)))

                        msg_nav = self.mav.nav_controller_output_encode(
                            nav_roll=float(drone.roll),
                            nav_pitch=float(drone.pitch),
                            nav_bearing=tb_deg,
                            target_bearing=tb_deg,
                            wp_dist=wp_dist_m,
                            alt_error=float(drone_tgt[2] - drone.pos[2]),
                            aspd_error=0.0,
                            xtrack_error=0.0,
                        )
                        pkt_nav = msg_nav.pack(self.mav)
                        bytes_sent += self.mavlink_sock.sendto(pkt_nav, (self.host, self.mavlink_port))

                    # 2e. GPS_RAW_INT (#24) for Mission Planner GPS Track (COG) & 3D fix status
                    if hasattr(self.mav, "gps_raw_int_encode"):
                        speed_xy = float(math.hypot(drone.vel[0], drone.vel[1]))
                        if speed_xy > 0.1:
                            cog_deg = (90.0 - math.degrees(math.atan2(drone.vel[1], drone.vel[0]))) % 360.0
                            cog_cdeg = int(round(cog_deg * 100.0)) % 36000
                        else:
                            cog_cdeg = hdg_cdeg

                        # MAVLink Glitch Injection on Tactical GNSS Denial (TKT-52)
                        is_denied = getattr(drone, "gnss_denied", False)
                        msg_gps = self.mav.gps_raw_int_encode(
                            time_usec=int(current_time * 1e6),
                            fix_type=0 if is_denied else 3,  # 0 = No GPS Fix
                            lat=0 if is_denied else int(lat_deg * 1e7),
                            lon=0 if is_denied else int(lon_deg * 1e7),
                            alt=int(alt_m * 1000.0),
                            eph=9999 if is_denied else 100,
                            epv=9999 if is_denied else 100,
                            vel=int(round(speed_xy * 100.0)),
                            cog=cog_cdeg,
                            satellites_visible=0 if is_denied else 16,
                        )
                        pkt_gps = msg_gps.pack(self.mav)
                        bytes_sent += self.mavlink_sock.sendto(pkt_gps, (self.host, self.mavlink_port))

            # 1b. Periodic HOME_POSITION broadcast (at 1 Hz) for all active airframes so Mission Planner
            # auto-centers on SIM base and acknowledges home for each vehicle (sysid 1..N)
            if send_hb:
                for drone in self.sim.drones:
                    if not drone.crashed:
                        bytes_sent += self.send_home_position(
                            (self.host, self.mavlink_port),
                            sys_id=int(drone.id) + 1,
                        )

        # 3. MISSION_ITEM_REACHED (#46) when active target reached
        target_reached = self.sim.is_target_reached()
        if target_reached and not self.last_reached_sent:
            self.last_reached_sent = True
            if hasattr(self.mav, "mission_item_reached_encode") and self.sim.leader is not None:
                self.mav.srcSystem = int(self.sim.leader.id) + 1
                seq = getattr(self.sim, "rng_station_count", 0)
                msg_reach = self.mav.mission_item_reached_encode(seq=seq)
                pkt_reach = msg_reach.pack(self.mav)
                bytes_sent += self.mavlink_sock.sendto(pkt_reach, (self.host, self.mavlink_port))
        elif not target_reached:
            self.last_reached_sent = False

        return bytes_sent

    # ==========================================================================
    # MAVLink Inbound Commands & Outbound Helpers (TKT-46)
    # ==========================================================================
    def send_mavlink_packet(self, msg: Any, dest_addr: Optional[Tuple[str, int]] = None) -> int:
        """Pack and send a MAVLink message to client or default loopback port."""
        if not HAVE_PYMAVLINK or self.mav is None:
            return 0
        try:
            pkt = msg.pack(self.mav)
            dest = dest_addr or self.last_client_mavlink_addr or (self.host, self.mavlink_port)
            return self.mavlink_sock.sendto(pkt, dest)
        except Exception:
            return 0

    def send_mavlink_ack(
        self,
        command: int,
        result: int,
        dest_addr: Optional[Tuple[str, int]] = None,
        src_sys: Optional[int] = None,
    ) -> int:
        """Send a MAVLink COMMAND_ACK (#77) packet back to external GCS.
        Ensures srcSystem matches the commanded vehicle sysid so GCS recognizes ACK."""
        if not HAVE_PYMAVLINK or self.mav is None:
            return 0
        prev_sys = self.mav.srcSystem
        if src_sys is not None:
            self.mav.srcSystem = int(src_sys)
        try:
            if hasattr(self.mav, "command_ack_encode"):
                msg_ack = self.mav.command_ack_encode(
                    command=int(command),
                    result=int(result),
                    progress=0,
                    result_param2=0,
                    target_system=getattr(self.mav, "srcSystem", 1),
                    target_component=1,
                )
                return self.send_mavlink_packet(msg_ack, dest_addr)
            return 0
        finally:
            self.mav.srcSystem = prev_sys

    def send_mavlink_statustext(
        self,
        text: str,
        severity: int = 6,  # MAV_SEVERITY_INFO
        dest_addr: Optional[Tuple[str, int]] = None,
        src_sys: Optional[int] = None,
    ) -> int:
        """Send a MAVLink STATUSTEXT (#253) notification message to external GCS."""
        if not HAVE_PYMAVLINK or self.mav is None:
            return 0
        prev_sys = self.mav.srcSystem
        if src_sys is not None:
            self.mav.srcSystem = int(src_sys)
        try:
            if hasattr(self.mav, "statustext_encode"):
                text_bytes = text[:50].encode("utf-8")
                msg_txt = self.mav.statustext_encode(
                    severity=int(severity),
                    text=text_bytes,
                )
                return self.send_mavlink_packet(msg_txt, dest_addr)
            return 0
        finally:
            self.mav.srcSystem = prev_sys

    def send_home_position(
        self,
        dest_addr: Optional[Tuple[str, int]] = None,
        sys_id: Optional[int] = None,
    ) -> int:
        """Send HOME_POSITION (#242) and GPS_GLOBAL_ORIGIN (#49) to external GCS.
        If sys_id is provided, sends for that system; otherwise broadcasts for all active swarm airframes."""
        if not HAVE_PYMAVLINK or self.mav is None:
            return 0
        bytes_sent = 0
        prev_sys = self.mav.srcSystem

        sys_ids = [sys_id] if sys_id is not None else [
            int(d.id) + 1 for d in self.sim.drones if not d.crashed
        ]
        if not sys_ids:
            sys_ids = [1]

        for sid in sys_ids:
            self.mav.srcSystem = sid
            if hasattr(self.mav, "home_position_encode"):
                msg_home = self.mav.home_position_encode(
                    latitude=int(self.home_lat * 1e7),
                    longitude=int(self.home_lon * 1e7),
                    altitude=int(self.home_alt * 1000.0),
                    x=0.0,
                    y=0.0,
                    z=0.0,
                    q=[1.0, 0.0, 0.0, 0.0],
                    approach_x=0.0,
                    approach_y=0.0,
                    approach_z=0.0,
                )
                bytes_sent += self.send_mavlink_packet(msg_home, dest_addr)

        # GPS_GLOBAL_ORIGIN from system 1
        self.mav.srcSystem = 1
        if hasattr(self.mav, "gps_global_origin_encode"):
            msg_origin = self.mav.gps_global_origin_encode(
                latitude=int(self.home_lat * 1e7),
                longitude=int(self.home_lon * 1e7),
                altitude=int(self.home_alt * 1000.0),
            )
            bytes_sent += self.send_mavlink_packet(msg_origin, dest_addr)

        self.mav.srcSystem = prev_sys
        return bytes_sent

    def send_autopilot_version(
        self,
        dest_addr: Optional[Tuple[str, int]] = None,
        sys_id: Optional[int] = None,
    ) -> int:
        """Send AUTOPILOT_VERSION (#148) with airframe hardware silicon UID and capabilities.
        If sys_id is provided, sends for that vehicle; otherwise sends for all active swarm airframes."""
        if not HAVE_PYMAVLINK or self.mav is None:
            return 0
        if not hasattr(self.mav, "autopilot_version_encode"):
            return 0

        bytes_sent = 0
        prev_sys = self.mav.srcSystem

        sys_ids = [sys_id] if sys_id is not None else [
            int(d.id) + 1 for d in self.sim.drones if not d.crashed
        ]
        if not sys_ids:
            sys_ids = [1]

        capabilities = (
            getattr(mavlink2, "MAV_PROTOCOL_CAPABILITY_MAVLINK2", 0)
            | getattr(mavlink2, "MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT", 0)
            | getattr(mavlink2, "MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT", 0)
            | getattr(mavlink2, "MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED", 0)
            | getattr(mavlink2, "MAV_PROTOCOL_CAPABILITY_COMMAND_INT", 0)
        )

        for sid in sys_ids:
            drone = next((d for d in self.sim.drones if d.id + 1 == sid), None)
            hw_uid_str = getattr(drone, "hardware_uid", f"3842A10F9B20{sid - 1:04X}")
            try:
                uid_int = int(hw_uid_str, 16)
            except (ValueError, TypeError):
                uid_int = 0x3842A10F9B200000 + sid

            uid_bytes = uid_int.to_bytes(8, "big")
            uid2 = list(uid_bytes.ljust(18, b"\x00"))

            self.mav.srcSystem = sid
            msg = self.mav.autopilot_version_encode(
                capabilities=capabilities,
                flight_sw_version=0x030C0000,  # v3.12.0
                middleware_sw_version=0,
                os_sw_version=0,
                board_version=0,
                flight_custom_version=[0] * 8,
                middleware_custom_version=[0] * 8,
                os_custom_version=[0] * 8,
                vendor_id=0x26AC,
                product_id=0x0001,
                uid=uid_int,
                uid2=uid2,
            )
            bytes_sent += self.send_mavlink_packet(msg, dest_addr)

        self.mav.srcSystem = prev_sys
        return bytes_sent

    def poll_mavlink_commands(self) -> List[Dict[str, Any]]:
        """Non-blocking poll of MAVLink socket for inbound command packets from Mission Planner/QGC.
        Parses SET_POSITION_TARGET_GLOBAL_INT, SET_POSITION_TARGET_LOCAL_NED, and COMMAND_LONG."""
        if not self.enable_mavlink or not HAVE_PYMAVLINK or self.mav is None:
            return []

        responses = []
        while True:
            try:
                data, addr = self.mavlink_sock.recvfrom(4096)
                if not data:
                    break
                self.last_client_mavlink_addr = addr

                # Decode MAVLink message(s) from datagram
                msgs = []
                try:
                    parsed = self.mav.parse_buffer(bytearray(data))
                    if parsed:
                        msgs.extend(parsed)
                except (TypeError, ValueError, struct.error, IndexError, KeyError, AttributeError):
                    pass

                if not msgs:
                    try:
                        single = self.mav.decode(bytearray(data))
                        if single:
                            msgs.append(single)
                    except (TypeError, ValueError, struct.error, IndexError, KeyError, AttributeError):
                        pass

                for msg in msgs:
                    res = self.handle_mavlink_message(msg, addr)
                    if res:
                        responses.append(res)
                        self.mavlink_commands_processed += 1

            except (BlockingIOError, socket.error):
                break
            except Exception as e:
                responses.append({"status": "ERROR", "error": str(e)})
                break

        return responses

    def handle_mavlink_message(self, msg: Any, addr: Tuple[str, int]) -> Optional[Dict[str, Any]]:
        """Process an individual inbound MAVLink message and execute corresponding C2 actions."""
        msg_id = msg.get_msgId()

        # 1. SET_POSITION_TARGET_GLOBAL_INT (#86) - Mission Planner "Fly Here" clicks
        if msg_id == mavlink2.MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
            lat_deg = msg.lat_int / 1e7
            lon_deg = msg.lon_int / 1e7
            alt_m = float(msg.alt)

            # Determine relative altitude Z
            # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_INT = 5
            if msg.coordinate_frame in (6, 11):
                z = alt_m
            else:
                z = (alt_m - self.home_alt) if alt_m > self.home_alt else alt_m

            # WGS-84 inverse projection to local Cartesian meters relative to SIM home (TKT-63)
            x, y, _ = gps_to_cartesian(
                self.home_lat, self.home_lon, self.home_alt, lat_deg, lon_deg, alt_m
            )

            dist_horiz = math.hypot(x, y)
            GEOFENCE_RADIUS = RANGE_HOWL  # Max GCS RF Howl uplink horizon / arena boundary (m)

            if dist_horiz > GEOFENCE_RADIUS:
                # Reject click beyond tactical geofence
                self.send_mavlink_statustext(
                    f"[GEOFENCE] Target rejected: {dist_horiz:.0f}m > {GEOFENCE_RADIUS:.0f}m",
                    severity=mavlink2.MAV_SEVERITY_WARNING,
                    dest_addr=addr,
                )
                self.send_mavlink_ack(command=86, result=mavlink2.MAV_RESULT_DENIED, dest_addr=addr)
                return {
                    "status": "REJECTED_GEOFENCE",
                    "msg": "SET_POSITION_TARGET_GLOBAL_INT",
                    "dist": round(dist_horiz, 1),
                    "pos": [round(x, 2), round(y, 2), round(z, 2)],
                }

            # Safe target: clamp altitude to safe corridor [5.0m, 45.0m]
            z_clamped = max(5.0, min(45.0, z))
            target_pos = [round(x, 2), round(y, 2), round(z_clamped, 2)]

            self.sim.patrol_active = False
            self.sim.set_target(target_pos)

            self.send_mavlink_statustext(
                f"[C2] Target accepted: X={x:.1f} Y={y:.1f} Z={z_clamped:.1f}",
                severity=mavlink2.MAV_SEVERITY_INFO,
                dest_addr=addr,
            )
            self.send_mavlink_ack(command=86, result=mavlink2.MAV_RESULT_ACCEPTED, dest_addr=addr)

            return {
                "status": "OK",
                "msg": "SET_POSITION_TARGET_GLOBAL_INT",
                "target": target_pos,
                "dist": round(dist_horiz, 1),
            }

        # 2. SET_POSITION_TARGET_LOCAL_NED (#84)
        elif msg_id == mavlink2.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
            # NED: x = North, y = East, z = Down
            x_sim = float(msg.y)
            y_sim = float(msg.x)
            z_sim = -float(msg.z)

            dist_horiz = math.hypot(x_sim, y_sim)
            GEOFENCE_RADIUS = RANGE_HOWL  # Max GCS RF Howl uplink horizon / arena boundary (m)

            if dist_horiz > GEOFENCE_RADIUS:
                self.send_mavlink_statustext(
                    f"[GEOFENCE] Target rejected: {dist_horiz:.0f}m > {GEOFENCE_RADIUS:.0f}m",
                    severity=mavlink2.MAV_SEVERITY_WARNING,
                    dest_addr=addr,
                )
                self.send_mavlink_ack(command=84, result=mavlink2.MAV_RESULT_DENIED, dest_addr=addr)
                return {"status": "REJECTED_GEOFENCE", "msg": "SET_POSITION_TARGET_LOCAL_NED", "dist": dist_horiz}

            z_clamped = max(5.0, min(45.0, z_sim))
            target_pos = [round(x_sim, 2), round(y_sim, 2), round(z_clamped, 2)]

            self.sim.patrol_active = False
            self.sim.set_target(target_pos)
            self.send_mavlink_ack(command=84, result=mavlink2.MAV_RESULT_ACCEPTED, dest_addr=addr)

            return {"status": "OK", "msg": "SET_POSITION_TARGET_LOCAL_NED", "target": target_pos}

        # 3. COMMAND_LONG (#76) - Mission Planner standard actions
        elif msg_id == mavlink2.MAVLINK_MSG_ID_COMMAND_LONG:
            cmd = msg.command
            target_sys = getattr(msg, "target_system", 1) or 1

            # MAV_CMD_NAV_TAKEOFF (24)
            if cmd == mavlink2.MAV_CMD_NAV_TAKEOFF:
                self.sim.command_scramble_swarm()
                for d in self.sim.drones:
                    d.armed = True
                self.send_mavlink_statustext("[C2] Swarm TAKEOFF / Scramble sequence engaged", dest_addr=addr, src_sys=target_sys)
                self.send_mavlink_ack(command=cmd, result=mavlink2.MAV_RESULT_ACCEPTED, dest_addr=addr, src_sys=target_sys)
                return {"status": "OK", "cmd": "MAV_CMD_NAV_TAKEOFF", "sys_id": target_sys}

            # MAV_CMD_NAV_RETURN_TO_LAUNCH (20)
            elif cmd == mavlink2.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                self.sim.command_rtb_hold()
                self.send_mavlink_statustext("[C2] Swarm RTB (Overhead Marshal) engaged", dest_addr=addr, src_sys=target_sys)
                self.send_mavlink_ack(command=cmd, result=mavlink2.MAV_RESULT_ACCEPTED, dest_addr=addr, src_sys=target_sys)
                return {"status": "OK", "cmd": "MAV_CMD_NAV_RETURN_TO_LAUNCH", "sys_id": target_sys}

            # MAV_CMD_NAV_LAND (21)
            elif cmd == mavlink2.MAV_CMD_NAV_LAND:
                queued = self.sim.command_land_swarm()
                self.send_mavlink_statustext(f"[C2] Swarm LAND engaged ({len(queued)} queued)", dest_addr=addr, src_sys=target_sys)
                self.send_mavlink_ack(command=cmd, result=mavlink2.MAV_RESULT_ACCEPTED, dest_addr=addr, src_sys=target_sys)
                return {"status": "OK", "cmd": "MAV_CMD_NAV_LAND", "queued": queued, "sys_id": target_sys}

            # MAV_CMD_COMPONENT_ARM_DISARM (400)
            elif cmd == mavlink2.MAV_CMD_COMPONENT_ARM_DISARM:
                arm = bool(msg.param1 == 1.0)
                for d in self.sim.drones:
                    d.armed = arm
                state_str = "ARMED" if arm else "DISARMED"
                self.send_mavlink_statustext(f"[C2] Swarm {state_str}", dest_addr=addr, src_sys=target_sys)
                self.send_mavlink_ack(command=cmd, result=mavlink2.MAV_RESULT_ACCEPTED, dest_addr=addr, src_sys=target_sys)
                return {"status": "OK", "cmd": "MAV_CMD_COMPONENT_ARM_DISARM", "armed": arm, "sys_id": target_sys}

            # MAV_CMD_DO_SET_HOME (179)
            elif cmd == getattr(mavlink2, "MAV_CMD_DO_SET_HOME", 179):
                if msg.param1 == 0 and abs(msg.param5) > 0.1 and abs(msg.param6) > 0.1:
                    self.home_lat = float(msg.param5)
                    self.home_lon = float(msg.param6)
                    if abs(msg.param7) > 0.1:
                        self.home_alt = float(msg.param7)
                self.send_mavlink_ack(command=cmd, result=mavlink2.MAV_RESULT_ACCEPTED, dest_addr=addr, src_sys=target_sys)
                self.send_home_position(dest_addr=addr, sys_id=target_sys)
                return {"status": "OK", "cmd": "MAV_CMD_DO_SET_HOME", "sys_id": target_sys}

            # MAV_CMD_GET_HOME_POSITION (410)
            elif cmd == getattr(mavlink2, "MAV_CMD_GET_HOME_POSITION", 410):
                self.send_mavlink_ack(command=cmd, result=mavlink2.MAV_RESULT_ACCEPTED, dest_addr=addr, src_sys=target_sys)
                self.send_home_position(dest_addr=addr, sys_id=target_sys)
                return {"status": "OK", "cmd": "MAV_CMD_GET_HOME_POSITION", "sys_id": target_sys}

            # MAV_CMD_REQUEST_MESSAGE (512)
            elif cmd == getattr(mavlink2, "MAV_CMD_REQUEST_MESSAGE", 512):
                req_msg = int(msg.param1)
                if req_msg == mavlink2.MAVLINK_MSG_ID_HOME_POSITION:
                    self.send_home_position(dest_addr=addr, sys_id=target_sys)
                elif req_msg == getattr(mavlink2, "MAVLINK_MSG_ID_AUTOPILOT_VERSION", 148):
                    self.send_autopilot_version(dest_addr=addr, sys_id=target_sys)
                self.send_mavlink_ack(command=cmd, result=mavlink2.MAV_RESULT_ACCEPTED, dest_addr=addr, src_sys=target_sys)
                return {"status": "OK", "cmd": "MAV_CMD_REQUEST_MESSAGE", "requested": req_msg, "sys_id": target_sys}

            # MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES (520) - Mission Planner / QGC Autopilot & Silicon UID probe
            elif cmd == getattr(mavlink2, "MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES", 520):
                self.send_autopilot_version(dest_addr=addr, sys_id=target_sys)
                self.send_mavlink_ack(command=cmd, result=mavlink2.MAV_RESULT_ACCEPTED, dest_addr=addr, src_sys=target_sys)
                return {"status": "OK", "cmd": "MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES", "sys_id": target_sys}

            else:
                self.send_mavlink_ack(command=cmd, result=mavlink2.MAV_RESULT_UNSUPPORTED, dest_addr=addr, src_sys=target_sys)
                return {"status": "UNSUPPORTED", "cmd": cmd, "sys_id": target_sys}

        # 3b. SET_HOME_POSITION (#243)
        elif msg_id == getattr(mavlink2, "MAVLINK_MSG_ID_SET_HOME_POSITION", 243):
            target_sys = getattr(msg, "target_system", 1) or 1
            lat_deg = getattr(msg, "latitude", 0) / 1e7
            lon_deg = getattr(msg, "longitude", 0) / 1e7
            alt_m = getattr(msg, "altitude", 0) / 1000.0
            if abs(lat_deg) > 0.1 and abs(lon_deg) > 0.1:
                self.home_lat = float(lat_deg)
                self.home_lon = float(lon_deg)
                if abs(alt_m) > 0.1:
                    self.home_alt = float(alt_m)
            self.send_home_position(dest_addr=addr, sys_id=target_sys)
            return {"status": "OK", "msg": "SET_HOME_POSITION", "sys_id": target_sys}

        # 4. PARAM_REQUEST_LIST (#21) - Mission Planner parameters download
        elif msg_id == mavlink2.MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            target_sys = getattr(msg, "target_system", 1) or 1
            prev_sys = self.mav.srcSystem
            self.mav.srcSystem = target_sys
            count = len(STANDARD_PARAMS)
            for idx, (param_name, ptype, val_fn) in enumerate(STANDARD_PARAMS):
                msg_val = self.mav.param_value_encode(
                    param_id=param_name.encode("ascii")[:16],
                    param_value=float(val_fn(target_sys)),
                    param_type=int(ptype),
                    param_count=count,
                    param_index=idx,
                )
                self.send_mavlink_packet(msg_val, addr)
            self.mav.srcSystem = prev_sys
            return {
                "status": "OK",
                "msg": "PARAM_REQUEST_LIST",
                "target_system": target_sys,
                "param_count": count,
            }

        # 5. PARAM_REQUEST_READ (#20)
        elif msg_id == mavlink2.MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            target_sys = getattr(msg, "target_system", 1) or 1
            param_idx = getattr(msg, "param_index", -1)
            raw_param_id = getattr(msg, "param_id", b"")
            if isinstance(raw_param_id, str):
                param_name = raw_param_id.strip("\x00")
            elif isinstance(raw_param_id, bytes):
                param_name = raw_param_id.decode("ascii", errors="ignore").strip("\x00")
            else:
                param_name = ""

            matched = None
            matched_idx = -1
            if 0 <= param_idx < len(STANDARD_PARAMS):
                matched = STANDARD_PARAMS[param_idx]
                matched_idx = param_idx
            elif param_name:
                for idx, item in enumerate(STANDARD_PARAMS):
                    if item[0] == param_name:
                        matched = item
                        matched_idx = idx
                        break

            if matched:
                prev_sys = self.mav.srcSystem
                self.mav.srcSystem = target_sys
                msg_val = self.mav.param_value_encode(
                    param_id=matched[0].encode("ascii")[:16],
                    param_value=float(matched[2](target_sys)),
                    param_type=int(matched[1]),
                    param_count=len(STANDARD_PARAMS),
                    param_index=matched_idx,
                )
                self.send_mavlink_packet(msg_val, addr)
                self.mav.srcSystem = prev_sys

            return {
                "status": "OK",
                "msg": "PARAM_REQUEST_READ",
                "target_system": target_sys,
                "param_id": param_name,
                "param_index": param_idx,
            }

        # 6. PARAM_SET (#23)
        elif msg_id == mavlink2.MAVLINK_MSG_ID_PARAM_SET:
            target_sys = getattr(msg, "target_system", 1) or 1
            raw_param_id = getattr(msg, "param_id", b"")
            if isinstance(raw_param_id, str):
                param_name = raw_param_id.strip("\x00")
            elif isinstance(raw_param_id, bytes):
                param_name = raw_param_id.decode("ascii", errors="ignore").strip("\x00")
            else:
                param_name = ""
            param_val = float(getattr(msg, "param_value", 0.0))
            param_type = int(getattr(msg, "param_type", 6))

            matched_idx = -1
            for idx, item in enumerate(STANDARD_PARAMS):
                if item[0] == param_name:
                    matched_idx = idx
                    break

            prev_sys = self.mav.srcSystem
            self.mav.srcSystem = target_sys
            msg_val = self.mav.param_value_encode(
                param_id=param_name.encode("ascii")[:16],
                param_value=param_val,
                param_type=param_type,
                param_count=len(STANDARD_PARAMS),
                param_index=matched_idx if matched_idx >= 0 else 0,
            )
            self.send_mavlink_packet(msg_val, addr)
            self.mav.srcSystem = prev_sys

            return {
                "status": "OK",
                "msg": "PARAM_SET",
                "target_system": target_sys,
                "param_id": param_name,
                "param_value": param_val,
            }

        return None

    def step(self, snapshot: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Unified update step: polls commands, broadcasts JSON state, and sends MAVLink."""
        cmd_responses = self.poll_commands()
        mav_responses = self.poll_mavlink_commands()
        tx_bytes = self.broadcast_telemetry(snapshot)
        mav_bytes = self.emit_mavlink_loopback()

        return {
            "commands_processed": len(cmd_responses) + len(mav_responses),
            "command_responses": cmd_responses,
            "mavlink_responses": mav_responses,
            "telemetry_bytes_sent": tx_bytes,
            "mavlink_bytes_sent": mav_bytes,
        }

    def close(self) -> None:
        """Close all network sockets cleanly."""
        try:
            self.tx_sock.close()
        except (OSError, socket.error):
            pass
        try:
            self.rx_sock.close()
        except (OSError, socket.error):
            pass
        try:
            self.mavlink_sock.close()
        except (OSError, socket.error):
            pass
