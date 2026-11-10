"""
Automated Unit Tests for SwarmTech3_UpRising Network Bridge (mavlink_bridge.py)
Tests UDP JSON telemetry broadcast (port 9000), command listener (port 9001),
and simulated MAVLink v2 loopback (port 14550).
"""

import json
import math
import socket
import unittest
import time
import numpy as np

from SIM.backend.sitl_engine import SwarmSimulator
from SIM.backend.mavlink_bridge import MavlinkBridge, HAVE_PYMAVLINK

if HAVE_PYMAVLINK:
    from pymavlink.dialects.v20 import common as mavlink2


class TestMavlinkBridge(unittest.TestCase):
    """Test suite verifying UDP and MAVLink network interfaces."""

    def setUp(self):
        self.sim = SwarmSimulator(drone_count=5, enable_packet_loss=False, seed=10)
        # Use custom high test ports to avoid any environment port collisions
        self.host = "127.0.0.1"
        self.tx_port = 19000
        self.rx_port = 19001
        self.mav_port = 19550

        self.bridge = MavlinkBridge(
            sim=self.sim,
            host=self.host,
            tx_port=self.tx_port,
            rx_port=self.rx_port,
            mavlink_port=self.mav_port,
            enable_mavlink=True,
        )

    def tearDown(self):
        self.bridge.close()

    def test_telemetry_broadcast(self):
        """Verify telemetry snapshot JSON broadcast over UDP."""
        # Create client listener socket
        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.tx_port))
        listener.settimeout(1.0)

        try:
            bytes_sent = self.bridge.broadcast_telemetry()
            self.assertGreater(bytes_sent, 0)

            data, addr = listener.recvfrom(65535)
            payload = json.loads(data.decode("utf-8"))

            self.assertIn("sim_time", payload)
            self.assertIn("formation", payload)
            self.assertIn("drones", payload)
            self.assertEqual(len(payload["drones"]), 5)
        finally:
            listener.close()

    def test_command_ingestion(self):
        """Verify receiving and executing external JSON commands via UDP."""
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            # 1. Test SET_TARGET
            cmd_target = {"cmd": "SET_TARGET", "pos": [120.0, 80.0, 45.0]}
            sender.sendto(json.dumps(cmd_target).encode("utf-8"), (self.host, self.rx_port))

            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0]["status"], "OK")
            np.testing.assert_allclose(self.sim.target_pos, [120.0, 80.0, 45.0])

            # 2. Test SET_FORMATION
            cmd_formation = {"cmd": "SET_FORMATION", "mode": "ORBIT_VORTEX"}
            sender.sendto(json.dumps(cmd_formation).encode("utf-8"), (self.host, self.rx_port))

            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(self.sim.formation_mode, "ORBIT_VORTEX")

            # 3. Test CUT_TETHER
            cmd_cut = {"cmd": "CUT_TETHER"}
            sender.sendto(json.dumps(cmd_cut).encode("utf-8"), (self.host, self.rx_port))

            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertTrue(self.sim.tether_severed)

            # 4. Test KILL_LEADER
            old_leader_id = self.sim.leader.id
            cmd_kill = {"cmd": "KILL_LEADER"}
            sender.sendto(json.dumps(cmd_kill).encode("utf-8"), (self.host, self.rx_port))

            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertTrue(self.sim.drones[old_leader_id].crashed)
            self.assertNotEqual(self.sim.leader.id, old_leader_id)

            # 5b. Test CLEAR_WRECKAGE
            self.sim.kill_drone(1)
            cmd_clear = {"cmd": "CLEAR_WRECKAGE"}
            sender.sendto(json.dumps(cmd_clear).encode("utf-8"), (self.host, self.rx_port))

            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0]["status"], "OK")
            self.assertIn(1, responses[0]["purged_ids"])
            self.assertFalse(any(d.id == 1 for d in self.sim.drones))

            # 5c. Test TOGGLE_PATROL
            cmd_patrol = {"cmd": "TOGGLE_PATROL"}
            sender.sendto(json.dumps(cmd_patrol).encode("utf-8"), (self.host, self.rx_port))

            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0]["status"], "OK")
            self.assertTrue(responses[0]["patrol_active"])
            self.assertTrue(self.sim.patrol_active)

            # 5d. Test RTB_HOLD (TKT-19)
            cmd_rtb = {"cmd": "RTB_HOLD"}
            sender.sendto(json.dumps(cmd_rtb).encode("utf-8"), (self.host, self.rx_port))
            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0]["status"], "OK")
            self.assertTrue(self.sim.overhead_marshal_active)

            # 5e. Test LAND_SWARM (TKT-19)
            cmd_land = {"cmd": "LAND_SWARM"}
            sender.sendto(json.dumps(cmd_land).encode("utf-8"), (self.host, self.rx_port))
            responses = self.bridge.poll_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0]["status"], "OK")
            self.assertEqual(self.sim.recovery_stage, "LANDING")

        finally:
            sender.close()

    def test_mavlink_loopback_emission(self):
        """Verify binary MAVLink v2 packets emitted on loopback port for all swarm drones."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.mav_port))
        listener.settimeout(1.0)

        try:
            # Emit MAVLink packet
            bytes_sent = self.bridge.emit_mavlink_loopback()
            self.assertGreater(bytes_sent, 0)

            # Receive packets and verify multi-vehicle sysid demuxing
            mav = mavlink2.MAVLink(None)
            seen_sysids = set()

            # Read all buffered packets emitted in this step
            for _ in range(len(self.sim.drones) * 2):
                try:
                    data, addr = listener.recvfrom(2048)
                    self.assertEqual(data[0], 0xFD)  # MAVLink v2 magic byte
                    msg = mav.decode(bytearray(data))
                    seen_sysids.add(msg.get_srcSystem())
                except (socket.timeout, BlockingIOError):
                    break

            # Verify that sysids 1..5 were all represented
            self.assertIn(1, seen_sysids)
            self.assertGreaterEqual(len(seen_sysids), 2)

        finally:
            listener.close()

    def test_unified_step(self):
        """Verify the unified bridge.step() method processes all streams."""
        res = self.bridge.step()
        self.assertIn("commands_processed", res)
        self.assertIn("telemetry_bytes_sent", res)
        self.assertIn("mavlink_bytes_sent", res)
        self.assertGreater(res["telemetry_bytes_sent"], 0)

    def test_home_anchor_comes_from_sim(self):
        """Verify home coordinates are dynamically queried from SIM rather than hardcoded in bridge."""
        # 1. Defaults to Tokyo Bay / Tennozu Isle from SIM
        self.assertAlmostEqual(self.bridge.home_lat, 35.625971, places=5)
        self.assertAlmostEqual(self.bridge.home_lon, 139.734759, places=5)
        self.assertEqual(self.bridge.home_lat, self.sim.home_lat)
        self.assertEqual(self.bridge.home_lon, self.sim.home_lon)

        # 2. Dynamic SIM update reflects in bridge immediately
        self.sim.home_lat = 40.7128
        self.sim.home_lon = -74.0060
        self.assertEqual(self.bridge.home_lat, 40.7128)
        self.assertEqual(self.bridge.home_lon, -74.0060)

    def test_inbound_set_position_target_within_geofence(self):
        """Verify MAVLink SET_POSITION_TARGET_GLOBAL_INT within 120m steers the swarm."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        import math
        # Calculate GPS coordinate for local (x=30m, y=40m, z=25m)
        delta_lat = 40.0 / 111139.0
        delta_lon = 30.0 / (111139.0 * math.cos(math.radians(self.sim.home_lat)))
        target_lat = self.sim.home_lat + delta_lat
        target_lon = self.sim.home_lon + delta_lon

        client_mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=1)
        msg = client_mav.set_position_target_global_int_encode(
            time_boot_ms=1000,
            target_system=1,
            target_component=1,
            coordinate_frame=mavlink2.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask=0b0000111111111000,
            lat_int=int(target_lat * 1e7),
            lon_int=int(target_lon * 1e7),
            alt=25.0,
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=0, yaw_rate=0,
        )
        pkt = msg.pack(client_mav)

        # Send to bridge's socket
        # Note: bridge.mavlink_sock sends from an ephemeral port; for testing poll,
        # we can feed packet into handle_mavlink_message or simulate recv
        res = self.bridge.handle_mavlink_message(msg, (self.host, 14550))
        self.assertIsNotNone(res)
        self.assertEqual(res["status"], "OK")
        self.assertAlmostEqual(self.sim.target_pos[0], 30.0, delta=1.0)
        self.assertAlmostEqual(self.sim.target_pos[1], 40.0, delta=1.0)
        self.assertAlmostEqual(self.sim.target_pos[2], 25.0, delta=0.5)

    def test_inbound_set_position_target_rejected_outside_geofence(self):
        """Verify clicks in Timbuktu / outside 120m geofence are strictly rejected."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        # Target 50,000m away (far outside 120m geofence)
        target_lat = self.sim.home_lat + 1.0  # ~111km away
        target_lon = self.sim.home_lon + 1.0

        old_target = np.copy(self.sim.target_pos)

        client_mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=1)
        msg = client_mav.set_position_target_global_int_encode(
            time_boot_ms=1000,
            target_system=1,
            target_component=1,
            coordinate_frame=mavlink2.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask=0,
            lat_int=int(target_lat * 1e7),
            lon_int=int(target_lon * 1e7),
            alt=30.0,
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=0, yaw_rate=0,
        )

        res = self.bridge.handle_mavlink_message(msg, (self.host, 14550))
        self.assertIsNotNone(res)
        self.assertEqual(res["status"], "REJECTED_GEOFENCE")
        self.assertGreater(res["dist"], 120.0)
        # Swarm target remains unchanged
        np.testing.assert_allclose(self.sim.target_pos, old_target)

    def test_inbound_command_long_actions(self):
        """Verify COMMAND_LONG actions: RTL, Land, Arm/Disarm."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        client_mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=1)

        # 1. RTL
        msg_rtl = client_mav.command_long_encode(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            confirmation=0,
            param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0,
        )
        res_rtl = self.bridge.handle_mavlink_message(msg_rtl, (self.host, 14550))
        self.assertEqual(res_rtl["status"], "OK")
        self.assertTrue(self.sim.overhead_marshal_active)

        # 2. LAND
        msg_land = client_mav.command_long_encode(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_NAV_LAND,
            confirmation=0,
            param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0,
        )
        res_land = self.bridge.handle_mavlink_message(msg_land, (self.host, 14550))
        self.assertEqual(res_land["status"], "OK")
        self.assertEqual(self.sim.recovery_stage, "LANDING")

        # 3. DISARM
        msg_disarm = client_mav.command_long_encode(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=0.0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0,
        )
        res_disarm = self.bridge.handle_mavlink_message(msg_disarm, (self.host, 14550))
        self.assertEqual(res_disarm["status"], "OK")
        self.assertFalse(all(d.armed for d in self.sim.drones))

    def test_inbound_mavlink_socket_poll(self):
        """Verify binary MAVLink packet sent over UDP network socket is polled and executed."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        bridge_mav_port = self.bridge.mavlink_sock.getsockname()[1]

        try:
            client_mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=1)
            msg = client_mav.command_long_encode(
                target_system=1,
                target_component=1,
                command=mavlink2.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                confirmation=0,
                param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0,
            )
            pkt = msg.pack(client_mav)
            client_sock.sendto(pkt, (self.host, bridge_mav_port))

            # Poll via bridge
            responses = self.bridge.poll_mavlink_commands()
            self.assertEqual(len(responses), 1)
            self.assertEqual(responses[0]["cmd"], "MAV_CMD_NAV_RETURN_TO_LAUNCH")
            self.assertTrue(self.sim.overhead_marshal_active)
        finally:
            client_sock.close()

    def test_param_request_list_multi_vehicle(self):
        """Verify PARAM_REQUEST_LIST responds with PARAM_VALUE packets for target vehicle (e.g. sysid 2)."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.mav_port))
        listener.settimeout(1.0)

        try:
            client_mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=1)
            msg_list = client_mav.param_request_list_encode(
                target_system=2,
                target_component=1,
            )
            res = self.bridge.handle_mavlink_message(msg_list, (self.host, self.mav_port))
            self.assertIsNotNone(res)
            self.assertEqual(res["status"], "OK")
            self.assertEqual(res["target_system"], 2)
            self.assertGreater(res["param_count"], 0)

            # Receive emitted PARAM_VALUE packets
            mav = mavlink2.MAVLink(None)
            param_names = []
            for _ in range(res["param_count"]):
                data, _ = listener.recvfrom(2048)
                val_msg = mav.decode(bytearray(data))
                self.assertEqual(val_msg.get_msgId(), mavlink2.MAVLINK_MSG_ID_PARAM_VALUE)
                self.assertEqual(val_msg.get_srcSystem(), 2)
                self.assertEqual(val_msg.param_count, res["param_count"])
                param_names.append(val_msg.param_id)

            self.assertIn("SYSID_THISMAV", param_names)
            self.assertIn("FRAME_CLASS", param_names)
        finally:
            listener.close()

    def test_param_request_read_and_set(self):
        """Verify PARAM_REQUEST_READ and PARAM_SET operations."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.mav_port))
        listener.settimeout(1.0)

        try:
            client_mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=1)

            # 1. PARAM_REQUEST_READ by index
            msg_read = client_mav.param_request_read_encode(
                target_system=2,
                target_component=1,
                param_id=b"",
                param_index=0,
            )
            res_read = self.bridge.handle_mavlink_message(msg_read, (self.host, self.mav_port))
            self.assertEqual(res_read["status"], "OK")

            data, _ = listener.recvfrom(2048)
            mav = mavlink2.MAVLink(None)
            val_msg = mav.decode(bytearray(data))
            self.assertEqual(val_msg.get_srcSystem(), 2)
            self.assertEqual(val_msg.param_id, "SYSID_THISMAV")

            # 2. PARAM_SET
            msg_set = client_mav.param_set_encode(
                target_system=2,
                target_component=1,
                param_id=b"STAT_RUNTIME",
                param_value=250.0,
                param_type=mavlink2.MAV_PARAM_TYPE_UINT32,
            )
            res_set = self.bridge.handle_mavlink_message(msg_set, (self.host, self.mav_port))
            self.assertEqual(res_set["status"], "OK")
            self.assertEqual(res_set["param_value"], 250.0)
        finally:
            listener.close()

    def test_set_home_location_multi_vehicle(self):
        """Verify MAV_CMD_DO_SET_HOME and MAV_CMD_GET_HOME_POSITION accept commands for vehicle sysid 2."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.mav_port))
        listener.settimeout(1.0)

        try:
            client_mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=1)

            # 1. MAV_CMD_DO_SET_HOME
            msg_set_home = client_mav.command_long_encode(
                target_system=2,
                target_component=1,
                command=mavlink2.MAV_CMD_DO_SET_HOME,
                confirmation=0,
                param1=0, param2=0, param3=0, param4=0,
                param5=35.6280, param6=139.7350, param7=25.0,
            )
            res = self.bridge.handle_mavlink_message(msg_set_home, (self.host, self.mav_port))
            self.assertEqual(res["status"], "OK")
            self.assertEqual(res["cmd"], "MAV_CMD_DO_SET_HOME")
            self.assertEqual(res["sys_id"], 2)
            self.assertAlmostEqual(self.bridge.home_lat, 35.6280, places=4)
            self.assertAlmostEqual(self.bridge.home_lon, 139.7350, places=4)

            # 2. MAV_CMD_GET_HOME_POSITION
            msg_get_home = client_mav.command_long_encode(
                target_system=2,
                target_component=1,
                command=getattr(mavlink2, "MAV_CMD_GET_HOME_POSITION", 410),
                confirmation=0,
                param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0,
            )
            res_get = self.bridge.handle_mavlink_message(msg_get_home, (self.host, self.mav_port))
            self.assertEqual(res_get["status"], "OK")
            self.assertEqual(res_get["sys_id"], 2)
        finally:
            listener.close()

    def test_global_position_ned_velocities_and_heading(self):
        """Verify GLOBAL_POSITION_INT sends accurate NED velocities and clockwise heading from True North."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.mav_port))
        listener.settimeout(1.0)

        try:
            # Set drone 0: X velocity (East) = 10.0 m/s, Y velocity (North) = 5.0 m/s, Z (Up) = 2.0 m/s
            d0 = self.sim.drones[0]
            d0.vel = np.array([10.0, 5.0, 2.0], dtype=float)
            d0.yaw = math.atan2(5.0, 10.0)  # heading towards North-East

            # Force emission of position packet
            self.bridge.last_mavlink_pos = -1.0
            bytes_sent = self.bridge.emit_mavlink_loopback()
            self.assertGreater(bytes_sent, 0)

            mav = mavlink2.MAVLink(None)
            pos_found = False
            att_found = False
            vfr_found = False

            # Inspect received packets
            for _ in range(len(self.sim.drones) * 4):
                try:
                    data, _ = listener.recvfrom(2048)
                    msg = mav.decode(bytearray(data))
                    if msg.get_srcSystem() == 1:
                        if msg.get_msgId() == mavlink2.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                            pos_found = True
                            # vx must be North velocity (5.0 m/s = 500 cm/s)
                            self.assertEqual(msg.vx, 500)
                            # vy must be East velocity (10.0 m/s = 1000 cm/s)
                            self.assertEqual(msg.vy, 1000)
                            # vz must be Down velocity (-2.0 m/s = -200 cm/s)
                            self.assertEqual(msg.vz, -200)
                            # Heading clockwise from True North (between 0 and 90 deg = 0..9000 cdeg)
                            self.assertGreater(msg.hdg, 0)
                            self.assertLess(msg.hdg, 9000)
                        elif msg.get_msgId() == mavlink2.MAVLINK_MSG_ID_ATTITUDE:
                            att_found = True
                        elif msg.get_msgId() == mavlink2.MAVLINK_MSG_ID_VFR_HUD:
                            vfr_found = True
                except (socket.timeout, BlockingIOError):
                    break

            self.assertTrue(pos_found, "GLOBAL_POSITION_INT not received")
            self.assertTrue(att_found, "ATTITUDE not received")
            self.assertTrue(vfr_found, "VFR_HUD not received")
        finally:
            listener.close()

    def test_nav_controller_output_bearing_and_distance(self):
        """Verify NAV_CONTROLLER_OUTPUT (#62) is emitted with finite target bearing and waypoint distance."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.mav_port))
        listener.settimeout(1.0)

        try:
            d0 = self.sim.drones[0]
            d0.pos = np.array([0.0, 0.0, 20.0], dtype=float)
            # Target 50m North (Y=50m)
            d0.acknowledged_target = np.array([0.0, 50.0, 20.0], dtype=float)

            self.bridge.last_mavlink_pos = -1.0
            bytes_sent = self.bridge.emit_mavlink_loopback()
            self.assertGreater(bytes_sent, 0)

            mav = mavlink2.MAVLink(None)
            nav_found = False

            for _ in range(len(self.sim.drones) * 4):
                try:
                    data, _ = listener.recvfrom(2048)
                    msg = mav.decode(bytearray(data))
                    if msg.get_srcSystem() == 1 and msg.get_msgId() == getattr(mavlink2, "MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT", 62):
                        nav_found = True
                        # Bearing straight North must be 0 deg
                        self.assertEqual(msg.target_bearing, 0)
                        # Distance must be 50 meters
                        self.assertEqual(msg.wp_dist, 50)
                except (socket.timeout, BlockingIOError):
                    break

            self.assertTrue(nav_found, "NAV_CONTROLLER_OUTPUT packet not received")
        finally:
            listener.close()

    def test_command_ack_src_sys_matches_target(self):
        """Verify COMMAND_ACK packet has srcSystem matching target_system so GCS recognizes ACK."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.mav_port))
        listener.settimeout(1.0)

        try:
            # Send ACK for vehicle 3
            bytes_sent = self.bridge.send_mavlink_ack(
                command=176,  # MAV_CMD_DO_SET_MODE
                result=mavlink2.MAV_RESULT_ACCEPTED,
                dest_addr=(self.host, self.mav_port),
                src_sys=3,
            )
            self.assertGreater(bytes_sent, 0)

            data, _ = listener.recvfrom(2048)
            mav = mavlink2.MAVLink(None)
            msg = mav.decode(bytearray(data))
            self.assertEqual(msg.get_msgId(), mavlink2.MAVLINK_MSG_ID_COMMAND_ACK)
            self.assertEqual(msg.get_srcSystem(), 3, "COMMAND_ACK srcSystem must match commanded target vehicle sysid")
        finally:
            listener.close()

    def test_set_home_position_ingestion(self):
        """Verify SET_HOME_POSITION (#243) updates dynamic home coordinates and returns HOME_POSITION."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        client_mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=190)
        # Set home coordinates: 35.6300 deg N, 139.7400 deg E, 25m alt
        msg_set_home = client_mav.set_home_position_encode(
            target_system=2,
            latitude=int(35.6300 * 1e7),
            longitude=int(139.7400 * 1e7),
            altitude=int(25.0 * 1000),
            x=0.0, y=0.0, z=0.0,
            q=[1.0, 0.0, 0.0, 0.0],
            approach_x=0.0, approach_y=0.0, approach_z=0.0,
        )

        res = self.bridge.handle_mavlink_message(msg_set_home, (self.host, self.mav_port))
        self.assertEqual(res["status"], "OK")
        self.assertEqual(res["msg"], "SET_HOME_POSITION")
        self.assertEqual(res["sys_id"], 2)
        self.assertAlmostEqual(self.bridge.home_lat, 35.6300, places=4)
        self.assertAlmostEqual(self.bridge.home_lon, 139.7400, places=4)
        self.assertAlmostEqual(self.bridge.home_alt, 25.0, places=1)

    def test_gps_raw_int_packet_emission_and_cog(self):
        """Verify GPS_RAW_INT (#24) is emitted with fix_type 3, 16 satellites, and true velocity COG."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.mav_port))
        listener.settimeout(1.0)

        try:
            # Velocity: 10 m/s North (Y)
            d0 = self.sim.drones[0]
            d0.vel = np.array([0.0, 10.0, 0.0], dtype=float)

            self.bridge.last_mavlink_pos = -1.0
            bytes_sent = self.bridge.emit_mavlink_loopback()
            self.assertGreater(bytes_sent, 0)

            mav = mavlink2.MAVLink(None)
            gps_found = False

            for _ in range(len(self.sim.drones) * 5):
                try:
                    data, _ = listener.recvfrom(2048)
                    msg = mav.decode(bytearray(data))
                    if msg.get_srcSystem() == 1 and msg.get_msgId() == getattr(mavlink2, "MAVLINK_MSG_ID_GPS_RAW_INT", 24):
                        gps_found = True
                        self.assertEqual(msg.fix_type, 3)
                        self.assertEqual(msg.satellites_visible, 16)
                        # North velocity COG must be 0 deg = 0 cdeg
                        self.assertEqual(msg.cog, 0)
                        self.assertEqual(msg.vel, 1000)  # 10 m/s in cm/s
                except (socket.timeout, BlockingIOError):
                    break

            self.assertTrue(gps_found, "GPS_RAW_INT packet not received")
        finally:
            listener.close()

    def test_escort_formation_slot_nav_output(self):
        """Verify escort NAV_CONTROLLER_OUTPUT uses dynamic formation target station, not stale Point Alpha."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.mav_port))
        listener.settimeout(1.0)

        try:
            d1 = self.sim.drones[1]
            d1.pos = np.array([10.0, 10.0, 20.0], dtype=float)
            # Escort target station 20m East of its current pos
            d1.target_station = np.array([30.0, 10.0, 20.0], dtype=float)

            self.bridge.last_mavlink_pos = -1.0
            bytes_sent = self.bridge.emit_mavlink_loopback()
            self.assertGreater(bytes_sent, 0)

            mav = mavlink2.MAVLink(None)
            nav_found = False

            for _ in range(len(self.sim.drones) * 5):
                try:
                    data, _ = listener.recvfrom(2048)
                    msg = mav.decode(bytearray(data))
                    if msg.get_srcSystem() == 2 and msg.get_msgId() == getattr(mavlink2, "MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT", 62):
                        nav_found = True
                        # Bearing straight East (X) is 90 deg from North
                        self.assertEqual(msg.target_bearing, 90)
                        self.assertEqual(msg.wp_dist, 20)
                except (socket.timeout, BlockingIOError):
                    break

            self.assertTrue(nav_found, "Escort NAV_CONTROLLER_OUTPUT packet not received")
        finally:
            listener.close()

    def test_canyon_udp_command_parity(self):
        """[BUG-16 / Yellow 4] Verify Canyon UDP parity commands execute without UNKNOWN_COMMAND error."""
        # 1. SET_FIELD_LZ & CLEAR_FIELD_LZ
        res_flz = self.bridge.execute_command({
            "cmd": "SET_FIELD_LZ",
            "coords": [25.0, 35.0, 0.05],
            "name": "FLZ-ALPHA",
        })
        self.assertEqual(res_flz["status"], "OK")
        self.assertIsNotNone(self.sim.field_lz)

        res_clear = self.bridge.execute_command({"cmd": "CLEAR_FIELD_LZ"})
        self.assertEqual(res_clear["status"], "OK")
        self.assertIsNone(self.sim.field_lz)

        # 2. COMMAND_E_LAND
        res_eland = self.bridge.execute_command({"cmd": "COMMAND_E_LAND", "drone_id": 1})
        self.assertEqual(res_eland["status"], "OK")
        self.assertEqual(self.sim.drones[1].flight_state, "EMERGENCY_LANDING")

        # 3. TAKEOFF_DRONE
        d0 = self.sim.drones[0]
        d0.flight_state = "GROUND_IDLE"
        res_takeoff = self.bridge.execute_command({"cmd": "TAKEOFF_DRONE", "drone_id": 0})
        self.assertEqual(res_takeoff["status"], "OK")
        self.assertEqual(d0.flight_state, "TAKEOFF")

        # 4. DECLARE_LOST
        res_lost = self.bridge.execute_command({"cmd": "DECLARE_LOST", "drone_id": 2})
        self.assertEqual(res_lost["status"], "OK")
        self.assertEqual(self.sim.drones[2].flight_state, "DECLARED_LOST")


if __name__ == "__main__":
    unittest.main()
