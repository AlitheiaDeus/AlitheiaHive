"""
SwarmTech3_UpRising: Unit Tests for Tactical GNSS Denial & Dead-Reckoning RTB (test_gnss_denial.py)
Validates [TKT-52] Electronic Warfare Defense:
1. Single & full-swarm GNSS denial injection.
2. MAVLink GPS glitch injection (fix=0, sats=0) & STATUSTEXT alert.
3. Vertical altitude staircase separation (Delta Z >= 2.0m) preventing Zone 4 prop clashes.
4. Time-scaled drift error circle expansion & lateral fan-out.
5. Reciprocal heading dead-reckoning & strict 'Cheater No Cheating' invariant.
6. RF Howl beacon gradient ascent and near-field blind soft-settle grounding.
7. C2 Web command routing.
"""

from __future__ import annotations
import math
import socket
import unittest
import numpy as np

from SIM.backend.sitl_engine import SITLWorldEngine, ZONE4_CLASH
from SIM.backend.mavlink_bridge import MavlinkBridge, HAVE_PYMAVLINK
from GCS.backend.web_server import SwarmWebServer

if HAVE_PYMAVLINK:
    from pymavlink.dialects.v20 import common as mavlink2


class TestTacticalGNSSDenial(unittest.TestCase):
    """Test suite for TKT-52: Tactical GNSS Denial & Dead-Reckoning RTB Failsafe."""

    def setUp(self):
        self.sim = SITLWorldEngine(drone_count=8, enable_packet_loss=False, seed=42)
        # Advance slightly to settle initial formation
        for _ in range(30):
            self.sim.step()

    def test_single_and_full_swarm_toggle_gnss_denial(self):
        """Test targeting individual drone vs entire fleet with GNSS denial."""
        # 1. Target single drone #2
        state = self.sim.toggle_gnss_denial(drone_id=2)
        self.assertTrue(state)
        d2 = self.sim.get_drone_by_id(2)
        self.assertIsNotNone(d2)
        self.assertTrue(d2.gnss_denied)
        self.assertEqual(d2.status, "GNSS_DENIED")
        self.assertAlmostEqual(d2.drift_radius, 0.5)

        # Other drones remain nominal
        d0 = self.sim.get_drone_by_id(0)
        d1 = self.sim.get_drone_by_id(1)
        self.assertFalse(d0.gnss_denied)
        self.assertFalse(d1.gnss_denied)

        # Toggle single drone #2 off
        state_off = self.sim.toggle_gnss_denial(drone_id=2)
        self.assertFalse(state_off)
        self.assertFalse(d2.gnss_denied)

        # 2. Global denial across entire fleet
        global_state = self.sim.toggle_gnss_denial()
        self.assertTrue(global_state)
        for d in self.sim.drones:
            self.assertTrue(d.gnss_denied)
            self.assertFalse(d.mission_active)

        # Toggle global denial off
        global_off = self.sim.toggle_gnss_denial()
        self.assertFalse(global_off)
        for d in self.sim.drones:
            self.assertFalse(d.gnss_denied)

    def test_mavlink_gps_glitch_injection_and_statustext(self):
        """MAVLink bridge encodes fix_type=0, satellites=0, eph=9999 when GNSS is denied."""
        if not HAVE_PYMAVLINK:
            self.skipTest("pymavlink not installed")

        test_port = 24552
        bridge = MavlinkBridge(sim=self.sim, host="127.0.0.1", mavlink_port=test_port)

        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", test_port))
        listener.settimeout(0.5)

        def drain_socket():
            listener.setblocking(False)
            while True:
                try:
                    listener.recvfrom(4096)
                except (BlockingIOError, socket.error):
                    break
            listener.settimeout(0.5)

        try:
            d1 = self.sim.get_drone_by_id(1)
            self.assertIsNotNone(d1)
            d1.gnss_denied = False

            # 1. Nominal GPS emission
            bridge.last_mavlink_pos = -1.0
            bridge.emit_mavlink_loopback()

            client_mav = mavlink2.MAVLink(None)
            gps_nom = None
            while True:
                try:
                    data, _ = listener.recvfrom(2048)
                    msg = client_mav.decode(bytearray(data))
                    if msg.get_srcSystem() == 2 and msg.get_msgId() == getattr(mavlink2, "MAVLINK_MSG_ID_GPS_RAW_INT", 24):
                        gps_nom = msg
                except (socket.timeout, BlockingIOError):
                    break

            self.assertIsNotNone(gps_nom, "Nominal GPS_RAW_INT packet not received for drone 1 (sysid 2)")
            self.assertEqual(gps_nom.fix_type, 3)
            self.assertEqual(gps_nom.satellites_visible, 16)
            self.assertEqual(gps_nom.eph, 100)

            # 2. Inject GNSS denial on Drone 1
            drain_socket()
            self.sim.toggle_gnss_denial(drone_id=1, denied=True)
            self.assertTrue(d1.gnss_denied)

            bridge.last_mavlink_pos = -1.0
            bridge.emit_mavlink_loopback()

            gps_denied = None
            while True:
                try:
                    data, _ = listener.recvfrom(2048)
                    msg = client_mav.decode(bytearray(data))
                    if msg.get_srcSystem() == 2 and msg.get_msgId() == getattr(mavlink2, "MAVLINK_MSG_ID_GPS_RAW_INT", 24):
                        gps_denied = msg
                except (socket.timeout, BlockingIOError):
                    break

            self.assertIsNotNone(gps_denied, "Denied GPS_RAW_INT packet not received for drone 1 (sysid 2)")
            self.assertEqual(gps_denied.fix_type, 0, "Denied drone must have fix_type=0 (No GPS Fix)")
            self.assertEqual(gps_denied.satellites_visible, 0, "Denied drone must have 0 satellites visible")
            self.assertEqual(gps_denied.eph, 9999, "Denied drone must report max eph error 9999")
            self.assertEqual(gps_denied.epv, 9999, "Denied drone must report max epv error 9999")
            self.assertEqual(gps_denied.lat, 0, "Denied drone latitude must be masked to 0")
            self.assertEqual(gps_denied.lon, 0, "Denied drone longitude must be masked to 0")

            # 3. Verify unaffected Drone 0 retains nominal 3D fix
            drain_socket()
            bridge.last_mavlink_pos = -1.0
            bridge.emit_mavlink_loopback()

            gps_drone0 = None
            while True:
                try:
                    data, _ = listener.recvfrom(2048)
                    msg = client_mav.decode(bytearray(data))
                    if msg.get_srcSystem() == 1 and msg.get_msgId() == getattr(mavlink2, "MAVLINK_MSG_ID_GPS_RAW_INT", 24):
                        gps_drone0 = msg
                except (socket.timeout, BlockingIOError):
                    break

            self.assertIsNotNone(gps_drone0, "GPS_RAW_INT packet not received for drone 0 (sysid 1)")
            self.assertEqual(gps_drone0.fix_type, 3, "Unaffected drone must retain fix_type=3 (3D Fix)")
            self.assertEqual(gps_drone0.satellites_visible, 16)
        finally:
            bridge.close()
            listener.close()

    def test_vertical_altitude_staircase_prevents_prop_clash(self):
        """Simultaneous full-swarm GNSS denial steps airframes into vertical shelves.
        Even with severe horizontal cloud drift, vertical separation guarantees d > 1.5m (Zone 4)."""
        # Place all 8 drones at identical (X, Y) cruise position but allow vertical shelves to establish
        self.sim.toggle_gnss_denial(denied=True)

        # Step simulation for 3.0s (180 steps) to allow vertical shelves to stabilize
        for _ in range(180):
            self.sim.step()

        # Verify vertical staircase shelves: Z_k = max(6.0, 25.0 - k * 2.0)
        for d in self.sim.drones:
            expected_z = max(6.0, 25.0 - (d.id * 2.0))
            self.assertAlmostEqual(d.pos[2], expected_z, delta=2.5)

        # Verify pairwise 3D clearance: no two drones must ever clash (d <= 1.5m)
        n = len(self.sim.drones)
        for i in range(n):
            di = self.sim.drones[i]
            self.assertFalse(di.crashed, f"Drone #{di.id} crashed unexpectedly!")
            for j in range(i + 1, n):
                dj = self.sim.drones[j]
                dist = float(np.linalg.norm(di.pos - dj.pos))
                self.assertGreater(
                    dist,
                    ZONE4_CLASH,
                    f"Zone 4 Clash detected between Drone #{di.id} and #{dj.id} (dist={dist:.2f}m <= {ZONE4_CLASH}m)",
                )

    def test_time_scaled_drift_uncertainty_and_lateral_fan_out(self):
        """Drift radius expands: R_drift(dt) = min(15.0m, 0.5m + 0.4*dt).
        Lateral fan-out provides Port vs Starboard separation bias."""
        d0 = self.sim.get_drone_by_id(0)
        d1 = self.sim.get_drone_by_id(1)
        self.sim.toggle_gnss_denial(denied=True)

        # Check initial drift radius
        self.assertAlmostEqual(d0.drift_radius, 0.5)

        # Advance 5.0 seconds (300 ticks)
        for _ in range(300):
            self.sim.step(dt=1.0 / 60.0)

        # Elapsed dt ~= 5.0s -> R_drift = min(15.0, 0.5 + 0.4 * 5.0) = 2.5m
        self.assertAlmostEqual(d0.drift_radius, 2.5, delta=0.2)

        # Check lateral fan-out: even ID #0 vs odd ID #1
        # Guidance vectors should have opposing lateral biases
        acc0 = self.sim.brain.compute_gnss_denied_guidance(d0, dt=1.0 / 60.0)
        acc1 = self.sim.brain.compute_gnss_denied_guidance(d1, dt=1.0 / 60.0)
        # Heading is reciprocal; check dot product of lateral vectors
        psi0 = d0.reciprocal_heading
        u_perp0 = np.array([-math.sin(psi0), math.cos(psi0)])
        psi1 = d1.reciprocal_heading
        u_perp1 = np.array([-math.sin(psi1), math.cos(psi1)])

        # Lateral bias direction should be opposing (Starboard vs Port)
        lat_proj0 = np.dot(acc0[:2], u_perp0)
        lat_proj1 = np.dot(acc1[:2], u_perp1)
        self.assertGreater(lat_proj0, 0.0)
        self.assertLess(lat_proj1, 0.0)

    def test_cheater_no_cheating_invariant(self):
        """Strict Invariant: Cheater No Cheating.
        OBC guidance logic MUST NOT peek into drone.pos[:2] or sim.target_pos during GNSS denial."""
        d = self.sim.get_drone_by_id(3)
        self.sim.toggle_gnss_denial(drone_id=3, denied=True)

        # Tamper with actual X, Y position to test if guidance algorithm is immune to coordinate shifts
        d.pos[0] = 999.0
        d.pos[1] = -888.0
        d.rssi = -80.0
        d.rssi_prev = -80.0
        d.reciprocal_heading = math.pi  # Westbound heading
        d.vel = np.array([-4.0, 0.0, 0.0], dtype=float)

        acc = self.sim.brain.compute_gnss_denied_guidance(d, dt=1.0 / 60.0)

        # Horizontal acceleration must ONLY depend on reciprocal heading, not the coordinate (999, -888)
        # Traveling westbound (-X) along math.pi heading: target velocity is [-8.0, lateral]
        # Since vel is [-4.0, 0.0], guidance must accelerate toward -X (acc[0] < 0)
        self.assertLess(acc[0], 0.0)
        # Should not attempt to steer toward GCS base coordinates directly
        self.assertNotAlmostEqual(acc[0], -999.0, delta=10.0)

    def test_rf_beacon_gradient_and_emergency_blind_settle(self):
        """Near-field beacon threshold (RSSI >= -45.0 dBm / d <= 15m) triggers soft blind settle."""
        d = self.sim.get_drone_by_id(4)
        # Position drone 12m from GCS base anchor (inside near-field threshold)
        d.pos = np.array([8.0, 6.0, 15.0], dtype=float)
        d.vel = np.array([-4.0, -3.0, 0.0], dtype=float)
        self.sim.toggle_gnss_denial(drone_id=4, denied=True)

        # Update comms to compute Howl beacon RSSI
        self.sim._update_radio_comms()
        # Distance = hypot(8, 6) = 10m.
        # Howl beacon RSSI = -20 - 20*log10(10) = -40 dBm >= -45.0 dBm
        self.assertGreaterEqual(d.rssi, -45.0)

        # Advance step: guidance must enter soft settle
        self.sim.step()
        self.assertTrue(d.gnss_settled)

        # In blind settle: horizontal velocity is damped, vertical velocity is -0.8 m/s
        for _ in range(60):
            self.sim.step()

        # Horizontal speed damped close to 0
        speed_xy = math.hypot(d.vel[0], d.vel[1])
        self.assertLess(speed_xy, 1.5)
        # Vertical descent active
        self.assertLess(d.vel[2], -0.2)

    def test_emergency_settle_timeout_grounds_drone(self):
        """25.0s timeout forces blind soft-settle descent and smooth touchdown (Z <= 0.52m -> LANDED)."""
        d = self.sim.get_drone_by_id(5)
        d.pos = np.array([40.0, 40.0, 2.0], dtype=float)  # Low altitude
        self.sim.toggle_gnss_denial(drone_id=5, denied=True)
        d.gnss_denied_timer = 25.1  # Force timeout

        # Step until touchdown
        for _ in range(150):
            self.sim.step()

        self.assertTrue(d.gnss_settled)
        self.assertEqual(d.flight_state, "LANDED")
        self.assertAlmostEqual(d.pos[2], 0.5, delta=0.05)
        self.assertEqual(d.rotor_rpm, 0.0)

    def test_c2_web_server_command_dispatch(self):
        """C2 Web Server handles TOGGLE_GNSS_DENIAL command."""
        server = SwarmWebServer(sim=self.sim, port=9999)
        res = server.execute_command({"cmd": "TOGGLE_GNSS_DENIAL", "id": 1})
        self.assertEqual(res["status"], "OK")
        self.assertTrue(res["gnss_denied"])

        d1 = self.sim.get_drone_by_id(1)
        self.assertIsNotNone(d1)
        self.assertTrue(d1.gnss_denied)

        # Toggle off
        res_off = server.execute_command({"cmd": "JAM_GNSS", "id": 1})
        self.assertEqual(res_off["status"], "OK")
        self.assertFalse(res_off["gnss_denied"])
        self.assertFalse(d1.gnss_denied)


if __name__ == "__main__":
    unittest.main()
