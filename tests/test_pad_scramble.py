"""
Unit tests for [TKT-41] Cold-Start Pad Scramble & Sequenced Climb-Out Corridor.
Validates:
- Drones initialize cold on numbered ground launch pads (Z = 0.05m, 0 RPM, GROUND_IDLE).
- Umbilical spool retracted at base anchor when leader is grounded.
- Sequenced scramble: Flagship #0 departs at T=0, followers staggered every 1.2s.
- Station-keeping along departure corridors (|pos_xy - pad_xy| <= 0.5m).
- Reinforcement staging on pad with tactical pre-briefing ("Gang in contact" vs "Gang lost").
- Subsequent operator "launch ok" (TAKEOFF_DRONE) transitioning to corridor climb.
"""

import unittest
import numpy as np

from SIM.backend.sitl_engine import (
    SITLWorldEngine,
    SwarmSimulator,
    TIER_LEADER,
    TIER_ESCORT,
    DT,
)
from SIM.backend.mavlink_bridge import MavlinkBridge
from OBC.backend.drone_firmware import Drone


class TestColdStartPadScramble(unittest.TestCase):
    def setUp(self):
        # Clean deterministic simulator with 5 drones
        self.sim = SwarmSimulator(count=5, seed=42, cold_start=True)

    def test_cold_start_pad_initialization(self):
        """Drones must initialize cold on numbered ground pads with 0 RPM."""
        self.assertEqual(len(self.sim.drones), 5)
        self.assertTrue(self.sim.cold_start)
        self.assertFalse(self.sim.scramble_active)

        for i, drone in enumerate(self.sim.drones):
            self.assertEqual(drone.flight_state, "GROUND_IDLE")
            self.assertEqual(drone.status, "GROUND_IDLE")
            self.assertEqual(drone.rotor_rpm, 0.0)
            self.assertAlmostEqual(drone.pos[2], 0.05, places=2)
            self.assertIsNotNone(drone.landing_pad)
            self.assertEqual(drone.landing_pad_idx, i % 8)

        # Leader on Pad 0
        leader = self.sim.leader
        self.assertIsNotNone(leader)
        self.assertEqual(leader.id, 0)
        self.assertEqual(leader.landing_pad_idx, 0)

        # Umbilical tether must be retracted (not deployed/visible) while leader is grounded
        snap = self.sim.get_state_snapshot()
        self.assertFalse(snap["tether"]["deployed"])
        self.assertFalse(snap["tether"]["visible"])

    def test_backward_compatibility_default_cruise(self):
        """Default cold_start=False must initialize drones airborne in cruise."""
        sim_default = SwarmSimulator(count=5, seed=42)
        self.assertFalse(sim_default.cold_start)
        for drone in sim_default.drones:
            self.assertEqual(drone.flight_state, "CRUISE")
            self.assertGreaterEqual(drone.pos[2], 19.0)
            self.assertEqual(drone.rotor_rpm, 5200.0)

    def test_sequenced_pad_scramble_timing(self):
        """Flagship launches at T=0; followers launch staggered every 1.2s."""
        res = self.sim.command_scramble_swarm(stagger_s=1.2)
        self.assertEqual(res["status"], "SCRAMBLE_INITIATED")
        self.assertEqual(res["stagger_s"], 1.2)
        self.assertTrue(self.sim.scramble_active)

        # Flagship #0 takes off immediately at T=0
        leader = self.sim.leader
        self.assertEqual(leader.flight_state, "TAKEOFF")
        self.assertEqual(leader.rotor_rpm, 5200.0)

        # Followers are still grounded at T=0
        for follower in self.sim.drones[1:]:
            self.assertEqual(follower.flight_state, "GROUND_IDLE")
            self.assertEqual(follower.rotor_rpm, 0.0)

        # Step 0.6s: Drone 0 is climbing, Drone 1 not yet launched (stagger=1.2s)
        for _ in range(int(0.6 / DT)):
            self.sim.step(DT)
        self.assertEqual(self.sim.drones[0].flight_state, "TAKEOFF")
        self.assertGreater(self.sim.drones[0].pos[2], 0.5)
        self.assertEqual(self.sim.drones[1].flight_state, "GROUND_IDLE")

        # Step to T=1.3s: Drone 1 launches, Drone 2 still grounded
        for _ in range(int(0.7 / DT)):
            self.sim.step(DT)
        self.assertEqual(self.sim.drones[1].flight_state, "TAKEOFF")
        self.assertEqual(self.sim.drones[1].rotor_rpm, 5200.0)
        self.assertEqual(self.sim.drones[2].flight_state, "GROUND_IDLE")

        # Step to T=2.5s: Drone 2 launches
        for _ in range(int(1.2 / DT)):
            self.sim.step(DT)
        self.assertEqual(self.sim.drones[2].flight_state, "TAKEOFF")

    def test_climb_out_corridor_station_keeping(self):
        """Drones climbing in TAKEOFF must hold horizontal station to their pad coordinates."""
        self.sim.command_scramble_swarm(stagger_s=1.2)
        leader = self.sim.leader
        pad_xy = np.copy(leader.landing_pad[:2])

        # Step 2 seconds into climb
        for _ in range(int(2.0 / DT)):
            self.sim.step(DT)

        self.assertEqual(leader.flight_state, "TAKEOFF")
        self.assertGreater(leader.pos[2], 5.0)
        # Horizontal displacement from pad column must remain tight
        disp_xy = np.linalg.norm(leader.pos[:2] - pad_xy)
        self.assertLess(disp_xy, 0.5)

    def test_reinforcement_staging_with_pre_briefing(self):
        """Staging a reinforcement drone places it on pad in GROUND_IDLE with tactical briefing."""
        # Swarm active and in comms
        sim_airborne = SwarmSimulator(count=3, seed=123, cold_start=False)
        reinforce = sim_airborne.spawn_reinforcement(auto_launch=False)

        self.assertIsNotNone(reinforce)
        self.assertEqual(reinforce.flight_state, "GROUND_IDLE")
        self.assertEqual(reinforce.rotor_rpm, 0.0)
        self.assertAlmostEqual(reinforce.pos[2], 0.05, places=2)
        self.assertIn("FLEET_ACTIVE: Gang in contact", reinforce.briefing)

        # Operator gives "launch ok" via command_takeoff_drone
        ok = sim_airborne.command_takeoff_drone(reinforce.id)
        self.assertTrue(ok)
        self.assertEqual(reinforce.flight_state, "TAKEOFF")
        self.assertEqual(reinforce.rotor_rpm, 5200.0)

    def test_reinforcement_briefing_lost_link(self):
        """Reinforcement staged when swarm is out of comms receives FLEET_LOST briefing."""
        sim_airborne = SwarmSimulator(count=2, seed=123, cold_start=False)
        # Move leader beyond comms range (e.g. 150m)
        sim_airborne.leader.pos = np.array([160.0, 160.0, 25.0])
        sim_airborne.step(DT)

        reinforce = sim_airborne.spawn_reinforcement(auto_launch=False)
        self.assertEqual(reinforce.flight_state, "GROUND_IDLE")
        self.assertIn("FLEET_LOST: Gang lost", reinforce.briefing)

    def test_cold_reset_command(self):
        """command_cold_start_swarm must ground all drones to numbered pads."""
        sim_airborne = SwarmSimulator(count=4, seed=99, cold_start=False)
        self.assertFalse(sim_airborne.cold_start)

        res = sim_airborne.command_cold_start_swarm()
        self.assertEqual(res["status"], "COLD_START_ACTIVE")
        self.assertTrue(sim_airborne.cold_start)
        self.assertEqual(len(sim_airborne.drones), 4)
        for d in sim_airborne.drones:
            self.assertEqual(d.flight_state, "GROUND_IDLE")
            self.assertEqual(d.rotor_rpm, 0.0)
            self.assertAlmostEqual(d.pos[2], 0.05, places=2)

    def test_mavlink_bridge_scramble_and_spawn_commands(self):
        """MavlinkBridge must execute SCRAMBLE_SWARM and SPAWN_REINFORCEMENT."""
        bridge = MavlinkBridge(sim=self.sim, tx_port=0, rx_port=0, mavlink_port=0)
        try:
            # Test SPAWN_REINFORCEMENT command
            spawn_res = bridge.execute_command({"cmd": "SPAWN_REINFORCEMENT"})
            self.assertEqual(spawn_res["status"], "OK")
            self.assertIn("briefing", spawn_res)
            self.assertEqual(len(self.sim.drones), 6)
            staged_drone = self.sim.drones[-1]
            self.assertEqual(staged_drone.flight_state, "GROUND_IDLE")

            # Test SCRAMBLE_SWARM command
            scramble_res = bridge.execute_command({"cmd": "SCRAMBLE_SWARM", "stagger": 1.2})
            self.assertEqual(scramble_res["status"], "SCRAMBLE_INITIATED")
            self.assertTrue(self.sim.scramble_active)
            self.assertEqual(self.sim.leader.flight_state, "TAKEOFF")
        finally:
            bridge.close()

    def test_operator_incompetence_unretrieved_pad_collision(self):
        """[FIX-09] Operator clears ATC pad registry prematurely without physical WE retrieval:
        descending drone collides with unretrieved airframe sitting on pad at touchdown."""
        sim = SwarmSimulator(count=2, seed=42, cold_start=False)
        # Place Drone 0 landed on Pad 0
        pad0 = sim.get_pad_position(0)
        d0 = sim.drones[0]
        d0.pos = np.copy(pad0)
        d0.flight_state = "LANDED"
        d0.status = "LANDED"
        d0.touchdown_confirmed = True
        d0.rotor_rpm = 0.0

        # Operator prematurely clears pad registry without ground crew physical retrieval
        sim.clear_pads()
        self.assertNotIn(0, sim.landing_pads)

        # Command Drone 1 to land: ATC allocates Pad 0 because registry was wiped
        d1 = sim.drones[1]
        sim.command_land_drone(d1.id)
        self.assertEqual(d1.landing_pad_idx, 0)
        self.assertEqual(d1.flight_state, "DESCENDING")

        # Position Drone 1 right above Pad 0 at touchdown threshold (Z = 0.50m)
        d1.pos = np.copy(pad0)
        d1.pos[2] = 0.50
        d1.vel = np.array([0.0, 0.0, -0.5])

        # Step physics: Touchdown check triggers Zone 4 clash with unretrieved Drone 0!
        sim.step(DT)
        self.assertTrue(d1.crashed)
        self.assertTrue(d0.crashed)
        self.assertEqual(d1.status, "CRASHED")
        self.assertEqual(d0.status, "CRASHED")

    def test_ground_crew_physical_retrieval_prevents_collision(self):
        """[FIX-09] Ground crew physically retrieves airframe from pad before next wave lands:
        incoming drone touches down safely with TOUCHDOWN_CONFIRMED."""
        sim = SwarmSimulator(count=2, seed=42, cold_start=False)
        pad0 = sim.get_pad_position(0)
        d0 = sim.drones[0]
        d0.pos = np.copy(pad0)
        d0.flight_state = "LANDED"
        d0.status = "LANDED"
        d0.touchdown_confirmed = True
        d0.rotor_rpm = 0.0

        # Ground crew retrieves Drone 0 from Pad 0 into hangar
        retrieved_id = sim.retrieve_drone_from_pad(0)
        self.assertEqual(retrieved_id, d0.id)
        self.assertTrue(d0.retrieved)
        self.assertEqual(d0.flight_state, "RETRIEVED")
        self.assertGreater(abs(d0.pos[0] - pad0[0]), 50.0)

        # Clear ATC registry and land Drone 1 on Pad 0
        sim.clear_pads()
        d1 = sim.drones[1]
        sim.command_land_drone(d1.id)
        self.assertEqual(d1.landing_pad_idx, 0)

        # Position Drone 1 at touchdown threshold
        d1.pos = np.copy(pad0)
        d1.pos[2] = 0.50
        d1.vel = np.array([0.0, 0.0, -0.5])

        # Step physics: Pad is physically clear, touchdown confirmed!
        sim.step(DT)
        self.assertFalse(d1.crashed)
        self.assertEqual(d1.flight_state, "LANDED")
        self.assertTrue(d1.touchdown_confirmed)


if __name__ == "__main__":
    unittest.main()
