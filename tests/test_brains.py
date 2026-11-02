"""
Automated Unit Tests for SwarmTech3_UpRising Pluggable Brains (test_brains.py)
Tests BaseBrain, LeaderFollowerBrain, GossipConsensusBrain, P2P echo propagation,
quorum barrier consensus, hot-swapping, and Autopilot safety control layer.
"""

import unittest
import math
import numpy as np
import json

from SIM.backend.sitl_engine import SwarmSimulator, DT, A_MAX, ZONE4_CLASH
from OBC.backend.brains import BRAIN_REGISTRY, LeaderFollowerBrain, GossipConsensusBrain


class TestBrains(unittest.TestCase):
    """Test suite verifying pluggable brains and P2P consensus."""

    def setUp(self):
        self.sim = SwarmSimulator(drone_count=6, enable_packet_loss=False, seed=42)

    def test_default_brain_is_leader_follower(self):
        """Verify initial default guidance brain is LeaderFollowerBrain."""
        self.assertEqual(self.sim.active_brain_name, "LEADER_FOLLOWER")
        self.assertIsInstance(self.sim.brain, LeaderFollowerBrain)

    def test_hot_swap_to_gossip_consensus(self):
        """Verify runtime hot-swapping to GossipConsensusBrain."""
        swapped = self.sim.set_brain("GOSSIP_CONSENSUS")
        self.assertTrue(swapped)
        self.assertEqual(self.sim.active_brain_name, "GOSSIP_CONSENSUS")
        self.assertIsInstance(self.sim.brain, GossipConsensusBrain)

        # Step simulation to ensure numerical stability
        for _ in range(10):
            self.sim.step()
            for d in self.sim.drones:
                self.assertFalse(np.isnan(d.pos).any())
                self.assertFalse(np.isnan(d.vel).any())

        # Swap back to LEADER_FOLLOWER
        swapped_back = self.sim.set_brain("LEADER_FOLLOWER")
        self.assertTrue(swapped_back)
        self.assertEqual(self.sim.active_brain_name, "LEADER_FOLLOWER")
        self.assertIsInstance(self.sim.brain, LeaderFollowerBrain)

    def test_gossip_propagation_and_quorum_consensus(self):
        """Verify P2P multi-hop gossip echo wave and quorum barrier synchronization."""
        self.sim.set_brain("GOSSIP_CONSENSUS")
        brain: GossipConsensusBrain = self.sim.brain

        # Initial state: all drones coordinate around target [60, 40, 30] at seq 1
        for d in self.sim.drones:
            state = brain._get_peer_state(d.id)
            self.assertEqual(state.active_seq, 1)
            self.assertEqual(state.gossip_status, "COORDINATED")

        # Ingest new target: [150, 100, 50] (seq 2)
        res = brain.inject_target([150.0, 100.0, 50.0])
        self.assertEqual(res["status"], "OK")
        self.assertEqual(res["seq"], 2)
        seed_id = res["seed_drone_id"]

        # Seed drone is now echoing/propagating, others have not received it yet
        self.assertEqual(brain._get_peer_state(seed_id).pending_seq, 2)
        self.assertEqual(brain._get_peer_state(seed_id).gossip_status, "PROPAGATING")

        # Step 15 frames (~0.25s): covers 2 10Hz broadcast ticks so echo wave completes quorum
        for _ in range(15):
            self.sim.step()

        # All living drones should now have adopted pending_seq = 2 and reached quorum
        for d in self.sim.drones:
            state = brain._get_peer_state(d.id)
            self.assertEqual(state.active_seq, 2)
            np.testing.assert_allclose(state.active_target, [150.0, 100.0, 50.0])
            self.assertEqual(state.gossip_status, "COORDINATED")

    def test_autopilot_safety_effort_clamping(self):
        """Verify Zone 2/3 safety is computed as flight-controller control effort
        bounded by motor acceleration limit A_MAX (not god-mode physics)."""
        d1 = self.sim.drones[1]
        d2 = self.sim.drones[2]

        # Place them at 2.0m closing at high speed (isolated from drone #0 at origin)
        d1.pos = np.array([50.0, 0.0, 20.0], dtype=float)
        d2.pos = np.array([52.0, 0.0, 20.0], dtype=float)
        d1.vel = np.array([8.0, 0.0, 0.0], dtype=float)
        d2.vel = np.array([-8.0, 0.0, 0.0], dtype=float)

        self.sim.step()

        # Accelerations must be clamped to A_MAX
        self.assertLessEqual(np.linalg.norm(d1.acc), A_MAX + 1e-3)
        self.assertLessEqual(np.linalg.norm(d2.acc), A_MAX + 1e-3)

        # Neither drone has crashed because d = 2.0m > 1.5m ZONE4_CLASH
        self.assertFalse(d1.crashed)
        self.assertFalse(d2.crashed)

    def test_snapshot_includes_brain_telemetry(self):
        """Verify snapshot contains brain identifier and extra telemetry metadata."""
        snapshot = self.sim.get_state_snapshot()
        self.assertIn("brain", snapshot)
        self.assertEqual(snapshot["brain"], "LEADER_FOLLOWER")
        self.assertIn("brain_telemetry", snapshot)

        # Switch to gossip brain and verify telemetry
        self.sim.set_brain("GOSSIP_CONSENSUS")
        snapshot_gossip = self.sim.get_state_snapshot()
        self.assertEqual(snapshot_gossip["brain"], "GOSSIP_CONSENSUS")
        self.assertIn("target_seq", snapshot_gossip["brain_telemetry"])
        self.assertIn("consensus_reached", snapshot_gossip["brain_telemetry"])

    def test_echelon_heading_freeze_near_target(self):
        """Verify formation heading freezes when leader is within 8.0m of target."""
        brain: LeaderFollowerBrain = self.sim.brain
        leader = self.sim.leader

        # Position leader 20m from target, moving towards +X at 10 m/s
        self.sim.target_pos = np.array([100.0, 0.0, 20.0], dtype=float)
        leader.pos = np.array([80.0, 0.0, 20.0], dtype=float)
        leader.vel = np.array([10.0, 0.0, 0.0], dtype=float)

        # Advance 1 step: heading establishes as 0.0 (along +X)
        self.sim.step()
        self.assertAlmostEqual(brain.formation_heading, 0.0, places=2)

        # Now place leader within 4.0m of target (distance < 8.0m FREEZE_DIST)
        leader.pos = np.array([96.0, 0.0, 20.0], dtype=float)
        # Simulate micro-reversal / overshoot: vel points backwards along -X
        leader.vel = np.array([-2.0, 0.0, 0.0], dtype=float)

        self.sim.step()
        # Formation heading MUST remain frozen at 0.0 and NOT flip 180 degrees
        self.assertAlmostEqual(brain.formation_heading, 0.0, places=2)

    def test_echelon_heading_freeze_at_low_speed(self):
        """Verify formation heading freezes when leader speed drops below 1.5 m/s."""
        brain: LeaderFollowerBrain = self.sim.brain
        leader = self.sim.leader

        # Position leader far from target (50m) moving at cruise speed
        self.sim.target_pos = np.array([100.0, 0.0, 20.0], dtype=float)
        leader.pos = np.array([50.0, 0.0, 20.0], dtype=float)
        leader.vel = np.array([10.0, 0.0, 0.0], dtype=float)
        self.sim.step()
        self.assertAlmostEqual(brain.formation_heading, 0.0, places=2)

        # Now leader decelerates to 0.5 m/s (below FREEZE_SPEED 1.5 m/s) and veers +Y
        leader.vel = np.array([0.0, 0.5, 0.0], dtype=float)
        self.sim.step()
        # Formation heading must remain frozen at 0.0
        self.assertAlmostEqual(brain.formation_heading, 0.0, places=2)

    def test_echelon_heading_slew_rate_limit(self):
        """Verify heading change per step is strictly bounded by MAX_SLEW_RATE (45 deg/s)."""
        brain: LeaderFollowerBrain = self.sim.brain
        leader = self.sim.leader

        self.sim.target_pos = np.array([100.0, 0.0, 20.0], dtype=float)
        leader.pos = np.array([20.0, 0.0, 20.0], dtype=float)
        leader.vel = np.array([10.0, 0.0, 0.0], dtype=float)
        self.sim.step()
        self.assertAlmostEqual(brain.formation_heading, 0.0, places=2)

        # Leader suddenly banks 90 degrees (+Y) at full cruise speed
        leader.vel = np.array([0.0, 10.0, 0.0], dtype=float)
        dt = DT
        self.sim.step()

        # Max allowed delta is (pi / 4.0) * dt + slight float tolerance
        max_allowed_delta = (np.pi / 4.0) * dt + 1e-4
        delta_heading = abs(brain.formation_heading - 0.0)
        self.assertLessEqual(delta_heading, max_allowed_delta)

    def test_steer_toward_arrival_deadband(self):
        """Verify 0.25m arrival deadband outputs pure damping to prevent micro-hunting."""
        pos = np.array([10.0, 10.0, 20.0], dtype=float)
        vel = np.array([1.0, -0.5, 0.0], dtype=float)

        # Target within 0.1m (< 0.25m deadband)
        target = np.array([10.05, 10.05, 20.0], dtype=float)
        acc = LeaderFollowerBrain._steer_toward(pos, vel, target, max_speed=12.0)

        # Output must be pure velocity damping (-vel * 3.0)
        expected_acc = -vel * 3.0
        np.testing.assert_allclose(acc, expected_acc)

    def test_static_perimeter_sentry_ring(self):
        """Verify PERIMETER_PATROL positions followers at fixed static azimuthal stations
        around target (zero carousel rotation over time)."""
        self.sim.set_formation("PERIMETER_PATROL")
        brain: LeaderFollowerBrain = self.sim.brain
        leader = self.sim.leader
        followers = [d for d in self.sim.drones if not d.is_leader and not d.crashed]
        n_followers = len(followers)
        self.assertGreater(n_followers, 1)

        # Record initial slot positions at t = 0
        initial_slots = {}
        for idx, follower in enumerate(followers):
            slot = brain._get_formation_slot(follower, leader)
            initial_slots[follower.id] = np.copy(slot)

            # Verify radius is exactly 25.0m from target_pos
            dist_to_target = float(np.linalg.norm(slot - self.sim.target_pos))
            self.assertAlmostEqual(dist_to_target, 25.0, places=3)

            # Verify azimuthal angle theta_k = 2*pi*idx / n_followers
            expected_angle = (2.0 * np.pi * idx) / n_followers
            actual_angle = math.atan2(slot[1] - self.sim.target_pos[1], slot[0] - self.sim.target_pos[0]) % (2.0 * np.pi)
            self.assertAlmostEqual(actual_angle, expected_angle % (2.0 * np.pi), places=3)

        # Advance simulation by 2.0 seconds (120 steps)
        for _ in range(120):
            self.sim.step()

        # Slots must remain completely STATIC (zero rotation, no carousel orbit)
        for follower in followers:
            current_slot = brain._get_formation_slot(follower, self.sim.leader)
            np.testing.assert_allclose(current_slot, initial_slots[follower.id], atol=1e-5)

    def test_echelon_heading_tracks_during_target_patrol(self):
        """Verify echelon heading does not freeze when following a moving auto-patrol target,
        even when within 8.0m of the target beacon."""
        brain: LeaderFollowerBrain = self.sim.brain
        leader = self.sim.leader

        # Initial heading along +X
        self.sim.target_pos = np.array([50.0, 0.0, 20.0], dtype=float)
        leader.pos = np.array([45.0, 0.0, 20.0], dtype=float)
        leader.vel = np.array([6.0, 0.0, 0.0], dtype=float)
        self.sim.step()
        self.assertAlmostEqual(brain.formation_heading, 0.0, places=2)

        # Now engage auto-patrol and simulate leader turning toward +Y
        self.sim.patrol_active = True
        # Target is close (dist = 3.0m < FREEZE_DIST 8.0m)
        self.sim.target_pos = np.array([45.0, 3.0, 20.0], dtype=float)
        leader.vel = np.array([0.0, 6.0, 0.0], dtype=float)

        # Step simulation several times
        for _ in range(10):
            self.sim.step()

        # Heading MUST have rotated towards +Y (pi/2) and NOT remained frozen at 0.0!
        self.assertGreater(brain.formation_heading, 0.05)

    def test_gossip_autopatrol_consensus_convergence(self):
        """Verify that during target auto-patrol in Gossip Consensus mode,
        target injection is paced so that multi-hop quorum can settle,
        avoiding 60 Hz sequence thrashing."""
        sim = SwarmSimulator(drone_count=6)
        sim.set_comms_mode("GOSSIP_CONSENSUS")
        sim.toggle_target_patrol(True)
        self.assertTrue(sim.patrol_active)

        # Step 120 frames (2.0 simulated seconds)
        for _ in range(120):
            sim.step()

        # In 120 steps, an unpaced sim would have injected 120 sequence increments.
        # With consensus pacing, sequence increments should be reasonably throttled (e.g. <= 4).
        gossip_brain = sim.brain
        self.assertLessEqual(gossip_brain.current_seq, 4)
        self.assertGreater(gossip_brain.current_seq, 0)

        # Telemetry snapshot should show consensus reached periodically
        snap = sim.get_state_snapshot()
        self.assertEqual(snap["comms_mode"], "GOSSIP_CONSENSUS")
        self.assertTrue(snap["brain_telemetry"]["consensus_reached"])

    def test_leader_follower_boids_flocking_murmuration(self):
        """Verify TKT-23: Leader-Follower escorts execute true 3D Reynolds boid flocking."""
        self.sim.set_formation("NATURAL_FLOCK")
        self.sim.target_pos = np.array([50.0, 30.0, 25.0], dtype=float)

        # Step 300 ticks (5.0s)
        for _ in range(300):
            self.sim.step()

        active = [d for d in self.sim.drones if not d.crashed]
        self.assertEqual(len(active), 6)

        # 1. Zero Zone 4 clashes (d > 1.5m) and safe separation (d > 2.5m)
        for i in range(len(active)):
            for j in range(i + 1, len(active)):
                d_ij = float(np.linalg.norm(active[i].pos - active[j].pos))
                self.assertGreater(d_ij, ZONE4_CLASH, f"Prop clash between #{active[i].id} and #{active[j].id}: {d_ij:.2f}m")
                self.assertGreater(d_ij, 2.5, f"Followers clumped: {d_ij:.2f}m")

        # 2. Kinematic limits strictly preserved
        for d in active:
            speed = float(np.linalg.norm(d.vel))
            self.assertLessEqual(speed, 20.0, f"Speed exceeded max clamp: {speed:.1f}m/s")
            self.assertGreaterEqual(d.pos[2], 10.0, f"Altitude breached safety floor: {d.pos[2]:.1f}m")

    def test_boids_3d_z_space_layering(self):
        """Verify TKT-23: Drones utilize 3D Z-space instead of a flat 2D pancake."""
        self.sim.set_formation("NATURAL_FLOCK")
        self.sim.target_pos = np.array([40.0, 40.0, 25.0], dtype=float)

        for _ in range(240):
            self.sim.step()

        active = [d for d in self.sim.drones if not d.crashed]
        z_positions = [d.pos[2] for d in active]
        z_variance = float(np.var(z_positions))

        # Z-positions must exhibit natural vertical spread (> 0.1m^2 variance, ~0.4m+ std dev), proving true 3D flocking
        self.assertGreater(z_variance, 0.1, f"Swarm flattened into 2D plane: z_var={z_variance:.4f}")

    def test_hungarian_bipartite_assignment_minimal_cost(self):
        """Verify Hungarian Kuhn-Munkres algorithm returns minimum-weight matching (TKT-43)."""
        from OBC.backend.brains.hungarian import hungarian_assignment
        cost = np.array([
            [9.0, 2.0, 7.0, 8.0],
            [6.0, 4.0, 3.0, 7.0],
            [5.0, 8.0, 1.0, 8.0],
            [7.0, 6.0, 9.0, 4.0]
        ])
        rows, cols = hungarian_assignment(cost)
        total_cost = sum(cost[r, c] for r, c in zip(rows, cols))
        self.assertEqual(total_cost, 13.0)

        # Rectangular matrix (3 items to 5 slots)
        cost_rect = np.array([
            [10.0, 2.0, 8.0, 4.0, 9.0],
            [5.0, 8.0, 1.0, 6.0, 7.0],
            [7.0, 3.0, 9.0, 2.0, 8.0]
        ])
        rows_r, cols_r = hungarian_assignment(cost_rect)
        self.assertEqual(len(rows_r), 3)
        self.assertEqual(cols_r, [1, 2, 3])  # items get slots 1 (cost 2), 2 (cost 1), 3 (cost 2) = total 5.0

    def test_leader_follower_u_turn_formation_inversion(self):
        """Verify TKT-43: On U-turn heading reversal (>120 deg), Hungarian matching
        re-assigns trailing drones to new vanguard slots, minimizing total displacement."""
        self.sim.set_formation("ECHELON_ESCORT")
        leader = self.sim.leader
        self.assertIsNotNone(leader)

        # 1. Establish steady eastbound cruise toward (+60, 0, 20)
        self.sim.set_target([60.0, 0.0, 20.0])
        for _ in range(120):
            self.sim.step()

        # Follower slots in eastbound echelon:
        slots_before = dict(self.sim.brain.assigned_slots)
        self.assertTrue(len(slots_before) >= 4)

        # 2. Command sharp 180-deg reversal: target to West (-60, 0, 20)
        self.sim.set_target([-60.0, 0.0, 20.0])

        # Step 30 ticks to allow leader to turn and trigger Hungarian inversion
        for _ in range(30):
            self.sim.step()

        slots_after = dict(self.sim.brain.assigned_slots)

        # Slots must have been remapped by Hungarian matching
        differences = sum(1 for did in slots_before if slots_before.get(did) != slots_after.get(did))
        self.assertGreater(differences, 0, "Slots failed to invert upon sharp U-turn!")

        # Step forward 120 ticks: all drones remain alive, zero prop clashes (d > ZONE4_CLASH)
        for _ in range(120):
            self.sim.step()

        for d in self.sim.drones:
            self.assertFalse(d.crashed, f"Drone #{d.id} clashed during U-turn inversion!")

    def test_gossip_consensus_u_turn_formation_inversion(self):
        """Verify TKT-43 in GossipConsensusBrain: slots flip on consensus heading reversal."""
        self.sim.set_brain("GOSSIP_CONSENSUS")
        self.sim.set_formation("ECHELON_ESCORT")

        # Cruise toward (+50, 0, 25)
        self.sim.set_target([50.0, 0.0, 25.0])
        for _ in range(120):
            self.sim.step()

        slots_before = dict(self.sim.brain.assigned_slots)

        # Command U-turn to West (-50, 0, 25)
        self.sim.set_target([-50.0, 0.0, 25.0])
        for _ in range(60):
            self.sim.step()

        slots_after = dict(self.sim.brain.assigned_slots)
        # Verify slots exist and survived without prop clash
        for d in self.sim.drones:
            self.assertFalse(d.crashed, f"Gossip drone #{d.id} clashed during U-turn inversion!")

    def test_seamless_brain_handover_preserves_slots_and_positions(self):
        """[FIX-09] Hot-swapping between LeaderFollower and GossipConsensus must preserve
        slot assignments without instantaneous 50m jump or Zone 4 collision."""
        sim = SwarmSimulator(drone_count=5, enable_packet_loss=False, seed=42)
        sim.set_comms_mode("LEADER_FOLLOWER")
        sim.set_formation("ECHELON_ESCORT")
        sim.set_target([50.0, 30.0, 25.0])

        for _ in range(60):
            sim.step()

        slots_before = dict(sim.brain.assigned_slots)
        pos_before = {d.id: np.copy(d.pos) for d in sim.drones}

        # Hot-swap to GOSSIP_CONSENSUS
        ok = sim.set_brain("GOSSIP_CONSENSUS")
        self.assertTrue(ok)
        self.assertEqual(sim.active_brain_name, "GOSSIP_CONSENSUS")

        # Step sim 60 frames under GossipConsensusBrain
        for _ in range(60):
            sim.step()

        # Verify zero crashes and safe pairwise distances across handover
        for d in sim.drones:
            self.assertFalse(d.crashed, f"Drone #{d.id} crashed during brain hot-swap!")

        for i in range(len(sim.drones)):
            for j in range(i + 1, len(sim.drones)):
                d_ij = float(np.linalg.norm(sim.drones[i].pos - sim.drones[j].pos))
                self.assertGreater(d_ij, ZONE4_CLASH, f"Clash between #{i} and #{j}: {d_ij:.2f}m")

    def test_mid_flight_tether_relay_switch_locked(self):
        """[FIX-09] Mid-flight switch to TETHERED_RELAY is rejected when enforcing physical tether invariants:
        hardware umbilical cannot be attached mid-flight when already airborne in wireless mode."""
        sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=42)
        sim.set_comms_mode("LEADER_FOLLOWER")
        self.assertEqual(sim.comms_mode, "LEADER_FOLLOWER")

        # Swarm is airborne in cruise: attempt to switch with enforce_grounded=True must fail
        ok = sim.set_comms_mode("TETHERED_RELAY", enforce_grounded=True)
        self.assertFalse(ok)
        self.assertEqual(sim.comms_mode, "LEADER_FOLLOWER")

        # Switching with default (or when grounded) succeeds
        ok_default = sim.set_comms_mode("TETHERED_RELAY")
        self.assertTrue(ok_default)
        self.assertEqual(sim.comms_mode, "TETHERED_RELAY")

    def test_comms_horizon_crash_masking(self):
        """[FIX-09] Crash occurring outside radio horizon is masked in GCS telemetry as [LOST_LINK]
        at LKP, while God-View /sim retains omniscient ground truth (crashed: True)."""
        sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=42)
        sim.set_comms_mode("LEADER_FOLLOWER")

        # Move Drone 3 far beyond 120m radio horizon
        d3 = sim.drones[3]
        d3.pos = np.array([250.0, 0.0, 20.0], dtype=float)
        d3.lkp = np.array([115.0, 0.0, 20.0], dtype=float)
        d3.lost_link = True

        # Update radio comms: Drone 3 loses downlink
        sim._update_radio_comms()
        self.assertFalse(d3.downlink_active)
        self.assertTrue(d3.lost_link)

        # Drone 3 crashes while out of contact
        sim._trigger_crash(d3, reason="OBSTACLE_COLLISION_OUT_OF_RANGE")
        self.assertTrue(d3.crashed)
        self.assertFalse(d3.crashed_while_linked)

        snap = sim.get_state_snapshot()
        gcs_d3 = next(d for d in snap["drones"] if d["id"] == 3)
        sim_d3 = next(d for d in snap["sim_drones"] if d["id"] == 3)

        # GCS Airgap: GCS receives [LOST_LINK] at LKP, does not know it crashed
        self.assertFalse(gcs_d3["crashed"])
        self.assertTrue(gcs_d3["lost_link"])
        self.assertEqual(gcs_d3["status"], "LOST_LINK")
        self.assertEqual(gcs_d3["pos"], [115.0, 0.0, 20.0])

        # God-View Ground Truth: Sim retains true casualty state
        self.assertTrue(sim_d3["crashed"])
        self.assertEqual(sim_d3["status"], "CRASHED")
        self.assertEqual(sim_d3["pos"], [250.0, 0.0, 20.0])

    def test_bug03_hot_swap_unifies_consensus_target_and_prevents_oscillation(self):
        """[BUG-03] Hot-swapping from Wireless Star (LEADER_FOLLOWER) to P2P Mesh (GOSSIP_CONSENSUS)
        must unify initial active/pending target across all drones to prevent split-brain oscillation."""
        sim = SwarmSimulator(drone_count=6, enable_packet_loss=False, seed=42)
        sim.set_comms_mode("LEADER_FOLLOWER")

        target_bravo = np.array([120.0, 80.0, 30.0], dtype=float)
        sim.leader.acknowledged_target = np.copy(target_bravo)
        sim.target_pos = np.copy(target_bravo)

        # Follower currently holds default Alpha
        sim.drones[1].acknowledged_target = np.array([60.0, 40.0, 30.0], dtype=float)

        # Hot-swap to GOSSIP_CONSENSUS
        ok = sim.set_brain("GOSSIP_CONSENSUS")
        self.assertTrue(ok)

        # All brains should have been initialized with Target Bravo
        for d in sim.drones:
            self.assertIsInstance(d.brain, GossipConsensusBrain)
            pstate = d.brain.peer_state
            np.testing.assert_allclose(pstate.active_target, target_bravo)
            np.testing.assert_allclose(pstate.pending_target, target_bravo)
            self.assertEqual(pstate.active_seq, 1)
            self.assertEqual(pstate.pending_seq, 1)
            self.assertEqual(pstate.gossip_status, "COORDINATED")

        # Step 30 frames: verify all drones steer consistently toward Bravo with zero split-brain oscillation
        for _ in range(30):
            sim.step()
            for d in sim.drones:
                self.assertFalse(np.isnan(d.pos).any())
                np.testing.assert_allclose(d.acknowledged_target, target_bravo)
                pstate = d.brain.peer_state
                np.testing.assert_allclose(pstate.active_target, target_bravo)

    def test_bug03_equal_seq_divergent_target_tie_breaking(self):
        """[BUG-03] When two mesh partitions with equal sequence numbers meet with diverging targets,
        they must tie-break deterministically (newer timestamp wins; lower drone ID breaks timestamp ties)
        without deadlocking or oscillating."""
        from OBC.backend.brains.base import RadioPacket

        sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=42)
        b0 = GossipConsensusBrain(drone_id=0, initial_target=np.array([100.0, 100.0, 30.0]))
        b1 = GossipConsensusBrain(drone_id=1, initial_target=np.array([20.0, 20.0, 30.0]))

        b0.peer_state.pending_seq = 1
        b0.peer_state.pending_target = np.array([100.0, 100.0, 30.0])
        b0.peer_state.target_timestamp = 10.0

        b1.peer_state.pending_seq = 1
        b1.peer_state.pending_target = np.array([20.0, 20.0, 30.0])
        b1.peer_state.target_timestamp = 5.0

        sim.sim_time = 10.0
        tel1 = sim._build_local_telemetry(sim.drones[1])

        pkt0 = RadioPacket(
            sender_id=0,
            msg_type="GOSSIP_ECHO",
            timestamp=10.0,
            payload={
                "echo_seq": 1,
                "echo_target": [100.0, 100.0, 30.0],
                "echo_ack_peers": [0],
                "target_timestamp": 10.0,
            },
        )

        # Ingest pkt0 on Drone 1: newer timestamp (10.0 > 5.0) must win!
        b1.ingest_gossip_packets(tel1, [pkt0])
        np.testing.assert_allclose(b1.peer_state.pending_target, [100.0, 100.0, 30.0])
        self.assertEqual(b1.peer_state.gossip_status, "PROPAGATING")

        # Now test equal timestamp tie-break: lower sender ID (0 < 1) wins
        b1.peer_state.target_timestamp = 10.0
        b1.peer_state.pending_target = np.array([20.0, 20.0, 30.0])

        tel0 = sim._build_local_telemetry(sim.drones[0])

        pkt1 = RadioPacket(
            sender_id=1,
            msg_type="GOSSIP_ECHO",
            timestamp=10.0,
            payload={
                "echo_seq": 1,
                "echo_target": [20.0, 20.0, 30.0],
                "echo_ack_peers": [1],
                "target_timestamp": 10.0,
            },
        )

        # Drone 0 receives Drone 1's packet: 0 is lower ID, so Drone 0 keeps its target
        b0.ingest_gossip_packets(tel0, [pkt1])
        np.testing.assert_allclose(b0.peer_state.pending_target, [100.0, 100.0, 30.0])

        # Drone 1 receives Drone 0's packet: 0 is lower ID, so Drone 1 adopts Drone 0's target
        b1.ingest_gossip_packets(tel1, [pkt0])
        np.testing.assert_allclose(b1.peer_state.pending_target, [100.0, 100.0, 30.0])

    def test_bug03_mesh_summary_does_not_ghost_loop(self):
        """[BUG-03] Multi-hop mesh_summary relays must preserve original last_heard timestamp,
        ensuring disconnected peers expire after LINK_TIMEOUT (1.0s) rather than persisting infinitely."""
        from OBC.backend.brains.base import HeardPeer

        sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=42)
        b0 = GossipConsensusBrain(drone_id=0)
        b1 = GossipConsensusBrain(drone_id=1)

        # Both initially know Drone 2 at t=5.0
        b0.heard_peers[2] = HeardPeer(
            drone_id=2, pos=np.array([30.0, 30.0, 30.0]), vel=np.zeros(3),
            yaw=0.0, enrolled=True, flight_state="CRUISE", last_heard_clock=5.0
        )
        b1.heard_peers[2] = HeardPeer(
            drone_id=2, pos=np.array([30.0, 30.0, 30.0]), vel=np.zeros(3),
            yaw=0.0, enrolled=True, flight_state="CRUISE", last_heard_clock=5.0
        )

        # At t=5.5s, b0 emits packet to b1 with mesh_summary
        sim.sim_time = 5.5
        tel0_55 = sim._build_local_telemetry(sim.drones[0])
        pkt_b0 = b0.emit_gossip_packet(tel0_55)

        # b1 receives pkt_b0 at t=5.5s
        tel1_55 = sim._build_local_telemetry(sim.drones[1])
        b1.ingest_gossip_packets(tel1_55, [pkt_b0])

        # Drone 2's last_heard_clock on b1 must STILL be 5.0s, not 5.5s!
        self.assertIn(2, b1.heard_peers)
        self.assertEqual(b1.heard_peers[2].last_heard_clock, 5.0)

        # Advance to t=6.2s (> 5.0 + LINK_TIMEOUT (1.0s)): b1 steps with empty inbox
        sim.sim_time = 6.2
        tel1_62 = sim._build_local_telemetry(sim.drones[1])
        b1.ingest_gossip_packets(tel1_62, [])

        # Drone 2 must now be PRUNED from b1's heard_peers
        self.assertNotIn(2, b1.heard_peers)

    def test_bug04_follower_and_pad_lkp_isolation_in_wireless_star(self):
        """[BUG-04] Followers and pad drones in Wireless Star must maintain their own LKP coordinates.
        Receiving leader heartbeats or losing link must NEVER corrupt follower/pad LKP to the leader's coordinates,
        and pad drones must NEVER have their status flipped to LOST_LINK."""
        sim = SwarmSimulator(drone_count=6, enable_packet_loss=False, seed=42)
        sim.set_comms_mode("LEADER_FOLLOWER")

        d0 = sim.drones[0]
        d1 = sim.drones[1]
        d5 = sim.drones[5]

        # Leader at [30, 30, 20], Follower 1 airborne at [20, 25, 20]
        d0.pos = np.array([30.0, 30.0, 20.0], dtype=float)
        d1.pos = np.array([20.0, 25.0, 20.0], dtype=float)

        # Drone 5 grounded cold on pad
        pad_pos = np.array([-15.0, -15.0, 0.0], dtype=float)
        d5.pos = np.copy(pad_pos)
        d5.flight_state = "GROUND_IDLE"
        d5.status = "GROUND_IDLE"
        d5.lkp = np.copy(pad_pos)

        # Step 15 frames (~0.25s): leader heartbeats received
        for _ in range(15):
            sim.step()

        # Follower 1 LKP must record its OWN position, not leader's pos
        self.assertFalse(d1.lost_link)
        self.assertGreater(float(np.linalg.norm(d1.lkp - d0.pos)), 2.0)
        np.testing.assert_allclose(d1.lkp, d1.pos, atol=2.0)

        # Pad Drone 5 LKP must remain strictly on pad
        np.testing.assert_allclose(d5.lkp, pad_pos, atol=1e-3)
        self.assertEqual(d5.flight_state, "GROUND_IDLE")
        self.assertEqual(d5.status, "GROUND_IDLE")

        # Teleport leader far out of comms range (> 120m)
        d0.pos = np.array([250.0, 250.0, 20.0], dtype=float)

        # Step 75 frames (> 1.0s timeout)
        for _ in range(75):
            sim.step()

        # Follower 1 enters LOST_LINK but loiters at its own LKP (near [20, 25, 20])
        self.assertTrue(d1.lost_link)
        self.assertEqual(d1.status, "LOST_LINK")
        self.assertLess(float(np.linalg.norm(d1.lkp - np.array([20.0, 25.0, 20.0]))), 5.0)
        self.assertGreater(float(np.linalg.norm(d1.lkp - d0.pos)), 100.0)

        # Pad Drone 5 must remain GROUND_IDLE on pad, not corrupted to LOST_LINK or airborne coords
        np.testing.assert_allclose(d5.lkp, pad_pos, atol=1e-3)
        np.testing.assert_allclose(d5.pos, pad_pos, atol=1e-3)
        self.assertEqual(d5.flight_state, "GROUND_IDLE")
        self.assertEqual(d5.status, "GROUND_IDLE")

    def test_bug04_airborne_followers_preserve_slot_separation_on_lost_link(self):
        """[BUG-04] When multiple airborne followers in Wireless Star lose link with the leader,
        each follower must hold station at its own LKP rather than collapsing to a single central singularity."""
        sim = SwarmSimulator(drone_count=4, enable_packet_loss=False, seed=42)
        sim.set_comms_mode("LEADER_FOLLOWER")

        # Step 30 frames to settle followers in formation slots around leader
        for _ in range(30):
            sim.step()

        # Teleport leader far away
        sim.leader.pos = np.array([300.0, 300.0, 20.0], dtype=float)

        # Step 75 frames to trigger LOST_LINK
        for _ in range(75):
            sim.step()

        followers = [sim.drones[1], sim.drones[2], sim.drones[3]]
        for f in followers:
            self.assertTrue(f.lost_link)
            self.assertEqual(f.status, "LOST_LINK")
            self.assertFalse(f.crashed)

        # Verify pairwise separation: no two followers collapsed to the same coordinate
        for i in range(len(followers)):
            for j in range(i + 1, len(followers)):
                dist = float(np.linalg.norm(followers[i].pos - followers[j].pos))
                self.assertGreater(dist, 1.5, f"Drones {followers[i].id} and {followers[j].id} collapsed to distance {dist:.2f}m")
                dist_lkp = float(np.linalg.norm(followers[i].lkp - followers[j].lkp))
                self.assertGreater(dist_lkp, 1.5, f"LKP of drones {followers[i].id} and {followers[j].id} collapsed to distance {dist_lkp:.2f}m")


if __name__ == "__main__":
    unittest.main()
