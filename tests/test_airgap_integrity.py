"""
SwarmTech3_UpRising: OBC Brain Airgap & Pure I/O Contract Verification (test_airgap_integrity.py)
Enforces TLK-01 Induced Coma architectural invariants:
  1. Zero SIM or GCS imports anywhere in OBC/.
  2. Zero .sim or self.sim references anywhere in OBC/.
  3. BaseBrain, LeaderFollowerBrain, and GossipConsensusBrain accept zero simulator handles.
  4. Option A Per-Drone Brain Isolation: each Drone owns a distinct BaseBrain instance.
  5. Pure I/O Boundary: step_drone(LocalTelemetry, [RadioPacket], dt) -> (FlightIntent, [RadioPacket])
     operates standalone without any simulator instance and masks XY coordinates during GNSS denial.
"""

from __future__ import annotations
import ast
import inspect
import math
import re
import sys
import unittest
from pathlib import Path
import numpy as np

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from OBC.backend.drone_firmware import Drone
from OBC.backend.brains import (
    BaseBrain,
    RadioPacket,
    LocalTelemetry,
    FlightIntent,
    LeaderFollowerBrain,
    GossipConsensusBrain,
)
from SIM.backend.sitl_engine import SITLWorldEngine, DT
from tests.quarantine import ACTIVE_HEARTBEAT_STAGE, QUARANTINE_REGISTRY


class TestAirgapIntegrity(unittest.TestCase):
    """Static AST and runtime verification of the Phase 4 OBC Brain Airgap."""

    def test_zero_sim_or_gcs_imports_and_attributes_in_obc(self):
        """Scan every .py file in OBC/ via AST and regex to forbid SIM/GCS imports or .sim access."""
        obc_dir = _REPO_ROOT / "OBC"
        py_files = list(obc_dir.rglob("*.py"))
        self.assertGreater(len(py_files), 0, "Expected Python files in OBC/")

        forbidden_modules = {"SIM", "GCS"}
        forbidden_tokens = re.compile(r"\b(self\.sim|SwarmSimulator|SITLWorldEngine)\b")

        for py_path in py_files:
            rel_path = py_path.relative_to(_REPO_ROOT)
            source = py_path.read_text(encoding="utf-8")

            # 1. Raw token scan
            match = forbidden_tokens.search(source)
            self.assertIsNone(
                match,
                f"Forbidden simulator token '{match.group(0) if match else ''}' found in {rel_path}",
            )

            # 2. AST structural scan
            tree = ast.parse(source, filename=str(rel_path))
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        root_pkg = alias.name.split(".")[0]
                        self.assertNotIn(
                            root_pkg,
                            forbidden_modules,
                            f"Forbidden import '{alias.name}' in {rel_path}:{node.lineno}",
                        )
                elif isinstance(node, ast.ImportFrom):
                    if node.module:
                        root_pkg = node.module.split(".")[0]
                        self.assertNotIn(
                            root_pkg,
                            forbidden_modules,
                            f"Forbidden from-import '{node.module}' in {rel_path}:{node.lineno}",
                        )
                elif isinstance(node, ast.Attribute):
                    self.assertNotEqual(
                        node.attr,
                        "sim",
                        f"Forbidden '.sim' attribute access in {rel_path}:{node.lineno}",
                    )

    def test_brain_constructors_accept_zero_sim_handles(self):
        """Verify BaseBrain, LeaderFollowerBrain, and GossipConsensusBrain do not accept 'sim'."""
        for cls in (BaseBrain, LeaderFollowerBrain, GossipConsensusBrain):
            sig = inspect.signature(cls.__init__)
            self.assertNotIn(
                "sim",
                sig.parameters,
                f"{cls.__name__}.__init__ must not accept a 'sim' parameter",
            )

    def test_option_a_per_drone_isolated_brain_instances(self):
        """Verify Option A: every Drone owns its own distinct BaseBrain instance."""
        sim = SITLWorldEngine(drone_count=4, seed=42)
        brain_ids = set()
        for d in sim.drones:
            self.assertIsInstance(d.brain, LeaderFollowerBrain)
            self.assertEqual(d.brain.drone_id, d.id)
            self.assertFalse(hasattr(d.brain, "sim"))
            brain_ids.add(id(d.brain))

        self.assertEqual(len(brain_ids), 4, "Each drone must own a distinct BaseBrain object in memory")

        # Hot-swap to GOSSIP_CONSENSUS and verify per-drone brain replacement
        sim.set_brain("GOSSIP_CONSENSUS")
        gossip_brain_ids = set()
        for d in sim.drones:
            self.assertIsInstance(d.brain, GossipConsensusBrain)
            self.assertEqual(d.brain.drone_id, d.id)
            self.assertFalse(hasattr(d.brain, "sim"))
            gossip_brain_ids.add(id(d.brain))

        self.assertEqual(len(gossip_brain_ids), 4)

    def test_pure_io_solo_navigation_without_simulator(self):
        """Verify Heartbeat 1: standalone LeaderFollowerBrain and GossipConsensusBrain steer
        purely from LocalTelemetry and RadioPacket inbox with zero simulator instantiated."""
        brain = LeaderFollowerBrain(drone_id=0)
        telemetry = LocalTelemetry(
            drone_id=0,
            tier=3,
            is_leader=True,
            enrolled=True,
            flight_state="CRUISE",
            status="FLEET_LEADER",
            clock_s=1.0,
            pos=np.array([0.0, 0.0, 20.0], dtype=float),
            vel=np.array([0.0, 0.0, 0.0], dtype=float),
            yaw=0.0,
            reciprocal_heading=math.pi,
            rssi=-45.0,
            rssi_prev=-45.0,
            link_quality=1.0,
            lost_link=False,
            uplink_active=True,
            downlink_active=True,
            gnss_denied=False,
            gnss_denied_timer=0.0,
            drift_radius=0.0,
            gnss_settled=False,
            battery=100.0,
            acknowledged_target=np.array([50.0, 0.0, 20.0], dtype=float),
            lkp=np.array([0.0, 0.0, 20.0], dtype=float),
            backtrack_target=None,
            holding_at_horizon=False,
            rejoin_target=None,
            mission_active=False,
            mission_paused=False,
            mission_loop=True,
            mission_waypoints=(),
            current_wp_idx=0,
            wp_loitering=False,
            wp_loiter_timer=0.0,
            wp_total_loiter=0.0,
            route_acceptance_radius=4.0,
            peel_off_timer=0.0,
            peel_off_dir=np.zeros(3, dtype=float),
            promotion_timer=0.0,
            enrollment_timer=0.0,
            tether_active=True,
            tether_lost=False,
            tether_jettisoned=False,
            tether_failsafe_policy="REJOIN",
            tether_anchor_pos=np.array([0.0, 0.0, 1.5], dtype=float),
            overhead_marshal_active=False,
            patrol_active=False,
        )

        # Deliver a GCS_UPLINK target command via inbox packet
        uplink_pkt = RadioPacket(
            sender_id=-1,
            msg_type="GCS_UPLINK",
            timestamp=1.0,
            seq=1,
            payload={"cmd": "SET_TARGET", "target": [100.0, 0.0, 20.0]},
        )
        intent, outbox = brain.step_drone(telemetry, [uplink_pkt], DT)
        self.assertIsInstance(intent, FlightIntent)
        self.assertGreater(intent.target_acc[0], 0.0, "Must accelerate toward +X target")
        np.testing.assert_allclose(intent.state_updates["acknowledged_target"], [100.0, 0.0, 20.0])

    def test_gnss_denied_telemetry_masks_xy_with_nan(self):
        """Verify SITLWorldEngine masks LocalTelemetry.pos[:2] to NaN when gnss_denied=True,
        physically preventing any brain from reading ground-truth XY coordinates."""
        sim = SITLWorldEngine(drone_count=2, seed=42)
        sim.toggle_gnss_denial(drone_id=0, denied=True)
        tel = sim._build_local_telemetry(sim.drones[0])
        self.assertTrue(np.isnan(tel.pos[0]))
        self.assertTrue(np.isnan(tel.pos[1]))
        self.assertFalse(np.isnan(tel.pos[2]))

        intent, _ = sim.drones[0].brain.step_drone(tel, [], DT)
        self.assertFalse(np.isnan(intent.target_acc).any(), "GNSS-denied guidance must not produce NaN")

    def test_local_telemetry_contains_zero_peer_drone_objects(self):
        """Red 1 Airgap Enforcement: LocalTelemetry must never smuggle foreign Drone objects
        or peer simulation memory handles into the OBC brain."""
        sim = SITLWorldEngine(drone_count=4, seed=42)
        for drone in sim.drones:
            tel = sim._build_local_telemetry(drone)
            self.assertFalse(
                hasattr(tel, "nearby_peers"),
                "LocalTelemetry must not have 'nearby_peers' attribute (prohibits object smuggling)",
            )
            # Inspect all fields of LocalTelemetry to ensure zero Drone instances exist
            for f_name in tel.__dataclass_fields__:
                val = getattr(tel, f_name)
                if isinstance(val, (list, tuple)):
                    for item in val:
                        self.assertNotIsInstance(
                            item,
                            Drone,
                            f"Field '{f_name}' in LocalTelemetry contains smuggled Drone object!",
                        )
                else:
                    self.assertNotIsInstance(
                        val,
                        Drone,
                        f"Field '{f_name}' in LocalTelemetry is a Drone instance!",
                    )

    def test_autonomous_outbox_emission_zero_sitl_forgery(self):
        """Red 8 Airgap Enforcement (BUG-17): Packets originate exclusively in OBC step_drone.
        SITLWorldEngine._deliver_radio_packets must never forge or append packets into outboxes."""
        sim = SITLWorldEngine(drone_count=4, seed=42)
        sim.set_brain("GOSSIP_CONSENSUS", force=True)

        for d in sim.drones:
            d.outbox.clear()

        # SITL deliver comms step with empty outboxes must not forge any packets
        sim._deliver_radio_packets(DT)
        for d in sim.drones:
            self.assertEqual(len(d.outbox), 0, f"Drone #{d.id} outbox was forged by SITL!")

        # OBC guidance step must autonomously populate outbox
        accs = sim._execute_obc_step(DT)
        for d in sim.drones:
            if not d.crashed and d.flight_state not in ("LANDED", "DESCENDING"):
                self.assertGreater(
                    len(d.outbox),
                    0,
                    f"Drone #{d.id} OBC brain step_drone failed to emit packet into outbox",
                )

    def test_dispatch_target_uplink_uses_inbox_packets_zero_ram_mutation(self):
        """Red 4 Airgap Enforcement (BUG-13): dispatch_target_uplink must deliver packets
        to transceiver inbox without mutating drone.acknowledged_target directly in RAM."""
        sim = SITLWorldEngine(drone_count=4, seed=42)
        sim.comms_mode = "LEADER_FOLLOWER"
        leader = sim.leader
        self.assertIsNotNone(leader)

        old_target = np.copy(leader.acknowledged_target)
        new_target = np.array([45.0, 35.0, 20.0], dtype=float)

        leader.inbox.clear()
        res = sim.dispatch_target_uplink(new_target)
        self.assertEqual(res["status"], "OK")

        # Zero direct RAM mutation: acknowledged_target must NOT change immediately
        self.assertTrue(
            np.allclose(leader.acknowledged_target, old_target),
            "dispatch_target_uplink cheated by mutating drone.acknowledged_target directly in RAM!",
        )

        # Target packet must be queued in inbox
        uplink_pkts = [p for p in leader.inbox if p.msg_type == "GCS_UPLINK"]
        self.assertEqual(len(uplink_pkts), 1, "Expected GCS_UPLINK packet in transceiver inbox")
        self.assertEqual(uplink_pkts[0].payload.get("cmd"), "SET_TARGET")

        # After step_drone execution, OBC brain ingests packet and updates target buffer
        sim.step()
        self.assertTrue(
            np.allclose(leader.acknowledged_target, new_target),
            "Leader OBC brain failed to update acknowledged_target from inbox packet during step()",
        )

    def test_no_quarantined_tests_remain_at_stage_3(self):
        """Anti-orphan gate: at Heartbeat Stage 3, zero tests may remain quarantined."""
        if ACTIVE_HEARTBEAT_STAGE >= 3:
            active_quarantines = [
                (hb, name, reason)
                for hb, name, reason in QUARANTINE_REGISTRY
                if ACTIVE_HEARTBEAT_STAGE < hb
            ]
            self.assertEqual(
                len(active_quarantines),
                0,
                f"Quarantined tests still skipped at Stage 3: {active_quarantines}",
            )


if __name__ == "__main__":
    unittest.main()
