"""
SwarmTech3_UpRising: Stage-Gated Test Quarantine Utility (quarantine.py)
Controls sequential subsystem resuscitation across the Induced Coma refactor (TLK-01):
  - Stage 1: Heartbeat 1 (Solo Navigation & Onboard Failsafes)
  - Stage 2: Heartbeat 2 (Leader-Follower Formations over RF Packets)
  - Stage 3: Heartbeat 3 (Decentralized P2P Gossip over 60m Mesh Packets)
"""

from __future__ import annotations
import os
import unittest
from typing import Callable, TypeVar

T = TypeVar("T")

ACTIVE_HEARTBEAT_STAGE: int = int(os.environ.get("SWARM_HEARTBEAT_STAGE", "2"))
QUARANTINE_REGISTRY: list[tuple[int, str, str]] = []


def quarantine_until(heartbeat: int, reason: str = "") -> Callable[[T], T]:
    """Skip decorated test method or class until ACTIVE_HEARTBEAT_STAGE >= heartbeat."""
    def decorator(obj: T) -> T:
        qualname = getattr(obj, "__qualname__", getattr(obj, "__name__", str(obj)))
        QUARANTINE_REGISTRY.append((heartbeat, qualname, reason))
        msg = (
            f"[QUARANTINED: Awaiting Heartbeat {heartbeat} | Active Stage={ACTIVE_HEARTBEAT_STAGE}] "
            f"{reason}"
        )
        return unittest.skipIf(ACTIVE_HEARTBEAT_STAGE < heartbeat, msg)(obj)
    return decorator
