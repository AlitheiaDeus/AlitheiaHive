"""
SwarmTech3_UpRising: Pluggable Guidance Brain Architecture (OBC Brain Airgap)
Defines BaseBrain interface, pure I/O boundary contracts, and registry for hot-swappable guidance plugins.
"""

from typing import Dict, Type
from .base import (
    BaseBrain,
    RadioPacket,
    HeardPeer,
    LocalTelemetry,
    FlightIntent,
)
from .leader_follower import LeaderFollowerBrain
from .gossip_consensus import GossipConsensusBrain

BRAIN_REGISTRY: Dict[str, Type[BaseBrain]] = {
    "LEADER_FOLLOWER": LeaderFollowerBrain,
    "GOSSIP_CONSENSUS": GossipConsensusBrain,
}


def register_brain(name: str):
    """Decorator to register a guidance brain plugin."""
    def decorator(cls: Type[BaseBrain]):
        BRAIN_REGISTRY[name.upper()] = cls
        cls.brain_name = name.upper()
        return cls
    return decorator


__all__ = [
    "BaseBrain",
    "RadioPacket",
    "HeardPeer",
    "LocalTelemetry",
    "FlightIntent",
    "BRAIN_REGISTRY",
    "register_brain",
    "LeaderFollowerBrain",
    "GossipConsensusBrain",
]
