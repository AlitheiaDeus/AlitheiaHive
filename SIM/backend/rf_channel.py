"""
SwarmTech3_UpRising: RF Channel & Signal Propagation Manager (rf_channel.py)
Bubble 2: SITL World Simulator.
Encapsulates RF link budgets (Canine Taxonomy), Friis path loss, electronic warfare jamming fields,
mesh telemetry gateway routing, and 3D urban obstacle ray-cast LOS shadowing (PLN-08 / TKT-49 / TKT-60).
"""

from __future__ import annotations
import math
from typing import Dict, List, Optional, Set, Tuple, Any, Union
import numpy as np

from SIM.backend.urban_geometry import ray_intersects_aabb, UrbanObstacle

# Radio Comms & Range Constants (Canine Taxonomy - TKT-49)
RANGE_HOWL: float = 140.0
RANGE_BARK: float = 90.0
RANGE_BORK: float = 70.0
RANGE_YIP: float = 50.0
RANGE_P2P: float = 60.0
RADIO_RANGE: float = RANGE_HOWL
HEARTBEAT_INTERVAL: float = 0.1   # 10 Hz baud rate (s)
LINK_TIMEOUT: float = 1.0         # Heartbeat timeout for LOST_LINK transition (s)
RF_MODE_BORK: str = "BORK"
TIER_LEADER: str = "TIER_1_LEADER"
TIER_ESCORT: str = "TIER_2_ESCORT"


def is_rf_occluded(origin: np.ndarray, target: np.ndarray, urban_obstacles: Optional[List[UrbanObstacle]] = None) -> bool:
    """Check if direct line-of-sight ray between origin and target intersects any RF-blocking obstacle (PLN-08 / TKT-60)."""
    if not urban_obstacles:
        return False
    for obs in urban_obstacles:
        if getattr(obs, "blocks_rf", True):
            if ray_intersects_aabb(origin, target, obs.bounds):
                return True
    return False


class RFChannelManager:
    """Manages RF channel propagation, signal attenuation, electronic warfare jamming,
    and mesh relay hops for the SITL world simulator."""

    def __init__(self, tether_anchor: Optional[np.ndarray] = None) -> None:
        self.tether_anchor: np.ndarray = (
            np.copy(tether_anchor) if tether_anchor is not None else np.array([0.0, 0.0, 1.5], dtype=float)
        )
        self.rf_jamming_rate: float = 0.0
        self.enable_packet_loss: bool = False
        self.last_heartbeat_broadcast: float = 0.0

    def set_rf_jamming(self, rate: float) -> float:
        """Set electronic warfare RF jamming rate [0.0, 0.5]."""
        self.rf_jamming_rate = float(np.clip(rate, 0.0, 0.5))
        if self.rf_jamming_rate > 0.0:
            self.enable_packet_loss = True
        return self.rf_jamming_rate

    def calculate_packet_drop_prob(
        self, distance: float, is_hardline: bool = False, max_range: Optional[float] = None
    ) -> float:
        """Calculate packet loss probability using distance-dependent path loss and jamming.
        P_loss(d) = gamma + (1 - gamma) * 0.25 * (d / R_max)^2"""
        if is_hardline:
            return 0.0
        r_max = max_range if max_range is not None else RADIO_RANGE
        if distance > r_max:
            return 1.0
        if not self.enable_packet_loss and self.rf_jamming_rate <= 0.0:
            return 0.0
        gamma = self.rf_jamming_rate
        norm_dist = distance / r_max
        prob = gamma + (1.0 - gamma) * 0.25 * (norm_dist ** 2)
        return float(np.clip(prob, 0.0, 1.0))

    def is_packet_dropped(
        self, distance: float, is_hardline: bool = False, max_range: Optional[float] = None
    ) -> bool:
        """Stochastically determine if packet is lost given distance and jamming rate."""
        p = self.calculate_packet_drop_prob(distance, is_hardline, max_range=max_range)
        if p <= 0.0:
            return False
        if p >= 1.0:
            return True
        return bool(np.random.random() < p)

    def calculate_rssi(self, distance: float, is_hardline: bool = False) -> Tuple[float, float]:
        """Calculate RSSI (dBm) and normalized link quality [0.0, 1.0].
        Hardline fiber: -35.0 dBm, 1.0 quality.
        Wireless: Free-space loss from -40.0 dBm at 1m with -40*gamma jamming penalty."""
        if is_hardline:
            return -35.0, 1.0
        d = max(1.0, float(distance))
        loss = 20.0 * math.log10(d)
        jamming_penalty = 40.0 * self.rf_jamming_rate
        rssi = -40.0 - loss - jamming_penalty
        rssi = float(np.clip(rssi, -115.0, -35.0))
        lq = max(0.0, min(1.0, (rssi - (-100.0)) / 60.0)) * (1.0 - self.rf_jamming_rate)
        return round(rssi, 1), round(float(lq), 2)

    def is_within_base_rf_dome(
        self, pos_or_dist: Union[float, List[float], np.ndarray], tether_anchor: Optional[np.ndarray] = None
    ) -> bool:
        """Check whether position or scalar distance is within 50m High-Gain RF Dome (TKT-42)."""
        anchor = tether_anchor if tether_anchor is not None else self.tether_anchor
        if isinstance(pos_or_dist, (int, float)):
            return float(pos_or_dist) <= 50.0
        pos_arr = np.array(pos_or_dist, dtype=float)
        return float(np.linalg.norm(pos_arr - anchor)) <= 50.0

    def calculate_base_rf_link(
        self, pos_or_dist: Union[float, List[float], np.ndarray], tether_anchor: Optional[np.ndarray] = None
    ) -> Tuple[float, float, float]:
        """Compute (drop_prob, rssi, link_quality) incorporating 50m Base Horizon High-Gain RF Dome (TKT-42).
        Within 50m radius of base anchor, high-gain base station radio guarantees 100% link quality."""
        anchor = tether_anchor if tether_anchor is not None else self.tether_anchor
        d = float(pos_or_dist) if isinstance(pos_or_dist, (int, float)) else float(np.linalg.norm(np.array(pos_or_dist, dtype=float) - anchor))
        if d <= 50.0:
            return 0.0, -35.0, 1.0
        rssi, lq = self.calculate_rssi(d)
        drop_p = self.calculate_packet_drop_prob(d)
        return drop_p, rssi, lq

    def update_radio_comms(
        self,
        sim_time: float,
        comms_mode: str,
        drones: List[Any],
        ground_repeaters: List[Any],
        tether_anchor: np.ndarray,
        tether_severed: bool,
        urban_obstacles: Optional[List[UrbanObstacle]] = None,
        current_leader: Optional[Any] = None,
    ) -> None:
        """Simulate realistic 10 Hz radio broadcast packets, asymmetric link budget (Canine Taxonomy),
        Mesh Telemetry Gateway relay, and timeout-based Lost-Link state transitions (TKT-49 / TKT-60)."""
        # Check if 10 Hz broadcast timer fired
        broadcast_tick = False
        if sim_time - self.last_heartbeat_broadcast >= HEARTBEAT_INTERVAL:
            self.last_heartbeat_broadcast = sim_time
            broadcast_tick = True

        living = [d for d in drones if not d.crashed]

        # Reset crashed drones
        for d in drones:
            if d.crashed:
                d.uplink_active = False
                d.downlink_active = False
                d.yip_lost = False
                d.gateway_id = None

        if comms_mode == "GOSSIP_CONSENSUS":
            # 1. Asymmetric Link Budget: Uplink & Direct Downlink
            direct_gateways: Set[int] = set()

            for d in living:
                d_gcs = float(np.linalg.norm(d.pos - tether_anchor))
                occluded_gcs = is_rf_occluded(tether_anchor, d.pos, urban_obstacles)

                # Base Horizon High-Gain RF Dome (TKT-42):
                if d_gcs <= 50.0 and not occluded_gcs:
                    d.uplink_active = True
                    d.downlink_active = True
                    d.yip_lost = False
                    d.gateway_id = d.id
                    direct_gateways.add(d.id)
                    continue

                dropped_up = occluded_gcs or self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_HOWL)
                d.uplink_active = (d_gcs <= RANGE_HOWL and not dropped_up)

                r_down = RANGE_BORK if getattr(d, "rf_power_mode", None) == RF_MODE_BORK else (RANGE_BARK if (d.tier == TIER_LEADER or d.is_leader) else RANGE_YIP)
                dropped_down = occluded_gcs or self.is_packet_dropped(d_gcs, is_hardline=False, max_range=r_down)
                has_direct = (d_gcs <= r_down and not dropped_down)

                if has_direct:
                    d.downlink_active = True
                    d.gateway_id = d.id  # Acting as its own direct gateway
                    direct_gateways.add(d.id)
                else:
                    d.downlink_active = False
                    d.gateway_id = None

            # 2. Mesh Telemetry Gateway (The Relay Proxy)
            queue = [(gw_id, gw_id) for gw_id in direct_gateways]
            visited = set(direct_gateways)

            # Direct link assessment for stationary ground repeaters (TKT-59 / PLN-04)
            for rep in ground_repeaters:
                if not rep.active:
                    rep.uplink_active = False
                    rep.downlink_active = False
                    continue
                d_rep_gcs = float(np.linalg.norm(rep.pos - tether_anchor))
                occluded_rep_gcs = is_rf_occluded(tether_anchor, rep.pos, urban_obstacles)
                rep.uplink_active = (d_rep_gcs <= RANGE_HOWL and not occluded_rep_gcs and not self.is_packet_dropped(d_rep_gcs, is_hardline=False, max_range=RANGE_HOWL))
                rep_direct = (d_rep_gcs <= 50.0 and not occluded_rep_gcs)
                if not rep_direct:
                    for gw_id in direct_gateways:
                        gw_d = next((d for d in living if d.id == gw_id), None)
                        if gw_d:
                            d_rg = float(np.linalg.norm(rep.pos - gw_d.pos))
                            occluded_rg = is_rf_occluded(rep.pos, gw_d.pos, urban_obstacles)
                            if d_rg <= RANGE_P2P and not occluded_rg and not self.is_packet_dropped(d_rg, is_hardline=False, max_range=RANGE_P2P):
                                rep_direct = True
                                break
                rep.downlink_active = rep_direct

            while queue:
                curr_id, root_gw = queue.pop(0)
                curr_drone = next((d for d in living if d.id == curr_id), None)
                if curr_drone is None:
                    continue

                for peer in living:
                    if peer.id not in visited:
                        d_peer = float(np.linalg.norm(peer.pos - curr_drone.pos))
                        occluded_peer = is_rf_occluded(curr_drone.pos, peer.pos, urban_obstacles)
                        dropped_peer = occluded_peer or self.is_packet_dropped(d_peer, is_hardline=False, max_range=RANGE_P2P)
                        if d_peer <= RANGE_P2P and not dropped_peer:
                            peer.downlink_active = True
                            peer.gateway_id = root_gw
                            visited.add(peer.id)
                            queue.append((peer.id, root_gw))

            # Extend mesh to living drones within RANGE_P2P of any linked ground repeater (TKT-59)
            for rep in ground_repeaters:
                if rep.active and rep.downlink_active:
                    for peer in living:
                        if peer.id not in visited:
                            d_peer = float(np.linalg.norm(peer.pos - rep.pos))
                            occluded_rep_peer = is_rf_occluded(rep.pos, peer.pos, urban_obstacles)
                            dropped_peer = occluded_rep_peer or self.is_packet_dropped(d_peer, is_hardline=False, max_range=RANGE_P2P)
                            if d_peer <= RANGE_P2P and not dropped_peer:
                                peer.downlink_active = True
                                peer.gateway_id = rep.carrier_id if rep.carrier_id is not None else 0
                                visited.add(peer.id)

            # 3. Yip-Lost State & Heartbeat / RSSI
            for d in living:
                d.yip_lost = bool(d.uplink_active and not d.downlink_active)

                d_gcs = float(np.linalg.norm(d.pos - tether_anchor))
                if d_gcs <= 50.0:
                    d.rssi, d.link_quality = -35.0, 1.0
                else:
                    min_dist = d_gcs
                    for peer in living:
                        if peer.id != d.id:
                            min_dist = min(min_dist, float(np.linalg.norm(d.pos - peer.pos)))
                    d.rssi, d.link_quality = self.calculate_rssi(min_dist, is_hardline=False)

                if broadcast_tick:
                    if d.downlink_active or d.uplink_active:
                        d.last_heartbeat_time = sim_time
                        d.lost_link = False
                        d.gcs_heartbeat_watchdog = 2.0
                        d.lkp = np.copy(d.pos)
                        if d.status == "LOST_LINK" and d.flight_state not in ("LANDED", "GROUND_IDLE"):
                            d.status = "ACTIVE"
                if sim_time - d.last_heartbeat_time > LINK_TIMEOUT:
                    d.lost_link = True
                    if d.flight_state not in ("LANDED", "GROUND_IDLE"):
                        d.status = "LOST_LINK"

        elif comms_mode == "TETHERED_RELAY":
            # Primary leader on hardline umbilical
            for drone in living:
                if drone.id == 0 or drone.is_leader:
                    if not tether_severed:
                        drone.rssi, drone.link_quality = self.calculate_rssi(0.0, is_hardline=True)
                        drone.uplink_active = True
                        drone.downlink_active = True
                        drone.yip_lost = False
                        drone.gateway_id = None
                        drone.lost_link = False
                        drone.gcs_heartbeat_watchdog = 2.0
                        drone.last_heartbeat_time = sim_time
                        if broadcast_tick:
                            drone.lkp = np.copy(drone.pos)
                        if drone.status == "LOST_LINK":
                            drone.status = "FLEET_LEADER"
                    else:
                        d_gcs = float(np.linalg.norm(drone.pos - tether_anchor))
                        if d_gcs <= 50.0:
                            drone.rssi, drone.link_quality = -35.0, 1.0
                            drone.uplink_active = True
                            drone.downlink_active = True
                        else:
                            drone.rssi, drone.link_quality = self.calculate_rssi(d_gcs, is_hardline=False)
                            dropped_up = self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_HOWL)
                            r_down = RANGE_BORK if getattr(drone, "rf_power_mode", None) == RF_MODE_BORK else RANGE_BARK
                            dropped_down = self.is_packet_dropped(d_gcs, is_hardline=False, max_range=r_down)
                            drone.uplink_active = (d_gcs <= RANGE_HOWL and not dropped_up)
                            drone.downlink_active = (d_gcs <= r_down and not dropped_down)
                        drone.yip_lost = bool(drone.uplink_active and not drone.downlink_active)
                        drone.gateway_id = None
                        if broadcast_tick and drone.uplink_active:
                            drone.last_heartbeat_time = sim_time
                            drone.lost_link = False
                            drone.gcs_heartbeat_watchdog = 2.0
                            drone.lkp = np.copy(drone.pos)
                            if drone.status == "LOST_LINK" and drone.flight_state not in ("LANDED", "GROUND_IDLE"):
                                drone.status = "FLEET_LEADER"
                        if sim_time - drone.last_heartbeat_time > LINK_TIMEOUT:
                            drone.lost_link = True
                            if drone.flight_state not in ("LANDED", "GROUND_IDLE"):
                                drone.status = "LOST_LINK"
                else:
                    # Escort in Tethered Relay
                    d_gcs = float(np.linalg.norm(drone.pos - tether_anchor))
                    dist_to_leader = float(np.linalg.norm(drone.pos - current_leader.pos)) if current_leader else 999.0
                    if d_gcs <= 50.0:
                        drone.rssi, drone.link_quality = -35.0, 1.0
                        drone.uplink_active = True
                        drone.downlink_active = True
                        drone.gateway_id = None
                    else:
                        dropped_up = self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_HOWL)
                        drone.uplink_active = (d_gcs <= RANGE_HOWL and not dropped_up)
                        drone.rssi, drone.link_quality = self.calculate_rssi(min(d_gcs, dist_to_leader), is_hardline=False)

                        dropped_lead = self.is_packet_dropped(dist_to_leader, is_hardline=False, max_range=RANGE_YIP)
                        if d_gcs <= RANGE_YIP and not self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_YIP):
                            drone.downlink_active = True
                            drone.gateway_id = None
                        elif current_leader and current_leader.downlink_active and dist_to_leader <= RANGE_YIP and not dropped_lead:
                            drone.downlink_active = True
                            drone.gateway_id = current_leader.id
                        else:
                            drone.downlink_active = False
                            drone.gateway_id = None

                    drone.yip_lost = bool(drone.uplink_active and not drone.downlink_active)

                    if broadcast_tick:
                        in_range_lead = (dist_to_leader <= RANGE_BARK and not self.is_packet_dropped(dist_to_leader, is_hardline=False, max_range=RANGE_BARK))
                        if current_leader and not current_leader.lost_link and in_range_lead:
                            drone.lkp = np.copy(drone.pos)
                            drone.last_heartbeat_time = sim_time
                            drone.lost_link = False
                            drone.gcs_heartbeat_watchdog = 2.0
                            if drone.status == "LOST_LINK" and drone.flight_state not in ("LANDED", "GROUND_IDLE"):
                                drone.status = "ACTIVE"
                    if sim_time - drone.last_heartbeat_time > LINK_TIMEOUT:
                        drone.lost_link = True
                        if drone.flight_state not in ("LANDED", "GROUND_IDLE"):
                            drone.status = "LOST_LINK"

        elif comms_mode == "LEADER_FOLLOWER":
            # Wireless Star
            for drone in living:
                d_gcs = float(np.linalg.norm(drone.pos - tether_anchor))
                occluded_gcs = is_rf_occluded(tether_anchor, drone.pos, urban_obstacles)
                dropped_up = occluded_gcs or self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_HOWL)
                drone.uplink_active = (d_gcs <= RANGE_HOWL and not dropped_up)

                if drone.is_leader:
                    drone.rssi, drone.link_quality = self.calculate_rssi(d_gcs, is_hardline=False)
                    r_down = RANGE_BORK if getattr(drone, "rf_power_mode", None) == RF_MODE_BORK else RANGE_BARK
                    dropped_down = occluded_gcs or self.is_packet_dropped(d_gcs, is_hardline=False, max_range=r_down)
                    drone.downlink_active = (d_gcs <= r_down and not dropped_down)
                    drone.gateway_id = None
                else:
                    dist_to_leader = float(np.linalg.norm(drone.pos - current_leader.pos)) if current_leader else 999.0
                    occluded_lead = is_rf_occluded(current_leader.pos, drone.pos, urban_obstacles) if current_leader else True
                    drone.rssi, drone.link_quality = self.calculate_rssi(min(d_gcs, dist_to_leader), is_hardline=False)

                    dropped_lead = occluded_lead or self.is_packet_dropped(dist_to_leader, is_hardline=False, max_range=RANGE_YIP)
                    if d_gcs <= RANGE_YIP and not occluded_gcs and not self.is_packet_dropped(d_gcs, is_hardline=False, max_range=RANGE_YIP):
                        drone.downlink_active = True
                        drone.gateway_id = None
                    elif current_leader and current_leader.downlink_active and dist_to_leader <= RANGE_YIP and not dropped_lead:
                        drone.downlink_active = True
                        drone.gateway_id = current_leader.id
                    else:
                        drone.downlink_active = False
                        drone.gateway_id = None
                        if ground_repeaters:
                            for rep in ground_repeaters:
                                if rep.active and rep.downlink_active:
                                    d_rep = float(np.linalg.norm(drone.pos - rep.pos))
                                    occluded_rep = is_rf_occluded(rep.pos, drone.pos, urban_obstacles)
                                    if d_rep <= RANGE_P2P and not occluded_rep and not self.is_packet_dropped(d_rep, is_hardline=False, max_range=RANGE_P2P):
                                        drone.downlink_active = True
                                        drone.gateway_id = rep.carrier_id if rep.carrier_id is not None else 0
                                        break

                drone.yip_lost = bool(drone.uplink_active and not drone.downlink_active)

            # Pass 2: In-Range Flagship Gateway Relay: if leader has no direct GCS link, check living escorts (BUG-06)
            if current_leader and not current_leader.uplink_active:
                for peer in living:
                    if peer.id != current_leader.id and peer.uplink_active:
                        dist_to_peer = float(np.linalg.norm(current_leader.pos - peer.pos))
                        peer_r = RANGE_BORK if getattr(peer, "rf_power_mode", None) == RF_MODE_BORK else RANGE_BARK
                        occluded_peer = is_rf_occluded(peer.pos, current_leader.pos, urban_obstacles)
                        if dist_to_peer <= peer_r and not occluded_peer and not self.is_packet_dropped(dist_to_peer, is_hardline=False, max_range=peer_r):
                            current_leader.uplink_active = True
                            current_leader.gateway_id = peer.id
                            current_leader.downlink_active = True
                            current_leader.yip_lost = False
                            break

            # Pass 3: Heartbeat updates & lost link flags
            for drone in living:
                if drone.is_leader:
                    if broadcast_tick and drone.uplink_active:
                        drone.last_heartbeat_time = sim_time
                        drone.lost_link = False
                        drone.gcs_heartbeat_watchdog = 2.0
                        drone.lkp = np.copy(drone.pos)
                        if drone.status == "LOST_LINK" and drone.flight_state not in ("LANDED", "GROUND_IDLE"):
                            drone.status = "FLEET_LEADER"
                    if sim_time - drone.last_heartbeat_time > LINK_TIMEOUT:
                        drone.lost_link = True
                        if drone.flight_state not in ("LANDED", "GROUND_IDLE"):
                            drone.status = "LOST_LINK"
                else:
                    if broadcast_tick:
                        dist_to_leader = float(np.linalg.norm(drone.pos - current_leader.pos)) if current_leader else 999.0
                        occluded_lead = is_rf_occluded(current_leader.pos, drone.pos, urban_obstacles) if current_leader else True
                        in_range_lead = (
                            dist_to_leader <= RANGE_BARK
                            and not occluded_lead
                            and not self.is_packet_dropped(dist_to_leader, is_hardline=False, max_range=RANGE_BARK)
                        )
                        if current_leader and not current_leader.lost_link and in_range_lead:
                            drone.lkp = np.copy(drone.pos)
                            drone.last_heartbeat_time = sim_time
                            drone.lost_link = False
                            drone.gcs_heartbeat_watchdog = 2.0
                            if drone.status == "LOST_LINK" and drone.flight_state not in ("LANDED", "GROUND_IDLE"):
                                drone.status = "ACTIVE"
                    if sim_time - drone.last_heartbeat_time > LINK_TIMEOUT:
                        drone.lost_link = True
                        if drone.flight_state not in ("LANDED", "GROUND_IDLE"):
                            drone.status = "LOST_LINK"

        # Update Howl RF Beacon tracking for GNSS-denied airframes across all modes (TKT-52)
        for d in living:
            if getattr(d, "gnss_denied", False):
                d_gcs = float(np.linalg.norm(d.pos - tether_anchor))
                howl_rssi = float(np.clip(-20.0 - 20.0 * math.log10(max(1.0, d_gcs)), -100.0, -20.0))
                d.rssi = round(howl_rssi, 1)
                d.link_quality = max(0.0, min(1.0, (d.rssi - (-100.0)) / 60.0))
