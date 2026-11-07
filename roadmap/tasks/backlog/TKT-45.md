# [TKT-45] Cryptographic Comms Guard: MAVLink v2 HMAC Signing & EW Anti-Spoofing

* **Grade**: ðŸš€ Feature | **Type**: Feature / Cyber & Electronic Warfare (`Backlog`) | **Priority**: Medium
* **Target Files**: `SIM/backend/mavlink_bridge.py`, `OBC/backend/drone_firmware.py`, `GCS/frontend/js/hud.js`
* **Technical Blueprint**:
  - Cryptographic MAVLink v2 Packet Signing (`MAVLINK_IFLAG_SIGNED`):
    - 32-byte shared secret key + HMAC-SHA256 signature + 48-bit monotonic microsecond timestamp to defeat replay attacks.
    - Autopilot firmware drops unsigned or invalid signature packets with `AUTH_FAILURE`.
  - Electronic Warfare (EW) Adversary Spoofing Simulator in SIM Lab:
    - Button: `[INJECT ROGUE C2 PACKET]` broadcasts spoofed `SET_TARGET` or `LAND` packet over UDP.
    - Behavior in Unsigned mode: Swarm accepts spoofed waypoint and diverts course toward enemy target.
    - Behavior in Signed mode: Drones verify HMAC signature, reject rogue packet, and trigger GCS HUD flashing alert: `[EW ALERT] Spoofed packet rejected from unauthorized sysid`.
  - Gossip Mesh Peer Tokens: Node HMAC signatures prevent rogue/Sybil peers from poisoning consensus target coordinates.
* **Targeted Test**: `python -m unittest tests/test_mavlink_bridge.py`
