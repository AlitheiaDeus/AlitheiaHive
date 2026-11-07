# [TKT-51] Physical Courier / Search-and-Recall Rescue Protocol (Data Mule to LKP)

* **Grade**: ðŸš€ Feature | **Type**: Autonomous Mission / Search & Rescue (`Backlog`) | **Priority**: Low
* **Target Files**: `SIM/backend/sitl_engine.py`, `GCS/backend/c2_gateway.py`, `GCS/frontend/js/hud.js`, `tests/test_gossip_formations.py`
* **Technical Blueprint**:
  - **The Reality of RF Blackout & Fragile Chains**: Daisy-chain relays suffer from single-point-of-failure disconnects and require god-view placement. A real tactical operator sends a "Data Mule / Messenger Pigeon" when a swarm goes dark at LKP.
  - **Part 1: Courier Dispatch & Mission Payload**:
    - When swarm enters `[LOST_LINK]` beyond comms horizon, operator can launch a dedicated Courier drone (`DISPATCH_COURIER_LKP`).
    - Courier is loaded with a stored flight directive: `ORDER = "RTB_IMMEDIATELY"` and target coordinates `p_search = LKP`.
  - **Part 2: Autonomous Blind Transit to LKP**:
    - Courier leaves GCS coverage and navigates autonomously to the LKP coordinates.
    - GCS has NO omniscient tracking of the courier once it leaves GCS range (`COURIER: EN ROUTE TO LKP (RADIO SILENCE)`).
  - **Part 3: Search Pattern & P2P Contact Handshake**:
    - Upon arriving at LKP, courier flies an expanding search loiter (radius $20\text{m} \dots 50\text{m}$) broadcasting P2P rescue beacons.
    - If contact is made ($d \le 60\text{m}$ to any swarm peer):
      - Courier handshakes with swarm, delivers the stored `RTB_IMMEDIATELY` command.
      - Swarm breaks loiter and follows courier (or consensus returns) back toward Base.
  - **Part 4: MIA Timeout & Return**:
    - If no peer responds within a 30s search window, courier concludes fleet is destroyed/MIA.
    - Courier turns around and flies back to Base GCS alone, downlinking `[MISSION REPORT: SWARM MIA AT LKP]`.
    - If courier is destroyed in transit, GCS receives nothing â€” full comms realism preserved.
* **Targeted Test**: `python -m unittest tests/test_gossip_formations.py`
