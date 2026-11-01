# 📘 AlitheiaHive: Real-World Hardware Deployment Blueprint
## The Zero-Fluff Physical Swarm Guide: From Bench to Flight

---

> **Purpose of this document**:  
> Whenever your head hurts, you feel lost in simulation physics, or you wonder *"How does this actually work on real drones with real hardware?"* — **read this document**.  
> It strips away all simulation math and describes the exact, 4-wire physical reality.
> 
> > [!NOTE]
> > **Evolutionary Roadmap Alignment (Phase 4: Real Stuff / HW-01)**:  
> > While Phases 2 and 3 focus on 60 FPS FC twin kinematics, 4-zone safety, and tactical gamified missions, this blueprint defines the physical hardware bridge when flashing real companion computers and binding physical airframes.

---

## 1. The Core Vision (The 4 Rules)

1. **Register Drones**: Drones boot, connect to Wi-Fi, and automatically show up as visual cards on your GCS dashboard.
2. **Identical Code on Every Drone**: Every companion computer (OBC) runs the exact same software.
3. **Click & Command Safely**: You select a formation (e.g. *Echelon*), click a target on a 3D map, and the drones coordinate safely.
4. **Safe Automated Landing**: You click `[LAND ALL]`, and they softly touch down and disarm.

---

## 2. Minimalist Physical Bill of Materials (BOM)

To build a functional 2-to-4 drone autonomous swarm, you need:

| Component | Example / Spec | Approximate Cost | Purpose |
| :--- | :--- | :--- | :--- |
| **Airframes** | 2× Small hobby GPS drones (BetaFPV Pavo, SpeedyBee, or custom 3" quad running **ArduPilot** or **INAV**) | Existing hobby hardware | Physical flight, motor stabilization, and GPS position hold. |
| **Companion OBC** | 2× **Seeed Studio XIAO C5** (or ESP32-C3 / Raspberry Pi Zero 2W) with stubby antenna | ~$5 - $8 each | The "Swarm Brain". Translates between Wi-Fi and Flight Controller serial. |
| **Network** | 1× Laptop with Mobile Hotspot enabled (2.4 GHz or 5 GHz Wi-Fi) | Free (existing laptop) | The wireless swarm bridge between your GCS and the drones. |
| **Centimeter GPS** | Free Community RTK Caster (e.g., [RTK2GO.com](http://rtk2go.com) or local NTRIP service) | Free | Drops GPS jitter from 3 meters down to **1.5 centimeters**. |
| **Operator GCS** | AlitheiaHive Tactical WebGL HUD (or standard **QGroundControl** / **Mission Planner**) | Free (this software) | The pilot screen where you see 3D cards and send mission intents. |

---

## 3. Physical Wiring (The 4 Wires)

You do **not** build sensors, gyros, or motor drivers. The drone's existing Flight Controller (FC) already has those.

You connect the XIAO companion computer to any spare UART port on the Flight Controller with **only 4 wires**:

```
FLIGHT CONTROLLER (Spare UART Pads)        XIAO C5 / ESP32 PINS
┌─────────────────────────────────┐        ┌───────────────────────┐
│ 5V Pad (Power from drone BEC)   │───────>│ VCC (5V In)           │
│ GND (Ground)                    │───────>│ GND                   │
│ UART TX (Telemetry Out)         │───────>│ RX  (Serial In)       │
│ UART RX (Command Setpoints In)  │───────>│ TX  (Serial Out)      │
└─────────────────────────────────┘        └───────────────────────┘
```

* **Physical Mounting**: Stick the postage-stamp-sized XIAO to the top plate of the drone frame with double-sided foam tape.  
* **Weight added**: Less than **3 grams**.

---

## 4. What Goes on the XIAO? (`drone_agent.py`)

Every drone runs the exact same ~120-line MicroPython script. It has **three simple jobs**:

1. **Connect to Wi-Fi Hotspot** on boot.
2. **Talk Serial to Flight Controller (115200 baud)**:
   * Reads current GPS position, altitude, and heading.
   * Sends target waypoints: `SET_POSITION_TARGET_LOCAL_NED(x, y, z)`.
   * Forwards incoming RTK correction bytes (`RTCM3`) straight into the GPS.
3. **Talk UDP to Laptop & Peers**:
   * Broadcasts its position at 10 Hz over UDP.
   * Listens for GCS mission intents (e.g. `FORMATION = ECHELON`).
   * Listens for Peer positions (Drone #2 listens to Drone #1).
   * Calculates its formation slot offset: `my_target = leader_pos + slot_offset`.

```
                  ┌──────────────────────────────┐
                  │    Laptop Hotspot (Wi-Fi)    │
                  │   AlitheiaHive GCS (8080)    │
                  │     MAVLink Port (14550)     │
                  └──────────────┬───────────────┘
                                 │
                 ┌───────────────┴───────────────┐
                 │ Wi-Fi UDP Packets             │
                 ▼                               ▼
       ┌───────────────────┐           ┌───────────────────┐
       │   XIAO C5 (OBC)   │           │   XIAO C5 (OBC)   │
       │     Drone #1      │◄─────────►│     Drone #2      │
       │    (Flagship)     │  P2P Mesh │     (Escort)      │
       └─────────┬─────────┘   Wi-Fi   └─────────┬─────────┘
                 │ 4-wire Serial                 │ 4-wire Serial
                 ▼                               ▼
       ┌───────────────────┐           ┌───────────────────┐
       │ Flight Controller │           │ Flight Controller │
       │  (ArduPilot/INAV) │           │  (ArduPilot/INAV) │
       │  Motors & Gyros   │           │  Motors & Gyros   │
       └───────────────────┘           └───────────────────┘
```

---

## 5. Step-by-Step Flight Day Procedure

When you are ready to fly:

### Step 1: Base Setup
1. Turn on **Mobile Hotspot** on your laptop (SSID: `AlitheiaSwarm`).
2. Start the AlitheiaHive server: `python run.py`.
3. Open your browser to `http://localhost:8080`.

### Step 2: Power Up
1. Place Drone #1 and Drone #2 on the grass $3\text{m}$ apart.
2. Plug in the flight batteries.
3. Within 3 seconds, both XIAOs connect to the laptop hotspot.
4. Your GCS dashboard chimes:
   * **`[DRONE #1: ONLINE]`** (Flagship, SysID 1)
   * **`[DRONE #2: ONLINE]`** (Escort, SysID 2)
   Both appear on your 3D cyberpunk map at their true GPS coordinates.

### Step 3: Automated Centimeter RTK
1. If your laptop has internet, AlitheiaHive pulls the free RTCM3 stream from RTK2GO.
2. It forwards these packets over Wi-Fi UDP to both drones.
3. The drones' GPS LEDs turn from blinking green to solid blue: **RTK FIXED** ($1.5\text{cm}$ precision).

### Step 4: One-Click Scramble
1. Click **`[ARM & SCRAMBLE]`** on the GCS.
2. Both drones spin up their propellers and ascend vertically to $Z = 15\text{m}$, holding stable station.

### Step 5: Autonomous Swarm Pursuit
1. Select **`[ECHELON]`** on the formation bar.
2. Click anywhere on the 3D map $50\text{m}$ away.
3. Drone #1 leads towards the target point.
4. Drone #2's OBC automatically maintains a $6\text{m}$ trailing wedge offset behind Drone #1, matching velocity and banking in sync.

### Step 6: Coordinated Soft Landing
1. Click **`[LAND ALL]`**.
2. Both drones descend smoothly at $-1.0\text{ m/s}$.
3. Flight controllers detect touchdown, instantly cut motor power, and disarm.

---

## 6. The Sanity Checklist (When You Feel Lost)

* **Do I need to code motor thrust or propeller physics?**  
  **NO.** The drone's flight controller (Betaflight / ArduPilot) does that with hardware gyros.
* **Do I need to emulate SPI or I2C sensor chips?**  
  **NO.** Real companion computers communicate via standard high-level UART serial messages.
* **Do I need expensive SIK radio modems?**  
  **NO.** Standard 2.4/5 GHz Wi-Fi easily covers $100\text{m}\dots 150\text{m}$ line-of-sight in an open field, which is plenty for a tactical swarm demonstration.
* **Why does AlitheiaHive support QGroundControl / Mission Planner?**  
  Because our backend outputs **MAVLink v2 on UDP port 14550**. You can connect standard, industry-standard ground control software to the exact same port anytime.
* **What is the simulator (`sitl_engine.py`) actually doing right now?**  
  It is a **visual mock**. When you don't have the 2 physical drones plugged in, `sitl_engine.py` pretends to be the 2 flight controllers so you can test your WebGL dashboard and mission planner on your sofa.

---

*This document is the absolute ground truth of how SwarmTech connects to the physical universe.*
