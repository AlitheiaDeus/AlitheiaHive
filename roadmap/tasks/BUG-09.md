# BUG-09: God-View Sim Viewport (/sim) Missing Drone ID Badges

> **Category**: 🐛 Bug | **Priority**: Normal | **State**: todo  
> **Operational Inspiration**: Sim viewport at `/sim` renders collision spheres and rotors but lacks drone ID badges

---

## 1. Operational Overview & Objectives

The SITL simulation server provides a God-View omniscient 3D viewport at `http://localhost:9000/sim` (`SIM/frontend/js/sim_app.js`).

**The Bug**:
- While the GCS tactical viewport (`/`) renders glowing 3D canvas billboard sprites above each drone displaying `D0`, `D1`, etc., the sim viewport (`/sim`) renders airframe geometry, rotors, and collision spheres without any vehicle identification tags.
- When observing swarm collisions, obstacle clashes, or formation flights in God-View, operators cannot distinguish which drone ID corresponds to which airframe in the 3D space.

**Objective**:
- Add canvas-based 2D/3D text billboard sprites above each drone in `SIM/frontend/js/sim_app.js`.
- Display clean alphanumeric IDs (`D0`, `D1`, ...) above the airframe meshes, updated at 60 FPS in Three.js.

---

## 2. Target Files & Architectural Scope

* `SIM/frontend/js/sim_app.js`: Implement `createDroneIdBadge(id)` using HTML5 canvas texture sprite, attach to drone mesh group, and update billboard position above airframe.

---

## 3. Implementation Blueprint

1. **Sprite Generator**:
   - In `sim_app.js`:
     ```javascript
     function createSimDroneBadge(droneId) {
         const canvas = document.createElement('canvas');
         canvas.width = 128;
         canvas.height = 64;
         const ctx = canvas.getContext('2d');
         ctx.fillStyle = 'rgba(10, 25, 38, 0.75)';
         ctx.fillRect(0, 0, 128, 64);
         ctx.strokeStyle = '#00f0ff';
         ctx.lineWidth = 2;
         ctx.strokeRect(2, 2, 124, 60);
         ctx.fillStyle = '#00f0ff';
         ctx.font = 'bold 28px "JetBrains Mono", monospace';
         ctx.textAlign = 'center';
         ctx.textBaseline = 'middle';
         ctx.fillText(`D${droneId}`, 64, 32);
         const texture = new THREE.CanvasTexture(canvas);
         const material = new THREE.SpriteMaterial({ map: texture, depthTest: false });
         const sprite = new THREE.Sprite(material);
         sprite.scale.set(1.5, 0.75, 1);
         sprite.position.set(0, 0, 1.2);
         return sprite;
     }
     ```
2. **Lifecycle Management**:
   - Add badge to `droneMeshes[id]` during initialization.
   - Synchronize position during telemetry animation ticks.

---

## 4. Acceptance Criteria & Invariants

* [x] Sim viewport at `/sim` displays readable `D0`, `D1`, ... text badges above each airframe.
* [x] Badges track airframe movement smoothly at 60 FPS without z-fighting or jitter.
* [x] Code passes syntax and lint validation without regressions.
