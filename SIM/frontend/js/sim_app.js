/**
 * SwarmTech3_UpRising: SITL God-View Three.js Viewport (sim_app.js)
 * Omniscient Simulation Lab Viewport:
 * Unfiltered ground truth physics, Zone 4 ghost collision spheres (1.5m),
 * 120m RF horizon dome, physical tether catenary, and trajectory traces.
 */

(function () {
    'use strict';

    const COLORS = {
        bg: 0x03070d,
        grid: 0x22052b,
        gridHighlight: 0xFF00EA,
        ring25: 0xFF00EA,
        ring50: 0xFF00EA,
        ring100: 0x9D00FF,
        ring120: 0xFF0055,
        leader: 0xFFB800,
        escort: 0xFF00EA,
        crashed: 0xFF2A55,
        tether: 0xFF00EA,
        ghostNormal: 0xFF00EA,
        ghostWarn: 0xFFB800,
        ghostClash: 0xFF2A55,
        rfDomeHowl: 0x00F0FF,
        rfDomeBark: 0xFFB800,
        rfDomeYip: 0x0088FF,
        target: 0x00FFAA,
        cmdTarget: 0x00F0FF,
    };

    let scene, camera, renderer, controls;
    let container;
    let dronesMap = new Map();
    let tetherMesh = null;
    let rfDome = null;
    let targetBeacon = null;
    let cmdTargetBeacon = null;
    let isChaseCam = false;
    let targetDroneId = 0;
    let fpsCounter = 0;
    let lastFpsUpdate = 0;
    let urbanObstacleMeshes = new Map();

    function init() {
        container = document.getElementById('viewport-container');
        if (!container) return;

        // 1. Scene & Cyber Fog
        scene = new THREE.Scene();
        scene.background = new THREE.Color(COLORS.bg);
        scene.fog = new THREE.FogExp2(COLORS.bg, 0.003);

        // 2. Camera Setup (Z-Up)
        camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.5, 1500);
        camera.up.set(0, 0, 1);
        camera.position.set(85, -85, 75);

        // 3. WebGL Renderer
        renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        container.appendChild(renderer.domElement);

        // 4. OrbitControls
        if (typeof THREE.OrbitControls !== 'undefined') {
            controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            controls.dampingFactor = 0.08;
            controls.maxPolarAngle = Math.PI / 2 - 0.02;
            controls.minDistance = 5;
            controls.maxDistance = 500;
            controls.target.set(0, 0, 15);
        }

        // 5. Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
        scene.add(ambientLight);

        const dirLight = new THREE.DirectionalLight(0xfff0ff, 0.85);
        dirLight.position.set(60, -40, 100);
        scene.add(dirLight);

        // 6. World Grid & Range Rings
        setupEnvironment();

        // 7. RF Horizon Bubble (120m Dome)
        setupRfHorizonDome();

        // 8. Target Beacons
        setupTargetBeacons();

        // 9. Resize & Key Handlers
        window.addEventListener('resize', onWindowResize);
        window.addEventListener('keydown', onKeyDown);

        // 10. Start Animation Loop
        requestAnimationFrame(animate);
    }

    function setupEnvironment() {
        // Magenta Coordinate Grid (300m x 300m)
        const gridHelper = new THREE.GridHelper(300, 60, COLORS.gridHighlight, COLORS.grid);
        gridHelper.rotation.x = Math.PI / 2;
        gridHelper.position.z = 0;
        scene.add(gridHelper);

        // Range Rings (Canine Acoustic Taxonomy: 50m Yip, 60m P2P, 90m Bark, 140m Howl)
        const rings = [
            { r: 25, color: 0x3d1059, op: 0.3 },
            { r: 50, color: COLORS.rfDomeYip, op: 0.6 },    // 50m Yip Escort Downlink
            { r: 60, color: 0x00FFAA, op: 0.4 },            // 60m P2P Mesh limit
            { r: 90, color: COLORS.rfDomeBark, op: 0.7 },   // 90m Bark Flagship Downlink
            { r: 140, color: COLORS.rfDomeHowl, op: 0.85, dash: true }, // 140m Howl GCS Uplink
        ];

        rings.forEach(({ r, color, op, dash }) => {
            const curve = new THREE.EllipseCurve(0, 0, r, r, 0, 2 * Math.PI, false, 0);
            const points = curve.getPoints(96);
            const geom = new THREE.BufferGeometry().setFromPoints(
                points.map(p => new THREE.Vector3(p.x, p.y, 0.05))
            );
            const mat = new THREE.LineBasicMaterial({
                color: color,
                transparent: true,
                opacity: op,
                linewidth: dash ? 2 : 1,
            });
            const ringLine = new THREE.Line(geom, mat);
            scene.add(ringLine);
        });

        // Launch Pad Base (P0-P7)
        setupBasePads();
    }

    function setupBasePads() {
        const padGroup = new THREE.Group();
        padGroup.name = "base_pads";

        // Tether Winch Anchor at (0, 0, 0.1)
        const anchorGeom = new THREE.CylinderGeometry(0.8, 1.0, 0.3, 16);
        const anchorMat = new THREE.MeshStandardMaterial({
            color: 0x2a0438,
            metalness: 0.8,
            roughness: 0.3,
            emissive: 0xFF00EA,
            emissiveIntensity: 0.3,
        });
        const anchorMesh = new THREE.Mesh(anchorGeom, anchorMat);
        anchorMesh.rotation.x = Math.PI / 2;
        anchorMesh.position.set(0, 0, 0.15);
        padGroup.add(anchorMesh);

        scene.add(padGroup);
    }

    function setupRfHorizonDome() {
        const domeGroup = new THREE.Group();
        domeGroup.name = "rf_canine_domes";

        // 1. 140m GCS Howl Uplink Wireframe Dome (Outer Cyan)
        const howlGeom = new THREE.SphereGeometry(140, 32, 16, 0, Math.PI * 2, 0, Math.PI / 2);
        howlGeom.rotateX(Math.PI / 2);
        const howlMat = new THREE.MeshBasicMaterial({
            color: COLORS.rfDomeHowl,
            wireframe: true,
            transparent: true,
            opacity: 0.10,
        });
        const howlDome = new THREE.Mesh(howlGeom, howlMat);
        domeGroup.add(howlDome);

        // 2. 90m Flagship Bark Downlink Wireframe Dome (Mid Amber)
        const barkGeom = new THREE.SphereGeometry(90, 24, 12, 0, Math.PI * 2, 0, Math.PI / 2);
        barkGeom.rotateX(Math.PI / 2);
        const barkMat = new THREE.MeshBasicMaterial({
            color: COLORS.rfDomeBark,
            wireframe: true,
            transparent: true,
            opacity: 0.08,
        });
        const barkDome = new THREE.Mesh(barkGeom, barkMat);
        domeGroup.add(barkDome);

        // 3. 50m Escort Yip Downlink Wireframe Dome (Inner Blue)
        const yipGeom = new THREE.SphereGeometry(50, 20, 10, 0, Math.PI * 2, 0, Math.PI / 2);
        yipGeom.rotateX(Math.PI / 2);
        const yipMat = new THREE.MeshBasicMaterial({
            color: COLORS.rfDomeYip,
            wireframe: true,
            transparent: true,
            opacity: 0.08,
        });
        const yipDome = new THREE.Mesh(yipGeom, yipMat);
        domeGroup.add(yipDome);

        scene.add(domeGroup);
    }

    function setupTargetBeacons() {
        // Acknowledged Target Beacon
        const geom = new THREE.OctahedronGeometry(1.2, 0);
        const mat = new THREE.MeshBasicMaterial({ color: COLORS.target, wireframe: true });
        targetBeacon = new THREE.Mesh(geom, mat);
        targetBeacon.position.set(0, 0, 20);
        scene.add(targetBeacon);

        // Commanded Target Beacon
        const cmdGeom = new THREE.OctahedronGeometry(1.5, 0);
        const cmdMat = new THREE.MeshBasicMaterial({ color: COLORS.cmdTarget, wireframe: true });
        cmdTargetBeacon = new THREE.Mesh(cmdGeom, cmdMat);
        cmdTargetBeacon.position.set(0, 0, 20);
        scene.add(cmdTargetBeacon);
    }

    function createSimDroneBadge(droneId, isLeader, tailNumber) {
        const canvas = document.createElement('canvas');
        canvas.width = 256;
        canvas.height = 48;
        const texture = new THREE.CanvasTexture(canvas);
        texture.minFilter = THREE.LinearFilter;

        const material = new THREE.SpriteMaterial({
            map: texture,
            transparent: true,
            depthTest: false,
        });
        const sprite = new THREE.Sprite(material);
        sprite.scale.set(2.2, 0.42, 1.0);
        sprite.position.set(0, 0, 1.15);
        sprite.userData = { canvas, texture, droneId, lastKey: '' };
        updateSimBadge(sprite, droneId, isLeader, false, tailNumber);
        return sprite;
    }

    function updateSimBadge(badge, droneId, isLeader, isCrashed, tailNumber) {
        if (!badge || !badge.userData) return;
        const { canvas, texture } = badge.userData;
        const tail = tailNumber || `ALITHEIA-V${String(droneId + 1).padStart(2, '0')}`;
        const key = `${droneId}_${isLeader}_${isCrashed}_${tail}`;
        if (badge.userData.lastKey === key) return;
        badge.userData.lastKey = key;

        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        const col = isCrashed ? '#FF3366' : (isLeader ? '#FFB800' : '#00F0FF');
        // Sleek minimal tactical ID pill
        ctx.fillStyle = 'rgba(6, 16, 25, 0.82)';
        ctx.fillRect(4, 4, canvas.width - 8, canvas.height - 8);
        ctx.strokeStyle = col;
        ctx.lineWidth = 2;
        ctx.strokeRect(4, 4, canvas.width - 8, canvas.height - 8);

        ctx.fillStyle = col;
        ctx.font = 'bold 20px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        const label = isCrashed ? `${tail} [CRASH]` : tail;
        ctx.fillText(label, canvas.width / 2, canvas.height / 2);
        texture.needsUpdate = true;
    }

    function createDroneMesh(droneId, isLeader) {
        const droneGroup = new THREE.Group();
        droneGroup.name = `drone_${droneId}`;

        // 1. Central Carbon Hull
        const bodyGeom = new THREE.BoxGeometry(0.6, 0.6, 0.18);
        const bodyColor = isLeader ? COLORS.leader : COLORS.escort;
        const bodyMat = new THREE.MeshStandardMaterial({
            color: bodyColor,
            metalness: 0.6,
            roughness: 0.2,
            emissive: bodyColor,
            emissiveIntensity: 0.4,
        });
        const body = new THREE.Mesh(bodyGeom, bodyMat);
        droneGroup.add(body);

        // 2. Cross Arms (X-Frame, 1.5m Wingspan)
        const armGeom = new THREE.CylinderGeometry(0.04, 0.04, 1.5, 8);
        const armMat = new THREE.MeshStandardMaterial({ color: 0x333333, metalness: 0.8 });

        const arm1 = new THREE.Mesh(armGeom, armMat);
        arm1.rotation.z = Math.PI / 4;
        droneGroup.add(arm1);

        const arm2 = new THREE.Mesh(armGeom, armMat);
        arm2.rotation.z = -Math.PI / 4;
        droneGroup.add(arm2);

        // 3. Rotors (4 spinning discs)
        const rotorGeom = new THREE.CircleGeometry(0.28, 12);
        const rotorMat = new THREE.MeshBasicMaterial({
            color: bodyColor,
            transparent: true,
            opacity: 0.5,
            side: THREE.DoubleSide,
        });

        const rotors = [];
        const offsets = [
            [0.53, 0.53],
            [-0.53, 0.53],
            [-0.53, -0.53],
            [0.53, -0.53],
        ];

        offsets.forEach(([ox, oy]) => {
            const rotor = new THREE.Mesh(rotorGeom, rotorMat);
            rotor.position.set(ox, oy, 0.12);
            droneGroup.add(rotor);
            rotors.push(rotor);
        });

        // 4. Zone 4 Ghost Collision Sphere (R = 1.5m wireframe)
        // Prop clash occurs strictly at d <= 1.5m!
        const ghostGeom = new THREE.SphereGeometry(1.5, 16, 12);
        const ghostMat = new THREE.MeshBasicMaterial({
            color: COLORS.ghostNormal,
            wireframe: true,
            transparent: true,
            opacity: 0.22,
        });
        const ghostSphere = new THREE.Mesh(ghostGeom, ghostMat);
        droneGroup.add(ghostSphere);

        // 5. Perspective Ground Drop-Line
        const dropLineGeom = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(0, 0, -30),
        ]);
        const dropLineMat = new THREE.LineBasicMaterial({
            color: bodyColor,
            transparent: true,
            opacity: 0.4,
        });
        const dropLine = new THREE.Line(dropLineGeom, dropLineMat);
        droneGroup.add(dropLine);

        // 6. 3D Billboard Tactical ID Badge (BUG-09)
        const badge = createSimDroneBadge(droneId, isLeader, null);
        droneGroup.add(badge);

        scene.add(droneGroup);

        return {
            badge: badge,
            group: droneGroup,
            body: body,
            bodyMat: bodyMat,
            rotors: rotors,
            ghostSphere: ghostSphere,
            ghostMat: ghostMat,
            dropLine: dropLine,
            dropLineGeom: dropLineGeom,
        };
    }

    function updateDrones(dronesList) {
        if (!Array.isArray(dronesList)) return;

        // Calculate minimum inter-drone distances to highlight Zone 4 / Zone 3
        const posMap = new Map();
        dronesList.forEach(d => {
            if (!d.crashed) posMap.set(d.id, d.pos);
        });

        const minDistanceMap = new Map();
        posMap.forEach((pos1, id1) => {
            let minD = 999.0;
            posMap.forEach((pos2, id2) => {
                if (id1 !== id2) {
                    const dx = pos1[0] - pos2[0];
                    const dy = pos1[1] - pos2[1];
                    const dz = pos1[2] - pos2[2];
                    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
                    if (dist < minD) minD = dist;
                }
            });
            minDistanceMap.set(id1, minD);
        });

        const activeIds = new Set();

        dronesList.forEach(d => {
            activeIds.add(d.id);
            let dObj = dronesMap.get(d.id);

            if (!dObj) {
                dObj = createDroneMesh(d.id, d.is_leader);
                dronesMap.set(d.id, dObj);
            }

            // Position & Heading
            dObj.group.position.set(d.pos[0], d.pos[1], d.pos[2]);
            if (d.heading !== undefined) {
                dObj.group.rotation.z = d.heading;
            }

            // Update ID Badge (BUG-09)
            updateSimBadge(dObj.badge, d.id, d.is_leader, d.crashed, d.tail_number);

            // Rotor Spin
            if (!d.crashed) {
                const rotSpeed = 0.35;
                dObj.rotors.forEach(r => (r.rotation.z += rotSpeed));
            }

            // Drop-Line
            const groundZ = -d.pos[2];
            dObj.dropLineGeom.setFromPoints([
                new THREE.Vector3(0, 0, 0),
                new THREE.Vector3(0, 0, groundZ),
            ]);

            // Visual Status & Ghost Collision Sphere
            const minD = minDistanceMap.get(d.id) || 999.0;

            if (d.crashed) {
                dObj.bodyMat.color.setHex(COLORS.crashed);
                dObj.bodyMat.emissive.setHex(COLORS.crashed);
                dObj.ghostMat.color.setHex(COLORS.crashed);
                dObj.ghostMat.opacity = 0.08;
            } else if (minD <= 1.5) {
                // ZONE 4 CLASH VIOLATION!
                dObj.ghostMat.color.setHex(COLORS.ghostClash);
                dObj.ghostMat.opacity = 0.85;
            } else if (minD <= 3.5) {
                // ZONE 3 MOMENTUM DAMPING ZONE
                dObj.ghostMat.color.setHex(COLORS.ghostWarn);
                dObj.ghostMat.opacity = 0.50;
            } else {
                // NOMINAL
                const col = d.is_leader ? COLORS.leader : COLORS.escort;
                dObj.bodyMat.color.setHex(col);
                dObj.bodyMat.emissive.setHex(col);
                dObj.ghostMat.color.setHex(COLORS.ghostNormal);
                dObj.ghostMat.opacity = 0.20;
            }
        });

        // Clean up purged drones
        for (const [id, dObj] of dronesMap.entries()) {
            if (!activeIds.has(id)) {
                if (dObj.badge && dObj.badge.material) {
                    if (dObj.badge.material.map) dObj.badge.material.map.dispose();
                    dObj.badge.material.dispose();
                }
                scene.remove(dObj.group);
                dronesMap.delete(id);
            }
        }

        // Chase Cam
        if (isChaseCam) {
            const targetDrone = dronesMap.get(targetDroneId);
            if (targetDrone && controls) {
                controls.target.copy(targetDrone.group.position);
            }
        }
    }

    function updateTether(tetherData, leaderPos) {
        if (!tetherData || !tetherData.visible || tetherData.severed || !leaderPos) {
            if (tetherMesh) {
                scene.remove(tetherMesh);
                tetherMesh = null;
            }
            return;
        }

        const anchor = new THREE.Vector3(
            tetherData.anchor[0],
            tetherData.anchor[1],
            tetherData.anchor[2]
        );
        const leader = new THREE.Vector3(leaderPos[0], leaderPos[1], leaderPos[2]);

        // 3D Catenary Curve
        const midPoint = new THREE.Vector3().addVectors(anchor, leader).multiplyScalar(0.5);
        const dist = anchor.distanceTo(leader);
        const sag = Math.min(dist * 0.12, 4.0);
        midPoint.z = Math.max(0.1, midPoint.z - sag);

        const curve = new THREE.QuadraticBezierCurve3(anchor, midPoint, leader);
        const points = curve.getPoints(24);
        const geom = new THREE.BufferGeometry().setFromPoints(points);

        if (!tetherMesh) {
            const mat = new THREE.LineBasicMaterial({
                color: COLORS.tether,
                linewidth: 2,
                transparent: true,
                opacity: 0.85,
            });
            tetherMesh = new THREE.Line(geom, mat);
            scene.add(tetherMesh);
        } else {
            tetherMesh.geometry.dispose();
            tetherMesh.geometry = geom;
        }
    }

    function updateUrbanObstacles(obstacles) {
        if (!scene) return;
        const activeIds = new Set();
        if (Array.isArray(obstacles)) {
            obstacles.forEach(obs => {
                activeIds.add(obs.id);
                let entry = urbanObstacleMeshes.get(obs.id);
                if (!entry) {
                    const group = new THREE.Group();
                    const bounds = obs.bounds; // [x_min, x_max, y_min, y_max, z_floor, z_ceiling]
                    const width = bounds[1] - bounds[0];   // dx (X axis)
                    const depth = bounds[3] - bounds[2];   // dy (Y axis)
                    const height = bounds[5] - bounds[4];  // dz (Z axis)

                    const cx = (bounds[0] + bounds[1]) / 2;
                    const cy = (bounds[2] + bounds[3]) / 2;
                    const cz = (bounds[4] + bounds[5]) / 2;

                    const hexColor = parseInt((obs.color || "#00F0FF").replace("#", "0x"), 16);

                    // 1. Translucent Shaded Box Body
                    const boxGeo = new THREE.BoxGeometry(width, depth, height);
                    const boxMat = new THREE.MeshStandardMaterial({
                        color: hexColor,
                        emissive: hexColor,
                        emissiveIntensity: 0.12,
                        roughness: 0.3,
                        metalness: 0.7,
                        transparent: true,
                        opacity: obs.obstacle_type === "SKYBRIDGE" ? 0.45 : 0.22,
                        side: THREE.DoubleSide,
                    });
                    const boxMesh = new THREE.Mesh(boxGeo, boxMat);
                    group.add(boxMesh);

                    // 2. High-Tech Holographic Wireframe Edges
                    const edgesGeo = new THREE.EdgesGeometry(boxGeo);
                    const edgesMat = new THREE.LineBasicMaterial({
                        color: hexColor,
                        linewidth: 2,
                        transparent: true,
                        opacity: 0.85,
                    });
                    const edgesLine = new THREE.LineSegments(edgesGeo, edgesMat);
                    group.add(edgesLine);

                    // 3. Highlighted Top Landing Deck for Landable Blocks
                    if (obs.is_landable) {
                        const deckGeo = new THREE.PlaneGeometry(width * 0.96, depth * 0.96);
                        const deckMat = new THREE.MeshBasicMaterial({
                            color: hexColor,
                            transparent: true,
                            opacity: 0.35,
                            side: THREE.DoubleSide,
                        });
                        const deckMesh = new THREE.Mesh(deckGeo, deckMat);
                        deckMesh.position.set(0, 0, height / 2 + 0.02);
                        group.add(deckMesh);
                    }

                    // 4. Tactical Elevation Billboard Label
                    const canvas = document.createElement('canvas');
                    canvas.width = 256;
                    canvas.height = 64;
                    const ctx = canvas.getContext('2d');
                    ctx.fillStyle = obs.color || '#00F0FF';
                    ctx.font = 'bold 22px monospace';
                    ctx.textAlign = 'center';
                    ctx.textBaseline = 'middle';
                    const elevStr = obs.obstacle_type === 'SKYBRIDGE' ? `Z:${bounds[4]}-${bounds[5]}m` : `+${bounds[5].toFixed(1)}m`;
                    ctx.fillText(`${obs.name || obs.id} [${elevStr}]`, 128, 32);
                    const texture = new THREE.CanvasTexture(canvas);
                    const spriteMat = new THREE.SpriteMaterial({ map: texture, transparent: true, opacity: 0.9 });
                    const sprite = new THREE.Sprite(spriteMat);
                    sprite.scale.set(7.0, 1.75, 1.0);
                    sprite.position.set(0, 0, height / 2 + 2.0);
                    group.add(sprite);

                    // Position group at center of AABB
                    group.position.set(cx, cy, cz);
                    scene.add(group);

                    entry = { group, boxMesh, edgesLine, obs };
                    urbanObstacleMeshes.set(obs.id, entry);
                }
            });
        }

        // Clean up removed obstacles
        for (const [id, entry] of urbanObstacleMeshes.entries()) {
            if (!activeIds.has(id)) {
                entry.group.traverse(child => {
                    if (child.geometry) child.geometry.dispose();
                    if (child.material) {
                        if (child.material.map) child.material.map.dispose();
                        child.material.dispose();
                    }
                });
                scene.remove(entry.group);
                urbanObstacleMeshes.delete(id);
            }
        }
    }

    function onTelemetryFrame(frame) {
        if (!frame) return;

        // Drones (Omniscient God-View Truth)
        const droneList = frame.sim_drones || frame.drones;
        if (droneList) {
            updateDrones(droneList);

            // Find leader pos for tether
            const leader = droneList.find(d => d.is_leader && !d.crashed);
            if (frame.tether && leader) {
                updateTether(frame.tether, leader.pos);
            } else if (frame.tether) {
                updateTether(frame.tether, null);
            }
        }

        // 3D Urban Obstacles & Elevation Shelves (TKT-67)
        if (frame.urban_obstacles) {
            updateUrbanObstacles(frame.urban_obstacles);
        }

        // Target Beacons
        if (frame.acknowledged_target && targetBeacon) {
            targetBeacon.position.set(
                frame.acknowledged_target[0],
                frame.acknowledged_target[1],
                frame.acknowledged_target[2]
            );
            targetBeacon.rotation.y += 0.02;
        }

        if (frame.commanded_target && cmdTargetBeacon) {
            cmdTargetBeacon.position.set(
                frame.commanded_target[0],
                frame.commanded_target[1],
                frame.commanded_target[2]
            );
            cmdTargetBeacon.rotation.y -= 0.015;
        }
    }

    function animate(time) {
        requestAnimationFrame(animate);

        if (controls) controls.update();

        // FPS Calculation
        fpsCounter++;
        if (time - lastFpsUpdate >= 1000) {
            const elFps = document.getElementById('hud-fps');
            if (elFps) elFps.textContent = `${fpsCounter} FPS`;
            fpsCounter = 0;
            lastFpsUpdate = time;
        }

        renderer.render(scene, camera);
    }

    function onWindowResize() {
        if (!camera || !renderer) return;
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    }

    function onKeyDown(e) {
        if (e.key === 'c' || e.key === 'C') {
            isChaseCam = !isChaseCam;
            console.log(`[SIM] Chase Cam: ${isChaseCam ? 'ENABLED' : 'DISABLED'}`);
        } else if (e.key === 'r' || e.key === 'R') {
            if (camera && controls) {
                camera.position.set(85, -85, 75);
                controls.target.set(0, 0, 15);
                isChaseCam = false;
            }
        }
    }

    // Initialize when client is ready
    window.addEventListener('DOMContentLoaded', () => {
        init();
        if (window.swarmClient) {
            window.swarmClient.on('telemetry', onTelemetryFrame);
        }
    });
})();
