/**
 * SwarmTech3_UpRising: Three.js WebGL Tactical Viewport (app.js)
 * 100% GPU-accelerated 3D rendering (zero CPU software projection).
 * True 3D quadcopters, spinning rotors, perspective drop-lines,
 * concentric range rings, and dynamic glowing catenary tether.
 */

(function () {
    'use strict';

    // =========================================================================
    // Configuration & Visual Constants
    // =========================================================================
    const COLORS = {
        bg: 0x061019,
        grid: 0x0B2A3D,
        gridHighlight: 0x00F0FF,
        ring25: 0x00F0FF,
        ring50: 0x00F0FF,
        ring100: 0x0088AA,
        leader: 0xFFB800,
        escort: 0x00F0FF,
        crashed: 0xFF3366,
        propagating: 0xFFB800,
        tether: 0x00F0FF,
        commandedTarget: 0x00F0FF,
        target: 0x00FFAA,
        dropLine: 0x00F0FF,
    };

    let scene, camera, renderer, controls;
    let container;
    let dronesMap = new Map();
    let tetherMesh = null;
    let targetBeacon = null;
    let commandedBeacon = null;
    let stagedBeacon = null;
    let gcsMast = null;

    let isChaseCam = false;
    let targetDroneId = 0;
    let lastTelemetryTime = 0;
    let fpsCounter = 0;
    let lastFpsUpdate = 0;

    // =========================================================================
    // 1. Scene & Renderer Initialization
    // =========================================================================
    function init() {
        container = document.getElementById('viewport-container');

        // 1. Three.js Scene with Cyber Fog
        scene = new THREE.Scene();
        scene.background = new THREE.Color(COLORS.bg);
        scene.fog = new THREE.FogExp2(COLORS.bg, 0.0035);

        // 2. Camera Setup (Z-Up aerospace coordinate standard)
        camera = new THREE.PerspectiveCamera(
            55,
            window.innerWidth / window.innerHeight,
            0.5,
            1200
        );
        camera.up.set(0, 0, 1);
        camera.position.set(70, -70, 65);

        // 3. WebGL Renderer with High-DPI Support
        renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        renderer.toneMappingExposure = 1.2;
        container.appendChild(renderer.domElement);

        // 4. Orbit Controls
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.06;
        controls.screenSpacePanning = false;
        controls.minDistance = 5;
        controls.maxDistance = 500;
        controls.maxPolarAngle = Math.PI / 2 - 0.02; // Don't go below floor
        controls.target.set(30, 20, 15);

        // 5. Lighting
        const ambientLight = new THREE.AmbientLight(0x1B2C3D, 1.2);
        scene.add(ambientLight);

        const dirLight = new THREE.DirectionalLight(0xFFFFFF, 1.5);
        dirLight.position.set(60, 40, 100);
        scene.add(dirLight);

        const blueLight = new THREE.PointLight(0x00F0FF, 1.8, 150);
        blueLight.position.set(0, 0, 30);
        scene.add(blueLight);

        // 6. Build Tactical Environment
        createGroundGrid();
        createTacticalRangeRings();
        createGcsMast();
        createTargetBeacon();
        createCommandedBeacon();
        createStagedBeacon();
        createCatenaryTether();
        createRecoveryPads();
        createMissionRouteGroup();

        // 7. Event Listeners
        window.addEventListener('resize', onWindowResize);
        window.addEventListener('keydown', onKeyDown);
        renderer.domElement.addEventListener('pointerdown', onPointerDown);
        renderer.domElement.addEventListener('pointerup', onPointerUp);

        // 8. Connect WebSocket Telemetry
        if (window.swarmClient) {
            window.swarmClient.on('telemetry', onTelemetryFrame);
            window.swarmClient.on('status', onConnectionStatus);
        }

        animate();
    }

    // =========================================================================
    // 2. Tactical Ground Environment & Grid
    // =========================================================================
    function createGroundGrid() {
        // Grid on Z=0 plane (XY)
        const size = 300;
        const divisions = 60;
        const grid = new THREE.GridHelper(size, divisions, COLORS.gridHighlight, COLORS.grid);
        grid.rotation.x = Math.PI / 2;
        grid.position.set(0, 0, 0);
        scene.add(grid);
    }

    function createTacticalRangeRings() {
        // Canine Acoustic Taxonomy: 50m Yip, 60m P2P, 90m Bark, 140m Howl (TKT-49)
        const ringDefs = [
            { r: 25, color: 0x0088AA, dash: false, op: 0.4 },
            { r: 50, color: 0x0088FF, dash: false, op: 0.6 },  // 50m Yip Escort Downlink
            { r: 60, color: 0x00FFAA, dash: true, op: 0.45 },  // 60m P2P Mesh Gossip Limit
            { r: 90, color: 0xFFB800, dash: false, op: 0.75 }, // 90m Bark Flagship Downlink
            { r: 140, color: 0x00F0FF, dash: true, op: 0.9 },  // 140m Howl GCS Uplink Horizon
        ];
        ringDefs.forEach(({ r, color, dash, op }) => {
            const curve = new THREE.EllipseCurve(
                0, 0, r, r, 0, 2 * Math.PI, false, 0
            );
            const points = curve.getPoints(120);
            const geometry = new THREE.BufferGeometry().setFromPoints(
                points.map(p => new THREE.Vector3(p.x, p.y, 0.05))
            );
            const material = dash ? new THREE.LineDashedMaterial({
                color: color,
                dashSize: r === 140 ? 5 : 3,
                gapSize: r === 140 ? 3 : 2,
                linewidth: 1,
                transparent: true,
                opacity: op,
            }) : new THREE.LineBasicMaterial({
                color: color,
                transparent: true,
                opacity: op,
            });
            const line = new THREE.Line(geometry, material);
            if (dash) line.computeLineDistances();
            scene.add(line);
        });
    }

    function createGcsMast() {
        gcsMast = new THREE.Group();

        // Base pad
        const baseGeom = new THREE.CylinderGeometry(2.5, 3.0, 0.6, 16);
        baseGeom.rotateX(Math.PI / 2);
        const baseMat = new THREE.MeshStandardMaterial({
            color: 0x0A1C29,
            metalness: 0.8,
            roughness: 0.3,
        });
        const baseMesh = new THREE.Mesh(baseGeom, baseMat);
        baseMesh.position.set(0, 0, 0.3);
        gcsMast.add(baseMesh);

        // Vertical mast
        const mastGeom = new THREE.CylinderGeometry(0.15, 0.25, 1.5, 8);
        mastGeom.rotateX(Math.PI / 2);
        const mastMat = new THREE.MeshStandardMaterial({
            color: 0x5A7E96,
            metalness: 0.9,
            roughness: 0.2,
        });
        const mastMesh = new THREE.Mesh(mastGeom, mastMat);
        mastMesh.position.set(0, 0, 0.75);
        gcsMast.add(mastMesh);

        // Flashing anchor beacon
        const beaconGeom = new THREE.SphereGeometry(0.35, 16, 16);
        const beaconMat = new THREE.MeshBasicMaterial({
            color: COLORS.leader,
            transparent: true,
            opacity: 0.9,
        });
        const beaconMesh = new THREE.Mesh(beaconGeom, beaconMat);
        beaconMesh.position.set(0, 0, 1.5);
        gcsMast.add(beaconMesh);

        scene.add(gcsMast);
    }

    function createTargetBeacon() {
        targetBeacon = new THREE.Group();

        // 1. Solid rotating emerald diamond with subtle edge glow (TKT-26)
        const octGeom = new THREE.OctahedronGeometry(1.2, 0);
        const octMat = new THREE.MeshStandardMaterial({
            color: COLORS.target,
            emissive: COLORS.target,
            emissiveIntensity: 0.6,
            roughness: 0.25,
            metalness: 0.75,
            wireframe: false,
        });
        const diamond = new THREE.Mesh(octGeom, octMat);
        diamond.position.set(0, 0, 0);
        diamond.name = 'diamond';

        const wireGeom = new THREE.WireframeGeometry(octGeom);
        const wireMat = new THREE.LineBasicMaterial({ color: 0xFFFFFF, transparent: true, opacity: 0.4 });
        const wire = new THREE.LineSegments(wireGeom, wireMat);
        diamond.add(wire);
        targetBeacon.add(diamond);

        // 2. Vertical laser beam to ground (FIX-01: unit length, dynamically scaled to ground Z=0)
        const beamGeom = new THREE.CylinderGeometry(0.08, 0.08, 1.0, 8);
        beamGeom.rotateX(Math.PI / 2);
        const beamMat = new THREE.MeshBasicMaterial({
            color: COLORS.target,
            transparent: true,
            opacity: 0.35,
        });
        const beam = new THREE.Mesh(beamGeom, beamMat);
        beam.name = 'beam';
        beam.scale.set(1, 1, 30);
        beam.position.set(0, 0, -15);
        targetBeacon.add(beam);

        // 3. Ground pulse ring
        const ringGeom = new THREE.RingGeometry(1.8, 2.2, 32);
        const ringMat = new THREE.MeshBasicMaterial({
            color: COLORS.target,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.6,
        });
        const ring = new THREE.Mesh(ringGeom, ringMat);
        ring.name = 'ring';
        targetBeacon.add(ring);

        targetBeacon.position.set(0, 0, 20);
        scene.add(targetBeacon);
    }

    function createCommandedBeacon() {
        commandedBeacon = new THREE.Group();

        // 1. Hollow rotating octahedron wireframe (Cyan 0x00F0FF) (TKT-26)
        const octGeom = new THREE.OctahedronGeometry(1.65, 0);
        const octMat = new THREE.MeshBasicMaterial({
            color: COLORS.commandedTarget,
            wireframe: true,
            transparent: true,
            opacity: 0.85,
        });
        const diamond = new THREE.Mesh(octGeom, octMat);
        diamond.name = 'diamond';
        commandedBeacon.add(diamond);

        // 2. Tactical targeting reticle ring around the diamond
        const reticleGeom = new THREE.RingGeometry(1.8, 2.05, 24);
        const reticleMat = new THREE.MeshBasicMaterial({
            color: COLORS.commandedTarget,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.7,
        });
        const reticle = new THREE.Mesh(reticleGeom, reticleMat);
        reticle.name = 'reticle';
        commandedBeacon.add(reticle);

        // 3. Thin dashed drop-line to ground (visible when diverged from acknowledged target)
        const dropGeom = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(0, 0, 0),
        ]);
        const dropMat = new THREE.LineDashedMaterial({
            color: COLORS.commandedTarget,
            dashSize: 1.0,
            gapSize: 0.8,
            transparent: true,
            opacity: 0.5,
        });
        const dropLine = new THREE.Line(dropGeom, dropMat);
        dropLine.name = 'dropLine';
        dropLine.visible = false;
        commandedBeacon.add(dropLine);

        commandedBeacon.position.set(0, 0, 20);
        scene.add(commandedBeacon);
    }

    function createStagedBeacon() {
        stagedBeacon = new THREE.Group();
        stagedBeacon.name = 'stagedBeacon';
        stagedBeacon.visible = false;

        const colorGold = 0xFFB800;

        // 1. Rotating gold octahedron wireframe (TKT-68)
        const octGeom = new THREE.OctahedronGeometry(1.45, 0);
        const octMat = new THREE.MeshBasicMaterial({
            color: colorGold,
            wireframe: true,
            transparent: true,
            opacity: 0.9,
        });
        const diamond = new THREE.Mesh(octGeom, octMat);
        diamond.name = 'diamond';
        stagedBeacon.add(diamond);

        // 2. Tactical targeting reticle ring
        const reticleGeom = new THREE.RingGeometry(1.65, 1.9, 24);
        const reticleMat = new THREE.MeshBasicMaterial({
            color: colorGold,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.8,
        });
        const reticle = new THREE.Mesh(reticleGeom, reticleMat);
        reticle.name = 'reticle';
        stagedBeacon.add(reticle);

        // 3. Dashed vertical ground guide line
        const dropGeom = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(0, 0, -20),
        ]);
        const dropMat = new THREE.LineDashedMaterial({
            color: colorGold,
            dashSize: 0.8,
            gapSize: 0.5,
            transparent: true,
            opacity: 0.75,
        });
        const dropLine = new THREE.Line(dropGeom, dropMat);
        dropLine.name = 'dropLine';
        stagedBeacon.add(dropLine);

        // 4. Ground pulse ring
        const ringGeom = new THREE.RingGeometry(1.8, 2.2, 32);
        const ringMat = new THREE.MeshBasicMaterial({
            color: colorGold,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.65,
        });
        const ring = new THREE.Mesh(ringGeom, ringMat);
        ring.name = 'ring';
        stagedBeacon.add(ring);

        // 5. Tactical Billboard Label
        const canvas = document.createElement('canvas');
        canvas.width = 256;
        canvas.height = 64;
        const ctx = canvas.getContext('2d');
        ctx.fillStyle = '#FFB800';
        ctx.font = 'bold 20px monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('STAGED TARGET', 128, 22);
        ctx.font = 'bold 15px monospace';
        ctx.fillStyle = '#FFFFFF';
        ctx.fillText('[CLICK SEND]', 128, 44);
        const texture = new THREE.CanvasTexture(canvas);
        const spriteMat = new THREE.SpriteMaterial({ map: texture, transparent: true, opacity: 0.95 });
        const sprite = new THREE.Sprite(spriteMat);
        sprite.scale.set(6.0, 1.5, 1.0);
        sprite.position.set(0, 0, 2.5);
        stagedBeacon.add(sprite);

        stagedBeacon.position.set(0, 0, 20);
        scene.add(stagedBeacon);
    }

    function createCatenaryTether() {
        // Dynamic curve tube
        const curve = new THREE.LineCurve3(
            new THREE.Vector3(0, 0, 1.5),
            new THREE.Vector3(0, 0, 20)
        );
        const geom = new THREE.BufferGeometry().setFromPoints(curve.getPoints(16));
        const mat = new THREE.LineBasicMaterial({
            color: COLORS.tether,
            linewidth: 2,
            transparent: true,
            opacity: 0.85,
        });
        tetherMesh = new THREE.Line(geom, mat);
        scene.add(tetherMesh);
    }

    // =========================================================================
    // 2b. Tactical Ground Landing Pads & Air Traffic Control (TKT-19)
    // =========================================================================
    const landingPads = [];

    function createRecoveryPads() {
        const padsGroup = new THREE.Group();
        padsGroup.name = 'recovery_pads';

        for (let idx = 0; idx < 8; idx++) {
            const col = idx % 4;
            const row = Math.floor(idx / 4);
            const x = (col - 1.5) * 2.8;
            const y = -6.0 - row * 2.8;

            const padGroup = new THREE.Group();
            padGroup.position.set(x, y, 0);

            // 1. Dark Slate Octagonal Base Pad (0.08m height, placed at ground)
            const baseGeom = new THREE.CylinderGeometry(1.25, 1.35, 0.08, 8);
            baseGeom.rotateX(Math.PI / 2);
            const baseMat = new THREE.MeshStandardMaterial({
                color: 0x0A1824,
                metalness: 0.85,
                roughness: 0.35,
            });
            const baseMesh = new THREE.Mesh(baseGeom, baseMat);
            baseMesh.position.set(0, 0, 0.04);
            padGroup.add(baseMesh);

            // 2. Neon Cyan Octagonal Perimeter Ring
            const ringGeom = new THREE.RingGeometry(1.12, 1.22, 8);
            const ringMat = new THREE.MeshBasicMaterial({
                color: 0x00F0FF,
                side: THREE.DoubleSide,
                transparent: true,
                opacity: 0.65,
            });
            const ringMesh = new THREE.Mesh(ringGeom, ringMat);
            ringMesh.position.set(0, 0, 0.085);
            padGroup.add(ringMesh);

            // 3. Center Status Target Light & Decal (Vacant: Cyan, Approaching: Amber, Landed: Emerald)
            const centerGeom = new THREE.CylinderGeometry(0.38, 0.38, 0.03, 16);
            centerGeom.rotateX(Math.PI / 2);
            const centerMat = new THREE.MeshStandardMaterial({
                color: 0x00A0B0,
                emissive: 0x00A0B0,
                emissiveIntensity: 0.35,
                transparent: true,
                opacity: 0.6,
            });
            const centerMesh = new THREE.Mesh(centerGeom, centerMat);
            centerMesh.position.set(0, 0, 0.09);
            padGroup.add(centerMesh);

            // 4. Inner Concentric Ring
            const innerRingGeom = new THREE.RingGeometry(0.65, 0.72, 16);
            const innerRingMat = new THREE.MeshBasicMaterial({
                color: 0x00F0FF,
                side: THREE.DoubleSide,
                transparent: true,
                opacity: 0.35,
            });
            const innerRingMesh = new THREE.Mesh(innerRingGeom, innerRingMat);
            innerRingMesh.position.set(0, 0, 0.088);
            padGroup.add(innerRingMesh);

            // 5. Pad Designation Marker Label ("PAD 0" .. "PAD 7")
            const canvas = document.createElement('canvas');
            canvas.width = 128;
            canvas.height = 64;
            const ctx = canvas.getContext('2d');
            ctx.fillStyle = 'rgba(6, 16, 25, 0.8)';
            ctx.fillRect(0, 0, 128, 64);
            ctx.strokeStyle = '#00F0FF';
            ctx.lineWidth = 3;
            ctx.strokeRect(3, 3, 122, 58);
            ctx.fillStyle = '#00F0FF';
            ctx.font = 'bold 24px "JetBrains Mono", Consolas, monospace';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(`PAD ${idx}`, 64, 32);

            const labelTexture = new THREE.CanvasTexture(canvas);
            const labelSpriteMat = new THREE.SpriteMaterial({
                map: labelTexture,
                transparent: true,
                opacity: 0.85,
            });
            const labelSprite = new THREE.Sprite(labelSpriteMat);
            labelSprite.scale.set(1.2, 0.6, 1);
            labelSprite.position.set(0, -0.9, 0.35);
            padGroup.add(labelSprite);

            padsGroup.add(padGroup);

            landingPads.push({
                idx: idx,
                group: padGroup,
                baseMesh: baseMesh,
                ringMesh: ringMesh,
                centerMesh: centerMesh,
                labelSprite: labelSprite,
                status: 'VACANT',
            });
        }

        scene.add(padsGroup);
    }

    // 2c. Tactical Field Landing Zone (FLZ) Rendering (TKT-63 / PLN-05)
    let fieldLZGroup = null;
    function updateFieldLZ(frame) {
        if (!frame || !frame.field_lz || !frame.field_lz.active) {
            if (fieldLZGroup) fieldLZGroup.visible = false;
            return;
        }

        const flz = frame.field_lz;
        const coords = flz.coords || [40, 20, 0.05];

        if (!fieldLZGroup) {
            fieldLZGroup = new THREE.Group();
            fieldLZGroup.name = 'field_landing_zone';

            // 1. Tactical Outer Ring
            const ringGeo = new THREE.RingGeometry(11.8, 12.0, 48);
            const ringMat = new THREE.MeshBasicMaterial({
                color: 0xE6B800,
                side: THREE.DoubleSide,
                transparent: true,
                opacity: 0.85,
            });
            const ringMesh = new THREE.Mesh(ringGeo, ringMat);
            ringMesh.rotation.x = -Math.PI / 2;
            ringMesh.position.y = 0.04;
            fieldLZGroup.add(ringMesh);

            // 2. Tactical Inner Radial Crosshairs
            const crossGeo = new THREE.RingGeometry(0.2, 0.5, 16);
            const crossMat = new THREE.MeshBasicMaterial({ color: 0xFFD700, side: THREE.DoubleSide });
            const crossMesh = new THREE.Mesh(crossGeo, crossMat);
            crossMesh.rotation.x = -Math.PI / 2;
            crossMesh.position.y = 0.05;
            fieldLZGroup.add(crossMesh);

            // 3. Canvas Sprite Label
            const canvas = document.createElement('canvas');
            canvas.width = 256;
            canvas.height = 64;
            const ctx = canvas.getContext('2d');
            ctx.fillStyle = '#FFD700';
            ctx.font = 'bold 22px Courier New, monospace';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(`FIELD LZ [${flz.name || 'FLZ-1'}]`, 128, 32);

            const texture = new THREE.CanvasTexture(canvas);
            const spriteMat = new THREE.SpriteMaterial({ map: texture, transparent: true });
            const sprite = new THREE.Sprite(spriteMat);
            sprite.scale.set(6.0, 1.5, 1.0);
            sprite.position.set(0, 2.5, 0);
            fieldLZGroup.add(sprite);

            scene.add(fieldLZGroup);
        }

        fieldLZGroup.position.set(coords[0], coords[1], coords[2] || 0.05);
        fieldLZGroup.visible = true;
    }

    function updateLandingPads(frame) {
        if (!landingPads.length) return;

        const droneMap = new Map();
        if (Array.isArray(frame.drones)) {
            frame.drones.forEach(d => droneMap.set(d.id, d));
        }

        landingPads.forEach(pad => {
            let assignedDroneId = null;
            if (frame.landing_pads && frame.landing_pads[pad.idx] !== undefined) {
                assignedDroneId = frame.landing_pads[pad.idx];
            }

            const drone = assignedDroneId !== null ? droneMap.get(assignedDroneId) : null;
            let status = 'VACANT';

            if (drone && !drone.crashed) {
                if (drone.flight_state === 'LANDED') {
                    status = 'OCCUPIED';
                } else if (drone.flight_state === 'DESCENDING') {
                    status = 'APPROACHING';
                }
            }

            pad.status = status;

            if (status === 'OCCUPIED') {
                pad.centerMesh.material.color.setHex(0x00FFAA);
                pad.centerMesh.material.emissive.setHex(0x00FFAA);
                pad.centerMesh.material.emissiveIntensity = 0.95;
                pad.centerMesh.material.opacity = 0.95;
                pad.ringMesh.material.color.setHex(0x00FFAA);
            } else if (status === 'APPROACHING') {
                const pulse = 0.5 + 0.5 * Math.sin(performance.now() * 0.008);
                pad.centerMesh.material.color.setHex(0xFFB800);
                pad.centerMesh.material.emissive.setHex(0xFFB800);
                pad.centerMesh.material.emissiveIntensity = 0.6 + 0.4 * pulse;
                pad.centerMesh.material.opacity = 0.85;
                pad.ringMesh.material.color.setHex(0xFFB800);
            } else {
                pad.centerMesh.material.color.setHex(0x00A0B0);
                pad.centerMesh.material.emissive.setHex(0x00A0B0);
                pad.centerMesh.material.emissiveIntensity = 0.25;
                pad.centerMesh.material.opacity = 0.5;
                pad.ringMesh.material.color.setHex(0x00F0FF);
            }
        });
    }

    // =========================================================================
    // 2d. Stationary Ground Repeaters & Cairn G1 Beacons (TKT-59 / PLN-04)
    // =========================================================================
    const groundRepeaterMeshes = new Map();

    function updateGroundRepeaters(repeaters) {
        if (!scene) return;
        const activeIds = new Set();
        if (Array.isArray(repeaters)) {
            repeaters.forEach(rep => {
                activeIds.add(rep.id);
                let entry = groundRepeaterMeshes.get(rep.id);
                if (!entry) {
                    const group = new THREE.Group();
                    group.position.set(rep.pos[0], rep.pos[1], rep.pos[2] || 0.05);

                    // Central Solar Pod Box
                    const podGeo = new THREE.CylinderGeometry(0.8, 0.9, 0.4, 6);
                    const podMat = new THREE.MeshStandardMaterial({
                        color: 0x00FFAA,
                        emissive: 0x00FFAA,
                        emissiveIntensity: 0.6,
                        roughness: 0.2,
                        metalness: 0.8,
                    });
                    const podMesh = new THREE.Mesh(podGeo, podMat);
                    podMesh.rotation.x = Math.PI / 2;
                    group.add(podMesh);

                    // Pulsing P2P Mesh Beacon Ring (60m radius)
                    const ringGeo = new THREE.RingGeometry(59.5, 60.5, 64);
                    const ringMat = new THREE.MeshBasicMaterial({
                        color: 0x00FFAA,
                        side: THREE.DoubleSide,
                        transparent: true,
                        opacity: 0.25,
                    });
                    const ringMesh = new THREE.Mesh(ringGeo, ringMat);
                    group.add(ringMesh);

                    scene.add(group);
                    entry = { group, podMesh, ringMesh };
                    groundRepeaterMeshes.set(rep.id, entry);
                }

                // Update position & pulse ring
                entry.group.position.set(rep.pos[0], rep.pos[1], rep.pos[2] || 0.05);
                const pulse = 0.15 + 0.15 * Math.sin(performance.now() * 0.005);
                entry.ringMesh.material.opacity = pulse;
            });
        }

        // Remove any repeaters no longer active
        for (const [id, entry] of groundRepeaterMeshes.entries()) {
            if (!activeIds.has(id)) {
                scene.remove(entry.group);
                groundRepeaterMeshes.delete(id);
            }
        }
    }

    // =========================================================================
    // 2e. Urban 3D Obstacle Shelves & Altitude Colliders (PLN-08 / TKT-60)
    // =========================================================================
    const urbanObstacleMeshes = new Map();

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
                scene.remove(entry.group);
                urbanObstacleMeshes.delete(id);
            }
        }
    }

    // =========================================================================
    // 2c. Discrete Waypoint Mission Route Visuals (TKT-28)
    // =========================================================================
    let missionRouteGroup = null;
    let routeLineMesh = null;
    const routeWaypointMarkers = [];

    function createMissionRouteGroup() {
        missionRouteGroup = new THREE.Group();
        missionRouteGroup.name = 'mission_route';
        missionRouteGroup.visible = false;

        const lineGeom = new THREE.BufferGeometry();
        const lineMat = new THREE.LineBasicMaterial({
            color: 0x00F0FF,
            transparent: true,
            opacity: 0.7,
            linewidth: 2,
        });
        routeLineMesh = new THREE.LineLoop(lineGeom, lineMat);
        missionRouteGroup.add(routeLineMesh);

        scene.add(missionRouteGroup);
    }

    function updateMissionRouteVisuals(routeData) {
        if (!missionRouteGroup) return;

        if (!routeData || !routeData.active || !Array.isArray(routeData.waypoints) || routeData.waypoints.length === 0) {
            missionRouteGroup.visible = false;
            return;
        }

        missionRouteGroup.visible = true;
        const wps = routeData.waypoints;
        const currentIdx = routeData.current_wp_idx || 0;

        // 1. Update polygon line points
        const points = wps.map(wp => new THREE.Vector3(wp[0], wp[1], wp[2]));
        routeLineMesh.geometry.setFromPoints(points);

        // 2. Manage waypoint vertex beacon spheres
        while (routeWaypointMarkers.length < wps.length) {
            const markerGroup = new THREE.Group();

            // Translucent glowing sphere
            const sphereGeom = new THREE.SphereGeometry(0.7, 12, 12);
            const sphereMat = new THREE.MeshBasicMaterial({
                color: 0x00F0FF,
                transparent: true,
                opacity: 0.45,
                wireframe: true,
            });
            const sphereMesh = new THREE.Mesh(sphereGeom, sphereMat);
            markerGroup.add(sphereMesh);

            // Ground projection drop circle
            const circleGeom = new THREE.RingGeometry(0.6, 0.85, 16);
            const circleMat = new THREE.MeshBasicMaterial({
                color: 0x00F0FF,
                side: THREE.DoubleSide,
                transparent: true,
                opacity: 0.35,
            });
            const circleMesh = new THREE.Mesh(circleGeom, circleMat);
            circleMesh.position.set(0, 0, 0.05);
            markerGroup.add(circleMesh);

            // Drop line from waypoint to ground
            const dropGeom = new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(0, 0, 0),
                new THREE.Vector3(0, 0, 1),
            ]);
            const dropMat = new THREE.LineDashedMaterial({
                color: 0x00F0FF,
                dashSize: 1.0,
                gapSize: 0.5,
                transparent: true,
                opacity: 0.35,
            });
            const dropLine = new THREE.Line(dropGeom, dropMat);
            dropLine.computeLineDistances();
            markerGroup.add(dropLine);

            missionRouteGroup.add(markerGroup);
            routeWaypointMarkers.push({
                group: markerGroup,
                sphere: sphereMesh,
                circle: circleMesh,
                dropLine: dropLine,
            });
        }

        // Highlight active target waypoint vs upcoming waypoints
        for (let i = 0; i < routeWaypointMarkers.length; i++) {
            const m = routeWaypointMarkers[i];
            if (i < wps.length) {
                m.group.visible = true;
                const wp = wps[i];
                m.group.position.set(wp[0], wp[1], 0);
                m.sphere.position.set(0, 0, wp[2]);

                // Update drop line geometry
                m.dropLine.geometry.setFromPoints([
                    new THREE.Vector3(0, 0, 0),
                    new THREE.Vector3(0, 0, wp[2]),
                ]);
                m.dropLine.computeLineDistances();

                if (i === currentIdx) {
                    const isPaused = !!routeData.paused;
                    const isLoitering = !!routeData.loitering;
                    const now = Date.now() * 0.005;
                    const pulse = 1.3 + 0.25 * Math.sin(now);

                    let activeColor = 0x00FFAA;
                    if (isPaused) {
                        activeColor = 0xFFB800; // Neon Amber for PAUSED
                    } else if (isLoitering) {
                        activeColor = 0xFFD700; // Glowing Gold for LOITER / DWELL
                    }

                    m.sphere.material.color.setHex(activeColor);
                    m.sphere.material.opacity = isLoitering ? 0.95 : 0.85;
                    m.sphere.scale.set(pulse, pulse, pulse);
                    m.circle.material.color.setHex(activeColor);
                    m.circle.material.opacity = 0.75;
                    m.dropLine.material.color.setHex(activeColor);
                    m.dropLine.material.opacity = 0.7;
                } else {
                    m.sphere.material.color.setHex(0x00F0FF);
                    m.sphere.material.opacity = 0.4;
                    m.sphere.scale.set(1.0, 1.0, 1.0);
                    m.circle.material.color.setHex(0x00F0FF);
                    m.circle.material.opacity = 0.25;
                    m.dropLine.material.color.setHex(0x00F0FF);
                    m.dropLine.material.opacity = 0.25;
                }
            } else {
                m.group.visible = false;
            }
        }
    }

    // =========================================================================
    // 3. True 3D Quadcopter Mesh Builder
    // =========================================================================
    function buildQuadcopterMesh(isLeader) {
        const droneGroup = new THREE.Group();
        const scale = isLeader ? 1.15 : 1.0;

        // 1. Central fuselage body
        const bodyGeom = new THREE.BoxGeometry(0.55 * scale, 0.45 * scale, 0.22 * scale);
        const bodyMat = new THREE.MeshStandardMaterial({
            color: 0x111D28,
            metalness: 0.85,
            roughness: 0.25,
        });
        const bodyMesh = new THREE.Mesh(bodyGeom, bodyMat);
        droneGroup.add(bodyMesh);

        // Glowing center core
        const coreGeom = new THREE.SphereGeometry(0.18 * scale, 12, 12);
        const coreMat = new THREE.MeshBasicMaterial({
            color: isLeader ? COLORS.leader : COLORS.escort,
        });
        const coreMesh = new THREE.Mesh(coreGeom, coreMat);
        coreMesh.name = 'core';
        droneGroup.add(coreMesh);

        // 2. Four Diagonal Carbon Arms
        const armLength = 0.75 * scale; // wingspan ~1.5m
        const armGeom = new THREE.CylinderGeometry(0.04, 0.04, armLength, 8);
        armGeom.rotateZ(Math.PI / 2);
        const armMat = new THREE.MeshStandardMaterial({
            color: 0x243542,
            metalness: 0.9,
            roughness: 0.3,
        });

        // Arm 1 & 2 (X configuration)
        const armA = new THREE.Mesh(armGeom, armMat);
        armA.rotation.z = Math.PI / 4;
        droneGroup.add(armA);

        const armB = new THREE.Mesh(armGeom, armMat);
        armB.rotation.z = -Math.PI / 4;
        droneGroup.add(armB);

        // 3. Motor Pods & Spinning Rotor Discs
        const rotors = [];
        const armOffset = (armLength / 2) * Math.SQRT1_2;
        const motorPositions = [
            [armOffset, armOffset],
            [-armOffset, armOffset],
            [armOffset, -armOffset],
            [-armOffset, -armOffset],
        ];

        const motorGeom = new THREE.CylinderGeometry(0.07, 0.07, 0.16, 8);
        motorGeom.rotateX(Math.PI / 2);
        const motorMat = new THREE.MeshStandardMaterial({ color: 0x334455, metalness: 0.8 });

        const rotorGeom = new THREE.CircleGeometry(0.38 * scale, 16);
        const rotorMat = new THREE.MeshBasicMaterial({
            color: isLeader ? COLORS.leader : COLORS.escort,
            transparent: true,
            opacity: 0.45,
            side: THREE.DoubleSide,
        });

        motorPositions.forEach((pos, idx) => {
            const motor = new THREE.Mesh(motorGeom, motorMat);
            motor.position.set(pos[0], pos[1], 0.08 * scale);
            droneGroup.add(motor);

            // Rotor Disc
            const rotor = new THREE.Mesh(rotorGeom, rotorMat.clone());
            rotor.position.set(pos[0], pos[1], 0.16 * scale);
            droneGroup.add(rotor);
            rotors.push(rotor);
        });

        droneGroup.userData.rotors = rotors;

        // 4. Orientation LED indicator (Front = Cyan/Green, Rear = Red)
        const fwdLedGeom = new THREE.SphereGeometry(0.06, 8, 8);
        const fwdLedMat = new THREE.MeshBasicMaterial({ color: 0x00FF88 });
        const fwdLed = new THREE.Mesh(fwdLedGeom, fwdLedMat);
        fwdLed.position.set(0.32 * scale, 0, 0);
        droneGroup.add(fwdLed);

        return droneGroup;
    }

    function createDropLine(isLeader) {
        const dropLineGeom = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(0, 0, 0),
        ]);
        const dropLineMat = new THREE.LineDashedMaterial({
            color: isLeader ? COLORS.leader : COLORS.dropLine,
            dashSize: 1.0,
            gapSize: 0.8,
            linewidth: 1,
            transparent: true,
            opacity: 0.5,
        });
        const dropLine = new THREE.Line(dropLineGeom, dropLineMat);
        scene.add(dropLine);
        return dropLine;
    }

    function createGroundShadow(isLeader) {
        const scale = isLeader ? 1.15 : 1.0;
        const shadowGeom = new THREE.RingGeometry(0.8 * scale, 1.0 * scale, 16);
        const shadowMat = new THREE.MeshBasicMaterial({
            color: isLeader ? COLORS.leader : COLORS.dropLine,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.45,
        });
        const shadowMesh = new THREE.Mesh(shadowGeom, shadowMat);
        scene.add(shadowMesh);
        return shadowMesh;
    }

    // 3D Billboard Tactical ID Badge Floating Above Drone Airframe
    function createDroneBadge(droneId, isLeader) {
        const canvas = document.createElement('canvas');
        canvas.width = 256;
        canvas.height = 64;
        const texture = new THREE.CanvasTexture(canvas);
        texture.minFilter = THREE.LinearFilter;

        const spriteMat = new THREE.SpriteMaterial({
            map: texture,
            transparent: true,
            depthTest: false,
        });
        const sprite = new THREE.Sprite(spriteMat);
        sprite.scale.set(2.4, 0.6, 1.0);
        sprite.userData = { canvas, texture, lastKey: '' };
        scene.add(sprite);
        return sprite;
    }

    function updateDroneBadge(sprite, d) {
        if (!sprite || !sprite.userData) return;
        const { canvas, texture } = sprite.userData;
        const key = `${d.id}_${d.is_leader}_${d.crashed}_${d.lost_link}_${d.yip_lost}_${d.flight_state}_${d.status}_${d.is_2ic}_${d.landing_pad_idx}`;
        if (sprite.userData.lastKey === key) return;
        sprite.userData.lastKey = key;

        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Cyberpunk tactical styling
        let borderColor = '#00F0FF';
        let bgColor = 'rgba(6, 16, 25, 0.88)';
        let textColor = '#00F0FF';
        let labelText = `#${d.id}`;
        let subText = 'ESCORT';
        let dotColor = '#00F0FF';

        if (d.crashed) {
            borderColor = '#FF3366';
            bgColor = 'rgba(45, 10, 20, 0.92)';
            textColor = '#FF3366';
            labelText = `#${d.id}`;
            subText = 'CRASHED';
            dotColor = '#FF3366';
        } else if (d.flight_state === 'FIELD_LANDED') {
            borderColor = '#00FFAA';
            bgColor = 'rgba(4, 30, 20, 0.94)';
            textColor = '#00FFAA';
            labelText = `#${d.id}`;
            const slotStr = d.field_lz_slot !== undefined && d.field_lz_slot !== null ? `S${d.field_lz_slot}` : 'FLZ';
            subText = `F-LAND [${slotStr}]`;
            dotColor = '#00FFAA';
        } else if (d.flight_state === 'FIELD_DESCENDING') {
            borderColor = '#FFD700';
            bgColor = 'rgba(42, 35, 4, 0.94)';
            textColor = '#FFD700';
            labelText = `#${d.id}`;
            subText = 'F-LANDING';
            dotColor = '#FFD700';
        } else if (d.flight_state === 'E_LANDED') {
            borderColor = '#FF9900';
            bgColor = 'rgba(50, 25, 5, 0.94)';
            textColor = '#FFB800';
            labelText = `#${d.id}`;
            subText = 'E-LANDED';
            dotColor = '#FF9900';
        } else if (d.flight_state === 'EMERGENCY_LANDING') {
            borderColor = '#FF3300';
            bgColor = 'rgba(55, 15, 15, 0.94)';
            textColor = '#FF4444';
            labelText = `#${d.id}`;
            subText = '⚠ E-LANDING';
            dotColor = '#FF3300';
        } else if (d.flight_state === 'LANDED') {
            borderColor = '#00FFAA';
            bgColor = 'rgba(4, 30, 20, 0.94)';
            textColor = '#00FFAA';
            labelText = `#${d.id}`;
            const padStr = d.landing_pad_idx !== undefined && d.landing_pad_idx !== null ? `P${d.landing_pad_idx}` : 'PAD';
            subText = `LANDED [${padStr}]`;
            dotColor = '#00FFAA';
        } else if (d.flight_state === 'DESCENDING') {
            borderColor = '#FFB800';
            bgColor = 'rgba(42, 28, 4, 0.94)';
            textColor = '#FFD24D';
            labelText = `#${d.id}`;
            const padStr = d.landing_pad_idx !== undefined && d.landing_pad_idx !== null ? `P${d.landing_pad_idx}` : 'PAD';
            subText = `LANDING [${padStr}]`;
            dotColor = '#FFB800';
        } else if (d.status === 'BREAKAWAY_JETTISON') {
            borderColor = '#FFAA00';
            bgColor = 'rgba(42, 28, 4, 0.95)';
            textColor = '#FFAA00';
            labelText = `#${d.id}`;
            subText = '⏏ JETTISON';
            dotColor = '#FFAA00';
        } else if (d.lost_link) {
            borderColor = '#FFB800';
            bgColor = 'rgba(38, 26, 5, 0.92)';
            textColor = '#FFD24D';
            labelText = `#${d.id}`;
            subText = d.is_leader ? 'LOST (FLAG)' : 'LOST LINK';
            dotColor = '#FFB800';
        } else if (d.is_leader) {
            borderColor = '#FFB800';
            bgColor = 'rgba(42, 28, 4, 0.92)';
            textColor = '#FFB800';
            labelText = `#${d.id}`;
            subText = 'FLAGSHIP';
            dotColor = '#FFB800';
        } else if (d.is_2ic) {
            borderColor = '#00F0FF';
            bgColor = 'rgba(6, 25, 35, 0.92)';
            textColor = '#00F0FF';
            labelText = `#${d.id}`;
            subText = '2IC';
            dotColor = '#00F0FF';
        } else if (d.flight_state === 'TAKEOFF') {
            borderColor = '#39FF14';
            bgColor = 'rgba(8, 35, 12, 0.92)';
            textColor = '#39FF14';
            labelText = `#${d.id}`;
            subText = 'TAKEOFF';
            dotColor = '#39FF14';
        } else if (d.flight_state === 'EN_ROUTE') {
            borderColor = '#00F0FF';
            bgColor = 'rgba(6, 25, 35, 0.92)';
            textColor = '#00F0FF';
            labelText = `#${d.id}`;
            subText = 'EN ROUTE';
            dotColor = '#00F0FF';
        } else if (d.yip_lost) {
            borderColor = '#FF9900';
            bgColor = 'rgba(40, 20, 0, 0.92)';
            textColor = '#FFB800';
            labelText = `#${d.id}`;
            subText = 'YIP LOST';
            dotColor = '#FF9900';
        } else if (d.flight_state === 'PERIMETER_HOLD' || d.status === 'PERIMETER_HOLD' || d.holding_at_horizon) {
            borderColor = '#FFB800';
            bgColor = 'rgba(38, 26, 5, 0.92)';
            textColor = '#FFD24D';
            labelText = `#${d.id}`;
            subText = 'PERIMETER HOLD';
            dotColor = '#FFB800';
        } else if (d.flight_state === 'BACKTRACK_HOLD' || d.status === 'BACKTRACK_HOLD') {
            borderColor = '#FF3366';
            bgColor = 'rgba(40, 10, 20, 0.92)';
            textColor = '#FF6699';
            labelText = `#${d.id}`;
            subText = 'BACKTRACKING';
            dotColor = '#FF3366';
        } else if (d.status === 'PROPAGATING') {
            borderColor = '#00FFAA';
            bgColor = 'rgba(4, 32, 20, 0.92)';
            textColor = '#00FFAA';
            labelText = `#${d.id}`;
            subText = 'ECHO';
            dotColor = '#00FFAA';
        }

        // Draw rounded tactical card
        const x = 4, y = 4, w = 248, h = 56, r = 8;
        ctx.beginPath();
        ctx.moveTo(x + r, y);
        ctx.lineTo(x + w - r, y);
        ctx.quadraticCurveTo(x + w, y, x + w, y + r);
        ctx.lineTo(x + w, y + h - r);
        ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
        ctx.lineTo(x + r, y + h);
        ctx.quadraticCurveTo(x, y + h, x, y + h - r);
        ctx.lineTo(x, y + r);
        ctx.quadraticCurveTo(x, y, x + r, y);
        ctx.closePath();

        ctx.fillStyle = bgColor;
        ctx.fill();

        ctx.lineWidth = 3;
        ctx.strokeStyle = borderColor;
        ctx.shadowColor = borderColor;
        ctx.shadowBlur = 12;
        ctx.stroke();
        ctx.shadowBlur = 0;

        // Status indicator dot
        ctx.beginPath();
        ctx.arc(28, 32, 6, 0, Math.PI * 2);
        ctx.fillStyle = dotColor;
        ctx.shadowColor = dotColor;
        ctx.shadowBlur = 8;
        ctx.fill();
        ctx.shadowBlur = 0;

        // Drone ID number
        ctx.font = 'bold 22px "JetBrains Mono", Consolas, monospace';
        ctx.fillStyle = '#FFFFFF';
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillText(labelText, 44, 32);

        // Role subtext
        ctx.font = 'bold 15px "JetBrains Mono", Consolas, monospace';
        ctx.fillStyle = textColor;
        ctx.textAlign = 'right';
        ctx.fillText(subText, 238, 32);

        texture.needsUpdate = true;
    }

    // =========================================================================
    // 4. Telemetry Frame Ingestion & Interpolation
    // =========================================================================
    function onTelemetryFrame(frame) {
        lastTelemetryTime = performance.now();

        // Update Top Bar HUD Info
        updateTopBar(frame);

        // Update Target Waypoint positions (TKT-26: Dual Target C2 Visuals)
        const cmdTarget = frame.commanded_target || frame.target;
        const ackTarget = frame.acknowledged_target || frame.target;

        if (cmdTarget && commandedBeacon) {
            commandedBeacon.position.set(cmdTarget[0], cmdTarget[1], cmdTarget[2]);
        }
        if (ackTarget && targetBeacon) {
            targetBeacon.position.set(ackTarget[0], ackTarget[1], ackTarget[2]);
        }

        // Update / Spawn Drones
        const activeIds = new Set();
        let currentLeaderDrone = null;

        if (Array.isArray(frame.drones)) {
            frame.drones.forEach(d => {
                activeIds.add(d.id);
                let entry = dronesMap.get(d.id);

                if (!entry) {
                    const mesh = buildQuadcopterMesh(d.is_leader);
                    scene.add(mesh);
                    const dropLine = createDropLine(d.is_leader);
                    const shadow = createGroundShadow(d.is_leader);
                    const badge = createDroneBadge(d.id, d.is_leader);
                    entry = {
                        mesh: mesh,
                        dropLine: dropLine,
                        shadow: shadow,
                        badge: badge,
                        targetPos: new THREE.Vector3(...d.pos),
                        currentPos: new THREE.Vector3(...d.pos),
                        yaw: d.yaw || 0,
                        pitch: d.pitch || 0,
                        roll: d.roll || 0,
                        is_leader: d.is_leader,
                        crashed: d.crashed,
                        flight_state: d.flight_state,
                        landing_pad_idx: d.landing_pad_idx,
                        rotorSpeed: (d.crashed || d.flight_state === 'LANDED') ? 0 : 0.4,
                    };
                    dronesMap.set(d.id, entry);
                }

                // Update target kinematics for interpolation
                // When telemetry link is lost, freeze avatar at LKP (The Operator Airgap)
                // Grounded airframes on pads remain fixed to pad positions, never snapping to airborne LKP
                const renderPos = ((d.lost_link || !d.downlink_active) && Array.isArray(d.lkp) && d.flight_state !== 'GROUND_IDLE' && d.flight_state !== 'LANDED') ? d.lkp : d.pos;
                entry.targetPos.set(renderPos[0], renderPos[1], renderPos[2]);
                entry.targetYaw = d.yaw || 0;
                entry.targetPitch = d.pitch || 0;
                entry.targetRoll = d.roll || 0;
                entry.crashed = d.crashed;
                entry.is_leader = d.is_leader;
                entry.status = d.status;
                entry.flight_state = d.flight_state;
                entry.landing_pad_idx = d.landing_pad_idx;
                entry.rotorSpeed = (d.crashed || d.flight_state === 'LANDED') ? 0 : 0.45;

                // Update 3D Tactical ID Badge
                updateDroneBadge(entry.badge, d);

                // Dynamic Rotor Disc, Core, Drop-Line & Shadow Role Tinting
                let rotorColor = COLORS.escort;
                let rotorOpacity = 0.45;
                let indicatorColor = COLORS.dropLine;

                if (d.crashed) {
                    rotorColor = 0x661122; // Dim dead red
                    rotorOpacity = 0.2;
                    indicatorColor = COLORS.crashed;
                } else if (d.flight_state === 'LANDED') {
                    rotorColor = 0x00FFAA; // Soft emerald pad disarm
                    rotorOpacity = 0.2;
                    indicatorColor = 0x00FFAA;
                } else if (d.flight_state === 'DESCENDING') {
                    rotorColor = 0xFFB800; // Amber descent
                    rotorOpacity = 0.6;
                    indicatorColor = 0xFFB800;
                } else if (d.status === 'BREAKAWAY_JETTISON') {
                    rotorColor = 0xFFAA00; // Bright amber jettison
                    rotorOpacity = 0.7;
                    indicatorColor = 0xFFAA00;
                } else if (d.flight_state === 'PERIMETER_HOLD' || d.status === 'PERIMETER_HOLD' || d.holding_at_horizon) {
                    rotorColor = 0xFFB800; // Amber horizon hover
                    rotorOpacity = 0.5;
                    indicatorColor = 0xFFB800;
                } else if (d.flight_state === 'BACKTRACK_HOLD' || d.status === 'BACKTRACK_HOLD') {
                    rotorColor = 0xFF3366; // Red-pink backtrack
                    rotorOpacity = 0.6;
                    indicatorColor = 0xFF3366;
                } else if (d.lost_link) {
                    rotorColor = 0xFFB800; // Amber ghost for lost-link LKP freeze
                    rotorOpacity = 0.3;
                    indicatorColor = 0xFFAA00;
                } else if (d.is_leader) {
                    rotorColor = COLORS.leader; // Bright neon amber
                    rotorOpacity = 0.65;
                    indicatorColor = COLORS.leader;
                } else if (d.flight_state === 'TAKEOFF') {
                    rotorColor = 0x39FF14; // Bright launch neon green
                    rotorOpacity = 0.65;
                    indicatorColor = 0x39FF14;
                } else if (d.flight_state === 'EN_ROUTE') {
                    rotorColor = 0x00E5FF; // High speed transit cyan
                    rotorOpacity = 0.55;
                    indicatorColor = 0x00E5FF;
                } else if (d.status === 'PROPAGATING') {
                    rotorColor = COLORS.propagating; // Pulse echo amber
                    rotorOpacity = 0.55;
                    indicatorColor = COLORS.propagating;
                }

                // Update rotor discs
                if (entry.mesh.userData.rotors) {
                    entry.mesh.userData.rotors.forEach(r => {
                        r.material.color.setHex(rotorColor);
                        r.material.opacity = rotorOpacity;
                    });
                }

                // Update core
                const coreMesh = entry.mesh.getObjectByName('core');
                if (coreMesh) {
                    coreMesh.material.color.setHex(indicatorColor);
                }

                // Update drop line and shadow colors
                if (entry.dropLine && entry.dropLine.material) {
                    entry.dropLine.material.color.setHex(indicatorColor);
                }
                if (entry.shadow && entry.shadow.material) {
                    entry.shadow.material.color.setHex(indicatorColor);
                }

                if (d.is_leader && !d.crashed) {
                    currentLeaderDrone = entry;
                }
            });
        }

        // Clean up vanished drones
        for (let [id, entry] of dronesMap.entries()) {
            if (!activeIds.has(id)) {
                scene.remove(entry.mesh);
                if (entry.dropLine) {
                    scene.remove(entry.dropLine);
                    entry.dropLine.geometry.dispose();
                    entry.dropLine.material.dispose();
                }
                if (entry.shadow) {
                    scene.remove(entry.shadow);
                    entry.shadow.geometry.dispose();
                    entry.shadow.material.dispose();
                }
                if (entry.badge) {
                    scene.remove(entry.badge);
                    if (entry.badge.material.map) entry.badge.material.map.dispose();
                    entry.badge.material.dispose();
                }
                if (targetDroneId === id) {
                    targetDroneId = null;
                }
                dronesMap.delete(id);
            }
        }

        // Update Dynamic Catenary Tether
        updateTetherCurve(frame.tether, currentLeaderDrone);

        // Update Tactical Landing Pads (TKT-19)
        updateLandingPads(frame);
        // Update Tactical Field Landing Zone (TKT-63 / PLN-05)
        updateFieldLZ(frame);

        // Update Discrete Mission Route Visuals (TKT-28)
        updateMissionRouteVisuals(frame.mission_route);

        // Update Stationary Ground Repeaters (TKT-59 / PLN-04)
        updateGroundRepeaters(frame.ground_repeaters);

        // Update Urban 3D Obstacles & Altitude Shelves (PLN-08 / TKT-60)
        updateUrbanObstacles(frame.urban_obstacles);
    }

    let tetherSeveredPrev = false;
    let tetherDropProgress = 0.0;
    let tetherLastTipPos = new THREE.Vector3(0, 0, 20);

    function updateTetherCurve(tetherData, leaderEntry) {
        if (!tetherMesh) return;

        if (!tetherData) {
            tetherMesh.visible = false;
            return;
        }

        // Wireless Modes (LEADER_FOLLOWER, GOSSIP_CONSENSUS): tether is not deployed
        if (tetherData.deployed === false || (!tetherData.deployed && tetherData.visible === false && !tetherData.severed)) {
            tetherMesh.visible = false;
            tetherSeveredPrev = false;
            tetherDropProgress = 0.0;
            return;
        }

        const anchor = new THREE.Vector3(0, 0, 1.5);

        // Check if tether is severed or leader is gone (TKT-16)
        if (tetherData.severed || !leaderEntry) {
            if (!tetherSeveredPrev) {
                tetherSeveredPrev = true;
                tetherDropProgress = 0.0;
                // Umbilical was anchored to primary flagship Drone 0
                const tetherDroneEntry = dronesMap.get(0);
                if (tetherDroneEntry) {
                    tetherLastTipPos.copy(tetherDroneEntry.currentPos);
                } else if (leaderEntry) {
                    tetherLastTipPos.copy(leaderEntry.currentPos);
                }
            }

            // Animate severed line falling to ground and dissolving (1.5s at 60 FPS, TKT-27)
            if (tetherDropProgress < 1.0) {
                tetherDropProgress = Math.min(1.0, tetherDropProgress + 0.011);
                tetherMesh.visible = true;

                // Severed tip drops to ground and slack sags
                const currentTip = new THREE.Vector3(
                    tetherLastTipPos.x,
                    tetherLastTipPos.y,
                    Math.max(0.2, tetherLastTipPos.z * (1.0 - tetherDropProgress))
                );

                const points = [];
                const segments = 16;
                for (let i = 0; i <= segments; i++) {
                    const t = i / segments;
                    const p = new THREE.Vector3().lerpVectors(anchor, currentTip, t);
                    // Heavy gravity sag as line goes completely slack
                    const sag = (2.5 + 8.0 * tetherDropProgress) * Math.sin(t * Math.PI);
                    p.z = Math.max(0.1, p.z - sag);
                    points.push(p);
                }

                tetherMesh.geometry.setFromPoints(points);
                tetherMesh.geometry.computeVertexNormals();
                tetherMesh.material.opacity = Math.max(0.0, 0.85 * (1.0 - tetherDropProgress));
            } else {
                tetherMesh.visible = false;
            }
            return;
        }

        // Tether connected: reset drop state
        tetherSeveredPrev = false;
        tetherDropProgress = 0.0;
        tetherMesh.material.opacity = 0.85;
        tetherMesh.visible = true;

        const leadPos = leaderEntry.currentPos;
        tetherLastTipPos.copy(leadPos);

        // Generate 16 catenary curve points with slight gravity sag
        const points = [];
        const segments = 16;
        for (let i = 0; i <= segments; i++) {
            const t = i / segments;
            const p = new THREE.Vector3().lerpVectors(anchor, leadPos, t);
            // Parabolic sag
            const sag = 2.5 * Math.sin(t * Math.PI);
            p.z = Math.max(0.2, p.z - sag);
            points.push(p);
        }

        tetherMesh.geometry.setFromPoints(points);
        tetherMesh.geometry.computeVertexNormals();
    }

    function updateTopBar(frame) {
        const simTimeElem = document.getElementById('hud-sim-time');
        const modeElem = document.getElementById('hud-mode');
        const commsElem = document.getElementById('hud-comms');
        const brainElem = document.getElementById('hud-brain');
        const dronesElem = document.getElementById('hud-drones');

        if (simTimeElem && frame.sim_time !== undefined) {
            simTimeElem.textContent = `${frame.sim_time.toFixed(1)}s`;
        }
        if (commsElem && frame.comms_mode) {
            commsElem.textContent = frame.comms_mode;
        }
        if (modeElem) {
            if (frame.recovery_stage === 'LANDING') {
                modeElem.textContent = 'ATC SOFT-LANDING';
            } else if (frame.overhead_marshal_active) {
                modeElem.textContent = 'OVERHEAD HOLD (25m)';
            } else if (frame.formation) {
                modeElem.textContent = frame.formation;
            }
        }
        if (brainElem && frame.brain) {
            brainElem.textContent = frame.brain;
        }
        if (dronesElem && Array.isArray(frame.drones)) {
            const alive = frame.drones.filter(d => !d.crashed).length;
            dronesElem.textContent = `${alive}/${frame.drones.length}`;
        }
    }

    function onConnectionStatus(connected) {
        const ind = document.getElementById('hud-conn-ind');
        const text = document.getElementById('hud-conn-text');
        if (ind && text) {
            if (connected) {
                ind.classList.remove('disconnected');
                text.textContent = 'ONLINE (60 FPS)';
            } else {
                ind.classList.add('disconnected');
                text.textContent = 'LINK LOST';
            }
        }
    }

    // =========================================================================
    // 5. Main Animation & Render Loop (60 FPS GPU)
    // =========================================================================
    function animate() {
        requestAnimationFrame(animate);

        // 1. Smooth Drone Position & Rotor Interpolation (lerp)
        for (let [id, entry] of dronesMap.entries()) {
            // Smoothly lerp towards target packet position
            entry.currentPos.lerp(entry.targetPos, 0.35);
            entry.mesh.position.copy(entry.currentPos);

            // Orientation lerp
            entry.mesh.rotation.z = entry.targetYaw;
            entry.mesh.rotation.y = entry.targetPitch;
            entry.mesh.rotation.x = entry.targetRoll;

            // Update 3D World Drop-Line from (x, y, z) down to (x, y, 0)
            if (entry.dropLine) {
                const p = entry.currentPos;
                if (p.z > 0.6 && !entry.crashed && entry.flight_state !== 'LANDED') {
                    entry.dropLine.visible = true;
                    entry.dropLine.geometry.setFromPoints([
                        new THREE.Vector3(p.x, p.y, p.z),
                        new THREE.Vector3(p.x, p.y, 0),
                    ]);
                    entry.dropLine.computeLineDistances();
                } else {
                    entry.dropLine.visible = false;
                }
            }

            // Update Ground Shadow ring: strictly flat on ground plane (z=0.05) directly underneath
            if (entry.shadow) {
                const p = entry.currentPos;
                entry.shadow.position.set(p.x, p.y, 0.05);
                entry.shadow.rotation.set(0, 0, 0);
                entry.shadow.visible = (!entry.crashed && entry.flight_state !== 'LANDED') || p.z > 0.6;
            }

            // Update 3D Tactical ID Badge Position (floating plumb 1.35m above drone)
            if (entry.badge) {
                entry.badge.position.set(entry.currentPos.x, entry.currentPos.y, entry.currentPos.z + 1.35);
            }
        }

        // 2. Animate Dual Target Waypoint Beacons (TKT-26)
        let targetsDiverged = false;
        if (targetBeacon && commandedBeacon) {
            const dist = targetBeacon.position.distanceTo(commandedBeacon.position);
            targetsDiverged = dist > 0.8;
        }

        if (targetBeacon) {
            const diamond = targetBeacon.getObjectByName('diamond');
            if (diamond) diamond.rotation.z += 0.015;

            const targetZ = Math.max(0.01, targetBeacon.position.z);
            const beam = targetBeacon.getObjectByName('beam');
            if (beam) {
                beam.scale.set(1, 1, targetZ);
                beam.position.set(0, 0, -targetZ / 2);
            }

            const ring = targetBeacon.getObjectByName('ring');
            if (ring) {
                const scale = 1.0 + 0.15 * Math.sin(performance.now() * 0.005);
                ring.scale.set(scale, scale, 1);
                ring.position.z = -targetBeacon.position.z + 0.05;
            }
        }

        if (commandedBeacon) {
            const cmdDiamond = commandedBeacon.getObjectByName('diamond');
            if (cmdDiamond) cmdDiamond.rotation.z -= 0.02;

            const reticle = commandedBeacon.getObjectByName('reticle');
            if (reticle) {
                reticle.rotation.z += 0.01;
                const rScale = 1.0 + 0.08 * Math.sin(performance.now() * 0.008);
                reticle.scale.set(rScale, rScale, 1);
            }

            const dropLine = commandedBeacon.getObjectByName('dropLine');
            if (dropLine) {
                if (targetsDiverged && commandedBeacon.position.z > 0.6) {
                    dropLine.visible = true;
                    dropLine.geometry.setFromPoints([
                        new THREE.Vector3(0, 0, 0),
                        new THREE.Vector3(0, 0, -commandedBeacon.position.z)
                    ]);
                    dropLine.computeLineDistances();
                } else {
                    dropLine.visible = false;
                }
            }
        }

        // 2b. Animate Staged Target Beacon (TKT-68)
        if (stagedBeacon && stagedBeacon.visible) {
            const stagedDiamond = stagedBeacon.getObjectByName('diamond');
            if (stagedDiamond) stagedDiamond.rotation.z += 0.025;

            const stagedReticle = stagedBeacon.getObjectByName('reticle');
            if (stagedReticle) {
                stagedReticle.rotation.z -= 0.02;
                const rScale = 1.0 + 0.1 * Math.sin(performance.now() * 0.007);
                stagedReticle.scale.set(rScale, rScale, 1);
            }

            const stagedRing = stagedBeacon.getObjectByName('ring');
            if (stagedRing) {
                const ringScale = 1.0 + 0.15 * Math.sin(performance.now() * 0.006);
                stagedRing.scale.set(ringScale, ringScale, 1);
                stagedRing.position.z = -stagedBeacon.position.z + 0.05;
            }

            const stagedDropLine = stagedBeacon.getObjectByName('dropLine');
            if (stagedDropLine && stagedBeacon.position.z > 0.6) {
                stagedDropLine.geometry.setFromPoints([
                    new THREE.Vector3(0, 0, 0),
                    new THREE.Vector3(0, 0, -stagedBeacon.position.z)
                ]);
                stagedDropLine.computeLineDistances();
            }
        }

        // 2b. Animate Approaching Landing Pads Pulse
        if (landingPads.length) {
            const padPulse = 0.5 + 0.5 * Math.sin(performance.now() * 0.008);
            landingPads.forEach(pad => {
                if (pad.status === 'APPROACHING') {
                    pad.centerMesh.material.emissiveIntensity = 0.5 + 0.5 * padPulse;
                }
            });
        }

        // 3. Chase Camera Logic
        if (isChaseCam) {
            let targetEntry = dronesMap.get(targetDroneId);
            if (!targetEntry) {
                // Default to first living drone
                for (let [_, e] of dronesMap) {
                    if (!e.crashed) { targetEntry = e; break; }
                }
            }

            if (targetEntry) {
                const fwdX = Math.cos(targetEntry.targetYaw);
                const fwdY = Math.sin(targetEntry.targetYaw);
                const chaseOffset = new THREE.Vector3(-fwdX * 18, -fwdY * 18, 9);
                const desiredCamPos = targetEntry.currentPos.clone().add(chaseOffset);

                camera.position.lerp(desiredCamPos, 0.08);
                controls.target.lerp(targetEntry.currentPos, 0.1);
            }
        }

        // 4. Update Click Ripples & Orbit Controls & Render
        updateRipples();
        controls.update();
        renderer.render(scene, camera);

        // 5. FPS Counter Calculation
        fpsCounter++;
        const now = performance.now();
        if (now - lastFpsUpdate >= 1000) {
            const fpsElem = document.getElementById('hud-fps');
            if (fpsElem) {
                fpsElem.textContent = `${fpsCounter} FPS`;
            }
            fpsCounter = 0;
            lastFpsUpdate = now;
        }
    }

    // =========================================================================
    // 6. User Interaction, Ground Click Dispatch & Window Controls
    // =========================================================================
    const groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    const clickRaycaster = new THREE.Raycaster();
    const clickPoint = new THREE.Vector3();
    let pointerDownX = 0, pointerDownY = 0;
    const activeRipples = [];

    function onPointerDown(e) {
        pointerDownX = e.clientX;
        pointerDownY = e.clientY;
    }

    function onPointerUp(e) {
        const dist = Math.hypot(e.clientX - pointerDownX, e.clientY - pointerDownY);
        // Only trigger on clean click without significant drag (< 6px) and left button
        if (dist < 6 && e.button === 0) {
            const chk = document.getElementById('chk-click-target');
            if (chk && !chk.checked) return;

            const rect = renderer.domElement.getBoundingClientRect();
            const mouseX = ((e.clientX - rect.left) / rect.width) * 2 - 1;
            const mouseY = -((e.clientY - rect.top) / rect.height) * 2 + 1;

            clickRaycaster.setFromCamera(new THREE.Vector2(mouseX, mouseY), camera);
            if (clickRaycaster.ray.intersectPlane(groundPlane, clickPoint)) {
                const targetX = Math.round(Math.max(-140, Math.min(140, clickPoint.x)));
                const targetY = Math.round(Math.max(-140, Math.min(140, clickPoint.y)));
                const targetZ = parseFloat(document.getElementById('tgt-input-z')?.value || 20.0);

                spawnClickRipple(targetX, targetY);

                // Stage target coordinates for operator review rather than immediate flight (TKT-68)
                if (window.tacticalHud && typeof window.tacticalHud.stageTarget === 'function') {
                    window.tacticalHud.stageTarget([targetX, targetY, targetZ], 'GROUND_CLICK');
                } else {
                    const ix = document.getElementById('tgt-input-x');
                    const iy = document.getElementById('tgt-input-y');
                    const iz = document.getElementById('tgt-input-z');
                    if (ix) ix.value = targetX;
                    if (iy) iy.value = targetY;
                    if (iz) iz.value = targetZ;
                    if (stagedBeacon) {
                        stagedBeacon.position.set(targetX, targetY, targetZ);
                        stagedBeacon.visible = true;
                    }
                }
            }
        }
    }

    function spawnClickRipple(x, y) {
        const geom = new THREE.RingGeometry(0.5, 0.9, 32);
        const mat = new THREE.MeshBasicMaterial({
            color: COLORS.target,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.9,
        });
        const ripple = new THREE.Mesh(geom, mat);
        ripple.position.set(x, y, 0.1);
        scene.add(ripple);

        activeRipples.push({
            mesh: ripple,
            born: performance.now(),
            lifetime: 600,
        });
    }

    function updateRipples() {
        const now = performance.now();
        for (let i = activeRipples.length - 1; i >= 0; i--) {
            const r = activeRipples[i];
            const age = now - r.born;
            if (age >= r.lifetime) {
                scene.remove(r.mesh);
                r.mesh.geometry.dispose();
                r.mesh.material.dispose();
                activeRipples.splice(i, 1);
            } else {
                const progress = age / r.lifetime;
                const scale = 1.0 + progress * 7.0;
                r.mesh.scale.set(scale, scale, 1);
                r.mesh.material.opacity = (1.0 - progress) * 0.9;
            }
        }
    }

    function onWindowResize() {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    }

    function onKeyDown(e) {
        if (e.key === 'c' || e.key === 'C') {
            isChaseCam = !isChaseCam;
            console.log(`[CAM] Chase Cam: ${isChaseCam ? 'ENABLED' : 'DISABLED'}`);
        } else if (e.key === 'r' || e.key === 'R') {
            isChaseCam = false;
            camera.position.set(70, -70, 65);
            controls.target.set(30, 20, 15);
            console.log('[CAM] Camera reset to tactical overview.');
        }
    }

    // Public API for HUD interaction
    window.selectDrone = function(id) {
        targetDroneId = id;
        isChaseCam = true;
        console.log(`[CAM] Focused camera on Drone #${id}`);
    };

    // Public API for Staged Target Preview (TKT-68)
    window.tacticalApp = {
        setStagedTarget: function(pos) {
            if (!stagedBeacon) return;
            stagedBeacon.position.set(pos[0], pos[1], pos[2]);
            stagedBeacon.visible = true;
        },
        clearStagedTarget: function() {
            if (!stagedBeacon) return;
            stagedBeacon.visible = false;
        }
    };

    // Initialize when DOM is ready
    window.addEventListener('DOMContentLoaded', init);

})();
