/**
 * SwarmTech3_UpRising: Cyberpunk Tactical HUD & Command Deck (hud.js)
 * Handles telemetry cards, mission dispatch, formation switching,
 * brain architecture hot-swapping, casualty lab controls, and event logging.
 */

(function () {
    'use strict';

    // State caching
    const droneCards = new Map();
    let currentLeaderId = null;
    let currentFormation = null;
    let currentBrain = null;
    let currentCommsMode = null;
    let isDraggingJammingSlider = false;
    let tetherSevered = false;
    let patrolActive = false;
    let swarmRtbActive = false;
    let overheadMarshalActive = false;
    let recoveryStage = 'IDLE';
    const deadDroneIds = new Set();
    const lostLinkIds = new Set();
    const takeoffDroneIds = new Set();
    const enrouteDroneIds = new Set();
    const jettisonDroneIds = new Set();
    const landedDroneIds = new Set();
    const descendingDroneIds = new Set();

    // DOM Elements
    let cardsContainer = null;
    let eventLogList = null;
    let inputX = null, inputY = null, inputZ = null;

    function init() {
        cardsContainer = document.getElementById('telemetry-cards');
        eventLogList = document.getElementById('event-log-list');
        inputX = document.getElementById('tgt-input-x');
        inputY = document.getElementById('tgt-input-y');
        inputZ = document.getElementById('tgt-input-z');

        setupEventListeners();

        if (window.swarmClient) {
            window.swarmClient.on('telemetry', onTelemetry);
            window.swarmClient.on('command_ack', onCommandAck);
            window.swarmClient.on('status', onWsStatus);
            window.swarmClient.on('hydration', onSessionHydration);
        }

        logEvent('SYSTEM', 'Tactical HUD & Command Deck Initialized.');
    }

    // =========================================================================
    // 1. UI Event Listeners & Command Dispatch
    // =========================================================================
    function setupEventListeners() {
        // Swarm Architecture & Comms Selectors (TKT-17)
        setupBtn('btn-mode-tether', () => dispatchCommsMode('TETHERED_RELAY'));
        setupBtn('btn-mode-star', () => dispatchCommsMode('LEADER_FOLLOWER'));
        setupBtn('btn-mode-mesh', () => dispatchCommsMode('GOSSIP_CONSENSUS'));



        // Formation Selectors
        setupBtn('btn-form-echelon', () => dispatchFormation('ECHELON_ESCORT'));
        setupBtn('btn-form-boids', () => dispatchFormation('NATURAL_FLOCK'));
        setupBtn('btn-form-vortex', () => dispatchFormation('ORBIT_VORTEX'));
        setupBtn('btn-form-patrol', () => dispatchFormation('PERIMETER_IN'));
        setupBtn('btn-form-patrol-out', () => dispatchFormation('PERIMETER_OUT'));

        // Preset Waypoints
        setupBtn('btn-wp-alpha', () => dispatchTarget([60.0, 40.0, 30.0], 'ALPHA'));
        setupBtn('btn-wp-bravo', () => dispatchTarget([-40.0, 70.0, 45.0], 'BRAVO'));
        setupBtn('btn-wp-charlie', () => dispatchTarget([50.0, -50.0, 25.0], 'CHARLIE'));
        setupBtn('btn-wp-rtb', () => dispatchTarget([0.0, 0.0, 20.0], 'RTB_BASE'));

        // Discrete Mission Routes (TKT-28)
        setupBtn('btn-route-box', () => deployRoutePreset('BOX_PATROL'));
        setupBtn('btn-route-diamond', () => deployRoutePreset('DIAMOND_SURVEY'));
        setupBtn('btn-route-hexagon', () => deployRoutePreset('HEXAGON_SWEEP'));
        setupBtn('btn-pause-route', () => togglePauseRoute());
        setupBtn('btn-skip-wp', () => skipRouteWaypoint());
        setupBtn('btn-stop-route', () => stopMissionRoute());

        setupBtn('btn-toggle-patrol', () => {
            logEvent('CMD', 'Toggling continuous 3D target auto-patrol loop: TOGGLE_PATROL');
            window.swarmClient?.send({ cmd: 'TOGGLE_PATROL' });
        });

        // Tactical RNG Touch-and-Go Reconnaissance (TKT-29)
        setupBtn('btn-toggle-rng', () => {
            logEvent('CMD', 'Toggling Tactical RNG Touch-and-Go Reconnaissance: TOGGLE_RNG_RECON');
            window.swarmClient?.send({ cmd: 'TOGGLE_RNG_RECON' });
        });

        // 1-Click Aux Mission: Patrol to BINGO RTB (TKT-58)
        setupBtn('btn-patrol-bingo', () => {
            logEvent('CMD', 'Deploying Aux Mission: Patrol to BINGO RTB (Threshold: 25.0%, Pattern: BOX_PATROL)');
            window.swarmClient?.send({
                cmd: 'DEPLOY_PATROL_TO_BINGO',
                threshold: 25.0,
                pattern: 'BOX_PATROL'
            });
        });

        // 1-Click Aux Mission: Physical Courier / Search-and-Recall Rescue Mule to LKP (TKT-51)
        setupBtn('btn-search-recall', () => {
            const recallBtn = document.getElementById('btn-search-recall');
            if (recallBtn && recallBtn.classList.contains('active-recall')) {
                logEvent('CMD', 'Aborting Aux Mission: Search & Recall Courier Mule');
                window.swarmClient?.send({ cmd: 'STOP_SEARCH_AND_RECALL' });
            } else {
                logEvent('CMD', 'Deploying Aux Mission: Search & Recall Courier Mule to LKP');
                window.swarmClient?.send({ cmd: 'DEPLOY_SEARCH_AND_RECALL' });
            }
        });

        // 1-Click Aux Mission: Cairn G1 Solar Repeater Payload Drop (TKT-59 / PLN-04)
        setupBtn('btn-payload-drop', () => {
            const dropBtn = document.getElementById('btn-payload-drop');
            if (dropBtn && dropBtn.classList.contains('active-payload')) {
                logEvent('CMD', 'Aborting Aux Mission: Cairn G1 Payload Drop');
                window.swarmClient?.send({ cmd: 'STOP_PAYLOAD_DROP' });
            } else {
                logEvent('CMD', 'Deploying Aux Mission: Cairn G1 Repeater Payload Drop');
                window.swarmClient?.send({ cmd: 'DEPLOY_PAYLOAD_DROP' });
            }
        });

        // Custom Target Coordinates
        const btnDispatchCustom = document.getElementById('btn-dispatch-custom');
        if (btnDispatchCustom) {
            btnDispatchCustom.addEventListener('click', () => {
                const x = parseFloat(inputX?.value || 0);
                const y = parseFloat(inputY?.value || 0);
                const z = parseFloat(inputZ?.value || 25);
                dispatchTarget([x, y, z], 'CUSTOM_COORD');
            });
        }

        // 2-Stage Tactical Recovery & Soft-Landing Controls (TKT-19)
        setupBtn('btn-rtb-hold', () => {
            logEvent('CMD', 'Initiating Stage 1: Swarm RTB to overhead sentry hold (0,0,25)');
            window.swarmClient?.send({ cmd: 'RTB_HOLD' });
        });

        setupBtn('btn-land-swarm', () => {
            logEvent('CMD', 'Engaging Stage 2: Swarm ATC landing pad allocation & staggered soft-landing');
            window.swarmClient?.send({ cmd: 'LAND_SWARM' });
        });

        setupBtn('btn-clear-pads', () => {
            logEvent('CMD', 'ATC Manual Override: Clearing occupied pads & releasing next wave from overhead marshal');
            window.swarmClient?.send({ cmd: 'CLEAR_PADS' });
        });

        setupBtn('btn-cancel-rtb', () => {
            logEvent('CMD', 'Cancelling recovery: Resuming normal swarm cruise');
            window.swarmClient?.send({ cmd: 'CANCEL_RTB' });
        });



        setupBtn('btn-scramble-swarm', () => {
            logEvent('CMD', 'Ordering sequenced pad scramble: SCRAMBLE_SWARM');
            window.swarmClient?.send({ cmd: 'SCRAMBLE_SWARM', stagger: 1.2 });
        });

        setupBtn('btn-spawn-pad', () => {
            logEvent('CMD', 'Staging standby reinforcement drone on pad: SPAWN_REINFORCEMENT');
            window.swarmClient?.send({ cmd: 'SPAWN_REINFORCEMENT' });
        });

        setupBtn('btn-reinforce-swarm', () => {
            logEvent('CMD', 'Launching reinforcement drone from base pad: LAUNCH_REINFORCEMENT');
            window.swarmClient?.send({ cmd: 'LAUNCH_REINFORCEMENT' });
        });

        setupBtn('btn-reboot-fleet', () => {
            logEvent('CMD', 'Executing tactical swarm reboot: REBOOT_FLEET');
            window.swarmClient?.send({ cmd: 'REBOOT_FLEET' });
        });

        // Panel Toggle & Edge Restoration Handlers (FIX-03)
        function setLeftPanelCollapsed(collapsed) {
            const lp = document.getElementById('left-panel');
            const lr = document.getElementById('tab-restore-left');
            if (lp) {
                lp.classList.toggle('collapsed', collapsed);
            }
            if (lr) {
                lr.classList.toggle('visible', collapsed);
            }
        }

        function setRightPanelCollapsed(collapsed) {
            const rp = document.getElementById('right-panel');
            const rr = document.getElementById('tab-restore-right');
            if (rp) {
                rp.classList.toggle('collapsed', collapsed);
            }
            if (rr) {
                rr.classList.toggle('visible', collapsed);
            }
        }

        const leftToggle = document.getElementById('toggle-left-panel');
        if (leftToggle) {
            leftToggle.addEventListener('click', (e) => {
                e.preventDefault();
                const lp = document.getElementById('left-panel');
                const isCollapsed = lp?.classList.contains('collapsed');
                setLeftPanelCollapsed(!isCollapsed);
            });
        }
        const leftRestore = document.getElementById('tab-restore-left');
        if (leftRestore) {
            leftRestore.addEventListener('click', (e) => {
                e.preventDefault();
                setLeftPanelCollapsed(false);
            });
        }

        const rightToggle = document.getElementById('toggle-right-panel');
        if (rightToggle) {
            rightToggle.addEventListener('click', (e) => {
                e.preventDefault();
                const rp = document.getElementById('right-panel');
                const isCollapsed = rp?.classList.contains('collapsed');
                setRightPanelCollapsed(!isCollapsed);
            });
        }
        const rightRestore = document.getElementById('tab-restore-right');
        if (rightRestore) {
            rightRestore.addEventListener('click', (e) => {
                e.preventDefault();
                setRightPanelCollapsed(false);
            });
        }
    }

    function setupBtn(id, handler) {
        const btn = document.getElementById(id);
        if (btn) {
            btn.addEventListener('click', (e) => {
                e.preventDefault();
                handler();
            });
        }
    }

    function dispatchCommsMode(modeName) {
        if (modeName === 'TETHERED_RELAY' && currentCommsMode !== 'TETHERED_RELAY') {
            const tetherBtn = document.getElementById('btn-mode-tether');
            if (tetherBtn && tetherBtn.disabled) {
                logEvent('WARN', 'Rejected: Physical tether umbilical cannot be attached mid-flight');
                return;
            }
        }
        logEvent('CMD', `Switching Swarm Architecture -> ${modeName}`);
        window.swarmClient?.send({ cmd: 'SET_COMMS_MODE', mode: modeName });
    }

    function dispatchBrain(brainName) {
        logEvent('CMD', `Hot-swapping Brain Architecture -> ${brainName}`);
        window.swarmClient?.send({ cmd: 'SET_BRAIN', brain: brainName });
    }

    function dispatchFormation(modeName) {
        logEvent('CMD', `Changing Swarm Formation -> ${modeName}`);
        window.swarmClient?.send({ cmd: 'SET_FORMATION', mode: modeName });
    }

    function dispatchTarget(pos, label) {
        patrolActive = false;
        const btnPatrol = document.getElementById('btn-toggle-patrol');
        if (btnPatrol) {
            btnPatrol.classList.remove('active-patrol');
            btnPatrol.innerHTML = '&#x221E; AUTO PATROL LOOP';
        }
        const btnRng = document.getElementById('btn-toggle-rng');
        if (btnRng) {
            btnRng.classList.remove('active-rng');
            btnRng.innerHTML = '&#x1F3B2; RNG RECON';
        }
        logEvent('CMD', `Dispatching Target [${label}] -> (${pos[0]}, ${pos[1]}, ${pos[2]})`);
        window.swarmClient?.send({ cmd: 'SET_TARGET', pos: pos });
        if (inputX) inputX.value = pos[0];
        if (inputY) inputY.value = pos[1];
        if (inputZ) inputZ.value = pos[2];
    }

    function deployRoutePreset(presetName) {
        logEvent('CMD', `Deploying Mission Route Preset -> ${presetName}`);
        window.swarmClient?.send({ cmd: 'START_ROUTE_PRESET', preset: presetName });
    }

    function stopMissionRoute() {
        logEvent('CMD', 'Aborting active mission route: STOP_MISSION_ROUTE');
        window.swarmClient?.send({ cmd: 'STOP_MISSION_ROUTE' });
    }

    function togglePauseRoute() {
        const isPaused = latestTelemetryFrame?.mission_route?.paused;
        if (isPaused) {
            logEvent('CMD', 'Resuming active mission route: RESUME_MISSION_ROUTE');
            window.swarmClient?.send({ cmd: 'RESUME_MISSION_ROUTE' });
        } else {
            logEvent('CMD', 'Pausing active mission route: PAUSE_MISSION_ROUTE');
            window.swarmClient?.send({ cmd: 'PAUSE_MISSION_ROUTE' });
        }
    }

    function skipRouteWaypoint() {
        logEvent('CMD', 'Skipping current waypoint: SKIP_MISSION_WAYPOINT');
        window.swarmClient?.send({ cmd: 'SKIP_MISSION_WAYPOINT' });
    }

    let latestTelemetryFrame = null;

    // =========================================================================
    // 2. Telemetry Ingestion & Real-Time Card Updates
    // =========================================================================
    function onTelemetry(frame) {
        if (!frame) return;
        latestTelemetryFrame = frame;

        // 0. Update Comms Mode & RF Jamming (TKT-17)
        if (frame.comms_mode && frame.comms_mode !== currentCommsMode) {
            currentCommsMode = frame.comms_mode;
            updateCommsButtons(currentCommsMode);
        }

        // Lock TETHER RELAY once airborne if currently in wireless star or P2P mesh
        const isAirborne = Array.isArray(frame.drones) && frame.drones.some(d => !d.crashed && d.flight_state !== 'LANDED' && d.flight_state !== 'GROUND_IDLE');
        const tetherBtn = document.getElementById('btn-mode-tether');
        if (tetherBtn) {
            if (currentCommsMode !== 'TETHERED_RELAY' && isAirborne) {
                tetherBtn.disabled = true;
                tetherBtn.style.opacity = '0.35';
                tetherBtn.style.cursor = 'not-allowed';
                tetherBtn.title = 'HARDLINE TETHER LOCKED: Umbilical anchored at mission prep in WE';
            } else {
                tetherBtn.disabled = false;
                tetherBtn.style.opacity = '1.0';
                tetherBtn.style.cursor = 'pointer';
                tetherBtn.title = 'Hardline tethered leader, star RF followers, immune to EW jamming';
            }
        }
        if (frame.rf_jamming !== undefined && !isDraggingJammingSlider) {
            const pct = Math.round(frame.rf_jamming * 100);
            const slider = document.getElementById('rf-jamming-slider');
            const val = document.getElementById('rf-jamming-val');
            const rfEnvStatus = document.getElementById('rf-env-status');
            if (slider && parseInt(slider.value) !== pct) slider.value = pct;
            if (val) val.textContent = `${pct}% JAMMING`;
            if (rfEnvStatus) {
                rfEnvStatus.innerHTML = pct > 0 ? `RF: <strong style="color:var(--neon-amber)">CONTESTED (${pct}%)</strong>` : `RF: <strong>NOMINAL</strong>`;
            }
        }

        // 1. Update Brain selection highlighting
        if (frame.brain && frame.brain !== currentBrain) {
            currentBrain = frame.brain;
            updateBrainButtons(currentBrain);
        }

        // 2. Update Formation selection highlighting
        if (frame.formation && frame.formation !== currentFormation) {
            currentFormation = frame.formation;
            updateFormationButtons(currentFormation);
        }

        // 3. Monitor Tether Severance
        if (frame.tether) {
            if (frame.tether.severed && !tetherSevered) {
                tetherSevered = true;
                logEvent('ALERT', 'Physical Tether Severed! Primary power cut.');
            } else if (!frame.tether.severed && tetherSevered) {
                tetherSevered = false;
                logEvent('STATUS', 'Tether re-anchored to GCS mast.');
            }
        }

        // 4. Update Target coordinate inputs & Dual Target C2 Status (TKT-26)
        const cmdTarget = frame.commanded_target || frame.target;
        const ackTarget = frame.acknowledged_target || frame.target;

        if (cmdTarget && inputX && !inputX.matches(':focus')) {
            inputX.value = cmdTarget[0];
            inputY.value = cmdTarget[1];
            inputZ.value = cmdTarget[2];
        }

        const valCmdTgt = document.getElementById('val-cmd-tgt');
        const valAckTgt = document.getElementById('val-ack-tgt');
        const tgtStatusLine = document.getElementById('hud-target-status');
        if (valCmdTgt && cmdTarget) {
            valCmdTgt.textContent = `[${cmdTarget[0]}, ${cmdTarget[1]}, ${cmdTarget[2]}]`;
        }
        if (valAckTgt && ackTarget) {
            valAckTgt.textContent = `[${ackTarget[0]}, ${ackTarget[1]}, ${ackTarget[2]}]`;
        }
        if (tgtStatusLine && cmdTarget && ackTarget) {
            const dx = cmdTarget[0] - ackTarget[0];
            const dy = cmdTarget[1] - ackTarget[1];
            const dz = cmdTarget[2] - ackTarget[2];
            const diverged = Math.hypot(dx, dy, dz) > 0.8;
            if (diverged) {
                tgtStatusLine.className = 'target-status-line status-diverged';
            } else {
                tgtStatusLine.className = 'target-status-line status-synced';
            }
        }

        // 4b. Monitor Auto-Patrol Loop State
        if (typeof frame.patrol_active === 'boolean' && frame.patrol_active !== patrolActive) {
            patrolActive = frame.patrol_active;
            const btnPatrol = document.getElementById('btn-toggle-patrol');
            if (btnPatrol) {
                if (patrolActive) {
                    btnPatrol.classList.add('active-patrol');
                    btnPatrol.innerHTML = '&#x221E; PATROL ACTIVE [ON]';
                    logEvent('STATUS', '3D Target Auto-Patrol Loop ENGAGED.');
                } else {
                    btnPatrol.classList.remove('active-patrol');
                    btnPatrol.innerHTML = '&#x221E; AUTO PATROL LOOP';
                    logEvent('STATUS', '3D Target Auto-Patrol Loop DISENGAGED.');
                }
            }
        }

        // 4b2. Monitor Discrete Mission Route State (TKT-28, TKT-30, TKT-56)
        const routeData = frame.mission_route;
        const routeActiveBadge = document.getElementById('route-active-badge');
        const routeControlRow = document.getElementById('route-control-row');
        const btnPauseRoute = document.getElementById('btn-pause-route');
        const btnStopRoute = document.getElementById('btn-stop-route');
        const btnRouteBox = document.getElementById('btn-route-box');
        const btnRouteDiamond = document.getElementById('btn-route-diamond');
        const btnRouteHexagon = document.getElementById('btn-route-hexagon');

        if (routeData && routeData.active) {
            const currentIdx = (routeData.current_wp_idx || 0) + 1;
            const totalWps = (routeData.total_wps || (routeData.waypoints ? routeData.waypoints.length : 0));
            const isPaused = !!routeData.paused;

            if (routeActiveBadge) {
                if (isPaused) {
                    routeActiveBadge.className = 'badge-route-active badge-route-paused';
                    routeActiveBadge.textContent = `${routeData.name} [PAUSED @ WP ${currentIdx}/${totalWps}]`;
                } else if (routeData.loitering && routeData.loiter_timer > 0) {
                    routeActiveBadge.className = 'badge-route-active badge-route-holding';
                    routeActiveBadge.textContent = `${routeData.name} [HOLDING: ${Number(routeData.loiter_timer).toFixed(1)}s (WP ${currentIdx}/${totalWps})]`;
                } else {
                    routeActiveBadge.className = 'badge-route-active';
                    routeActiveBadge.textContent = `${routeData.name} [${currentIdx}/${totalWps}]`;
                }
            }

            if (routeControlRow) routeControlRow.style.display = 'grid';
            if (btnPauseRoute) {
                if (isPaused) {
                    btnPauseRoute.innerHTML = '&#x25B6; RESUME';
                    btnPauseRoute.classList.add('btn-active-pause');
                } else {
                    btnPauseRoute.innerHTML = '&#x23F8; PAUSE';
                    btnPauseRoute.classList.remove('btn-active-pause');
                }
            }
            if (btnStopRoute) btnStopRoute.style.display = 'block';

            if (btnRouteBox) btnRouteBox.classList.toggle('active-route', routeData.name === 'BOX_PATROL');
            if (btnRouteDiamond) btnRouteDiamond.classList.toggle('active-route', routeData.name === 'DIAMOND_SURVEY');
            if (btnRouteHexagon) btnRouteHexagon.classList.toggle('active-route', routeData.name === 'HEXAGON_SWEEP');
        } else {
            if (routeActiveBadge) {
                routeActiveBadge.className = 'badge-route-inactive';
                routeActiveBadge.textContent = 'STANDBY';
            }
            if (routeControlRow) routeControlRow.style.display = 'none';
            if (btnStopRoute) btnStopRoute.style.display = 'none';
            if (btnRouteBox) btnRouteBox.classList.remove('active-route');
            if (btnRouteDiamond) btnRouteDiamond.classList.remove('active-route');
            if (btnRouteHexagon) btnRouteHexagon.classList.remove('active-route');
        }

        // 4b3. Monitor Tactical RNG Reconnaissance State (TKT-29 & TKT-31)
        const rngData = frame.rng_recon;
        const btnRng = document.getElementById('btn-toggle-rng');
        if (rngData && rngData.active) {
            const isOffline = (rngData.status === 'UPLINK_OFFLINE' || !!rngData.paused);
            if (btnRng) {
                btnRng.classList.add('active-rng');
                btnRng.classList.toggle('btn-rng-offline', isOffline);
                if (isOffline) {
                    btnRng.innerHTML = `&#x1F3B2; UPLINK OFFLINE`;
                } else if (rngData.dwelling) {
                    btnRng.innerHTML = `&#x1F3B2; DWELL (${Number(rngData.dwell_timer || 0).toFixed(1)}s)`;
                } else {
                    btnRng.innerHTML = `&#x1F3B2; STATION #${rngData.station_count || 0}`;
                }
            }
            if (routeActiveBadge && (!routeData || !routeData.active)) {
                routeActiveBadge.className = isOffline ? 'badge-route-active badge-route-offline' : 'badge-route-active';
                if (isOffline) {
                    routeActiveBadge.textContent = 'RNG [LINK LOST: HOLDING BEACON]';
                } else if (rngData.dwelling) {
                    routeActiveBadge.textContent = `RNG [DWELL ${Number(rngData.dwell_timer || 0).toFixed(1)}s]`;
                } else {
                    routeActiveBadge.textContent = `RNG [STATION #${rngData.station_count || 0}]`;
                }
            }
        } else {
            if (btnRng) {
                btnRng.classList.remove('active-rng', 'btn-rng-offline');
                btnRng.innerHTML = '&#x1F3B2; RNG RECON';
            }
        }

        // 4b4. Monitor 1-Click Aux Mission: Patrol to BINGO RTB (TKT-58)
        const bingoData = frame.patrol_to_bingo;
        const btnBingo = document.getElementById('btn-patrol-bingo');
        if (bingoData && bingoData.active) {
            if (btnBingo) {
                btnBingo.classList.add('active-bingo');
                if (bingoData.triggered) {
                    btnBingo.innerHTML = `&#x26A1; BINGO RTB [${bingoData.stage}]`;
                } else {
                    btnBingo.innerHTML = `&#x26A1; PATROL &rarr; BINGO [${bingoData.threshold}%]`;
                }
            }
            if (routeActiveBadge) {
                routeActiveBadge.className = 'badge-route-active badge-route-bingo';
                if (bingoData.triggered) {
                    routeActiveBadge.textContent = `BINGO RTB [${bingoData.stage}]`;
                } else {
                    routeActiveBadge.textContent = `PATROL [BINGO ${bingoData.threshold}%]`;
                }
            }
        } else {
            if (btnBingo) {
                btnBingo.classList.remove('active-bingo');
                btnBingo.innerHTML = '&#x26A1; PATROL &rarr; BINGO RTB';
            }
        }

        // 4b5. Monitor 1-Click Aux Mission: Search & Recall Courier Mule (TKT-51)
        const recallData = frame.search_and_recall;
        const btnRecall = document.getElementById('btn-search-recall');
        if (recallData && recallData.active) {
            if (btnRecall) {
                btnRecall.classList.add('active-recall');
                btnRecall.innerHTML = `&#x1F50D; S&amp;R [${recallData.stage}]`;
            }
            if (routeActiveBadge && (!bingoData || !bingoData.active)) {
                routeActiveBadge.className = 'badge-route-active badge-route-recall';
                routeActiveBadge.textContent = `S&amp;R MULE #${recallData.courier_id ?? '?'} [${recallData.stage}]`;
            }
        } else {
            if (btnRecall) {
                btnRecall.classList.remove('active-recall');
                btnRecall.innerHTML = '&#x1F50D; S&amp;R ESCORT (LKP)';
            }
        }

        // 4b6. Monitor 1-Click Aux Mission: Cairn G1 Repeater Payload Drop (TKT-59)
        const payloadData = frame.payload_deposit;
        const btnPayload = document.getElementById('btn-payload-drop');
        if (payloadData && payloadData.active) {
            if (btnPayload) {
                btnPayload.classList.add('active-payload');
                btnPayload.innerHTML = `&#x1F4E6; DROP [${payloadData.stage}]`;
            }
            if (routeActiveBadge && (!bingoData || !bingoData.active) && (!recallData || !recallData.active)) {
                routeActiveBadge.className = 'badge-route-active badge-route-payload';
                routeActiveBadge.textContent = `CAIRN G1 [${payloadData.stage}]`;
            }
        } else {
            if (btnPayload) {
                btnPayload.classList.remove('active-payload');
                const count = payloadData?.repeaters_count ?? 0;
                btnPayload.innerHTML = count > 0 ? `&#x1F4E6; CAIRN G1 DROP (${count} ACTIVE)` : '&#x1F4E6; CAIRN G1 REPEATER DROP';
            }
        }

        // 4c. Monitor Tactical Recovery & Soft-Landing (TKT-19)
        const marshalActive = !!frame.overhead_marshal_active;
        const stage = frame.recovery_stage || 'IDLE';

        if (marshalActive !== overheadMarshalActive) {
            overheadMarshalActive = marshalActive;
            const btnRtb = document.getElementById('btn-rtb-hold');
            if (btnRtb) {
                btnRtb.classList.toggle('active-recovery', overheadMarshalActive);
            }
            if (overheadMarshalActive) {
                logEvent('RECOVERY', 'Stage 1: Swarm transiting to Base (0, 0, 25m) overhead marshal hold.');
            }
        }

        if (stage !== recoveryStage) {
            recoveryStage = stage;
            const btnLand = document.getElementById('btn-land-swarm');
            if (btnLand) {
                btnLand.classList.toggle('active-recovery', recoveryStage === 'LANDING');
            }
            if (recoveryStage === 'LANDING') {
                logEvent('RECOVERY', 'Stage 2: Coordinated ATC Soft-Landing initiated with 2.0s stagger intervals.');
            } else if (recoveryStage === 'IDLE' && !overheadMarshalActive) {
                logEvent('STATUS', 'Recovery sequence cleared. Fleet in normal cruise.');
            }
        }

        const btnCancelRtb = document.getElementById('btn-cancel-rtb');
        if (btnCancelRtb) {
            if (overheadMarshalActive || recoveryStage === 'LANDING') {
                btnCancelRtb.style.display = 'block';
            } else {
                btnCancelRtb.style.display = 'none';
            }
        }

        const btnClearPads = document.getElementById('btn-clear-pads');
        if (btnClearPads) {
            if (frame.atc && (frame.atc.waiting_marshal_count > 0 || frame.atc.turnaround_timer > 0)) {
                btnClearPads.style.display = 'block';
                if (frame.atc.turnaround_timer > 0) {
                    btnClearPads.textContent = `CLEAR PADS // NEXT WAVE (${frame.atc.turnaround_timer}s)`;
                } else {
                    btnClearPads.textContent = `CLEAR PADS // NEXT WAVE (${frame.atc.waiting_marshal_count} WAITING)`;
                }
            } else if (frame.landing_pads && Object.keys(frame.landing_pads).length > 0) {
                btnClearPads.style.display = 'block';
                btnClearPads.textContent = 'CLEAR PADS // NEXT WAVE';
            } else {
                btnClearPads.style.display = 'none';
            }
        }

        // 4d. Update Top Bar Failsafe Indicator (TKT-16, FIX-02)
        const failsafePill = document.getElementById('hud-failsafe-pill');
        const failsafeText = document.getElementById('hud-failsafe-text');
        if (failsafePill && failsafeText) {
            const isTetherSevered = frame.tether && frame.tether.severed && frame.comms_mode === 'TETHERED_RELAY';
            const hasLostLinkLoiter = frame.lost_link_loiter_timer !== undefined && frame.lost_link_loiter_timer > 0;
            const allDronesLostLink = Array.isArray(frame.drones) && frame.drones.length > 0 && frame.drones.every(d => d.crashed || d.lost_link);
            const anyYipLost = Array.isArray(frame.drones) && frame.drones.some(d => !d.crashed && d.yip_lost);
            const anyHoldingHorizon = Boolean(frame.fleet_holding_at_horizon) || (Array.isArray(frame.drones) && frame.drones.some(d => !d.crashed && (d.holding_at_horizon || d.status === 'PERIMETER_HOLD' || d.flight_state === 'PERIMETER_HOLD')));
            const anyBacktracking = Boolean(frame.fleet_backtracking) || (Array.isArray(frame.drones) && frame.drones.some(d => !d.crashed && (d.flight_state === 'BACKTRACK_HOLD' || d.status === 'BACKTRACK_HOLD')));

            if (anyHoldingHorizon || anyBacktracking || isTetherSevered || hasLostLinkLoiter || allDronesLostLink || frame.swarm_rtb_active || anyYipLost) {
                failsafePill.classList.remove('hidden');
                if (anyHoldingHorizon) {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-loiter';
                    failsafeText.textContent = 'FLEET HOLDING AT RADIO HORIZON (OUT-OF-RANGE TARGET)';
                } else if (anyBacktracking) {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-loiter';
                    failsafeText.textContent = 'WATCHDOG TIMEOUT: BACKTRACKING ALONG BREADCRUMBS';
                } else if (frame.recovery_stage === 'LANDING' || frame.recovery_stage === 'COMPLETE') {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-rtb';
                    failsafeText.textContent = 'TACTICAL RECOVERY: COORD SOFT-LANDING ACTIVE';
                } else if (frame.swarm_rtb_active) {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-rtb';
                    failsafeText.textContent = 'UPLINK LOST: DOCTRINE A SWARM RTB TO BASE ACTIVE';
                } else if (hasLostLinkLoiter) {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-loiter';
                    const loiterCause = (frame.comms_mode === 'TETHERED_RELAY') ? 'TETHER SEVERED' : 'RF HORIZON EXCEEDED';
                    failsafeText.textContent = `${loiterCause}: AUTONOMOUS LOITER ${frame.lost_link_loiter_timer.toFixed(1)}s -> RTB`;
                } else if (isTetherSevered) {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-untethered';
                    failsafeText.textContent = 'UNTETHERED CRUISE (MISSION RESUMED)';
                } else if (allDronesLostLink) {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-loiter';
                    failsafeText.textContent = 'RF LINK LOST: FLEET OUTSIDE 140m HOWL HORIZON';
                } else if (anyYipLost) {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-untethered';
                    failsafeText.textContent = 'UPLINK: ACTIVE | DOWNLINK: SILENT (YIP LOST)';
                } else {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-loiter';
                    failsafeText.textContent = 'RF LINK LOST: FLEET OUTSIDE 140m HOWL HORIZON';
                }
            } else {
                failsafePill.classList.add('hidden');
            }
        }

        // Failsafe Event Logging
        if (frame.tether && frame.tether.severed && !tetherSevered) {
            tetherSevered = true;
            const twoIcStr = frame.two_ic_id !== null && frame.two_ic_id !== undefined ? `#${frame.two_ic_id}` : '#1';
            logEvent('FAILSAFE', `Tactical umbilical severed! 2IC ${twoIcStr} assumed command. Drone #0 breakaway dive initiated.`);
            logEvent('DOCTRINE_A', `Doctrine A engaged: Holding 30s defensive loiter before automated RTB.`);
        } else if (frame.tether && !frame.tether.severed && tetherSevered) {
            tetherSevered = false;
            logEvent('STATUS', `Tactical umbilical reconnected / swarm rebooted.`);
        }

        if (frame.swarm_rtb_active && !swarmRtbActive) {
            swarmRtbActive = true;
            logEvent('RTB', `Doctrine A 30s loiter grace period expired. Swarm RTB initiated to Base (0, 0, 25m).`);
        } else if (!frame.swarm_rtb_active && swarmRtbActive) {
            swarmRtbActive = false;
        }

        // 5. Update Drone Telemetry Cards
        if (Array.isArray(frame.drones) && cardsContainer) {
            let leaderFound = null;

            // Purge card DOM elements and tracking sets for vanished drones (e.g. cleared wreckage)
            const activeIds = new Set(frame.drones.map(d => d.id));
            for (let [id, card] of droneCards.entries()) {
                if (!activeIds.has(id)) {
                    card.el.remove();
                    droneCards.delete(id);
                    deadDroneIds.delete(id);
                    lostLinkIds.delete(id);
                    takeoffDroneIds.delete(id);
                    enrouteDroneIds.delete(id);
                    landedDroneIds.delete(id);
                    descendingDroneIds.delete(id);
                }
            }

            frame.drones.forEach(d => {
                if (d.is_leader && !d.crashed) {
                    leaderFound = d.id;
                }

                // Detect Autonomic Events & State Changes
                const dName = d.tail_number || `Drone #${d.id}`;
                if (d.crashed && !deadDroneIds.has(d.id)) {
                    deadDroneIds.add(d.id);
                    logEvent('CASUALTY', `${dName} crashed! Ballistic drop initiated.`);
                } else if (!d.crashed && deadDroneIds.has(d.id)) {
                    deadDroneIds.delete(d.id);
                    logEvent('STATUS', `${dName} rebooted and active.`);
                }

                if (d.flight_state === 'TAKEOFF' && !takeoffDroneIds.has(d.id)) {
                    takeoffDroneIds.add(d.id);
                    logEvent('LAUNCH', `Reinforcement ${dName} launched from base pad.`);
                } else if (d.flight_state !== 'TAKEOFF' && takeoffDroneIds.has(d.id)) {
                    takeoffDroneIds.delete(d.id);
                    if (d.flight_state === 'EN_ROUTE') {
                        logEvent('STATUS', `${dName} reached transition altitude (20m). En route to formation.`);
                    }
                }

                if (d.flight_state === 'EN_ROUTE' && !enrouteDroneIds.has(d.id)) {
                    enrouteDroneIds.add(d.id);
                } else if (d.flight_state === 'CRUISE' && enrouteDroneIds.has(d.id)) {
                    enrouteDroneIds.delete(d.id);
                    logEvent('STATUS', `${dName} reached formation slot. Status: CRUISE.`);
                }

                if (d.flight_state === 'DESCENDING' && !descendingDroneIds.has(d.id)) {
                    descendingDroneIds.add(d.id);
                    const padStr = d.landing_pad_idx !== undefined && d.landing_pad_idx !== null ? `Pad #${d.landing_pad_idx}` : 'designated pad';
                    logEvent('ATC', `${dName} allocated ${padStr}. Commencing 2-stage deceleration descent.`);
                } else if (d.flight_state !== 'DESCENDING' && descendingDroneIds.has(d.id)) {
                    descendingDroneIds.delete(d.id);
                }

                const isGrounded = (d.flight_state === 'LANDED' || d.flight_state === 'GROUND_IDLE');
                if (isGrounded && !landedDroneIds.has(d.id)) {
                    landedDroneIds.add(d.id);
                    const padStr = d.landing_pad_idx !== undefined && d.landing_pad_idx !== null ? `Pad #${d.landing_pad_idx}` : 'pad';
                    if (d.flight_state === 'GROUND_IDLE') {
                        logEvent('STATUS', `${dName} staged on ${padStr} in GROUND IDLE.`);
                    } else {
                        logEvent('TOUCHDOWN', `${dName} touched down on ${padStr}. Motors disarmed (0 RPM).`);
                    }
                } else if (!isGrounded && landedDroneIds.has(d.id)) {
                    landedDroneIds.delete(d.id);
                    logEvent('TAKEOFF', `${dName} launching from pad. Rejoining swarm.`);
                }

                if (d.lost_link && !lostLinkIds.has(d.id)) {
                    lostLinkIds.add(d.id);
                    logEvent('WARN', `${dName} LOST LINK! Seeking LKP (${d.lkp[0]}, ${d.lkp[1]}, ${d.lkp[2]}).`);
                } else if (!d.lost_link && lostLinkIds.has(d.id)) {
                    lostLinkIds.delete(d.id);
                    logEvent('STATUS', `${dName} link restored. Rejoined swarm.`);
                }

                if (d.status === 'BREAKAWAY_JETTISON' && !jettisonDroneIds.has(d.id)) {
                    jettisonDroneIds.add(d.id);
                    logEvent('FAILSAFE', `${dName} executing 3D breakaway dive (ΔZ=-4m) and cable purge.`);
                } else if (d.status !== 'BREAKAWAY_JETTISON' && jettisonDroneIds.has(d.id)) {
                    jettisonDroneIds.delete(d.id);
                    logEvent('STATUS', `${dName} umbilical purge complete. Rejoining formation as unencumbered escort.`);
                }

                // Update / Build Card DOM
                try {
                    updateOrCreateDroneCard(d);
                } catch (cardErr) {
                    console.error(`[GCS HUD] Error updating card for Drone #${d?.id}:`, cardErr);
                }
            });

            // Detect Autonomic Succession
            if (leaderFound !== null && currentLeaderId !== null && leaderFound !== currentLeaderId) {
                logEvent('SUCCESSION', `Autonomic Succession: Drone #${leaderFound} promoted to Swarm Leader!`);
            }
            currentLeaderId = leaderFound;
        }
    }

    function updateOrCreateDroneCard(drone) {
        let card = droneCards.get(drone.id);

        if (!card) {
            try {
                card = createCardElement(drone);
                cardsContainer.appendChild(card.el);
                droneCards.set(drone.id, card);
            } catch (err) {
                console.error(`[GCS HUD] Failed to create telemetry card for Drone #${drone?.id}:`, err);
                const fallbackEl = document.createElement('div');
                fallbackEl.className = 'drone-telemetry-card crashed-card';
                fallbackEl.id = `card-drone-${drone?.id ?? 'err'}`;
                fallbackEl.innerHTML = `
                    <div class="card-header">
                        <strong class="drone-id-label">DRONE #${drone?.id ?? '?'}</strong>
                        <span class="status-tag tag-crashed">RENDER ERROR</span>
                    </div>
                    <div class="card-briefing-sub" style="display:block; color:#ff4466;">Telemetry render failed</div>
                `;
                cardsContainer.appendChild(fallbackEl);
                droneCards.set(drone.id, { el: fallbackEl });
                return;
            }
        }

        // Update In-Place for High-FPS smoothness
        if (card.elIdLabel) {
            card.elIdLabel.textContent = drone.tail_number || `ALITHEIA-V${String(drone.id + 1).padStart(2, '0')}`;
        }
        if (card.elSysidBadge) {
            card.elSysidBadge.textContent = `SYSID ${drone.sysid || (drone.id + 1)}`;
        }
        if (card.elTierBadge) {
            card.elTierBadge.textContent = drone.full_tier_designation || `TIER-${drone.tier || 1}`;
        }
        if (card.elUidSub) {
            card.elUidSub.textContent = `UID: ${drone.hardware_uid || 'UNKNOWN'}`;
        }
        if (card.elBriefingSub) {
            if (drone.briefing && (drone.flight_state === 'GROUND_IDLE' || drone.flight_state === 'TAKEOFF' || drone.enrollment_status === 'SCOUT')) {
                card.elBriefingSub.style.display = 'block';
                card.elBriefingSub.textContent = drone.briefing;
            } else {
                card.elBriefingSub.style.display = 'none';
            }
        }

        const vel = drone.vel || [0, 0, 0];
        const pos = drone.pos || [0, 0, 0];
        const battery = typeof drone.battery === 'number' ? drone.battery : 100;
        const speed = Math.hypot(vel[0] || 0, vel[1] || 0, vel[2] || 0).toFixed(1);
        const alt = (pos[2] || 0).toFixed(1);

        if (card.elAlt) card.elAlt.textContent = `${alt}m`;
        if (card.elVel) card.elVel.textContent = `${speed}m/s`;
        if (card.elBat) card.elBat.textContent = `${battery.toFixed(0)}%`;
        if (card.elBatBar) card.elBatBar.style.width = `${Math.max(0, Math.min(100, battery))}%`;

        // Signal RSSI & Link Quality (TKT-17)
        if (card.elRssi) {
            const rssi = drone.rssi !== undefined ? drone.rssi.toFixed(0) : '-45';
            const lq = drone.link_quality !== undefined ? drone.link_quality : 1.0;
            card.elRssi.textContent = `${rssi}dBm`;
            if (lq < 0.35) {
                card.elRssi.style.color = 'var(--neon-red)';
            } else if (lq < 0.7) {
                card.elRssi.style.color = 'var(--neon-amber)';
            } else {
                card.elRssi.style.color = 'var(--neon-cyan)';
            }
            if (card.elSignalBar) {
                const sigPct = Math.max(0, Math.min(100, Math.round(lq * 100)));
                card.elSignalBar.style.width = `${sigPct}%`;
                if (lq < 0.35) {
                    card.elSignalBar.style.background = 'var(--neon-red)';
                } else if (lq < 0.7) {
                    card.elSignalBar.style.background = 'var(--neon-amber)';
                } else {
                    card.elSignalBar.style.background = 'linear-gradient(90deg, var(--neon-amber), var(--neon-cyan))';
                }
            }
        }

        // Status Styling & Badges
        card.el.classList.remove('leader-card', 'crashed-card', 'lkp-card', 'gossip-card', 'takeoff-card', 'enroute-card', 'jettison-card', 'landed-card', 'landing-card', 'hold-card', 'card-gnss-denied');
        card.elRole.className = 'drone-role-badge';

        function setActionBtn(text, className, title) {
            if (!card.elActionBtn) return;
            card.elActionBtn.textContent = text;
            card.elActionBtn.className = className;
            if (title) card.elActionBtn.title = title;
        }

        if (drone.crashed || drone.declared_destroyed) {
            card.el.classList.add('crashed-card');
            const crashStatus = drone.declared_destroyed
                ? 'DECLARED LOST'
                : (drone.collision_inferred
                    ? 'COLLISION CONFIRMED'
                    : (drone.mayday_beacon ? 'IMPACT CONFIRMED' : 'CRASHED'));
            card.elStatus.textContent = crashStatus;
            card.elStatus.className = 'status-tag tag-crashed';
            card.elRole.textContent = drone.declared_destroyed
                ? 'OPERATOR DECLARED'
                : (drone.collision_inferred ? 'PROP CLASH' : 'CASUALTY');
            if (card.elLandBtn) card.elLandBtn.style.display = 'none';
        } else if (drone.flight_state === 'LANDED' || drone.flight_state === 'GROUND_IDLE') {
            card.el.classList.add('landed-card');
            const padStr = drone.landing_pad_idx !== undefined && drone.landing_pad_idx !== null ? `P${drone.landing_pad_idx}` : '';
            if (drone.flight_state === 'GROUND_IDLE') {
                card.elStatus.textContent = padStr ? `GROUND [${padStr}]` : 'GROUND IDLE';
                card.elStatus.className = 'status-tag tag-landed';
                card.elRole.textContent = 'STANDBY';
            } else {
                card.elStatus.textContent = padStr ? `LANDED [${padStr}]` : 'LANDED';
                card.elStatus.className = 'status-tag tag-landed';
                card.elRole.textContent = 'DISARMED';
            }
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAUNCH';
                card.elLandBtn.className = 'card-btn-action btn-launch';
            }
            setActionBtn('KILL', 'card-btn-action btn-kill');
        } else if (drone.flight_state === 'DESCENDING') {
            card.el.classList.add('landing-card');
            const padStr = drone.landing_pad_idx !== undefined && drone.landing_pad_idx !== null ? `P${drone.landing_pad_idx}` : '';
            card.elStatus.textContent = padStr ? `LANDING [${padStr}]` : 'LANDING';
            card.elStatus.className = 'status-tag tag-landing';
            card.elRole.textContent = 'ATC DESCENT';
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LANDING';
                card.elLandBtn.className = 'card-btn-action btn-landing-busy';
            }
            setActionBtn('KILL', 'card-btn-action btn-kill');
        } else if (drone.gnss_denied) {
            card.el.classList.add('card-gnss-denied');
            if (drone.gnss_settled) {
                card.elStatus.textContent = 'BLIND SETTLING';
                card.elStatus.className = 'status-tag tag-gnss-denied';
                card.elRole.textContent = 'SOFT DESCENT';
            } else {
                card.elStatus.textContent = 'DEAD-RECKONING RTB';
                card.elStatus.className = 'status-tag tag-gnss-denied';
                card.elRole.textContent = `GNSS DENIED [±${Number(drone.drift_radius || 0).toFixed(1)}m]`;
            }
            if (card.elLandBtn) card.elLandBtn.style.display = 'none';
            setActionBtn('KILL', 'card-btn-action btn-kill');
        } else if (drone.status === 'BREAKAWAY_JETTISON') {
            card.el.classList.add('jettison-card');
            card.elStatus.textContent = 'JETTISONING';
            card.elStatus.className = 'status-tag tag-jettison';
            card.elRole.textContent = '⏏ JETTISON';
            card.elRole.classList.add('badge-jettison');
            if (card.elLandBtn) card.elLandBtn.style.display = 'none';
            setActionBtn('KILL', 'card-btn-action btn-kill');
        } else if (drone.lost_link) {
            card.el.classList.add('lkp-card');
            card.elStatus.textContent = 'LKP RECOVERY';
            card.elStatus.className = 'status-tag tag-lkp';
            const rfMode = drone.rf_power_mode || 'YIP';
            card.elRole.textContent = drone.is_leader ? `LKP (LEADER) • ${rfMode}` : `LKP • ${rfMode}`;
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'DECLARE DESTROYED';
                card.elLandBtn.className = 'card-btn-action btn-danger';
                card.elLandBtn.title = 'Manual Operator Action: Confirm airframe destroyed at LKP';
            }
        } else if (drone.is_leader) {
            card.el.classList.add('leader-card');
            card.elStatus.textContent = overheadMarshalActive ? 'OVERHEAD HOLD' : 'LEADER';
            card.elStatus.className = overheadMarshalActive ? 'status-tag tag-hold' : 'status-tag tag-leader';
            const rfMode = drone.rf_power_mode || 'BORK';
            card.elRole.textContent = `FLAGSHIP • ${rfMode}`;
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAND';
                card.elLandBtn.className = 'card-btn-action btn-land';
            }
            setActionBtn('KILL', 'card-btn-action btn-kill');
        } else if (drone.is_2ic) {
            card.elStatus.textContent = overheadMarshalActive ? 'OVERHEAD HOLD' : 'ACTIVE';
            card.elStatus.className = overheadMarshalActive ? 'status-tag tag-hold' : 'status-tag tag-active';
            const rfMode = drone.rf_power_mode || 'YIP';
            card.elRole.textContent = `2IC • ${rfMode}`;
            card.elRole.classList.add('badge-2ic');
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAND';
                card.elLandBtn.className = 'card-btn-action btn-land';
            }
            setActionBtn('KILL', 'card-btn-action btn-kill');
        } else if (drone.flight_state === 'TAKEOFF') {
            card.el.classList.add('takeoff-card');
            card.elStatus.textContent = 'TAKEOFF';
            card.elStatus.className = 'status-tag tag-takeoff';
            const rfMode = drone.rf_power_mode || 'YIP';
            card.elRole.textContent = (drone.enrolled === false) ? 'SCOUT TAKEOFF' : `TAKEOFF • ${rfMode}`;
            if (card.elLandBtn) card.elLandBtn.style.display = 'none';
            setActionBtn('KILL', 'card-btn-action btn-kill');
        } else if (drone.flight_state === 'EN_ROUTE') {
            card.el.classList.add('enroute-card');
            card.elStatus.textContent = (drone.enrolled === false) ? 'SCOUT REJOIN' : 'EN ROUTE';
            card.elStatus.className = 'status-tag tag-enroute';
            const rfMode = drone.rf_power_mode || 'YIP';
            card.elRole.textContent = (drone.enrolled === false) ? 'SCOUT' : `EN ROUTE • ${rfMode}`;
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAND';
                card.elLandBtn.className = 'card-btn-action btn-land';
            }
            setActionBtn('KILL', 'card-btn-action btn-kill');
        } else if (drone.yip_lost) {
            card.el.classList.add('yip-lost-card');
            card.elStatus.textContent = 'YIP LOST';
            card.elStatus.className = 'status-tag tag-yip-lost';
            const relayStr = (drone.gateway_id !== null && drone.gateway_id !== undefined && drone.gateway_id !== drone.id)
                ? ` [RELAY #${drone.gateway_id}]` : ' [SILENT]';
            card.elRole.textContent = `LOST • YIP${relayStr}`;
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAND';
                card.elLandBtn.className = 'card-btn-action btn-land';
            }
            setActionBtn('KILL', 'card-btn-action btn-kill');
        } else if (drone.status === 'PROPAGATING') {
            card.el.classList.add('gossip-card');
            card.elStatus.textContent = 'ECHO WAVE';
            card.elStatus.className = 'status-tag tag-gossip';
            const relayStr = (drone.gateway_id !== null && drone.gateway_id !== undefined && drone.gateway_id !== drone.id)
                ? ` [GW #${drone.gateway_id}]` : '';
            const rfMode = drone.rf_power_mode || 'YIP';
            card.elRole.textContent = `ECHO • ${rfMode}${relayStr}`;
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAND';
                card.elLandBtn.className = 'card-btn-action btn-land';
            }
            setActionBtn('KILL', 'card-btn-action btn-kill');
        } else {
            card.elStatus.textContent = overheadMarshalActive ? 'OVERHEAD HOLD' : 'ACTIVE';
            card.elStatus.className = overheadMarshalActive ? 'status-tag tag-hold' : 'status-tag tag-active';
            const relayStr = (drone.gateway_id !== null && drone.gateway_id !== undefined && drone.gateway_id !== drone.id)
                ? ` [GW #${drone.gateway_id}]` : '';
            const rfMode = drone.rf_power_mode || 'YIP';
            card.elRole.textContent = `ESCORT • ${rfMode}${relayStr}`;
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAND';
                card.elLandBtn.className = 'card-btn-action btn-land';
            }
            setActionBtn('KILL', 'card-btn-action btn-kill');
        }
    }

    function createCardElement(drone) {
        const el = document.createElement('div');
        el.className = 'drone-telemetry-card';
        el.id = `card-drone-${drone.id}`;

        const tailNum = drone.tail_number || `ALITHEIA-V${String(drone.id + 1).padStart(2, '0')}`;
        const sysId = drone.sysid || (drone.id + 1);
        const hwUid = drone.hardware_uid || 'UNKNOWN';
        const tierDesig = drone.full_tier_designation || `TIER-${drone.tier || 1}`;

        el.innerHTML = `
            <div class="card-header">
                <div class="card-id-group">
                    <span class="drone-dot"></span>
                    <strong class="drone-id-label">${tailNum}</strong>
                    <span class="drone-sysid-badge">SYSID ${sysId}</span>
                    <span class="drone-tier-badge">${tierDesig}</span>
                    <span class="drone-role-badge">STANDBY</span>
                </div>
                <span class="status-tag tag-active">ACTIVE</span>
            </div>
            <div class="card-uid-sub">UID: ${hwUid}</div>
            <div class="card-briefing-sub" style="display: none;"></div>
            <div class="card-metrics">
                <div class="metric-col">
                    <span class="metric-label">ALT</span>
                    <strong class="metric-val metric-alt">0.0m</strong>
                </div>
                <div class="metric-col">
                    <span class="metric-label">SPEED</span>
                    <strong class="metric-val metric-vel">0.0m/s</strong>
                </div>
                <div class="metric-col">
                    <span class="metric-label">BATTERY</span>
                    <strong class="metric-val metric-bat">100%</strong>
                </div>
                <div class="metric-col">
                    <span class="metric-label">SIGNAL</span>
                    <strong class="metric-val metric-rssi">-45dBm</strong>
                </div>
            </div>
            <div class="card-bars-container">
                <div class="card-battery-progress" title="Battery Level">
                    <div class="card-battery-fill" style="width: 100%;"></div>
                </div>
                <div class="card-signal-progress" title="RF Link Quality">
                    <div class="card-signal-fill" style="width: 100%;"></div>
                </div>
            </div>
            <div class="card-footer">
                <button class="card-btn-action btn-land-action btn-land" style="width: 100%;">LAND</button>
            </div>
        `;

        const landBtn = el.querySelector('.btn-land-action');
        const actionBtn = el.querySelector('.btn-kill') || el.querySelector('.card-btn-action:not(.btn-land-action)') || null;

        if (actionBtn) {
            actionBtn.addEventListener('click', (e) => {
                e.stopPropagation();
                if (deadDroneIds.has(drone.id)) {
                    logEvent('CMD', `Rebooting drone #${drone.id}`);
                    window.swarmClient?.send({ cmd: 'REBOOT_DRONE', id: drone.id });
                } else {
                    logEvent('CMD', `Terminating drone #${drone.id}`);
                    window.swarmClient?.send({ cmd: 'KILL_DRONE', id: drone.id });
                }
            });
        }

        if (landBtn) {
            landBtn.addEventListener('click', (e) => {
                e.stopPropagation();
                if (drone.lost_link && !deadDroneIds.has(drone.id)) {
                    logEvent('CMD', `Operator confirmed Drone #${drone.id} destroyed: DECLARE_DESTROYED`);
                    window.swarmClient?.send({ cmd: 'DECLARE_DESTROYED', id: drone.id });
                } else if (landedDroneIds.has(drone.id)) {
                    logEvent('CMD', `Launching Drone #${drone.id} from pad: TAKEOFF_DRONE`);
                    window.swarmClient?.send({ cmd: 'TAKEOFF_DRONE', id: drone.id });
                } else {
                    logEvent('CMD', `Commanding Drone #${drone.id} to soft-land: LAND_DRONE`);
                    window.swarmClient?.send({ cmd: 'LAND_DRONE', id: drone.id });
                }
            });
        }

        el.addEventListener('click', () => {
            logEvent('NAV', `Tactical camera focused on Drone #${drone.id}`);
            if (window.selectDrone) {
                window.selectDrone(drone.id);
            }
        });

        return {
            el: el,
            elIdLabel: el.querySelector('.drone-id-label'),
            elSysidBadge: el.querySelector('.drone-sysid-badge'),
            elTierBadge: el.querySelector('.drone-tier-badge'),
            elUidSub: el.querySelector('.card-uid-sub'),
            elBriefingSub: el.querySelector('.card-briefing-sub'),
            elAlt: el.querySelector('.metric-alt'),
            elVel: el.querySelector('.metric-vel'),
            elBat: el.querySelector('.metric-bat'),
            elBatBar: el.querySelector('.card-battery-fill'),
            elRssi: el.querySelector('.metric-rssi'),
            elSignalBar: el.querySelector('.card-signal-fill'),
            elStatus: el.querySelector('.status-tag'),
            elRole: el.querySelector('.drone-role-badge'),
            elLandBtn: landBtn,
            elActionBtn: actionBtn,
        };
    }

    // =========================================================================
    // 3. Selection Highlighting
    // =========================================================================
    function updateCommsButtons(activeMode) {
        const tetherBtn = document.getElementById('btn-mode-tether');
        const starBtn = document.getElementById('btn-mode-star');
        const meshBtn = document.getElementById('btn-mode-mesh');

        [tetherBtn, starBtn, meshBtn].forEach(b => b?.classList.remove('active-selection'));

        if (activeMode === 'TETHERED_RELAY') {
            tetherBtn?.classList.add('active-selection');
        } else if (activeMode === 'LEADER_FOLLOWER') {
            starBtn?.classList.add('active-selection');
        } else if (activeMode === 'GOSSIP_CONSENSUS') {
            meshBtn?.classList.add('active-selection');
        }
    }

    function updateBrainButtons(activeBrain) {
        const lfBtn = document.getElementById('btn-brain-lf');
        const gossipBtn = document.getElementById('btn-brain-gossip');

        if (activeBrain === 'LEADER_FOLLOWER') {
            lfBtn?.classList.add('active-selection');
            gossipBtn?.classList.remove('active-selection');
        } else if (activeBrain === 'GOSSIP_CONSENSUS') {
            gossipBtn?.classList.add('active-selection');
            lfBtn?.classList.remove('active-selection');
        }
    }

    function updateFormationButtons(activeFormation) {
        const map = {
            'ECHELON_ESCORT': 'btn-form-echelon',
            'NATURAL_FLOCK': 'btn-form-boids',
            'ORBIT_VORTEX': 'btn-form-vortex',
            'PERIMETER_PATROL': 'btn-form-patrol',
            'PERIMETER_IN': 'btn-form-patrol',
            'PERIMETER_OUT': 'btn-form-patrol-out',
        };

        for (let [mode, btnId] of Object.entries(map)) {
            const btn = document.getElementById(btnId);
            if (btn) {
                if (mode === activeFormation) {
                    btn.classList.add('active-selection');
                } else {
                    btn.classList.remove('active-selection');
                }
            }
        }
    }

    // =========================================================================
    // 4. Tactical Event Logging Feed
    // =========================================================================
    function logEvent(type, message) {
        if (!eventLogList) return;

        const timeStr = new Date().toTimeString().split(' ')[0];
        const item = document.createElement('li');
        item.className = `event-item type-${type.toLowerCase()}`;
        item.innerHTML = `<span class="event-time">[${timeStr}]</span> <span class="event-type">[${type}]</span> <span class="event-msg">${message}</span>`;

        eventLogList.appendChild(item);

        // Keep maximum 30 log lines
        while (eventLogList.children.length > 30) {
            eventLogList.removeChild(eventLogList.firstChild);
        }

        // Auto-scroll to latest
        eventLogList.scrollTop = eventLogList.scrollHeight;
    }

    function onCommandAck(ack) {
        if (!ack || !ack.cmd) return;
        if (ack.status === 'OUT_OF_RANGE' || ack.status === 'DROPPED') {
            logEvent('WARN', `[RF REJECTED] ${ack.cmd}: ${ack.error || ack.reason || 'Fleet outside 140m GCS Howl horizon'}`);
            const failPill = document.getElementById('hud-failsafe-pill');
            const failText = document.getElementById('hud-failsafe-text');
            if (failPill && failText) {
                failPill.classList.remove('hidden');
                failText.textContent = 'RF LINK OUT OF RANGE (>140m)';
                setTimeout(() => failPill.classList.add('hidden'), 4000);
            }
            return;
        }
        if (ack.cmd === 'CLEAR_WRECKAGE' && Array.isArray(ack.purged_ids)) {
            logEvent('ACK', `Salvage complete: Cleared ${ack.count} wreckage [${ack.purged_ids.join(', ')}]. Remaining: ${ack.remaining_drones}.`);
        } else {
            logEvent('ACK', `Command ${ack.cmd} acknowledged: ${JSON.stringify(ack)}`);
        }
    }

    function onSessionHydration(bundle) {
        if (!bundle) return;

        // 1. Rehydrate Event History Log (TKT-32)
        if (bundle.event_history && Array.isArray(bundle.event_history) && eventLogList) {
            eventLogList.innerHTML = '';
            bundle.event_history.forEach(evt => {
                const timeStr = evt.wall_time || (evt.sim_time !== undefined && evt.sim_time !== null ? `T+${evt.sim_time}s` : '00:00:00');
                const item = document.createElement('li');
                item.className = `event-item type-${(evt.type || 'info').toLowerCase()}`;
                item.innerHTML = `<span class="event-time">[${timeStr}]</span> <span class="event-type">[${evt.type}]</span> <span class="event-msg">${evt.message}</span>`;
                eventLogList.appendChild(item);
            });
            while (eventLogList.children.length > 50) {
                eventLogList.removeChild(eventLogList.firstChild);
            }
            eventLogList.scrollTop = eventLogList.scrollHeight;
        }

        // 2. Rehydrate Comms Mode Selector
        if (bundle.comms_mode) {
            ['btn-mode-tether', 'btn-mode-star', 'btn-mode-mesh'].forEach(id => {
                const el = document.getElementById(id);
                if (el) {
                    if ((id === 'btn-mode-tether' && bundle.comms_mode === 'TETHERED_RELAY') ||
                        (id === 'btn-mode-star' && bundle.comms_mode === 'LEADER_FOLLOWER') ||
                        (id === 'btn-mode-mesh' && bundle.comms_mode === 'GOSSIP_CONSENSUS')) {
                        el.classList.add('active');
                    } else {
                        el.classList.remove('active');
                    }
                }
            });
        }

        // 3. Rehydrate Formation Mode Selector
        if (bundle.formation) {
            const formMap = {
                'ECHELON_ESCORT': 'btn-form-echelon',
                'NATURAL_FLOCK': 'btn-form-boids',
                'ORBIT_VORTEX': 'btn-form-vortex',
                'PERIMETER_PATROL': 'btn-form-patrol',
                'PERIMETER_IN': 'btn-form-patrol',
                'PERIMETER_OUT': 'btn-form-patrol-out'
            };
            const activeId = formMap[bundle.formation];
            ['btn-form-echelon', 'btn-form-boids', 'btn-form-vortex', 'btn-form-patrol', 'btn-form-patrol-out'].forEach(id => {
                const el = document.getElementById(id);
                if (el) {
                    if (id === activeId) el.classList.add('active');
                    else el.classList.remove('active');
                }
            });
        }

        // 4. Rehydrate RF Jamming Slider & Display
        if (bundle.rf_jamming !== undefined) {
            const jammingSlider = document.getElementById('slider-rf-jamming');
            const jammingReadout = document.getElementById('jamming-value');
            if (jammingSlider) {
                jammingSlider.value = Math.round(bundle.rf_jamming * 100);
            }
            if (jammingReadout) {
                jammingReadout.textContent = `${Math.round(bundle.rf_jamming * 100)}%`;
            }
        }

        // 5. Rehydrate Target Inputs
        if (bundle.target && Array.isArray(bundle.target) && bundle.target.length === 3) {
            if (inputX) inputX.value = bundle.target[0];
            if (inputY) inputY.value = bundle.target[1];
            if (inputZ) inputZ.value = bundle.target[2];
        }

        logEvent('HYDRATE', `Session state hydrated: ${bundle.event_history ? bundle.event_history.length : 0} historical events synchronized.`);
    }

    function onWsStatus(connected) {
        if (connected) {
            logEvent('NET', 'Duplex WebSocket link operational (60 FPS telemetry stream).');
        } else {
            logEvent('NET', 'WebSocket disconnected. Entering reconnection loop...');
        }
    }

    // Export logger for global access (e.g. from app.js)
    window.tacticalHud = {
        logEvent: logEvent,
    };

    window.addEventListener('DOMContentLoaded', init);
})();
