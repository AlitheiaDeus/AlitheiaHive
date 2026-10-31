/**
 * SwarmTech3_UpRising: SITL God-View HUD & Fault Injection Deck (sim_hud.js)
 * Manages the God-mode casualty deck, physical truth telemetry cards,
 * Zone 4 safety separation monitoring, and SITL event logging.
 */

(function () {
    'use strict';

    let droneCards = new Map();
    let deadDroneIds = new Set();
    let currentLeaderId = 0;
    let isDraggingJammingSlider = false;
    let zone4BreachesCount = 0;

    // DOM Elements
    let elSimTime, elDroneCount, elTetherStatus, elRfStatus, elConnStatus;
    let elMinDist, elZone4Breaches, elTetherLen, elJammingVal;
    let truthCardsContainer, eventLogList;

    function init() {
        // Top Header
        elSimTime = document.getElementById('hud-sim-time');
        elDroneCount = document.getElementById('hud-drones');
        elTetherStatus = document.getElementById('hud-tether');
        elRfStatus = document.getElementById('hud-rf');
        elConnStatus = document.getElementById('hud-conn-text');

        // Metrics Deck
        elMinDist = document.getElementById('val-min-dist');
        elZone4Breaches = document.getElementById('val-zone4-breaches');
        elTetherLen = document.getElementById('val-tether-len');
        elJammingVal = document.getElementById('rf-jamming-val');

        truthCardsContainer = document.getElementById('truth-cards-container');
        eventLogList = document.getElementById('event-log-list');

        setupEventListeners();
        setupClientCallbacks();

        logEvent('SYSTEM', 'SITL Simulation Lab HUD Initialized. Omniscient truth physics active.');
    }

    function setupEventListeners() {
        // 1. Electronic Warfare Jamming Slider
        const jammingSlider = document.getElementById('rf-jamming-slider');
        if (jammingSlider) {
            jammingSlider.addEventListener('input', (e) => {
                isDraggingJammingSlider = true;
                const pct = parseInt(e.target.value);
                const rate = pct / 100.0;
                if (elJammingVal) elJammingVal.textContent = `${pct}% JAMMING`;
                window.swarmClient?.send({ cmd: 'SET_RF_JAMMING', rate: rate });
            });
            jammingSlider.addEventListener('change', () => {
                isDraggingJammingSlider = false;
            });
        }

        // Tactical GNSS Denial & Dead-Reckoning RTB Trigger (TKT-52)
        setupBtn('btn-toggle-gnss', () => {
            logEvent('EW', 'Toggling Tactical GNSS Denial across Swarm: TOGGLE_GNSS_DENIAL');
            window.swarmClient?.send({ cmd: 'TOGGLE_GNSS_DENIAL' });
        });

        // 2. Physical Fault Injection Deck
        setupBtn('btn-kill-leader', () => {
            logEvent('CASUALTY', 'Injected flagship motor failure: KILL_LEADER');
            window.swarmClient?.send({ cmd: 'KILL_LEADER' });
        });

        setupBtn('btn-cut-tether', () => {
            logEvent('FAILSAFE', 'Manually severed umbilical cable: CUT_TETHER');
            window.swarmClient?.send({ cmd: 'CUT_TETHER' });
        });

        setupBtn('btn-kill-random', () => {
            const aliveEscorts = [];
            for (let [id, card] of droneCards) {
                if (!deadDroneIds.has(id) && id !== currentLeaderId) {
                    aliveEscorts.push(id);
                }
            }
            if (aliveEscorts.length > 0) {
                const victimId = aliveEscorts[Math.floor(Math.random() * aliveEscorts.length)];
                logEvent('CASUALTY', `Injected motor failure on escort: KILL_DRONE #${victimId}`);
                window.swarmClient?.send({ cmd: 'KILL_DRONE', id: victimId });
            } else {
                logEvent('WARN', 'No living escorts available to terminate.');
            }
        });

        setupBtn('btn-kill-select', () => {
            const sel = document.getElementById('select-kill-drone');
            if (sel) {
                const droneId = parseInt(sel.value);
                logEvent('CASUALTY', `Terminating selected airframe: KILL_DRONE #${droneId}`);
                window.swarmClient?.send({ cmd: 'KILL_DRONE', id: droneId });
            }
        });

        // 3. Wreckage Salvage & Fleet Resurrect
        setupBtn('btn-respawn-dead', () => {
            const deadCount = deadDroneIds.size;
            if (deadCount > 0) {
                logEvent('STATUS', `Respawning ${deadCount} destroyed airframe(s) in-place: REBOOT_ALL_DEAD`);
                window.swarmClient?.send({ cmd: 'REBOOT_ALL_DEAD' });
            } else {
                logEvent('INFO', 'No destroyed casualties to respawn. Fleet 100% operational.');
            }
        });

        setupBtn('btn-clear-wreckage', () => {
            const deadCount = deadDroneIds.size;
            if (deadCount > 0) {
                logEvent('STATUS', `Purging ${deadCount} crashed wreckage from battlefield: CLEAR_WRECKAGE`);
                window.swarmClient?.send({ cmd: 'CLEAR_WRECKAGE' });
            } else {
                logEvent('INFO', 'No wreckage on battlefield to salvage.');
            }
        });

        setupBtn('btn-retrieve-pads', () => {
            logEvent('CMD', 'Ground crew retrieving landed airframes/wreckage from launch pads: RETRIEVE_ALL_PADS');
            window.swarmClient?.send({ cmd: 'RETRIEVE_ALL_PADS' });
        });

        setupBtn('btn-cold-reset', () => {
            logEvent('CMD', 'Resetting fleet to cold ground pads: COLD_RESET');
            window.swarmClient?.send({ cmd: 'COLD_RESET' });
        });

        setupBtn('btn-reinforce-swarm', () => {
            logEvent('CMD', 'Spawning reinforcement airframe at base pad: LAUNCH_REINFORCEMENT');
            window.swarmClient?.send({ cmd: 'LAUNCH_REINFORCEMENT' });
        });

        setupBtn('btn-reboot-fleet', () => {
            logEvent('CMD', 'Full simulation factory reset: REBOOT_FLEET');
            window.swarmClient?.send({ cmd: 'REBOOT_FLEET' });
        });

        // 4. Panel Collapsing & Edge Tabs (FIX-03)
        const btnToggleLeft = document.getElementById('toggle-left-panel');
        const btnToggleRight = document.getElementById('toggle-right-panel');
        const tabRestoreLeft = document.getElementById('tab-restore-left');
        const tabRestoreRight = document.getElementById('tab-restore-right');

        function setLeftCollapsed(collapsed) {
            const p = document.getElementById('left-panel');
            if (p) p.classList.toggle('collapsed', collapsed);
            if (tabRestoreLeft) tabRestoreLeft.classList.toggle('visible', collapsed);
        }

        function setRightCollapsed(collapsed) {
            const p = document.getElementById('right-panel');
            if (p) p.classList.toggle('collapsed', collapsed);
            if (tabRestoreRight) tabRestoreRight.classList.toggle('visible', collapsed);
        }

        if (btnToggleLeft) btnToggleLeft.addEventListener('click', () => setLeftCollapsed(true));
        if (tabRestoreLeft) tabRestoreLeft.addEventListener('click', () => setLeftCollapsed(false));

        if (btnToggleRight) btnToggleRight.addEventListener('click', () => setRightCollapsed(true));
        if (tabRestoreRight) tabRestoreRight.addEventListener('click', () => setRightCollapsed(false));
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

    function setupClientCallbacks() {
        if (!window.swarmClient) return;

        window.swarmClient.on('status', (online) => {
            if (elConnStatus) {
                elConnStatus.textContent = online ? 'SITL ONLINE' : 'DISCONNECTED';
            }
            const ind = document.getElementById('hud-conn-ind');
            if (ind) {
                ind.classList.toggle('disconnected', !online);
            }
        });

        window.swarmClient.on('telemetry', (frame) => {
            updateSimHUD(frame);
        });

        window.swarmClient.on('hydration', (data) => {
            logEvent('SYSTEM', `Session Hydrated: ${data.event_history?.length || 0} events loaded.`);
            if (Array.isArray(data.event_history)) {
                data.event_history.forEach(evt => {
                    logEvent(evt.tag || 'SYSTEM', evt.msg || evt.text || '', evt.time);
                });
            }
        });
    }

    function updateSimHUD(frame) {
        if (!frame) return;

        // Top Bar
        if (elSimTime) elSimTime.textContent = `${frame.sim_time.toFixed(1)}s`;

        const droneList = frame.sim_drones || frame.drones || [];
        const totalDrones = droneList.length;
        const deadCount = droneList.filter(d => d.crashed).length;
        const aliveCount = totalDrones - deadCount;

        if (elDroneCount) elDroneCount.textContent = `${aliveCount}/${totalDrones} ALIVE`;

        // Tether Status
        if (frame.tether) {
            if (frame.tether.severed) {
                if (elTetherStatus) elTetherStatus.innerHTML = '<strong style="color:var(--neon-red)">SEVERED</strong>';
                if (elTetherLen) elTetherLen.textContent = 'SEVERED';
            } else if (!frame.tether.deployed) {
                if (elTetherStatus) elTetherStatus.innerHTML = '<strong>STOWED</strong>';
                if (elTetherLen) elTetherLen.textContent = 'STOWED';
            } else {
                if (elTetherStatus) elTetherStatus.innerHTML = '<strong style="color:var(--neon-green)">INTACT</strong>';
                if (elTetherLen) elTetherLen.textContent = `${frame.tether.length.toFixed(1)}m`;
            }
        }

        // RF Jamming Slider Sync
        if (!isDraggingJammingSlider && frame.rf_jamming !== undefined) {
            const pct = Math.round(frame.rf_jamming * 100);
            const jammingSlider = document.getElementById('rf-jamming-slider');
            if (jammingSlider) jammingSlider.value = pct;
            if (elJammingVal) elJammingVal.textContent = `${pct}% JAMMING`;
            if (elRfStatus) {
                elRfStatus.innerHTML = pct > 0 ? `<strong style="color:var(--neon-amber)">CONTESTED (${pct}%)</strong>` : '<strong>NOMINAL</strong>';
            }
        }

        // Zone 4 Separation & Distance Calculation
        if (droneList.length > 0) {
            updateTruthCards(droneList);
            updateKillSelector(droneList);
        }
    }

    function updateTruthCards(drones) {
        if (!truthCardsContainer) return;

        // Calculate minimum inter-drone distances
        let minDist = 999.0;
        const posMap = new Map();
        drones.forEach(d => {
            if (!d.crashed) posMap.set(d.id, d.pos);
        });

        posMap.forEach((p1, id1) => {
            posMap.forEach((p2, id2) => {
                if (id1 < id2) {
                    const dx = p1[0] - p2[0];
                    const dy = p1[1] - p2[1];
                    const dz = p1[2] - p2[2];
                    const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
                    if (d < minDist) minDist = d;
                }
            });
        });

        if (elMinDist) {
            if (minDist < 900) {
                elMinDist.textContent = `${minDist.toFixed(2)}m`;
                if (minDist <= 1.5) {
                    elMinDist.className = 'sim-metric-val val-red';
                    zone4BreachesCount++;
                } else if (minDist <= 3.5) {
                    elMinDist.className = 'sim-metric-val val-amber';
                } else {
                    elMinDist.className = 'sim-metric-val val-green';
                }
            } else {
                elMinDist.textContent = '--';
            }
        }

        if (elZone4Breaches) {
            elZone4Breaches.textContent = `${zone4BreachesCount}`;
            if (zone4BreachesCount > 0) elZone4Breaches.className = 'sim-metric-val val-red';
        }

        // Render Truth Cards
        const currentIds = new Set();
        drones.forEach(d => {
            currentIds.add(d.id);
            if (d.crashed) {
                deadDroneIds.add(d.id);
            } else {
                deadDroneIds.delete(d.id);
            }
            if (d.is_leader) currentLeaderId = d.id;

            let card = droneCards.get(d.id);
            if (!card) {
                card = createTruthCardElement(d);
                truthCardsContainer.appendChild(card.el);
                droneCards.set(d.id, card);
            }

            // Update in-place
            card.elPos.textContent = `X:${d.pos[0].toFixed(1)} Y:${d.pos[1].toFixed(1)} Z:${d.pos[2].toFixed(1)}`;
            const speed = Math.hypot(d.vel[0], d.vel[1], d.vel[2]).toFixed(1);
            card.elSpeed.textContent = `${speed}m/s`;
            card.elBat.textContent = `${d.battery.toFixed(0)}%`;

            if (d.crashed) {
                card.el.className = 'drone-truth-card crashed';
                card.elTag.textContent = 'CRASHED';
                card.elTag.className = 'status-tag tag-crashed';
                card.btnKill.style.display = 'none';
                card.btnRevive.style.display = 'inline-block';
            } else if (d.gnss_denied) {
                card.el.className = 'drone-truth-card gnss-denied';
                card.elTag.textContent = d.gnss_settled ? 'BLIND SETTLE' : 'DEAD-RECKONING RTB';
                card.elTag.className = 'status-tag tag-gnss-denied';
                card.btnKill.style.display = 'inline-block';
                card.btnRevive.style.display = 'none';
            } else if (d.is_leader) {
                card.el.className = 'drone-truth-card';
                card.elTag.textContent = 'FLAGSHIP';
                card.elTag.className = 'status-tag tag-leader';
                card.btnKill.style.display = 'inline-block';
                card.btnRevive.style.display = 'none';
            } else {
                card.el.className = 'drone-truth-card';
                card.elTag.textContent = `ESCORT [T${d.tier || 1}]`;
                card.elTag.className = 'status-tag';
                card.btnKill.style.display = 'inline-block';
                card.btnRevive.style.display = 'none';
            }
        });

        // Clean up purged cards
        for (const [id, card] of droneCards.entries()) {
            if (!currentIds.has(id)) {
                card.el.remove();
                droneCards.delete(id);
            }
        }
    }

    function createTruthCardElement(d) {
        const el = document.createElement('div');
        el.className = 'drone-truth-card';
        el.id = `truth-card-${d.id}`;

        el.innerHTML = `
            <div class="card-header-line">
                <span class="drone-id-tag">DRONE #${d.id}</span>
                <span class="status-tag">ACTIVE</span>
            </div>
            <div class="coord-grid">
                <span class="card-pos">X:0.0 Y:0.0 Z:0.0</span>
                <span>SPD: <strong class="card-spd">0.0m/s</strong></span>
                <span>BAT: <strong class="card-bat">100%</strong></span>
            </div>
            <div class="card-actions-row">
                <button class="btn-card-kill">KILL MOTOR</button>
                <button class="btn-card-revive" style="display:none;">REVIVE</button>
            </div>
        `;

        const btnKill = el.querySelector('.btn-card-kill');
        const btnRevive = el.querySelector('.btn-card-revive');

        btnKill.addEventListener('click', () => {
            logEvent('CASUALTY', `Terminating Drone #${d.id}`);
            window.swarmClient?.send({ cmd: 'KILL_DRONE', id: d.id });
        });

        btnRevive.addEventListener('click', () => {
            logEvent('STATUS', `Reviving Drone #${d.id}`);
            window.swarmClient?.send({ cmd: 'REBOOT_DRONE', id: d.id });
        });

        return {
            el: el,
            elTag: el.querySelector('.status-tag'),
            elPos: el.querySelector('.card-pos'),
            elSpeed: el.querySelector('.card-spd'),
            elBat: el.querySelector('.card-bat'),
            btnKill: btnKill,
            btnRevive: btnRevive,
        };
    }

    function updateKillSelector(drones) {
        const sel = document.getElementById('select-kill-drone');
        if (!sel) return;

        const currentVal = sel.value;
        sel.innerHTML = '';
        drones.forEach(d => {
            if (!d.crashed) {
                const opt = document.createElement('option');
                opt.value = d.id;
                opt.textContent = `Drone #${d.id} (${d.is_leader ? 'Flagship' : 'Escort'})`;
                sel.appendChild(opt);
            }
        });
        if (currentVal) sel.value = currentVal;
    }

    function logEvent(tag, msg, customTime) {
        if (!eventLogList) return;

        const now = customTime !== undefined ? customTime.toFixed(1) : (elSimTime ? elSimTime.textContent : '0.0s');
        const li = document.createElement('li');
        li.className = 'log-entry';

        li.innerHTML = `
            <span class="log-time">[${now}]</span>
            <span class="log-tag tag-${tag}">${tag}</span>
            <span class="log-msg">${msg}</span>
        `;

        eventLogList.appendChild(li);

        // Keep maximum 80 entries in DOM
        while (eventLogList.children.length > 80) {
            eventLogList.removeChild(eventLogList.firstChild);
        }

        eventLogList.scrollTop = eventLogList.scrollHeight;
    }

    // Expose for external calls
    window.simHud = {
        init: init,
        logEvent: logEvent,
    };

    window.addEventListener('DOMContentLoaded', () => {
        init();
    });
})();
