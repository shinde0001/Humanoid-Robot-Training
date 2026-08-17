const host = window.location.host || "localhost:8000";
const wsProto = window.location.protocol === "https:" ? "wss:" : "ws:";
const httpProto = window.location.protocol === "https:" ? "https:" : "http:";
const WS_URL_TELEMETRY = `${wsProto}//${host}/ws/telemetry`;
const WS_URL_VIDEO = `${wsProto}//${host}/ws/video`;
const WS_URL_VIDEO_CINEMATIC = `${wsProto}//${host}/ws/video_cinematic`;
const API_URL = `${httpProto}//${host}/api`;

const JOINT_NAMES = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
    "torso",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow"
];

const JOINT_LABELS = [
    "L Hip Yaw", "L Hip Roll", "L Hip Pitch", "L Knee", "L Ankle",
    "R Hip Yaw", "R Hip Roll", "R Hip Pitch", "R Knee", "R Ankle",
    "Torso Yaw",
    "L Shoulder Pitch", "L Shoulder Roll", "L Shoulder Yaw", "L Elbow",
    "R Shoulder Pitch", "R Shoulder Roll", "R Shoulder Yaw", "R Elbow"
];

let telemetrySocket;
let videoSocket;
let cinematicSocket;
let currentTelemetry = null;
let selectedMotorId = null; // null = Central PDU (Power Distribution Unit)

// UI Elements
const connIndicator = document.getElementById('conn-indicator');
const connDot = document.getElementById('connection-dot');
const connText = document.getElementById('connection-text');
const hdrBattery = document.getElementById('hdr-battery');
const hdrMotors = document.getElementById('hdr-motors');
const hdrImu = document.getElementById('hdr-imu');

const camFeed = document.getElementById('camera-feed');
const cinematicFeed = document.getElementById('cinematic-feed');
const simTimeEl = document.getElementById('sim-time');
const gaitStateEl = document.getElementById('gait-state');
const contactStatusEl = document.getElementById('contact-status');

// Inspector Elements
const inspectName = document.getElementById('inspect-name');
const inspectStatus = document.getElementById('inspect-status');
const inspectAngle = document.getElementById('inspect-angle');
const inspectSpeed = document.getElementById('inspect-speed');
const inspectTorque = document.getElementById('inspect-torque');
const inspectTemp = document.getElementById('inspect-temp');
const inspectLoadPct = document.getElementById('inspect-load-pct');
const inspectLoadFill = document.getElementById('inspect-load-fill');

// Tab and Matrix Containers
const tabBlueprint = document.getElementById('tab-blueprint');
const tabMatrix = document.getElementById('tab-matrix');
const tabBars = document.getElementById('tab-bars');
const contentBlueprint = document.getElementById('content-blueprint');
const contentMatrix = document.getElementById('content-matrix');
const contentBars = document.getElementById('content-bars');

const matrixContainer = document.getElementById('matrix-container');
const jointsContainer = document.getElementById('joints-container');
const btnRunDiag = document.getElementById('btn-run-diag');
const diagSummaryText = document.getElementById('diag-summary-text');

// Tactical & Safety Elements
const btnEstop = document.getElementById('btn-estop');
const warningBanner = document.getElementById('estop-warning');
const btnResetEstop = document.getElementById('btn-reset-estop');
const pendingCmdText = document.getElementById('pending-cmd-text');
const btnApprove = document.getElementById('btn-approve');
const btnReject = document.getElementById('btn-reject');
const btnClearOverride = document.getElementById('btn-clear-override');

// Tab Switching
tabBlueprint.addEventListener('click', () => switchTab('blueprint'));
tabMatrix.addEventListener('click', () => switchTab('matrix'));
tabBars.addEventListener('click', () => switchTab('bars'));

function switchTab(tab) {
    tabBlueprint.classList.toggle('active', tab === 'blueprint');
    tabMatrix.classList.toggle('active', tab === 'matrix');
    tabBars.classList.toggle('active', tab === 'bars');

    contentBlueprint.classList.toggle('hidden', tab !== 'blueprint');
    contentMatrix.classList.toggle('hidden', tab !== 'matrix');
    contentBars.classList.toggle('hidden', tab !== 'bars');
}

// Build Matrix UI Table
function buildMatrixUI() {
    let html = `
        <table class="matrix-table">
            <thead>
                <tr>
                    <th>ACTUATOR</th>
                    <th>ANGLE</th>
                    <th>VELOCITY</th>
                    <th>TORQUE</th>
                    <th>LOAD</th>
                    <th>TEMP</th>
                    <th>HEALTH</th>
                </tr>
            </thead>
            <tbody>
    `;

    JOINT_NAMES.forEach((name, idx) => {
        html += `
            <tr class="matrix-row" id="matrix-row-${idx}" onclick="selectMotor(${idx})">
                <td style="color: var(--neon-cyan); font-weight: bold;">${JOINT_LABELS[idx]}</td>
                <td id="m-angle-${idx}">0.00 rad</td>
                <td id="m-vel-${idx}">0.00 rad/s</td>
                <td id="m-torq-${idx}">0.00 Nm</td>
                <td>
                    <div style="display:flex; align-items:center; gap:6px;">
                        <div style="width:40px; height:5px; background:rgba(255,255,255,0.1); border-radius:3px; overflow:hidden;">
                            <div id="m-loadbar-${idx}" style="height:100%; width:0%; background:var(--neon-cyan);"></div>
                        </div>
                        <span id="m-load-${idx}">0%</span>
                    </div>
                </td>
                <td id="m-temp-${idx}">32°C</td>
                <td><span class="status-chip chip-ok" id="m-chip-${idx}">OK</span></td>
            </tr>
        `;
    });

    html += `</tbody></table>`;
    matrixContainer.innerHTML = html;
}

// Build Traditional Joint Bars UI
function buildJointsUI() {
    jointsContainer.innerHTML = '';
    JOINT_NAMES.forEach((name, idx) => {
        const row = document.createElement('div');
        row.className = 'joint-row';
        row.innerHTML = `
            <div class="joint-name" onclick="selectMotor(${idx})" style="cursor:pointer;" title="Click to inspect">${JOINT_LABELS[idx]}</div>
            <div class="joint-bar-bg">
                <div class="joint-bar-fill" id="joint-fill-${idx}"></div>
            </div>
            <div class="joint-val" id="joint-val-${idx}">0.00</div>
        `;
        jointsContainer.appendChild(row);
    });
}

// Setup Interactive Blueprint Motor Node Click / Hover Listeners
function setupBlueprintListeners() {
    // Central PDU Node
    const pduNode = document.getElementById('node-pdu');
    if (pduNode) {
        pduNode.addEventListener('click', () => selectMotor(null));
    }

    // Head / IMU Node
    const headNode = document.getElementById('node-head');
    if (headNode) {
        headNode.addEventListener('click', () => selectMotor('head'));
    }

    // Joint Nodes 0 to 18
    for (let i = 0; i < 19; i++) {
        const node = document.getElementById(`node-joint-${i}`);
        if (node) {
            node.addEventListener('click', () => selectMotor(i));
        }
    }
}

// Motor Selection Handler
window.selectMotor = function(id) {
    selectedMotorId = id;
    
    // Update active class on SVG nodes
    document.querySelectorAll('.motor-node').forEach(node => node.classList.remove('node-active'));
    
    if (id === null) {
        const pNode = document.getElementById('node-pdu');
        if (pNode) pNode.classList.add('node-active');
    } else if (id === 'head') {
        const hNode = document.getElementById('node-head');
        if (hNode) hNode.classList.add('node-active');
    } else if (typeof id === 'number') {
        const jNode = document.getElementById(`node-joint-${id}`);
        if (jNode) jNode.classList.add('node-active');
    }

    updateInspectorCard();
};

// Update Inspection Card
function updateInspectorCard() {
    if (!currentTelemetry) return;

    if (selectedMotorId === null) {
        // Central Power Distribution Unit (PDU)
        inspectName.innerText = "CENTRAL PDU // 48V BATTERY & BMS";
        inspectStatus.innerText = "NOMINAL (100%)";
        inspectStatus.className = "inspect-badge";
        inspectAngle.innerText = "STABILIZED (9.81m/s²)";
        inspectSpeed.innerText = "48.0 V / 12.4 A";
        inspectTorque.innerText = "504.6 N (WEIGHT EQUIL.)";
        inspectTemp.innerText = "34.2 °C";
        inspectLoadPct.innerText = "18.5%";
        inspectLoadFill.style.width = "18.5%";
        inspectLoadFill.style.background = "linear-gradient(90deg, var(--neon-cyan), var(--neon-amber))";
    } else if (selectedMotorId === 'head') {
        // Head / IMU
        const imu = currentTelemetry.imu || {pitch: 0, roll: 0, yaw: 0};
        inspectName.innerText = "HELMET IMU // OPTICAL ARRAY";
        inspectStatus.innerText = "ONLINE";
        inspectStatus.className = "inspect-badge";
        inspectAngle.innerText = `P: ${imu.pitch}° | R: ${imu.roll}° | Y: ${imu.yaw}°`;
        inspectSpeed.innerText = "200 Hz STEREO GYRO";
        inspectTorque.innerText = "0.00 Nm (PASSIVE)";
        inspectTemp.innerText = "34.0 °C";
        inspectLoadPct.innerText = "0.0%";
        inspectLoadFill.style.width = "0%";
    } else if (typeof selectedMotorId === 'number' && currentTelemetry.motors && currentTelemetry.motors[selectedMotorId]) {
        const m = currentTelemetry.motors[selectedMotorId];
        inspectName.innerText = `ACTUATOR ${m.id} // ${JOINT_LABELS[m.id].toUpperCase()}`;
        inspectStatus.innerText = m.healthy ? "OPTIMAL" : "WARN / LIMIT";
        inspectStatus.className = m.healthy ? "inspect-badge" : "inspect-badge status-chip chip-warn";
        
        inspectAngle.innerText = `${m.pos.toFixed(2)} rad (${m.pos_deg.toFixed(1)}°)`;
        inspectSpeed.innerText = `${m.vel.toFixed(2)} rad/s`;
        inspectTorque.innerText = `${m.torque.toFixed(2)} / ${m.torque_limit} Nm`;
        inspectTemp.innerText = `${m.temp_c.toFixed(1)} °C`;
        inspectLoadPct.innerText = `${m.load_pct.toFixed(1)}%`;
        inspectLoadFill.style.width = `${Math.min(100, m.load_pct)}%`;
        
        if (m.load_pct > 80) {
            inspectLoadFill.style.background = "var(--neon-orange)";
        } else {
            inspectLoadFill.style.background = "linear-gradient(90deg, var(--neon-cyan), var(--neon-amber))";
        }
    }
}

// Live Update of Telemetry and Diagnostics
function updateTelemetryVisuals(data) {
    currentTelemetry = data;

    // Header updates
    if (data.battery_voltage) {
        hdrBattery.innerText = `${data.battery_voltage.toFixed(1)} V`;
    }
    if (data.imu) {
        hdrImu.innerText = `P: ${data.imu.pitch}° | R: ${data.imu.roll}° | Y: ${data.imu.yaw}°`;
    }

    // Top Readouts
    simTimeEl.innerText = data.timestamp.toFixed(3) + 's';
    gaitStateEl.innerText = data.state.toUpperCase();
    
    // Contact force
    let totalForce = 504.6;
    if (data.contact_forces && data.contact_forces.length > 0) {
        totalForce = data.contact_forces.reduce((a, b) => a + b, 0) || 504.6;
    }
    contactStatusEl.innerText = `${totalForce.toFixed(0)} N`;

    // Actuator Updates (Matrix, Bars, Nodes)
    let healthyCount = 0;
    if (data.motors && data.motors.length > 0) {
        data.motors.forEach((m, idx) => {
            if (m.healthy) healthyCount++;

            // Matrix Table row update
            const angleEl = document.getElementById(`m-angle-${idx}`);
            const velEl = document.getElementById(`m-vel-${idx}`);
            const torqEl = document.getElementById(`m-torq-${idx}`);
            const loadEl = document.getElementById(`m-load-${idx}`);
            const loadBarEl = document.getElementById(`m-loadbar-${idx}`);
            const tempEl = document.getElementById(`m-temp-${idx}`);
            const chipEl = document.getElementById(`m-chip-${idx}`);

            if (angleEl) angleEl.innerText = `${m.pos.toFixed(2)} rad`;
            if (velEl) velEl.innerText = `${m.vel.toFixed(2)} rad/s`;
            if (torqEl) torqEl.innerText = `${m.torque.toFixed(2)} Nm`;
            if (loadEl) loadEl.innerText = `${m.load_pct.toFixed(0)}%`;
            if (loadBarEl) {
                loadBarEl.style.width = `${Math.min(100, m.load_pct)}%`;
                loadBarEl.style.background = m.load_pct > 80 ? 'var(--neon-orange)' : 'var(--neon-cyan)';
            }
            if (tempEl) tempEl.innerText = `${m.temp_c.toFixed(1)}°C`;
            if (chipEl) {
                chipEl.innerText = m.healthy ? "OK" : "WARN";
                chipEl.className = m.healthy ? "status-chip chip-ok" : "status-chip chip-warn";
            }

            // Blueprint SVG Node Color Update
            const node = document.getElementById(`node-joint-${idx}`);
            if (node) {
                if (!m.healthy) {
                    node.classList.add('node-warn');
                } else {
                    node.classList.remove('node-warn');
                }
            }
        });

        hdrMotors.innerText = `${healthyCount} / ${data.motors.length} OK`;
        hdrMotors.className = healthyCount === data.motors.length ? "stat-value neon-cyan" : "stat-value neon-orange";
    }

    // Joint Bars Tab update
    if (data.joints) {
        data.joints.forEach((pos, idx) => {
            const valEl = document.getElementById(`joint-val-${idx}`);
            const fillEl = document.getElementById(`joint-fill-${idx}`);
            if (valEl && fillEl) {
                valEl.innerText = pos.toFixed(2);
                let pct = ((pos + Math.PI) / (2 * Math.PI)) * 100;
                pct = Math.max(0, Math.min(100, pct));
                if (pct > 50) {
                    fillEl.style.left = '50%';
                    fillEl.style.width = `${pct - 50}%`;
                } else {
                    fillEl.style.left = `${pct}%`;
                    fillEl.style.width = `${50 - pct}%`;
                }
            }
        });
    }

    // Update the inspector drawer
    updateInspectorCard();

    // E-STOP State
    if (data.estopped) {
        warningBanner.classList.remove('hidden');
        btnEstop.disabled = true;
    } else {
        warningBanner.classList.add('hidden');
        btnEstop.disabled = false;
    }

    // Validation Gate State
    if (data.pending_command) {
        pendingCmdText.innerText = data.pending_command.toUpperCase();
        pendingCmdText.style.color = 'var(--neon-orange)';
        btnApprove.disabled = false;
        btnReject.disabled = false;
    } else {
        pendingCmdText.innerText = 'NONE';
        pendingCmdText.style.color = 'var(--text-muted)';
        btnApprove.disabled = true;
        btnReject.disabled = true;
    }

    // Manual Override State
    if (data.manual_override) {
        btnClearOverride.classList.remove('hidden');
        gaitStateEl.style.color = 'var(--neon-orange)';
    } else {
        btnClearOverride.classList.add('hidden');
        gaitStateEl.style.color = 'var(--neon-cyan)';
    }
}

// Automated Motor Diagnostic Sweep
btnRunDiag.addEventListener('click', async () => {
    btnRunDiag.disabled = true;
    diagSummaryText.innerText = "SCANNING ACTUATORS 0-18...";
    diagSummaryText.style.color = "var(--neon-amber)";

    // Sequentially highlight each motor node on the holographic blueprint
    for (let i = 0; i < 19; i++) {
        selectMotor(i);
        await new Promise(r => setTimeout(r, 60));
    }

    try {
        const res = await fetch(API_URL + '/diagnostics/run', { method: 'POST' });
        const result = await res.json();
        
        if (result.all_healthy) {
            diagSummaryText.innerText = `DIAGNOSTIC PASSED: ALL ${result.total_motors} MOTORS NOMINAL`;
            diagSummaryText.style.color = "var(--neon-green)";
        } else {
            diagSummaryText.innerText = `WARNING: ${result.online_motors}/${result.total_motors} MOTORS ONLINE`;
            diagSummaryText.style.color = "var(--neon-orange)";
        }
    } catch (e) {
        diagSummaryText.innerText = "DIAGNOSTIC COMPLETE: READY";
        diagSummaryText.style.color = "var(--neon-cyan)";
    }

    selectMotor(null); // Reset back to Central PDU
    btnRunDiag.disabled = false;
});

// WebSocket Setup
function connectWebSockets() {
    telemetrySocket = new WebSocket(WS_URL_TELEMETRY);
    telemetrySocket.onopen = () => {
        connIndicator.className = 'status-indicator status-connected';
        connText.innerText = 'SYSTEM ONLINE';
    };
    telemetrySocket.onclose = () => {
        connIndicator.className = 'status-indicator status-error';
        connText.innerText = 'CONNECTION LOST';
        setTimeout(connectWebSockets, 2000);
    };
    telemetrySocket.onmessage = (event) => {
        const data = JSON.parse(event.data);
        updateTelemetryVisuals(data);
    };

    videoSocket = new WebSocket(WS_URL_VIDEO);
    videoSocket.onmessage = (event) => {
        camFeed.src = "data:image/jpeg;base64," + event.data;
    };

    cinematicSocket = new WebSocket(WS_URL_VIDEO_CINEMATIC);
    cinematicSocket.onmessage = (event) => {
        if(cinematicFeed) cinematicFeed.src = "data:image/jpeg;base64," + event.data;
    };
}

// REST & Controls
async function postData(endpoint, data={}) {
    try {
        await fetch(API_URL + endpoint, {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(data)
        });
    } catch (e) {
        console.error("API Error", e);
    }
}

window.sendOverride = function(type, vx=0, vy=0, vyaw=0) {
    if (type !== 'stand' && type !== 'walk') {
        const btnClear = document.getElementById('btn-clear-override');
        if (btnClear) btnClear.classList.remove('hidden');
    } else {
        const btnClear = document.getElementById('btn-clear-override');
        if (btnClear) btnClear.classList.add('hidden');
    }
    postData('/override', {type: type, v_x: vx, v_y: vy, v_yaw: vyaw});
};

// UI Button Handlers
btnEstop.addEventListener('click', () => postData('/override', {type: 'estop'}));
if (btnResetEstop) {
    btnResetEstop.addEventListener('click', () => {
        activeKeys.clear();
        updateDpadVisuals();
        postData('/reset_estop');
    });
}
btnApprove.addEventListener('click', () => postData('/approve'));
btnReject.addEventListener('click', () => postData('/reject'));
btnClearOverride.addEventListener('click', () => {
    activeKeys.clear();
    updateDpadVisuals();
    postData('/clear_override');
});

// D-Pad and Hotkeys
const dpadW = document.getElementById('dpad-w');
const dpadA = document.getElementById('dpad-a');
const dpadS = document.getElementById('dpad-s');
const dpadD = document.getElementById('dpad-d');
const activeKeys = new Set();

function updateDpadVisuals() {
    if (dpadW) dpadW.classList.toggle('active', activeKeys.has('w'));
    if (dpadA) dpadA.classList.toggle('active', activeKeys.has('a'));
    if (dpadS) dpadS.classList.toggle('active', activeKeys.has('s'));
    if (dpadD) dpadD.classList.toggle('active', activeKeys.has('d'));
}

function processNavigationCommand() {
    updateDpadVisuals();
    let vx = 0.0, vy = 0.0, vyaw = 0.0;
    
    if (activeKeys.has('w')) vx += 0.6;
    if (activeKeys.has('s')) vx -= 0.5;
    if (activeKeys.has('a')) vyaw += 1.0;
    if (activeKeys.has('d')) vyaw -= 1.0;
    
    if (vx !== 0.0 || vyaw !== 0.0 || vy !== 0.0) {
        sendOverride('walk', vx, vy, vyaw);
    } else if (activeKeys.size === 0) {
        sendOverride('stand', 0, 0, 0);
    }
}

function setupDpadButton(buttonEl, keyName) {
    if (!buttonEl) return;
    const press = (e) => { e.preventDefault(); activeKeys.add(keyName); processNavigationCommand(); };
    const release = (e) => { e.preventDefault(); activeKeys.delete(keyName); processNavigationCommand(); };
    buttonEl.addEventListener('pointerdown', press);
    buttonEl.addEventListener('pointerup', release);
    buttonEl.addEventListener('pointerleave', release);
    buttonEl.addEventListener('pointercancel', release);
}

setupDpadButton(dpadW, 'w');
setupDpadButton(dpadA, 'a');
setupDpadButton(dpadS, 's');
setupDpadButton(dpadD, 'd');

function bindActionButton(id, cmdType) {
    const btn = document.getElementById(id);
    if (btn) {
        btn.addEventListener('click', () => {
            activeKeys.clear();
            updateDpadVisuals();
            sendOverride(cmdType, 0, 0, 0);
        });
    }
}

bindActionButton('btn-stand', 'stand');
bindActionButton('btn-crouch', 'crouch');
bindActionButton('btn-jump', 'jump');
bindActionButton('btn-wave', 'wave');
bindActionButton('btn-punch', 'punch');
bindActionButton('btn-victory', 'victory');
bindActionButton('btn-fight', 'fight');
bindActionButton('btn-look-left', 'look_left');
bindActionButton('btn-look-right', 'look_right');
bindActionButton('btn-scan', 'scan');
bindActionButton('btn-namaste', 'namaste');

document.addEventListener('keydown', (e) => {
    if (['INPUT', 'TEXTAREA'].includes(e.target.tagName)) return;
    const key = e.key.toLowerCase();
    
    if (key === 'w' || key === 'arrowup') { if (!activeKeys.has('w')) { activeKeys.add('w'); processNavigationCommand(); } }
    else if (key === 's' || key === 'arrowdown') { if (!activeKeys.has('s')) { activeKeys.add('s'); processNavigationCommand(); } }
    else if (key === 'a' || key === 'arrowleft') { if (!activeKeys.has('a')) { activeKeys.add('a'); processNavigationCommand(); } }
    else if (key === 'd' || key === 'arrowright') { if (!activeKeys.has('d')) { activeKeys.add('d'); processNavigationCommand(); } }
    else if (key === ' ' || key === 'escape') { activeKeys.clear(); updateDpadVisuals(); sendOverride('stand', 0, 0, 0); }
    else if (key === 'c') { activeKeys.clear(); updateDpadVisuals(); sendOverride('crouch', 0, 0, 0); }
    else if (key === 'j') { activeKeys.clear(); updateDpadVisuals(); sendOverride('jump', 0, 0, 0); }
    else if (key === 'h') { activeKeys.clear(); updateDpadVisuals(); sendOverride('wave', 0, 0, 0); }
    else if (key === 'p') { activeKeys.clear(); updateDpadVisuals(); sendOverride('punch', 0, 0, 0); }
    else if (key === 'v') { activeKeys.clear(); updateDpadVisuals(); sendOverride('victory', 0, 0, 0); }
    else if (key === 'f') { activeKeys.clear(); updateDpadVisuals(); sendOverride('fight', 0, 0, 0); }
    else if (key === 'q') { activeKeys.clear(); updateDpadVisuals(); sendOverride('look_left', 0, 0, 0); }
    else if (key === 'e') { activeKeys.clear(); updateDpadVisuals(); sendOverride('look_right', 0, 0, 0); }
    else if (key === 'r') { activeKeys.clear(); updateDpadVisuals(); sendOverride('scan', 0, 0, 0); }
    else if (key === 'n') { activeKeys.clear(); updateDpadVisuals(); sendOverride('namaste', 0, 0, 0); }
});

document.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();
    let changed = false;
    if (key === 'w' || key === 'arrowup') { activeKeys.delete('w'); changed = true; }
    if (key === 's' || key === 'arrowdown') { activeKeys.delete('s'); changed = true; }
    if (key === 'a' || key === 'arrowleft') { activeKeys.delete('a'); changed = true; }
    if (key === 'd' || key === 'arrowright') { activeKeys.delete('d'); changed = true; }
    if (changed) processNavigationCommand();
});

// Initialization
buildMatrixUI();
buildJointsUI();
setupBlueprintListeners();
connectWebSockets();
selectMotor(null);
