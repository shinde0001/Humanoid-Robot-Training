const host = window.location.host || "localhost:8000";
const wsProto = window.location.protocol === "https:" ? "wss:" : "ws:";
const httpProto = window.location.protocol === "https:" ? "https:" : "http:";
const WS_URL_TELEMETRY = `${wsProto}//${host}/ws/telemetry`;
const WS_URL_VIDEO = `${wsProto}//${host}/ws/video`;
const API_URL = `${httpProto}//${host}/api`;

const JOINT_NAMES = [
    "l_hip_y", "l_hip_r", "l_hip_p", "l_knee", "l_ankle",
    "r_hip_y", "r_hip_r", "r_hip_p", "r_knee", "r_ankle",
    "torso",
    "l_sho_p", "l_sho_r", "l_sho_y", "l_elbow",
    "r_sho_p", "r_sho_r", "r_sho_y", "r_elbow"
];

let telemetrySocket;
let videoSocket;

// UI Elements
const connDot = document.getElementById('connection-dot');
const connText = document.getElementById('connection-text');
const camFeed = document.getElementById('camera-feed');
const jointsContainer = document.getElementById('joints-container');
const simTimeEl = document.getElementById('sim-time');
const gaitStateEl = document.getElementById('gait-state');
const btnEstop = document.getElementById('btn-estop');
const warningBanner = document.getElementById('estop-warning');
const btnResetEstop = document.getElementById('btn-reset-estop');
const pendingCmdText = document.getElementById('pending-cmd-text');
const btnApprove = document.getElementById('btn-approve');
const btnReject = document.getElementById('btn-reject');
const btnClearOverride = document.getElementById('btn-clear-override');

// Build Joint UI
function buildJointsUI() {
    jointsContainer.innerHTML = '';
    JOINT_NAMES.forEach((name, idx) => {
        const row = document.createElement('div');
        row.className = 'joint-row';
        row.innerHTML = `
            <div class="joint-name">${name}</div>
            <div class="joint-bar-bg">
                <div class="joint-bar-fill" id="joint-fill-${idx}"></div>
            </div>
            <div class="joint-val" id="joint-val-${idx}">0.00</div>
        `;
        jointsContainer.appendChild(row);
    });
}

function updateJoints(positions) {
    positions.forEach((pos, idx) => {
        const valEl = document.getElementById(`joint-val-${idx}`);
        const fillEl = document.getElementById(`joint-fill-${idx}`);
        if(valEl && fillEl) {
            valEl.innerText = pos.toFixed(2);
            // Map rad to percentage for visualization (-3.14 to 3.14 approx)
            let pct = ((pos + Math.PI) / (2 * Math.PI)) * 100;
            pct = Math.max(0, Math.min(100, pct));
            
            // Draw from center (50%)
            if(pct > 50) {
                fillEl.style.left = '50%';
                fillEl.style.width = `${pct - 50}%`;
            } else {
                fillEl.style.left = `${pct}%`;
                fillEl.style.width = `${50 - pct}%`;
            }
        }
    });
}

function connectWebSockets() {
    // Telemetry
    telemetrySocket = new WebSocket(WS_URL_TELEMETRY);
    telemetrySocket.onopen = () => {
        connDot.parentElement.className = 'status-indicator status-connected';
        connText.innerText = 'SYSTEM ONLINE';
    };
    telemetrySocket.onclose = () => {
        connDot.parentElement.className = 'status-indicator status-error';
        connText.innerText = 'CONNECTION LOST';
        setTimeout(connectWebSockets, 2000);
    };
    telemetrySocket.onmessage = (event) => {
        const data = JSON.parse(event.data);
        
        simTimeEl.innerText = data.timestamp.toFixed(3) + 's';
        gaitStateEl.innerText = data.state.toUpperCase();
        
        updateJoints(data.joints);
        
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
        if(data.manual_override) {
            btnClearOverride.classList.remove('hidden');
            gaitStateEl.style.color = 'var(--neon-orange)';
        } else {
            btnClearOverride.classList.add('hidden');
            gaitStateEl.style.color = 'var(--neon-cyan)';
        }
    };

    // Video
    videoSocket = new WebSocket(WS_URL_VIDEO);
    videoSocket.onmessage = (event) => {
        camFeed.src = "data:image/jpeg;base64," + event.data;
    };
}

// UI Button Elements
const dpadW = document.getElementById('dpad-w');
const dpadA = document.getElementById('dpad-a');
const dpadS = document.getElementById('dpad-s');
const dpadD = document.getElementById('dpad-d');
const btnStand = document.getElementById('btn-stand');
const btnCrouch = document.getElementById('btn-crouch');

// Active key / pointer state tracking
const activeKeys = new Set();

function updateDpadVisuals() {
    if (dpadW) dpadW.classList.toggle('active', activeKeys.has('w'));
    if (dpadA) dpadA.classList.toggle('active', activeKeys.has('a'));
    if (dpadS) dpadS.classList.toggle('active', activeKeys.has('s'));
    if (dpadD) dpadD.classList.toggle('active', activeKeys.has('d'));
}

function processNavigationCommand() {
    updateDpadVisuals();
    
    let vx = 0.0;
    let vy = 0.0;
    let vyaw = 0.0;
    
    if (activeKeys.has('w')) vx += 0.6;
    if (activeKeys.has('s')) vx -= 0.5;
    if (activeKeys.has('a')) vyaw += 1.0;  // Turn left
    if (activeKeys.has('d')) vyaw -= 1.0;  // Turn right
    
    if (vx !== 0.0 || vyaw !== 0.0 || vy !== 0.0) {
        sendOverride('walk', vx, vy, vyaw);
    } else if (activeKeys.size === 0) {
        sendOverride('stand', 0, 0, 0);
    }
}

// REST Actions
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

// Global Manual Override Dispatcher
window.sendOverride = function(type, vx=0, vy=0, vyaw=0) {
    postData('/override', {type: type, v_x: vx, v_y: vy, v_yaw: vyaw});
};

// Event Listeners for System Controls
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

// Setup D-pad on-screen buttons
function setupDpadButton(buttonEl, keyName) {
    if (!buttonEl) return;
    
    const press = (e) => {
        e.preventDefault();
        activeKeys.add(keyName);
        processNavigationCommand();
    };
    
    const release = (e) => {
        e.preventDefault();
        activeKeys.delete(keyName);
        processNavigationCommand();
    };
    
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
bindActionButton('btn-bow', 'bow');

// Keyboard binding for WASD / Arrow keys and Action Hotkeys
document.addEventListener('keydown', (e) => {
    if (['INPUT', 'TEXTAREA'].includes(e.target.tagName)) return;
    
    const key = e.key.toLowerCase();
    
    if (key === 'w' || key === 'arrowup') {
        if (!activeKeys.has('w')) {
            activeKeys.add('w');
            processNavigationCommand();
        }
    } else if (key === 's' || key === 'arrowdown') {
        if (!activeKeys.has('s')) {
            activeKeys.add('s');
            processNavigationCommand();
        }
    } else if (key === 'a' || key === 'arrowleft') {
        if (!activeKeys.has('a')) {
            activeKeys.add('a');
            processNavigationCommand();
        }
    } else if (key === 'd' || key === 'arrowright') {
        if (!activeKeys.has('d')) {
            activeKeys.add('d');
            processNavigationCommand();
        }
    } else if (key === ' ' || key === 'escape') {
        activeKeys.clear();
        updateDpadVisuals();
        sendOverride('stand', 0, 0, 0);
    } else if (key === 'c') {
        activeKeys.clear();
        updateDpadVisuals();
        sendOverride('crouch', 0, 0, 0);
    } else if (key === 'j') {
        activeKeys.clear();
        updateDpadVisuals();
        sendOverride('jump', 0, 0, 0);
    } else if (key === 'h') {
        activeKeys.clear();
        updateDpadVisuals();
        sendOverride('wave', 0, 0, 0);
    } else if (key === 'p') {
        activeKeys.clear();
        updateDpadVisuals();
        sendOverride('punch', 0, 0, 0);
    } else if (key === 'v') {
        activeKeys.clear();
        updateDpadVisuals();
        sendOverride('victory', 0, 0, 0);
    } else if (key === 'f') {
        activeKeys.clear();
        updateDpadVisuals();
        sendOverride('fight', 0, 0, 0);
    } else if (key === 'q') {
        activeKeys.clear();
        updateDpadVisuals();
        sendOverride('look_left', 0, 0, 0);
    } else if (key === 'e') {
        activeKeys.clear();
        updateDpadVisuals();
        sendOverride('look_right', 0, 0, 0);
    } else if (key === 'r') {
        activeKeys.clear();
        updateDpadVisuals();
        sendOverride('scan', 0, 0, 0);
    } else if (key === 'b') {
        activeKeys.clear();
        updateDpadVisuals();
        sendOverride('bow', 0, 0, 0);
    }
});

document.addEventListener('keyup', (e) => {
    const key = e.key.toLowerCase();
    let changed = false;
    
    if (key === 'w' || key === 'arrowup') { activeKeys.delete('w'); changed = true; }
    if (key === 's' || key === 'arrowdown') { activeKeys.delete('s'); changed = true; }
    if (key === 'a' || key === 'arrowleft') { activeKeys.delete('a'); changed = true; }
    if (key === 'd' || key === 'arrowright') { activeKeys.delete('d'); changed = true; }
    
    if (changed) {
        processNavigationCommand();
    }
});

// Init
buildJointsUI();
connectWebSockets();
