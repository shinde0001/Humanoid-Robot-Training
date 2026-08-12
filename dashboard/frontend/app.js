const WS_URL_TELEMETRY = "ws://127.0.0.1:8000/ws/telemetry";
const WS_URL_VIDEO = "ws://127.0.0.1:8000/ws/video";
const API_URL = "http://127.0.0.1:8000/api";

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

// Event Listeners
btnEstop.addEventListener('click', () => postData('/override', {type: 'estop'}));
btnApprove.addEventListener('click', () => postData('/approve'));
btnReject.addEventListener('click', () => postData('/reject'));
btnClearOverride.addEventListener('click', () => postData('/clear_override'));

window.sendOverride = function(type, vx, vy, vyaw) {
    postData('/override', {type: type, v_x: vx, v_y: vy, v_yaw: vyaw});
};

// Keyboard binding for WASD manual override
document.addEventListener('keydown', (e) => {
    if(e.repeat) return; // Prevent spamming
    const speed = 0.5;
    if(e.key.toLowerCase() === 'w') sendOverride('walk', speed, 0, 0);
    if(e.key.toLowerCase() === 's') sendOverride('walk', -speed, 0, 0);
    if(e.key.toLowerCase() === 'a') sendOverride('walk', 0, speed, 0);
    if(e.key.toLowerCase() === 'd') sendOverride('walk', 0, -speed, 0);
});

document.addEventListener('keyup', (e) => {
    const k = e.key.toLowerCase();
    if(['w','a','s','d'].includes(k)) {
        // Simple release -> stand. For multi-key tracking, we'd need more complex logic.
        sendOverride('stand', 0, 0, 0);
    }
});

// Init
buildJointsUI();
connectWebSockets();
