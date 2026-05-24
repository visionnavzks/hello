const canvas = document.getElementById('simCanvas');
const ctx = canvas.getContext('2d');

// Visualization scale
const scale = 150; // pixels per meter
const cartY = 250; // Y position of the cart rail
const l1 = 0.5; // length of pole 1
const l2 = 0.5; // length of pole 2

let state = { x: 0, th1: 0, th2: 0, u: 0 };
let uMax = 50.0;

// SSE connection
const evtSource = new EventSource('/stream');

evtSource.onmessage = function(event) {
    const data = JSON.parse(event.data);
    state = data;
    updateTelemetry();
    requestAnimationFrame(draw);
};

function updateTelemetry() {
    document.getElementById('uCurrent').innerText = state.u.toFixed(2);

    // Update u-bar
    const uBar = document.getElementById('uBar');
    // Map -uMax to uMax -> 0% to 100% of container width (50% is center)
    // Actually simpler: position at 50%, width based on magnitude, direction based on sign
    const normalized = Math.min(Math.max(state.u / uMax, -1), 1);

    if (normalized > 0) {
        uBar.style.left = '50%';
        uBar.style.width = `${normalized * 50}%`;
        uBar.style.backgroundColor = '#2ecc71'; // Green for positive force
    } else {
        uBar.style.left = `${50 + (normalized * 50)}%`;
        uBar.style.width = `${-normalized * 50}%`;
        uBar.style.backgroundColor = '#e74c3c'; // Red for negative force
    }
}

function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw rail
    ctx.beginPath();
    ctx.moveTo(0, cartY);
    ctx.lineTo(canvas.width, cartY);
    ctx.strokeStyle = '#aaa';
    ctx.lineWidth = 2;
    ctx.stroke();

    // Map cart x to canvas x
    // x=0 is center of canvas
    const cx = canvas.width / 2 + state.x * scale;

    // Draw cart
    const cartWidth = 60;
    const cartHeight = 30;
    ctx.fillStyle = '#34495e';
    ctx.fillRect(cx - cartWidth/2, cartY - cartHeight/2, cartWidth, cartHeight);

    // Calculate pendulum positions
    // Note: y axis is inverted in canvas. Theta = 0 is straight UP.
    // So x = l * sin(theta), y = -l * cos(theta)

    const p1x = cx + l1 * scale * Math.sin(state.th1);
    const p1y = cartY - l1 * scale * Math.cos(state.th1);

    const p2x = p1x + l2 * scale * Math.sin(state.th2);
    const p2y = p1y - l2 * scale * Math.cos(state.th2);

    // Draw Pole 1
    ctx.beginPath();
    ctx.moveTo(cx, cartY);
    ctx.lineTo(p1x, p1y);
    ctx.strokeStyle = '#e67e22';
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    ctx.stroke();

    // Draw Pole 2
    ctx.beginPath();
    ctx.moveTo(p1x, p1y);
    ctx.lineTo(p2x, p2y);
    ctx.strokeStyle = '#d35400';
    ctx.lineWidth = 4;
    ctx.stroke();

    // Draw joints
    ctx.beginPath();
    ctx.arc(cx, cartY, 6, 0, 2 * Math.PI);
    ctx.fillStyle = '#ecf0f1';
    ctx.fill();
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(p1x, p1y, 5, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(p2x, p2y, 4, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();

    // Draw force vector indicator
    if (Math.abs(state.u) > 0.1) {
        ctx.beginPath();
        const fLength = (state.u / uMax) * 50;

        ctx.moveTo(cx, cartY);
        ctx.lineTo(cx + fLength, cartY);

        // Arrow head
        const headlen = 10;
        const angle = state.u > 0 ? 0 : Math.PI;
        ctx.lineTo(cx + fLength - headlen * Math.cos(angle - Math.PI / 6), cartY - headlen * Math.sin(angle - Math.PI / 6));
        ctx.moveTo(cx + fLength, cartY);
        ctx.lineTo(cx + fLength - headlen * Math.cos(angle + Math.PI / 6), cartY - headlen * Math.sin(angle + Math.PI / 6));

        ctx.strokeStyle = state.u > 0 ? '#2ecc71' : '#e74c3c';
        ctx.lineWidth = 3;
        ctx.stroke();
    }
}

// UI Event Listeners
document.getElementById('mpcToggle').addEventListener('change', (e) => {
    fetch('/api/settings', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ mpc_enabled: e.target.checked })
    });
});

document.getElementById('uMaxSlider').addEventListener('input', (e) => {
    uMax = parseFloat(e.target.value);
    document.getElementById('uMaxVal').innerText = uMax;
    fetch('/api/settings', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ u_max: uMax })
    });
});

function applyDisturbance(fx, fth1, fth2) {
    fetch('/api/disturb', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ force_x: fx, force_th1: fth1, force_th2: fth2 })
    });
}

function resetSim() {
    fetch('/api/settings', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ reset: true })
    });
}
