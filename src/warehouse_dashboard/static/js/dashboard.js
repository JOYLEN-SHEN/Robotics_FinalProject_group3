/**
 * Warehouse Fleet Monitor Dashboard
 * Real-time visualization via Socket.IO
 */

// ===== Constants =====
const WAREHOUSE_W = 32;   // meters (world)
const WAREHOUSE_H = 22;   // meters (world)
const CANVAS_W    = 700;
const CANVAS_H    = 467;
const SCALE_X     = CANVAS_W / WAREHOUSE_W;
const SCALE_Y     = CANVAS_H / WAREHOUSE_H;

const ROBOT_COLORS = {
  0: '#22c55e',  // IDLE
  1: '#3b82f6',  // NAVIGATING
  2: '#a855f7',  // LOADING
  3: '#a855f7',  // UNLOADING
  4: '#f97316',  // CHARGING
  5: '#ef4444',  // ERROR
  6: '#eab308',  // WAITING
};
const STATE_NAMES = ['IDLE','NAVIGATING','LOADING','UNLOADING','CHARGING','ERROR','WAITING'];

// Warehouse static elements (in world coords, origin at center)
const SHELVES = [];
for (const rowX of [-10, -4, 4, 10]) {
  for (const shelfY of [-8, -4, 0, 4, 8]) {
    SHELVES.push({x: rowX, y: shelfY, w: 1.0, h: 0.5});
  }
}

const ZONES = [
  {name:'Loading 1',  x:-14, y: 3,  color:'#eab308', r:1.0},
  {name:'Loading 2',  x:-14, y:-3,  color:'#eab308', r:1.0},
  {name:'Unload 1',   x: 14, y: 3,  color:'#22c55e', r:1.0},
  {name:'Unload 2',   x: 14, y:-3,  color:'#22c55e', r:1.0},
  {name:'Charge 1',   x:-13, y: 9,  color:'#f97316', r:0.8},
  {name:'Charge 2',   x: 13, y: 9,  color:'#f97316', r:0.8},
  {name:'Charge 3',   x:-13, y:-9,  color:'#f97316', r:0.8},
  {name:'Charge 4',   x: 13, y:-9,  color:'#f97316', r:0.8},
];

// ===== State =====
let robotData = {};
let logEntries = [];

// ===== Canvas helpers =====
const canvas = document.getElementById('warehouse-canvas');
const ctx = canvas.getContext('2d');

function worldToCanvas(wx, wy) {
  // World: x in [-16, 16], y in [-11, 11] → canvas: (0,0) top-left
  const cx = (wx + WAREHOUSE_W / 2) * SCALE_X;
  const cy = (WAREHOUSE_H / 2 - wy) * SCALE_Y;  // y-flip
  return [cx, cy];
}

function drawWarehouse() {
  ctx.clearRect(0, 0, CANVAS_W, CANVAS_H);

  // Floor
  ctx.fillStyle = '#1a2332';
  ctx.fillRect(0, 0, CANVAS_W, CANVAS_H);

  // Warehouse boundary
  const [wx0, wy0] = worldToCanvas(-15, 11);
  const [wx1, wy1] = worldToCanvas( 15,-11);
  ctx.strokeStyle = '#475569';
  ctx.lineWidth = 2;
  ctx.strokeRect(wx0, wy0, wx1 - wx0, wy1 - wy0);

  // Zones
  for (const z of ZONES) {
    const [cx, cy] = worldToCanvas(z.x, z.y);
    const rx = z.r * SCALE_X;
    ctx.beginPath();
    ctx.arc(cx, cy, rx, 0, Math.PI * 2);
    ctx.fillStyle = z.color + '40';
    ctx.fill();
    ctx.strokeStyle = z.color;
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.fillStyle = z.color;
    ctx.font = '9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(z.name, cx, cy + rx + 10);
  }

  // Shelves
  for (const s of SHELVES) {
    const [sx, sy] = worldToCanvas(s.x - s.w/2, s.y + s.h/2);
    const sw = s.w * SCALE_X;
    const sh = s.h * SCALE_Y;
    ctx.fillStyle = '#4a3728';
    ctx.fillRect(sx, sy, sw, sh);
    ctx.strokeStyle = '#6b4c36';
    ctx.lineWidth = 1;
    ctx.strokeRect(sx, sy, sw, sh);
  }

  // Robots
  for (const [id, r] of Object.entries(robotData)) {
    const [rx, ry] = worldToCanvas(r.x, r.y);
    const color = ROBOT_COLORS[r.state] || '#94a3b8';

    // Shadow
    ctx.beginPath();
    ctx.arc(rx, ry, 12, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(0,0,0,0.4)';
    ctx.fill();

    // Body
    ctx.beginPath();
    ctx.arc(rx, ry, 10, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Label
    ctx.fillStyle = '#0f172a';
    ctx.font = 'bold 8px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(id.replace('robot_', 'R'), rx, ry + 3);

    // Battery indicator
    const battFrac = r.battery / 100;
    const barW = 20;
    const barX = rx - barW / 2;
    const barY = ry + 14;
    ctx.fillStyle = '#1e293b';
    ctx.fillRect(barX, barY, barW, 3);
    const battColor = battFrac > 0.5 ? '#22c55e' : battFrac > 0.2 ? '#eab308' : '#ef4444';
    ctx.fillStyle = battColor;
    ctx.fillRect(barX, barY, barW * battFrac, 3);
  }
}

// ===== Robot cards =====
function updateRobotCards(robots) {
  const container = document.getElementById('robots-container');
  robots.forEach(r => {
    const color = ROBOT_COLORS[r.state] || '#94a3b8';
    let card = document.getElementById(`card-${r.id}`);
    if (!card) {
      card = document.createElement('div');
      card.className = 'robot-card';
      card.id = `card-${r.id}`;
      container.appendChild(card);
    }
    const taskStr = r.task_id ? `Task: ${r.task_id.slice(0,8)}` : 'Idle';
    const errStr  = r.error ? `<span style="color:#ef4444"> ⚠ ${r.error}</span>` : '';
    const battColor = r.battery > 50 ? '#22c55e' : r.battery > 20 ? '#eab308' : '#ef4444';
    card.innerHTML = `
      <div class="robot-card-header">
        <div class="robot-dot" style="background:${color}"></div>
        <span class="robot-name">${r.id}</span>
        <span class="robot-state">${STATE_NAMES[r.state] || 'UNKNOWN'}</span>
      </div>
      <div class="robot-details">
        Pos: (${r.x}, ${r.y}) &nbsp; ${taskStr}${errStr}
        <br>Vel: ${r.linear_vel} m/s
      </div>
      <div class="battery-bar">
        <div class="battery-fill" style="width:${r.battery}%;background:${battColor}"></div>
      </div>
      <div style="font-size:0.65rem;color:#64748b;margin-top:2px">${r.battery}% battery</div>
      ${r.task_id ? `
      <div class="progress-bar-wrap">
        <div class="progress-fill" style="width:${r.progress}%"></div>
      </div>
      <div style="font-size:0.65rem;color:#64748b;margin-top:2px">${r.progress}% task progress</div>
      ` : ''}
    `;
  });
}

// ===== Stats =====
function updateStats(stats) {
  document.getElementById('stat-active').textContent    = stats.active_tasks;
  document.getElementById('stat-pending').textContent   = stats.pending_tasks;
  document.getElementById('stat-completed').textContent = stats.completed_tasks;
  document.getElementById('stat-failed').textContent    = stats.failed_tasks;
  document.getElementById('stat-utilization').textContent = stats.utilization + '%';
  document.getElementById('stat-avgtime').textContent   = stats.avg_time + 's';
  document.getElementById('stat-conflicts').textContent = stats.conflicts;
}

// ===== Event log =====
function addLog(message, type = 'info') {
  const log = document.getElementById('event-log');
  const ts  = new Date().toTimeString().slice(0,8);
  const div = document.createElement('div');
  div.className = `log-entry ${type}`;
  div.textContent = `[${ts}] ${message}`;
  log.insertBefore(div, log.firstChild);
  if (log.children.length > 200) log.removeChild(log.lastChild);
}

function clearLog() {
  document.getElementById('event-log').innerHTML = '';
}

// ===== Socket.IO =====
const socket = io();

socket.on('connect', () => {
  document.getElementById('connection-status').textContent = 'Connected';
  document.getElementById('connection-status').className = 'badge badge-green';
  addLog('Connected to warehouse server', 'success');
});

socket.on('disconnect', () => {
  document.getElementById('connection-status').textContent = 'Disconnected';
  document.getElementById('connection-status').className = 'badge badge-red';
  addLog('Disconnected from server', 'error');
});

socket.on('fleet_status', (data) => {
  // Update robot map state
  data.robots.forEach(r => { robotData[r.id] = r; });
  drawWarehouse();
  updateRobotCards(data.robots);
  updateStats(data.stats);
});

socket.on('task_result', (data) => {
  const icon = data.success ? '✓' : '✗';
  const type = data.success ? 'success' : 'error';
  addLog(
    `${icon} Task ${data.task_id.slice(0,8)} — ${data.robot_id} — ` +
    `${data.success ? `done in ${data.duration}s` : `FAILED: ${data.failure}`}`,
    type
  );
});

// ===== Initial draw =====
drawWarehouse();

// ===== Task form =====
document.getElementById('task-form').addEventListener('submit', async (e) => {
  e.preventDefault();
  const pickupSel  = document.getElementById('pickup-zone');
  const dropoffSel = document.getElementById('dropoff-zone');
  const priority   = parseInt(document.getElementById('priority').value);

  const pickupOpt  = pickupSel.options[pickupSel.selectedIndex];
  const dropoffOpt = dropoffSel.options[dropoffSel.selectedIndex];

  const payload = {
    pickup_x:    parseFloat(pickupOpt.dataset.x),
    pickup_y:    parseFloat(pickupOpt.dataset.y),
    dropoff_x:   parseFloat(dropoffOpt.dataset.x),
    dropoff_y:   parseFloat(dropoffOpt.dataset.y),
    pickup_zone: pickupOpt.value,
    dropoff_zone:dropoffOpt.value,
    priority:    priority,
  };

  try {
    const res  = await fetch('/api/task', {
      method:  'POST',
      headers: {'Content-Type': 'application/json'},
      body:    JSON.stringify(payload),
    });
    const data = await res.json();
    if (data.success) {
      addLog(`Task submitted: ${data.task_id} (${pickupOpt.value} → ${dropoffOpt.value})`, 'info');
    } else {
      addLog(`Task submission failed: ${data.error}`, 'error');
    }
  } catch (err) {
    addLog(`Network error: ${err.message}`, 'error');
  }
});
