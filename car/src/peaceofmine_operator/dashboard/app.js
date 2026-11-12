// The search lane runs along world +X, matching the ROS convention where a
// rover at yaw 0 faces +X. Cross-lane offsets are world Y.
const LANE_START_M = -2;
const LANE_END_M = 22;
const LANE_HALF_WIDTH_M = 2;
const CAMERA_RANGE_M = 26;

const DRIVE_KEEPALIVE_MS = 50;
const DRIVE_MIN_INTERVAL_MS = 8;
const PAD_UI_PERIOD_MS = 0; // Input graphics follow requestAnimationFrame.
const LATENCY_PERIOD_MS = 1000;
const LATENCY_HISTORY_LENGTH = 30;
const GATEWAY_TIMEOUT_MS = 8000;
const TELEMETRY_TIMEOUT_MS = 3000;
const MAX_LINEAR_MPS = 0.8;
const MAX_ANGULAR_RAD_S = 1.4;
const DRIVE_EPSILON = 1e-4;

// search-alert:start
// Used by the preview when the car does not report an alert state.
const SEARCH_ALERT_HOLD_MS = 150;
const SEARCH_ALERT_REARM_MS = 1500;
// Pure step: returns the next alert state and whether it just fired.
// phase: idle → pending (above level) → active (fired) → cooling (below) → idle.
function stepSearchAlert(alert, {enabled, level, signal, direction, hysteresis, live, suppressed, now}) {
  const next = {...alert};
  let fired = false;
  if (!enabled || !live || suppressed || ![signal, level, hysteresis].every(Number.isFinite)) {
    if (next.phase === 'active' || next.phase === 'cooling') next.clearedAt = now;
    next.phase = 'idle';
    return {alert: next, fired};
  }
  const distance = (signal - level) * direction;
  if ((next.phase === 'active' || next.phase === 'cooling') && (signal-next.peak)*direction > 0) next.peak = signal;
  switch (next.phase) {
    case 'pending':
      if (distance < 0) next.phase = 'idle';
      else if (now - next.since >= SEARCH_ALERT_HOLD_MS) {
        Object.assign(next, {phase: 'active', firedAt: now, peak: signal});
        fired = true;
      }
      break;
    case 'active':
      if (distance < -hysteresis) Object.assign(next, {phase: 'cooling', since: now});
      break;
    case 'cooling':
      if (distance >= 0) next.phase = 'active';
      else if (now - next.since >= SEARCH_ALERT_REARM_MS) Object.assign(next, {phase: 'idle', clearedAt: now});
      break;
    default:
      if (distance >= 0) Object.assign(next, {phase: 'pending', since: now});
  }
  return {alert: next, fired};
}
// search-alert:end

const state = {
  connected: false,
  drive: {
    armed: false,
    deadman: false,
    publishing: false,
    cmd_linear_x: 0,
    cmd_angular_z: 0,
    client_count: 0,
    you_control_owner: false,
    control_owner_present: false,
    command_source: 'none',
    joy: {seen: false},
  },
  robot: {x: 0, y: 0, yaw: 0, speed_mps: 0, pose_fresh: false},
  detector: {
    signal_ratio: 0,
    threshold_ratio: 1.0,
    detected: false,
    fixture_angle_deg: 0,
    sweep_enabled: false,
    sweep_speed_deg_s: 36,
    sweep_speed_min: 5,
    sweep_speed_max: 180,
    beam_half_angle_deg: 45,
    beam: [],
  },
  probe: {
    depth_mm: 0,
    target_mm: 0,
    max_depth_mm: 110,
    pressure_ratio: 0,
    pressure_history: [],
    fault: false,
  },
  detections: [],
  cameras: {},
  safety: {allowed: false, mode: 'unknown', reason: 'Waiting for PX4 safety status'},
  arm_servo: {},
  inspection: {phase: 'IDLE', active: false, message: '', completed: 0, total: 0, confirmed_angle_deg: null, points: []},
};

// Display samples are separate from authoritative telemetry and controls.
const motionFrames = [];
function recordMotionFrame(now) {
  if (motionFrames.length && now-motionFrames.at(-1).at > 250) motionFrames.length = 0;
  motionFrames.push({at: now, robot: {...state.robot}, depth: state.probe.depth_mm});
  while (motionFrames.length > 8) motionFrames.shift();
}
function displayMotion(now) {
  const fallback = {robot: state.robot, depth: state.probe.depth_mm};
  if (!state.connected || motionFrames.length < 2) return fallback;
  const at = now-50;
  for (let i=motionFrames.length-1; i>0; --i) {
    const a=motionFrames[i-1], b=motionFrames[i];
    if (a.at <= at && at <= b.at && b.at > a.at && b.at-a.at <= 250) {
      const f=(at-a.at)/(b.at-a.at);
      const mix=(x,y)=>Number.isFinite(x)&&Number.isFinite(y)?x+(y-x)*f:y;
      const robot={...b.robot};
      for (const key of ['x','y','speed_mps']) robot[key]=mix(a.robot[key],b.robot[key]);
      // Interpolate headings through the short arc across +/- pi.
      const delta=Math.atan2(Math.sin(b.robot.yaw-a.robot.yaw),Math.cos(b.robot.yaw-a.robot.yaw));
      robot.yaw=a.robot.yaw+delta*f;
      return {robot, depth: mix(a.depth,b.depth)};
    }
    if (b.at < at) break;
  }
  return fallback; // Never extrapolate through a telemetry gap.
}
function drawProbeMotion(depth) {
  const ratio=Number.isFinite(depth)?Math.max(0,Math.min(1,depth/(state.probe.max_depth_mm||1))):0;
  $('probe-arm').style.transform=`scaleY(${ratio})`;
}

const $ = id => document.getElementById(id);
const forward = $('forward-view');
const cameraPanel = $('camera-panel');
const map = $('map-view');
const preferences = {};
if (!['controller', 'keyboard', 'wheel', 'rc'].includes(preferences.input)) preferences.input = 'controller';
if (!Number.isFinite(preferences.sensitivity)) preferences.sensitivity = 100;
preferences.sensitivity = Math.max(50, Math.min(150, preferences.sensitivity));
if (!Number.isInteger(preferences.keyboardMaxThrottle) || preferences.keyboardMaxThrottle < 0 || preferences.keyboardMaxThrottle > 100) preferences.keyboardMaxThrottle = 100;
preferences.keyboardReverseThrottle = 100;
let drivingPending = null;
let drivingSaveTimer;
let lastRcEnable = -Infinity;
preferences.wheel = Object.assign({steering: 0, throttle: 1, brake: 2, deadman: 0,
  invertSteering: false, invertThrottle: true, invertBrake: true, deadzone: 0.05}, preferences.wheel);
preferences.cameras = Object.assign({forward: 'virtual', auxiliary: 'off'}, preferences.cameras);
for (const slot of ['forward', 'auxiliary']) {
  if (typeof preferences.cameras[slot] !== 'string' || !preferences.cameras[slot]) {
    preferences.cameras[slot] = slot === 'forward' ? 'virtual' : 'off';
  }
}
preferences.cameraRotation = Object.assign({forward: 0, auxiliary: 0}, preferences.cameraRotation);
for (const slot of ['forward', 'auxiliary']) {
  if (![0, 90, 180, 270].includes(preferences.cameraRotation[slot])) preferences.cameraRotation[slot] = 0;
}
const keyboardRamp = {throttle: 0, steer: 0, updatedAt: performance.now()};
let selectedPadIndex = null;
let selectedPadId = null;

const keys = new Set();
const activeSliders = new Set();
if (!['auto', 'standard', 'xbox360'].includes(preferences.mapping)) preferences.mapping = 'auto';
preferences.searchAlert = Object.assign({enabled: true, level: 60, sound: true}, preferences.searchAlert);
preferences.searchAlert.enabled = preferences.searchAlert.enabled !== false;
preferences.searchAlert.sound = preferences.searchAlert.sound !== false;
const triggerSeen = {lt: false, rt: false};
const lastInput = {
  linear: 0,
  angular: 0,
  steer: 0,
  throttle: 0,
  deadman: false,
  mapping: '',
  stickX: 0,
  stickY: 0,
  lt: 0,
  rt: 0,
  lb: false,
  rb: false,
  a: false,
  b: false,
  x: false,
  y: false,
};
const lastSentDrive = {linear: 0, angular: 0, deadman: false};
const padEls = {
  controller: $('controller'),
  mapping: $('controller-mapping'),
  raw: $('pad-raw'),
  ls: $('pad-ls'),
  lt: $('pad-lt'),
  rt: $('pad-rt'),
  lb: $('pad-lb'),
  rb: $('pad-rb'),
  a: $('pad-a'),
  b: $('pad-b'),
  x: $('pad-x'),
  y: $('pad-y'),
  driveBlock: $('drive-block'),
  inputLock: $('input-lock'),
};
let socket;
let gamepad = null;
let lastPadId = null;
let lastPadTimestamp = -1;
let prevA = false;
let cameraFocused = false;
let keyboardSweep = false;
let lastDriveSent = 0;
let lastPadUi = 0;
let noticeExpiry = 0;
let noticeText = null;
let pendingPumpMark = null;
let latencyProbeId = 0;
let latencyProbeStarted = 0;
let linkLatencyMs = null;
let lastTelemetryAt = 0;
let lastServerMessageAt = 0;
let reconnectTimer = null;
const latencyHistory = [];
const cameraStreams = {forward: null, auxiliary: null};
let networkCameraSignature = '';
let autoForwardPiDone = false;
let localCameras = [];
let cameraDeviceRefresh = 0;
const cameraGeneration = {forward: 0, auxiliary: 0};
const cameraSources = {forward: null, auxiliary: null};
const cameraRetries = {forward: null, auxiliary: null};
const cameraWatchdogs = {forward: null, auxiliary: null};
const cameraTopics = {forward: null, auxiliary: null};
let cameraWasOffline = navigator.onLine === false;
const cameraErrors = {forward: '', auxiliary: ''};

function throttle(periodMs, action) {
  let last = 0;
  return value => {
    const now = performance.now();
    if (now - last < periodMs) return;
    last = now;
    action(value);
  };
}

/* ------------------------------------------------------------ Transport --- */

function send(message) {
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

function sendDrive(linear, angular, deadman) {
  if (socket?.readyState !== WebSocket.OPEN || preferences.input === 'rc' || state.drive.input === 'rc') return;
  socket.send(
    `{"type":"drive","input":"${preferences.input}","linear_x":${linear},"angular_z":${angular},"deadman":${deadman}}`,
  );
}

function setConnection(connected, label, detail = '', overlay = !connected) {
  state.connected = connected;
  if (!connected) {
    motionFrames.length = 0;
    drivingPending = null;
    clearTimeout(drivingSaveTimer);
    lastRcEnable = -Infinity;
  }
  $('connection').textContent = label;
  $('connection-dot').style.background = connected ? '#37b98e' : '#d36458';
  $('connection-overlay').classList.toggle('hidden', !overlay);
  $('connection-title').textContent = connected ? 'Connected' : label;
  $('connection-detail').textContent = detail || 'Waiting for the Raspberry Pi gateway. This page will reconnect automatically.';
  updateHud();
}

function drawLatencyGraph() {
  const canvas = $('latency-graph');
  const context = canvas.getContext('2d');
  const width = canvas.width;
  const height = canvas.height;
  context.clearRect(0, 0, width, height);
  if (latencyHistory.length < 2) return;

  const maximum = Math.max(100, ...latencyHistory);
  const sampleSpan = Math.max(1, latencyHistory.length - 1);
  context.beginPath();
  latencyHistory.forEach((latency, index) => {
    const x = index / sampleSpan * width;
    const y = height - Math.min(latency / maximum, 1) * (height - 3) - 1;
    if (index === 0) context.moveTo(x, y);
    else context.lineTo(x, y);
  });
  context.strokeStyle = '#55c9a1';
  context.lineWidth = 2;
  context.stroke();
}

function measureLatency() {
  if (socket?.readyState !== WebSocket.OPEN) return;
  if (latencyProbeStarted && performance.now() - latencyProbeStarted < GATEWAY_TIMEOUT_MS) return;
  latencyProbeId += 1;
  latencyProbeStarted = performance.now();
  send({type: 'latency_ping', probe_id: latencyProbeId});
}

function recordLatency(probeId) {
  if (probeId !== latencyProbeId || !latencyProbeStarted) return;
  const latency = Math.round(performance.now() - latencyProbeStarted);
  linkLatencyMs = latency;
  latencyProbeStarted = 0;
  latencyHistory.push(latency);
  if (latencyHistory.length > LATENCY_HISTORY_LENGTH) latencyHistory.shift();
  $('latency-value').textContent = `${latency} ms`;
  drawLatencyGraph();
}

function controlUnavailable() {
  state.drive.armed = state.drive.deadman = state.drive.you_control_owner = state.drive.control_owner_present = false;
  state.drive.publishing = false;
  state.drive.cmd_linear_x = state.drive.cmd_angular_z = 0;
  pendingPumpMark = null;
  clearHeldInput();
  setConnection(false, 'Waiting for control telemetry', 'The gateway is linked. Controls recover when ROS telemetry returns.', !lastTelemetryAt);
}

let historyStream = null;
let historyCursor = 0;
let historyTime = 0;
let historyRequest = false;
let detectorPoints = []; // Exact samples from the car, aged relative to historyTime.
let backfillReceivedAt = 0;
let liveReceivedAt = 0;
let liveDetectorPoints = null;
let liveDetectorSeq = null;

function receiveDetectorStream(sensor, now) {
  if (sensor.history_stream !== historyStream) {
    historyStream = sensor.history_stream;
    historyCursor = historyTime = backfillReceivedAt = liveReceivedAt = 0;
    detectorPoints = [];
    liveDetectorPoints = liveDetectorSeq = null;
    queueMicrotask(refreshDetectorHistory);
  }
  sensor.history = liveDetectorPoints || detectorPoints;
  detectorHistoryAt = liveDetectorPoints ? liveReceivedAt : backfillReceivedAt;
  // Short HTTP gaps are already covered by the next delta. While bootstrapping
  // or during a longer outage, display each distinct measured telemetry packet.
  const latestRecordedAt = backfillReceivedAt - (historyTime - historyCursor) * 1000;
  if (detectorPoints.length && now - latestRecordedAt < 1000 && !liveDetectorPoints) return;
  const packet = sensor.packet;
  const adc = packet?.precise_adc ?? packet?.amplitude_adc;
  if (!sensor.fresh || !Number.isFinite(packet?.seq) || packet.seq === liveDetectorSeq ||
      !Number.isFinite(adc) || !Number.isFinite(packet.filtered_adc) || !Number.isFinite(sensor.age_ms)) return;
  liveDetectorSeq = packet.seq;
  if (!liveDetectorPoints) liveDetectorPoints = detectorPoints.map(point => ({...point}));
  const elapsed = Math.max(0, (now - detectorHistoryAt) / 1000);
  for (const point of liveDetectorPoints) point.age_s += elapsed;
  while (liveDetectorPoints.length && liveDetectorPoints[0].age_s > 120) liveDetectorPoints.shift();
  const age = Math.max(0, sensor.age_ms / 1000);
  if (!liveDetectorPoints.length || age < liveDetectorPoints.at(-1).age_s) {
    liveDetectorPoints.push({age_s: age, adc, filtered_adc: packet.filtered_adc,
      ...(Object.hasOwn(packet, 'angle_deg') ? {angle_deg: packet.angle_deg, timestamp_s: packet.timestamp_s} : {})});
  }
  sensor.history = liveDetectorPoints;
  liveReceivedAt = detectorHistoryAt = now;
}

async function refreshDetectorHistory() {
  const stream = state.detector?.sensor?.history_stream;
  if (!state.connected || !stream || historyRequest || document.hidden) return;
  historyRequest = true;
  // A new browser can need two minutes of history over the field link.
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), historyCursor ? 5000 : 20000);
  try {
    const query = new URLSearchParams({stream, after:String(historyCursor)});
    const response = await fetch('/api/detector-history?' + query, {cache:'no-store', signal:controller.signal});
    if (!response.ok) return;
    const batch = await response.json();
    if (batch.stream !== stream || state.detector?.sensor?.history_stream !== stream ||
        !Number.isFinite(batch.now) || !Array.isArray(batch.points)) return;
    const elapsed = batch.now - historyTime;
    detectorPoints = detectorPoints.filter(p => p.age_s + elapsed <= 120);
    for (const point of detectorPoints) point.age_s += elapsed;
    for (const [stamp, adc, filtered_adc, angle_deg, timestamp_s] of batch.points) {
      if (stamp <= historyCursor || ![stamp, adc, filtered_adc].every(Number.isFinite)) continue;
      detectorPoints.push({age_s:batch.now-stamp, adc, filtered_adc,
        ...(timestamp_s === undefined ? {} : {angle_deg, timestamp_s})});
      historyCursor = stamp;
    }
    historyTime = batch.now;
    // Keep a newer live trace when a receiver or gateway returns an empty or
    // stale batch. Once it catches up, use its exact, deduplicated history.
    if (liveDetectorPoints && (!historyCursor || batch.now - historyCursor > 1) &&
        state.detector.sensor.fresh) return;
    liveDetectorPoints = null;
    backfillReceivedAt = detectorHistoryAt = performance.now();
    state.detector.sensor.history = detectorPoints;
  } catch { /* Received packets continue to draw while the history request retries. */ }
  finally { clearTimeout(timeout); historyRequest = false; }
}

setInterval(refreshDetectorHistory, 400);
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) refreshDetectorHistory();
});

function connect() {
  clearTimeout(reconnectTimer);
  stopInput();
  state.drive.armed = state.drive.deadman = state.drive.you_control_owner = false;
  latencyProbeStarted = 0;
  if (socket && socket.readyState < WebSocket.CLOSING) socket.close();
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  const connectingSocket = new WebSocket(`${scheme}://${location.host}/ws`);
  let awaitingFirstState = true;
  socket = connectingSocket;
  lastServerMessageAt = performance.now();
  setConnection(false, 'Connecting to operator stack', 'The dashboard is loaded. Waiting for telemetry from the Raspberry Pi gateway.');

  connectingSocket.onopen = () => {
    if (socket !== connectingSocket) return;
    $('connection').textContent = 'Gateway linked · waiting for telemetry';
    measureLatency();
  };

  connectingSocket.onclose = () => {
    if (socket !== connectingSocket) return;
    latencyProbeStarted = 0;
    linkLatencyMs = null;
    latencyHistory.length = 0;
    $('latency-value').textContent = '-- ms';
    drawLatencyGraph();
    state.drive.armed = false;
    state.drive.deadman = false;
    state.drive.you_control_owner = false;
    pendingPumpMark = null;
    stopLocalInput();
    setConnection(false, 'Gateway connection lost', 'The Raspberry Pi gateway is unreachable. Retrying automatically…');
    reconnectTimer = setTimeout(connect, 1200);
  };

  connectingSocket.onerror = () => connectingSocket.close();

  connectingSocket.onmessage = event => {
    if (socket !== connectingSocket) return;
    let message;
    try { message = JSON.parse(event.data); } catch { return; }
    if (!message || typeof message !== 'object' || typeof message.type !== 'string') return;
    lastServerMessageAt = performance.now();
    if (message.type === 'control_status' && message.available === false) {
      if (!lastTelemetryAt || performance.now()-lastTelemetryAt >= 1000) controlUnavailable();
      return;
    }
    if (message.type === 'latency_pong') {
      recordLatency(message.probe_id);
      return;
    }
    if (message.type === 'settings_result' && armSetupForms.has(message.section)) {
      armSetupForms.get(message.section).result(message);
      return;
    }
    if (message.type === 'settings_result' && message.section === 'inspection') {
      if (!message.ok) {
        pendingPasses = null;
        $('confirm-passes-feedback').textContent = message.message || 'Confirmation settings could not be saved.';
      }
      updateMetalSettings();
      return;
    }
    if (message.type === 'settings_result' && message.section === 'gnss') {
      window.operatorGnssResult?.(message);
      return;
    }
    if (message.type === 'settings_result' && message.section === 'marking') {
      window.markingSettingsResult?.(message);
      return;
    }
    if (message.type === 'settings_result' && message.section === 'driving') {
      if (!message.ok) {
        failDriveSave(message.message || 'Drive settings rejected.');
      }
      return;
    }
    if (message.type === 'settings_result') {
      if (message.section === 'display' && !message.ok) {
        displaySignature = '';
        applyDisplaySettings(state.settings?.display);
        showNotice(message.message, true);
      }
      window.operatorSettingsResult?.(message);
      return;
    }
    if (message.type === 'adc_result') { window.operatorADCResult?.(message); return; }
    if (message.type === 'error') {
      const request = message.request_id;
      const pumpMatch = request && pendingPumpMark?.id === request;
      const metalMatch = request && metalPending?.id === request;
      const servoMatch = servoSettings.some(panel => panel.error(message.message, request));
      if (pumpMatch) pendingPumpMark = null;
      if (metalMatch) { metalPending = null; metalFeedback = message.message; }
      if (request && ((message.command_type === 'pump_mark' && !pumpMatch)
          || (message.command_type === 'detector_calibrate' && !metalMatch)
          || (message.command_type === 'arm_servo' && !servoMatch))) return;
      showNotice(message.message, true);
      return;
    }
    if (message.type !== 'state') return;
    lastTelemetryAt = performance.now();
    const recoveringControl = !state.connected;
    if (recoveringControl) setConnection(true, 'Raspberry Pi connected');
    const sensor = message.detector?.sensor;
    if (sensor) {
      if (sensor.history_stream) receiveDetectorStream(sensor, lastTelemetryAt);
      else if (Object.hasOwn(sensor, 'history')) {
        if (Array.isArray(sensor.history)) detectorHistoryAt = lastTelemetryAt - Math.max(0, sensor.history_age_s ?? 0) * 1000;
      } else sensor.history = state.detector.sensor?.history;
    }
    if (message.probe && !Object.hasOwn(message.probe, 'contact_samples')) {
      message.probe.contact_samples = state.probe.contact_samples;
      message.probe.history_time_s = state.probe.history_time_s;
    } else if (Array.isArray(message.probe?.contact_samples)) {
      pressureSource = message.probe.contact_samples;
      pressureReceivedAt = lastTelemetryAt - Math.max(0, message.probe.history_age_s ?? 0) * 1000;
    }
    if (recoveringControl) {
      observedDescent = null;
      lastSeenProbeResultTime = message.probe?.last_result?.timestamp || 0;
    }
    Object.assign(state, message);
    state.connected = true;
    updateDrivingSettings(message.settings?.driving);
    applyDisplaySettings(message.settings?.display);
    window.operatorSettingsState?.(message.settings);
    recordDetectorAngle(lastTelemetryAt);
    recordMotionFrame(lastTelemetryAt);
    updateNetworkCameraOptions(awaitingFirstState);
    awaitingFirstState = false;
    updateCameraLatency();
    updateHud();
  };
}

setInterval(measureLatency, LATENCY_PERIOD_MS);
setInterval(() => {
  if (socket && socket.readyState < WebSocket.CLOSING &&
      performance.now() - lastServerMessageAt > GATEWAY_TIMEOUT_MS) {
    setConnection(false, 'Gateway connection lost', 'No messages received from the Raspberry Pi gateway. Retrying automatically…');
    socket.close();
  } else if (state.connected && performance.now() - lastTelemetryAt > 1000) {
    controlUnavailable();
  }
}, 500);

$('retry-connection').onclick = connect;

// Rejected commands remain visible independently of live workflow telemetry.
function showNotice(text, persistent = false) {
  if (persistent && noticeExpiry === Infinity && noticeText === text && !$('notice').classList.contains('hidden')) return;
  noticeText = text;
  const time = new Date().toLocaleTimeString([], {hour: '2-digit', minute: '2-digit', second: '2-digit'});
  $('notice').textContent = `${time} · ${text}`;
  $('notice').classList.remove('hidden');
  noticeExpiry = persistent ? Infinity : performance.now() + 6000;
}

/* ------------------------------------------------------------------ HUD --- */

function setRange(input, min, max, step) {
  if (Number(input.min) !== min) input.min = min;
  if (Number(input.max) !== max) input.max = max;
  if (Number(input.step) !== step) input.step = step;
}

function updatePower() {
  const power = state.power || {};
  const battery = power.battery || {};
  const live = state.connected && battery.available && !battery.stale && battery.present;
  const fmt = (value, unit, digits = 1) => Number.isFinite(value) ? `${value.toFixed(digits)} ${unit}` : '—';
  const text = (id, value) => { $(id).textContent = value; };
  text('power-status', !state.connected ? 'DISCONNECTED' : battery.stale ? 'STALE' : live ? 'LIVE' : 'NO BATTERY DATA');
  $('power-status').className = `badge ${live ? 'safe' : 'neutral'}`;
  text('battery-watts', live ? fmt(battery.power_w, 'W') : '—');
  text('battery-charge', live ? fmt(battery.remaining_pct, '%', 0) : '—');
  text('battery-voltage', live ? fmt(battery.voltage_v, 'V', 2) : '—');
  text('battery-current', live ? fmt(battery.current_a, 'A', 2) : '—');
  text('battery-temperature', live ? fmt(battery.temperature_c, '°C') : '—');
  text('battery-cells', live && Number.isFinite(battery.cell_delta_v) ? fmt(battery.cell_delta_v * 1000, 'mV', 0) : '—');
  $('battery-level').hidden = !live || !Number.isFinite(battery.remaining_pct);
  if (live && Number.isFinite(battery.remaining_pct)) $('battery-level').value = battery.remaining_pct;
  const history = live ? (power.history || []).filter(p => Number.isFinite(p.watts) && p.age_s <= 60) : [];
  const low = Math.min(0, ...history.map(p => p.watts)), high = Math.max(1, ...history.map(p => p.watts));
  $('power-history-line').setAttribute('points', history.map(p => `${300 * (1 - p.age_s / 60)},${35 - 32 * (p.watts - low) / (high - low)}`).join(' '));
  const esc = power.esc || {};
  const escLive = state.connected && esc.available && !esc.stale;
  text('esc-status', !state.connected ? 'Disconnected' : esc.stale ? 'Stale' : escLive ? 'Live' : 'No telemetry');
  const rows = $('esc-readings');
  rows.replaceChildren();
  if (escLive && esc.motors?.length) {
    esc.motors.forEach(motor => {
      const row = document.createElement('div');
      row.textContent = motor.online ? `ESC ${motor.id} · ${fmt(motor.power_w, 'W')} · ${fmt(motor.current_a, 'A')} · ${fmt(motor.temperature_c, '°C')} · ${fmt(motor.rpm, 'rpm', 0)}${motor.faults ? ' · FAULT' : ''}` : `ESC ${motor.id} · Offline`;
      rows.append(row);
    });
  } else rows.textContent = esc.stale ? 'ESC readings are stale' : 'No ESC measurements received';
  const arm = state.actuators?.arm || state.arm_servo || {};
  text('power-arm', state.connected && arm.connected ? `ID ${arm.servo_id} · ${fmt(arm.voltage, 'V')} · ${fmt(arm.current_a, 'A', 3)} · ${fmt(arm.temperature_c, '°C', 0)}` : 'No telemetry');
}

/* -------------------------------------------------- Target Log & Probe Cycle --- */

let targetLog = [];

let lastSeenProbeResultTime = 0;
let observedDescent = null;
let probeAlertTimeout = null;
let audioCtx = null;

function playContactAlertSound() {
  try {
    if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    if (audioCtx.state === 'suspended') audioCtx.resume();
    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();
    osc.connect(gain);
    gain.connect(audioCtx.destination);
    const now = audioCtx.currentTime;
    osc.type = 'sawtooth';
    osc.frequency.setValueAtTime(880, now);
    osc.frequency.setValueAtTime(1760, now + 0.1);
    osc.frequency.setValueAtTime(880, now + 0.2);
    osc.frequency.setValueAtTime(1760, now + 0.3);
    gain.gain.setValueAtTime(0.3, now);
    gain.gain.exponentialRampToValueAtTime(0.01, now + 0.45);
    osc.start(now);
    osc.stop(now + 0.45);
  } catch (e) {}
}

/* ------------------------------------------------------- Search alert ---
 * Warns the operator that a reading may be worth stopping for. Uses the
 * pure stepSearchAlert() above; never sends commands.
 */
const SEARCH_ALERT_LINGER_MS = 8000;
let searchAlert = {phase: 'idle'};
let searchAlertDismissedAt = -Infinity;
let searchAlertLevelKey = '';

function ensureAudio() {
  try {
    if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    if (audioCtx.state === 'suspended') audioCtx.resume();
  } catch {}
}
// Browsers only allow sound after a click or key press, so unlock it early.
window.addEventListener('pointerdown', ensureAudio, {once: true, capture: true});
window.addEventListener('keydown', ensureAudio, {once: true, capture: true});

function playSearchAlertSound() {
  try {
    ensureAudio();
    const now = audioCtx.currentTime;
    for (const start of [0, 0.18]) {
      const osc = audioCtx.createOscillator();
      const gain = audioCtx.createGain();
      osc.connect(gain);
      gain.connect(audioCtx.destination);
      osc.type = 'sine';
      osc.frequency.setValueAtTime(740, now + start);
      gain.gain.setValueAtTime(0.0001, now + start);
      gain.gain.exponentialRampToValueAtTime(0.25, now + start + 0.02);
      gain.gain.exponentialRampToValueAtTime(0.0001, now + start + 0.14);
      osc.start(now + start);
      osc.stop(now + start + 0.15);
    }
  } catch {}
}

function searchAlertVolts() {
  const settings = preferences.searchAlert;
  if (Number.isFinite(settings.volts)) return settings.volts;
  const sensor = state.detector.sensor || {};
  const base = detectorVolts(sensor.baseline_adc, sensor.reference_voltage);
  const full = detectorVolts(sensor.full_response_adc, sensor.reference_voltage);
  if (base === null || full === null || !Number.isFinite(settings.level)) return NaN;
  return base + (full - base) * settings.level / 100;
}

function renderSearchAlertLevel() {
  const {enabled} = preferences.searchAlert;
  const volts = searchAlertVolts();
  const range = detectorVoltageRange(state.detector.sensor || {});
  const field = $('search-alert-level');
  const canEdit = state.connected && state.drive.you_control_owner && !!state.settings?.display;
  field.disabled = !enabled || !canEdit;
  const key = JSON.stringify([enabled, volts, range, canEdit]);
  if (key === searchAlertLevelKey) return;
  searchAlertLevelKey = key;
  if (document.activeElement !== field || field.disabled) field.value = Number.isFinite(volts) ? Number(volts.toFixed(6)) : '';
  const quick = $('detector-alert-input');
  quick.disabled = field.disabled;
  if (document.activeElement !== quick || quick.disabled) quick.value = Number.isFinite(volts) ? Number(volts.toFixed(3)) : '';
  const value = voltageLabel(volts);
  const marker = $('alert-marker');
  marker.hidden = !enabled || !range || !Number.isFinite(volts) || volts < range.min || volts > range.max;
  if (!marker.hidden) marker.style.left = (volts - range.min) / (range.max - range.min) * 100 + '%';
  $('detector-alert-note').textContent = enabled ? 'Alert at ' + value : 'Alert off';
  $('metal-alert-level').textContent = enabled ? value : 'Off';
}

function updateSearchAlert(now) {
  const settings = preferences.searchAlert;
  const detector = state.detector;
  const live = state.connected && detector.fresh !== false && detector.sensor?.fresh !== false;
  const suppressed = Boolean(state.drive.calibrating || state.inspection?.active || state.probe?.active || state.probe?.cycle_active);
  const sensor = detector.sensor || {};
  const signal = detectorVolts(sensor.packet?.filtered_adc, sensor.reference_voltage);
  const span = detectorVolts(sensor.full_response_adc - sensor.baseline_adc, sensor.reference_voltage);
  const shared = state.search_alert;
  if (shared) {
    const fired = shared.fired_at && shared.fired_at !== searchAlert.eventId;
    const first = searchAlert.eventId === undefined;
    searchAlert = {phase:shared.phase, peak:shared.peak_volts, eventId:shared.fired_at || 0,
      firedAt:shared.fired_at ? lastTelemetryAt - (shared.server_time-shared.fired_at)*1000 : -Infinity,
      clearedAt:shared.cleared_at ? lastTelemetryAt - (shared.server_time-shared.cleared_at)*1000 : -Infinity};
    if (fired && !first && settings.sound) playSearchAlertSound();
  } else {
    const step = stepSearchAlert(searchAlert, {enabled:settings.enabled, level:searchAlertVolts(), signal, direction:Math.sign(span),
      hysteresis:span === null ? NaN : Math.abs(span)*.05, live, suppressed, now});
    searchAlert = step.alert;
    if (step.fired && settings.sound) playSearchAlertSound();
  }
  renderSearchAlertLevel();

  const active = searchAlert.phase === 'active' || searchAlert.phase === 'cooling';
  const recent = !active && Number.isFinite(searchAlert.firedAt) && Number.isFinite(searchAlert.clearedAt)
    && searchAlert.clearedAt >= searchAlert.firedAt && now - searchAlert.clearedAt < SEARCH_ALERT_LINGER_MS;
  const shown = state.connected && (active || recent) && searchAlertDismissedAt < searchAlert.firedAt;
  const banner = $('search-alert');
  if (banner.hidden === shown) banner.hidden = !shown;
  document.body.dataset.searchAlert = active ? 'active' : recent ? 'recent' : '';
  if (!shown) return;
  const peak = voltageLabel(searchAlert.peak);
  const reading = voltageLabel(signal);
  banner.dataset.kind = active ? 'active' : 'recent';
  const title = active ? 'Possible metal here' : 'Possible metal behind you';
  const detail = active
    ? `${reading} now · peak ${peak}. Stop and run Check the spot.`
    : `Peak ${peak} ${Math.max(1, Math.round((now - searchAlert.firedAt) / 1000))} s ago. Back up to check it.`;
  if ($('search-alert-title').textContent !== title) $('search-alert-title').textContent = title;
  if ($('search-alert-detail').textContent !== detail) $('search-alert-detail').textContent = detail;
}

window.operatorFieldLog = entries => {
  if (!Array.isArray(entries)) return;
  targetLog = entries;
  renderTargetLog();
};

function logTime(stamp) {
  return typeof stamp === 'number' ? new Date(stamp * 1000).toLocaleString() : stamp;
}

function renderTargetLog() {
  const tbody = $('target-log-body');
  $('target-count').textContent = `${targetLog.length} ${targetLog.length === 1 ? 'ENTRY' : 'ENTRIES'}`;
  $('mission-record-count').textContent = `${targetLog.length} ${targetLog.length === 1 ? 'entry' : 'entries'}`;
  $('clear-target-log').disabled = !state.connected || !state.drive.you_control_owner || !targetLog.length;
  $('export-target-log').disabled = !targetLog.length;
  tbody.replaceChildren();
  if (!targetLog.length) {
    const row = tbody.insertRow();
    row.className = 'empty-row';
    const cell = row.insertCell(); cell.colSpan = 7;
    cell.textContent = 'No entries yet. Log a pose, activate the marking pump, or run a probe cycle.';
    return;
  }
  for (const entry of targetLog) {
    const row = tbody.insertRow();
    row.className = ['contact', 'clear', 'marked'].includes(entry.statusType) ? entry.statusType : '';
    const quality = entry.gps_status?.replaceAll('_', ' ') || 'GNSS';
    const accuracy = Number.isFinite(entry.horizontal_accuracy_m) ? `±${entry.horizontal_accuracy_m.toFixed(2)} m` : 'accuracy unknown';
    const position = Number.isFinite(entry.latitude) && Number.isFinite(entry.longitude)
      ? `${entry.latitude.toFixed(6)}, ${entry.longitude.toFixed(6)} · ${quality} · ${accuracy}`
      : Number.isFinite(entry.x) && Number.isFinite(entry.y) ? `(${entry.x.toFixed(2)}, ${entry.y.toFixed(2)})` : '—';
    for (const value of [String(entry.id).slice(0, 8), position, Number.isFinite(entry.detector_volts) ? entry.detector_volts.toFixed(4) : '',
                         entry.contact_depth_mm ?? '—', Number.isFinite(entry.contact_volts) ? entry.contact_volts.toFixed(4) : '', [entry.status, entry.latitude == null && entry.gps_status && quality].filter(Boolean).join(' · '), logTime(entry.timestamp)]) {
      row.insertCell().textContent = value;
    }
  }

}

function exportTargetLogCsv() {
  if (!targetLog.length) return;
  const header = ['ID', 'X (m)', 'Y (m)', 'Heading (deg)', 'Filtered detector (V)', 'Contact depth (mm)', 'Filtered probe sensor (V)', 'Status', 'Timestamp', 'Latitude', 'Longitude', 'Horizontal error (m)', 'GNSS status'];
  const rows = targetLog.map(t => [t.id, t.x ?? '', t.y ?? '', t.heading_deg ?? '', t.detector_volts ?? '', t.contact_depth_mm ?? '', t.contact_volts ?? '', t.status, t.timestamp, t.latitude ?? '', t.longitude ?? '', t.horizontal_accuracy_m ?? '', t.gps_status ?? '']);
  const csv = [header.join(','), ...rows.map(row => row.map(value => '"' + String(value).replaceAll('"', '""') + '"').join(','))].join('\n');
  const blob = new Blob([csv], {type: 'text/csv;charset=utf-8;'});
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `peaceofmine-targets-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.csv`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function markCurrentPose() {
  if (!state.connected || state.robot.pose_fresh !== true || ![state.robot?.x, state.robot?.y, state.robot?.yaw].every(Number.isFinite)) { showNotice('Fresh vehicle position is required to log a pose.'); return; }
  if (state.drive.you_control_owner) send({type:'log_pose'});
}

function operationBlock(allowManualTarget = false) {
  if (!state.connected) return 'Connect to the operator stack.';
  if (!state.drive.you_control_owner) return 'Start driving first.';
  if (!state.safety?.servo_allowed || !state.drive.armed) return state.safety?.servo_reason || 'RC must permit servo motion.';
  if (state.drive.calibrating) return 'Finish actuator calibration before operating.';
  if (state.inspection?.active) return 'A scan is running. Stop it first.';
  if (state.probe.cycle_active) return 'Probe cycle is running. Stop probe to cancel.';
  if (!allowManualTarget && (state.probe.active || state.probe.holding)) return 'Stop probe before starting another action.';
  if (state.pump_active || state.drive.pump_active) return 'Wait for the marking pump to stop.';
  return '';
}

function probeActionBlock(allowManualTarget = false) {
  const blocked = operationBlock(allowManualTarget);
  if (blocked) return blocked;
  if (state.actuators?.probe?.connected === false) return 'Waiting for probe encoder telemetry.';
  if (!Number.isFinite(state.probe.max_depth_mm) || state.probe.max_depth_mm <= 0) return 'Save the measured travel in Setup → Probe.';
  if (!state.probe.ready || !Number.isFinite(state.probe.depth_mm)) return 'Waiting for probe encoder telemetry.';
  if (state.probe.fault) return 'Clear the probe fault in Setup → Probe.';
  return '';
}

function pumpAuthorityBlock() {
  if (!state.marking?.settings) return 'Waiting for marking settings from the car.';
  if (!state.safety?.servo_allowed) return state.safety?.servo_reason || 'Waiting for RC and PX4 arming state.';
  if (state.safety.mode !== 'simulation' && state.drive.px4_armed !== true) return 'Arm PX4 to operate the pump.';
  return '';
}

function triggerPumpMark() {
  const blocked = operationBlock() || pumpAuthorityBlock();
  if (blocked) { showNotice(blocked); return; }
  if (pendingPumpMark) return;
  pendingPumpMark = {at: performance.now(), id: `mark-${Date.now()}-${Math.random().toString(36).slice(2)}`};
  send({type: 'pump_mark', request_id: pendingPumpMark.id});
  showNotice('Marking pump requested. Waiting for the operator stack.');
}

function startProbeCycle() {
  const blocked = probeActionBlock() || window.operatorContactBlock?.();
  if (blocked) { showNotice(blocked); return; }
  if (state.detector.sweep_enabled) { showNotice('Stop sweeping before probing.'); return; }
  const contact = state.probe.adc_contact_state;
  if (!contact?.enabled || !contact.fresh) { showNotice('Save a contact limit and wait for a fresh reading.'); return; }
  const input = $('probe-cycle-target');
  if (!input.reportValidity()) return;
  send({type: 'start_probe_cycle', depth_mm: input.valueAsNumber});
  showNotice(`Probe cycle requested to ${input.valueAsNumber} mm.`);
}

function checkProbeCycleState() {
  const probe = state.probe || {};
  const alertEl = $('probe-cycle-alert');
  const alertBadge = $('probe-cycle-alert-badge');
  const alertText = $('probe-cycle-alert-text');

  if (probe.cycle_active && probe.cycle_phase === 'DESCENDING') {
    observedDescent = probe.cycle_started_at;
    alertEl?.classList.add('hidden');
    if (probeAlertTimeout) clearTimeout(probeAlertTimeout);
  } else {
    if (probe.cycle_active && probe.contact_detected === true && Number.isFinite(probe.contact_depth_mm)) {
      alertEl?.classList.remove('hidden');
      if (alertBadge) {
        alertBadge.textContent = 'CONTACT ON DESCENT';
        alertBadge.className = 'badge active';
      }
      if (alertText) alertText.textContent = `Hit at ${probe.contact_depth_mm.toFixed(1)} mm on descent. Retracting.`;
      if (state.connected && !document.hidden && Number.isFinite(observedDescent) &&
          observedDescent === probe.cycle_started_at) playContactAlertSound();
    } else if (probe.cycle_active) {
      alertEl?.classList.add('hidden');
    }
    observedDescent = null;
  }

  // Completed result
  const lastRes = probe.last_result;
  if (!probe.cycle_active && lastRes && lastRes.timestamp && lastRes.timestamp !== lastSeenProbeResultTime) {
    lastSeenProbeResultTime = lastRes.timestamp;
    if (lastRes.contact) {
      if (alertEl) alertEl.classList.remove('hidden');
      if (alertBadge) {
        alertBadge.textContent = 'CONTACT ON DESCENT';
        alertBadge.className = 'badge active';
      }
      if (alertText) {
        alertText.textContent = `Contact trigger at ${lastRes.contact_depth_mm ?? '—'} mm${Number.isFinite(lastRes.contact_volts) ? ` · ${lastRes.contact_volts.toFixed(4)} V` : ''}. Probe retracted.`;
      }

    } else if (lastRes.status === 'CLEAR') {
      if (alertEl) alertEl.classList.remove('hidden');
      if (alertBadge) {
        alertBadge.textContent = 'DEPTH REACHED';
        alertBadge.className = 'badge safe';
      }
      if (alertText) {
        alertText.textContent = `No contact trigger to ${Math.round(lastRes.target_depth_mm)} mm. Probe retracted.`;
      }
      showNotice(`No contact trigger to ${Math.round(lastRes.target_depth_mm)} mm.`);
    } else if (['ABORTED', 'TIMEOUT'].includes(lastRes.status)) {
      if (alertEl) alertEl.classList.remove('hidden');
      if (alertBadge) { alertBadge.textContent = lastRes.status === 'TIMEOUT' ? 'TIMED OUT' : 'STOPPED'; alertBadge.className = 'badge blocked'; }
      const contact = probe.contact_detected ? `Contact${Number.isFinite(probe.contact_depth_mm) ? ` at ${probe.contact_depth_mm.toFixed(1)} mm` : ' detected'}. ` : '';
      const message = `${contact}${contact ? 'Return' : 'Cycle'} stopped: ${lastRes.reason || lastRes.status}. Check probe position.`;
      if (alertText) alertText.textContent = message;
    }

    if (probeAlertTimeout) clearTimeout(probeAlertTimeout);
    probeAlertTimeout = setTimeout(() => {
      if (alertEl && !state.probe?.cycle_active) alertEl.classList.add('hidden');
    }, 8000);
  }
}

function probeOffsetDeg() {
  // Older car processes used a fixed 45° mount and do not publish the setting.
  return state.inspection?.probe_offset_deg ?? state.settings?.probe_alignment?.offset_deg ?? 45;
}

function getProbeAlignment() {
  const detector = state.detector || {};
  const half = detector.beam_half_angle_deg ?? 45;
  const minAngle = detector.beam_min_angle_deg ?? -half;
  const maxAngle = detector.beam_max_angle_deg ?? half;

  const peak = detector.latest_peak;
  const peakAngle = detector.fresh === true && peak?.fresh === true && Number.isFinite(peak.angle_deg) ? peak.angle_deg : null;
  const peakSignal = peakAngle === null ? 0 : peak.signal;

  if (peakAngle === null) {
    return {
      hasPeak: false,
      peakAngle: null,
      peakSignal: 0,
      pivotAngle: null,
      reachable: false,
      deficitDeg: 0,
      reason: 'Sweep to record a fresh detection peak.'
    };
  }

  const pivotAngle = peakAngle - probeOffsetDeg();

  const isReachable = pivotAngle >= minAngle && pivotAngle <= maxAngle;
  const deficitDeg = pivotAngle < minAngle ? Math.round((minAngle - pivotAngle) * 10) / 10 : 0;

  let reason = '';
  if (!isReachable) {
    if (pivotAngle < minAngle) {
      reason = `Target is outside the arm range. Reposition the rover and scan again.`;
    } else {
      reason = `Target angle outside arm limits (${pivotAngle.toFixed(1)}° vs max ${maxAngle.toFixed(1)}°).`;
    }
  } else {
    reason = `Pivot arm to ${pivotAngle.toFixed(1)}° to align probe with target peak at ${peakAngle.toFixed(1)}°.`;
  }

  return {
    hasPeak: true,
    peakAngle,
    peakSignal,
    pivotAngle,
    reachable: isReachable,
    deficitDeg,
    reason
  };
}

function armSetupBlock() {
  const arm = state.actuators?.arm;
  if (arm?.connected !== true) return arm?.reason || 'Connect the arm in Setup → Arm.';
  if (!arm.arm_calibration && !arm.calibration) return 'Calibrate the arm in Setup → Arm.';
  return '';
}

function alignProbeToPeak() {
  const blocked = operationBlock() || armSetupBlock();
  if (blocked) { showNotice(blocked); return; }
  const align = getProbeAlignment();
  if (!align.hasPeak || !align.reachable) { showNotice(align.reason); return; }
  send({type: 'align_probe', target_bearing_deg: align.peakAngle, angle_deg: align.pivotAngle});
  showNotice(`Arm alignment requested to ${Number(align.pivotAngle.toFixed(1))}°.`);
}

function inspectionBlock(action) {
  const blocked = operationBlock(true) || armSetupBlock();
  if (blocked) return blocked;
  if (action === 'confirm') return state.detector.fresh === true ? '' : 'Wait for fresh detector readings before confirming.';
  const probeBlocked = probeActionBlock(true);
  if (probeBlocked) return probeBlocked;
  if (!Number.isFinite(state.inspection?.confirmed_angle_deg)) return 'Confirm the location first.';
  if (state.inspection?.probe_reachable === false) return 'Confirmed target is outside probe reach. Reposition the rover and confirm again.';
  return ''; // The probe checks the applied contact input when each descent begins.
}

function startInspection(action) {
  const blocked = inspectionBlock(action);
  if (blocked) { showNotice(blocked); return; }
  const command = {type: 'inspection', action};
  if (action === 'probe_scan') {
    const depth = $('probe-cycle-target');
    if (!depth.reportValidity()) return;
    command.depth_mm = depth.valueAsNumber;
  }
  stopInput();
  send(command);
}

function returningProbePoint(inspection, probe) {
  if (!state.connected || inspection.phase !== 'PROBING' || !inspection.active || !inspection.probe_goal_id
      || probe.cycle_goal_id !== inspection.probe_goal_id || !probe.cycle_active || probe.cycle_phase !== 'RETRACTING') return null;
  const angle = inspection.probe_angles_deg?.[inspection.completed];
  if (!Number.isFinite(angle) || inspection.completed !== inspection.points?.length) return null;
  if (probe.contact_detected === false) return {angle_deg:angle, status:'CLEAR', returning:true};
  if (probe.contact_detected === true && Number.isFinite(probe.contact_depth_mm) && probe.contact_depth_mm >= 0)
    return {angle_deg:angle, status:'CONTACT', contact_depth_mm:probe.contact_depth_mm, returning:true};
  return null;
}

function probePlanLabel(count, spacing, requested = count) {
  if (!Number.isInteger(count) || count < 1) return '';
  let label = count + (count === 1 ? ' position' : ' positions');
  if (Number.isInteger(requested) && requested > count) label = `${count} of ${requested} positions in range`;
  if (count > 1 && Number.isFinite(spacing) && spacing > 0) label += ` · ${spacing}° spacing`;
  return label;
}

function recordedProbeSpacing(inspection) {
  if (Number.isFinite(inspection.scan_spacing_deg)) return inspection.scan_spacing_deg;
  // Older snapshots may only have the ordered bearings. Clipped intervals
  // cannot establish the original setting when their spacing varies.
  const angles = inspection.probe_angles_deg;
  if (!Array.isArray(angles) || angles.length < 2 || !angles.every(Number.isFinite)) return null;
  const spacing = angles[1] - angles[0];
  return spacing > 0 && angles.slice(1).every((angle, i) => Math.abs(angle - angles[i] - spacing) < .001)
    ? Number(spacing.toFixed(4)) : null;
}

function clippedProbeEdges(angles, bearing, count, spacing) {
  if (!angles?.length || !Number.isFinite(bearing) || !Number.isInteger(count) || !Number.isFinite(spacing)) return '';
  const halfSpan = (count - 1) * spacing / 2;
  const limits = [];
  if (angles[0] > bearing - halfSpan + .0001) limits.push(`right limit ${angles[0].toFixed(1)}°`);
  if (angles.at(-1) < bearing + halfSpan - .0001) limits.push(`left limit ${angles.at(-1).toFixed(1)}°`);
  return limits.join(' · ');
}

function renderProbeScanMap(inspection, points) {
  const map = $('probe-scan-map');
  const olderScan = points.length > 0 || inspection.phase === 'PROBING' || inspection.phase === 'COMPLETE';
  const oldCount = olderScan && Number.isInteger(inspection.total) && inspection.total <= 31
    ? Math.max(points.length, inspection.total) : points.length;
  const plan = Array.isArray(inspection.probe_angles_deg) && inspection.probe_angles_deg.length
    ? inspection.probe_angles_deg : Array.from({length:oldCount}, (_, i) => points[i]?.angle_deg);
  const label = probePlanLabel(plan.length, recordedProbeSpacing(inspection), inspection.scan_point_count);
  const clipped = clippedProbeEdges(inspection.probe_angles_deg, inspection.probe_target_bearing_deg,
                                    inspection.scan_point_count, inspection.scan_spacing_deg);
  $('probe-map-plan').textContent = clipped ? label.replace('° spacing', '° requested') + ' · ' + clipped : label;
  const results = $('probe-map-readings');
  const target = inspection.probe_target_depth_mm;
  const bearing = inspection.probe_target_bearing_deg ?? inspection.confirmed_angle_deg;
  const angleLabel = value => Number.isFinite(value) ? `${Number(value.toFixed(3))}°` : '—';
  $('probe-map-centers').hidden = !plan.length;
  $('probe-map-bearing').textContent = angleLabel(bearing);
  $('probe-map-arm-center').textContent = angleLabel(inspection.probe_arm_center_deg);
  $('probe-map-depth').textContent = Number.isFinite(target) ? `Target depth ${Number(target.toFixed(1))} mm` : '';
  map.toggleAttribute('hidden', !plan.length);
  $('probe-map-empty').hidden = !!plan.length;
  map.replaceChildren();
  if (!plan.length) { results.hidden = true; results.replaceChildren(); return; }

  const make = (tag, attributes, content) => {
    const node = document.createElementNS('http://www.w3.org/2000/svg', tag);
    for (const [name, value] of Object.entries(attributes)) node.setAttribute(name, value);
    if (content !== undefined) node.textContent = content;
    return node;
  };
  // Use one scale around the target bearing. A clipped scan stays on the
  // correct side of the centerline and wide arcs can extend behind the arm.
  const knownAngles = plan.filter(Number.isFinite);
  const first = knownAngles.length ? Math.min(...knownAngles) : 0;
  const last = knownAngles.length ? Math.max(...knownAngles) : 0;
  const center = Number.isFinite(bearing) ? bearing : (first + last) / 2;
  const radius = 700;
  const at = (angle, r) => {
    const radians = (angle - center) * Math.PI / 180;
    return [-r * Math.sin(radians), radius - r * Math.cos(radians)];
  };
  const steps = Math.min(180, Math.max(1, Math.ceil((last-first)/4)));
  const arc = Array.from({length:steps+1}, (_, i) => first + (last-first)*i/steps);
  const outline = arc.flatMap(angle => [at(angle, radius), at(angle, radius-65)]);
  const side = Math.max(...outline.map(([x]) => Math.abs(x)));
  const bottom = Math.max(90, ...outline.map(([, y]) => y));
  const missing = plan.length-knownAngles.length;
  const scale = Math.max(1, (2*side+76)/460, (bottom+52)/145);
  const width = Math.max(460, 2*side+76*scale);
  const height = bottom+(52+(missing ? 28 : 0))*scale;
  map.setAttribute('viewBox', `${-width/2} ${-26*scale} ${width} ${height}`);
  map.style.setProperty('--scan-label-size', `${12*scale}px`);
  const positions = knownAngles.map(angle => at(angle, radius));
  const overlap = positions.some(([x,y], i) => positions.slice(0,i).some(([x2,y2]) =>
    Math.abs(x-x2) < 55*scale && Math.abs(y-y2) < 46*scale));
  const dense = plan.length > 7 || missing > 0 || overlap;
  results.hidden = !dense;
  if (knownAngles.length) {
    for (const r of [radius-40, radius-20, radius]) {
      map.append(make('polyline', {class:'scan-ring', points:arc.map(angle => at(angle, r).join(',')).join(' ')}));
    }
    if (Number.isFinite(bearing)) map.append(make('path', {class:'scan-center', d:'M0,-14 V65'}));
  }
  let unlocated = 0;
  const legend = [];
  const rows = [];
  plan.forEach((angle, index) => {
    const point = points[index];
    const validAngle = Number.isFinite(angle);
    const matches = validAngle && Number.isFinite(point?.angle_deg) && Math.abs(angle - point.angle_deg) < .001;
    const hit = matches && point.status === 'CONTACT' && Number.isFinite(point.contact_depth_mm) && point.contact_depth_mm >= 0;
    const clear = matches && point.status === 'CLEAR';
    const depth = hit ? point.contact_depth_mm : clear && Number.isFinite(target) ? target : null;
    const depthLabel = Number.isFinite(depth) ? `${Number(depth.toFixed(1))} mm` : '—';
    const status = hit ? `Early contact at ${depthLabel}` : clear ? (depth === null ? 'Target reached' : `Target reached at ${depthLabel}`)
      : !point && inspection.phase === 'PROBING' ? 'Pending' : 'Unknown';
    const label = angleLabel(angle);
    const description = `${validAngle ? label : 'Angle unavailable'}: ${status}${point?.returning ? ' · retracting' : ''}`;
    legend.push(description);
    const [x, y] = validAngle ? at(angle, radius) : [(missing === 1 ? 0 : .5 - unlocated++/(missing-1))*width*.85, bottom+16*scale];
    const size = 5*scale;
    const mark = make('g', {'data-result': hit ? 'contact' : clear ? 'clear' : 'unknown'});
    mark.append(make('title', {}, description));
    if (validAngle) mark.append(make('path', {class:'scan-ray', d:`M${at(angle, radius-65)} L${x},${y}`}));
    mark.append(hit ? make('path', {class:'map-contact', d:`M${x},${y-size} l${size},${size} -${size},${size} -${size},-${size} z`})
      : make('circle', {class:clear ? 'map-clear' : 'map-unknown', cx:x, cy:y, r:size}));
    // Keep a single overview of the entire scan as the point count grows.
    // The grid follows the fan from left to right and labels every marker.
    if (!dense) mark.append(make('text', {x, y:y-12*scale}, label));
    if (!dense) mark.append(make('text', {class:'scan-depth', x, y:y+20*scale}, depthLabel));
    else {
      const row = document.createElement('li');
      row.dataset.result = mark.dataset.result;
      row.setAttribute('aria-label', description);
      row.title = description;
      const symbol = document.createElement('span');
      symbol.className = 'probe-map-symbol';
      symbol.setAttribute('aria-hidden', 'true');
      symbol.textContent = hit ? '◆' : clear ? '●' : '○';
      const angleText = document.createElement('span');
      angleText.textContent = label;
      const depthText = document.createElement('strong');
      depthText.textContent = depthLabel;
      row.append(symbol, angleText, depthText);
      rows.push(row);
    }
    map.append(mark);
  });
  results.replaceChildren(...rows.reverse());
  map.setAttribute('aria-label', 'Probe scan map. ' + legend.join('. '));
}

let inspectionPointsKey = '';
let inspectionTaskKey = '';
function renderInspection() {
  const inspection = state.inspection;
  const phase = inspection.phase || 'IDLE';
  $('inspection-status').textContent = inspection.message || ({IDLE: 'Sweep to find a hit, then confirm, probe and mark it.', CONFIRMING: 'Comparing sweeps across the target…', CONFIRMED: 'Confirmation complete. Ready for probe scan.', PROBING: 'Probe scan in progress…', COMPLETE: 'Probe scan complete.', STOPPED: 'Inspection stopped.', ERROR: 'Inspection stopped. Check actuator status.'})[phase];
  const total = Number.isFinite(inspection.total) && inspection.total > 0 ? inspection.total : 0;
  const completed = Number.isFinite(inspection.completed) ? inspection.completed : 0;
  const probe = state.probe;
  if (phase === 'PROBING' && probe.cycle_active) {
    const movement = probe.cycle_phase === 'DESCENDING' ? 'Probing' : 'Retracting';
    const depth = Number.isFinite(probe.depth_mm) ? ` · ${probe.depth_mm.toFixed(1)} mm` : '';
    $('inspection-status').textContent = `Position ${Math.min(total, completed + 1)} of ${total}: ${movement}${depth}.`;
  }
  const taskKey = `${phase}:${Boolean(probe.cycle_active)}`;
  if (taskKey !== inspectionTaskKey) {
    if (probe.cycle_active || ['CONFIRMED', 'PROBING', 'COMPLETE'].includes(phase)) $('probe-task').open = true;
    else if (['IDLE', 'CONFIRMING'].includes(phase)) $('search-task').open = true;
    inspectionTaskKey = taskKey;
  }
  const progress = $('inspection-progress');
  progress.hidden = total === 0 || ['ERROR', 'STOPPED'].includes(phase);
  progress.max = total || 1;
  if (inspection.active && phase === 'CONFIRMING' && !Number.isFinite(inspection.progress)) {
    progress.removeAttribute('value');
    progress.setAttribute('aria-valuetext', inspection.message || 'Moving to sweep start');
  } else {
    const value = Math.max(0, Math.min(total, Number.isFinite(inspection.progress) ? inspection.progress : completed));
    progress.value = value;
    const percent = total ? Math.floor(value / total * 100) : 0;
    progress.setAttribute('aria-valuetext', phase === 'CONFIRMING'
      ? `${percent}% · pass ${Math.min(total, Math.floor(value) + 1)} of ${total}`
      : `${percent}% · ${Math.round(value)} of ${total} complete`);
  }
  $('inspection-stop').disabled = !state.connected || !state.drive.you_control_owner || !inspection.active;
  $('inspection-stop').hidden = !inspection.active;
  $('mode-sweep-btn').dataset.active = String(state.detector.sweep_enabled);
  $('mode-align-btn').dataset.active = String(phase === 'CONFIRMING');
  $('mode-probe-btn').dataset.active = String(phase === 'PROBING');
  const points = Array.isArray(inspection.points) ? inspection.points : [];
  $('inspection-points').parentElement.hidden = !points.length;
  const returning = returningProbePoint(inspection, probe);
  const mapPoints = returning ? [...points, returning] : points;
  const key = JSON.stringify([mapPoints, inspection.probe_angles_deg, inspection.probe_target_depth_mm, inspection.probe_target_bearing_deg, inspection.probe_arm_center_deg, inspection.confirmed_angle_deg, inspection.total, inspection.scan_spacing_deg, inspection.scan_point_count, phase]);
  if (key === inspectionPointsKey) return;
  inspectionPointsKey = key;
  renderProbeScanMap(inspection, mapPoints);
  $('inspection-points').replaceChildren(...points.map(point => {
    const item = document.createElement('li');
    const angle = Number.isFinite(point?.angle_deg) ? `${point.angle_deg.toFixed(1)}°` : 'Angle unavailable';
    const status = point?.status === 'CLEAR' ? 'Target reached' : point?.status === 'CONTACT' && Number.isFinite(point.contact_depth_mm) && point.contact_depth_mm >= 0 ? 'Early contact' : 'Unknown result';
    const depth = status === 'Early contact' ? ` · ${point.contact_depth_mm.toFixed(1)} mm` : '';
    item.textContent = `${angle} · ${status}${depth}`;
    return item;
  }));
}

function updateHud() {
  updateAlignmentSettings();
  updatePower();
  updateArmSettings();
  updateMetalSettings();
  const {drive, robot, detector, probe} = state;
  const owner = state.connected && drive.you_control_owner;
  if ((cameraFocused || keyboardSweep) && (!owner || !state.safety?.servo_allowed || !drive.armed)) {
    const wasDriving = lastInput.deadman;
    clearHeldInput();
    if (wasDriving && owner) sendDrive(0, 0, false);
  }

  $('client-count').textContent = `${drive.client_count} ${drive.client_count === 1 ? 'viewer' : 'viewers'}`;
  $('control-owner').textContent = owner ? "You're driving" : (drive.control_owner_present ? 'Watching only' : 'Nobody driving');
  $('control-owner').className = `badge ${owner ? 'safe' : 'neutral'}`;
  $('take-control').textContent = owner ? 'Stop driving' : drive.control_owner_present ? 'Take over driving' : 'Start driving';

  $('take-control').disabled = !state.connected || owner;
  const safe = state.safety?.allowed === true;
  const servoSafe = state.safety?.servo_allowed === true && drive.armed;
  const px4Armed = state.connected ? drive.px4_armed : null;
  const px4Badge = $('px4-arming');
  const known = typeof px4Armed === 'boolean';
  const simulated = state.connected && state.safety?.simulated && !known;
  const unknown = state.connected && drive.px4_state_status === 'stale' ? 'Car status unknown' : 'Car status unknown';
  setText(px4Badge, known ? (px4Armed ? 'Car unlocked' : 'Car locked') : simulated ? 'Car lock: n/a' : unknown);
  px4Badge.className = `badge ${known ? (px4Armed ? 'active' : 'neutral') : simulated ? 'neutral' : 'warn'}`;
  px4Badge.title = known ? 'Vehicle arming reported by MAVROS' : simulated ? 'Simulation has no PX4 arming telemetry' : 'Vehicle arming telemetry is missing or stale';
  for (const [setupId, badge] of [['setup-arming', px4Badge], ['setup-owner', $('control-owner')]]) {
    const copy = $(setupId);
    setText(copy, badge.textContent);
    copy.className = badge.className;
  }
  const canConfigureDrive = owner && !!state.settings?.driving && !drivingPending;
  for (const id of ['drive-control-source', 'keyboard-max-throttle', 'keyboard-reverse-throttle'])
    $(id).disabled = !canConfigureDrive;
  $('rc-safety').textContent = state.safety?.mode === 'simulation' ? 'Simulation' : safe ? 'Remote: driving allowed' : 'Remote: driving blocked';
  if (servoSafe && !safe) $('rc-safety').textContent += ' · arm can move';
  $('rc-safety').title = state.safety?.reason || 'Waiting for PX4 safety status';
  $('rc-safety').className = `badge ${safe ? 'ready' : 'blocked'}`;

  $('environment-label').textContent = state.safety?.mode === 'simulation' ? 'Simulation' : state.connected ? 'Vehicle operator' : 'Connecting';
  $('stop-controls').disabled = !owner;
  const blocked = probeActionBlock();
  const otherActionBlock = operationBlock();
  const align = getProbeAlignment();
  const contact = probe.adc_contact_state;
  const contactBlock = !contact?.enabled || !contact.fresh ? 'Save a contact limit and wait for a fresh reading.' : '';
  const cycleBlock = blocked || window.operatorContactBlock?.() || (detector.sweep_enabled ? 'Stop sweeping before probing.' : '') || contactBlock;
  const alignBlock = otherActionBlock || armSetupBlock() || (!align.hasPeak || !align.reachable ? align.reason : '');
  const markBlock = otherActionBlock || pumpAuthorityBlock();
  const sweepBlock = otherActionBlock || armSetupBlock();
  const confirmBlock = inspectionBlock('confirm');
  const scanBlock = inspectionBlock('probe_scan');
  $('operation-help').textContent = otherActionBlock || 'Drive with step 1 on until the detector reacts, stop, then run steps 2 to 4. STOP halts everything.';
  const shownBlocks = new Set([otherActionBlock]);
  for (const [id, reason] of [['mode-sweep-help', sweepBlock], ['mode-align-help', confirmBlock],
    ['mode-probe-help', scanBlock], ['mode-mark-help', markBlock], ['probe-cycle-help', scanBlock]]) {
    const text = reason && !shownBlocks.has(reason) ? reason : '';
    $(id).textContent = text;
    $(id).hidden = !text;
    if (text) shownBlocks.add(text);
  }
  $('detector-align-probe').disabled = Boolean(alignBlock);
  $('detector-align-probe').title = alignBlock || align.reason;
  $('align-probe-hint').textContent = alignBlock || align.reason;
  $('mode-align-btn').disabled = Boolean(confirmBlock);
  $('mode-align-btn').title = confirmBlock || `Compare ${state.settings?.inspection?.passes ?? 2} passes across the target.`;
  $('mode-align-btn').querySelector('.mode-text').textContent = state.inspection.phase === 'CONFIRMING' ? 'Checking…' : 'Check the spot';
  $('mode-align-pill').textContent = state.inspection.phase === 'CONFIRMING' ? `${state.inspection.completed || 0}/${state.inspection.total}` : confirmBlock ? 'BLOCKED' : `${state.settings?.inspection?.passes ?? 2} PASSES`;
  $('start-probing').disabled = Boolean(cycleBlock);
  $('start-probing').title = cycleBlock;
  $('start-probing').textContent = probe.cycle_active ? (probe.cycle_phase === 'DESCENDING' ? 'Probing…' : 'Retracting…') : 'Probe once';
  $('mode-probe-btn').disabled = Boolean(scanBlock);
  const scan = state.inspection;
  const scanSettings = state.settings?.probe_scan;
  const scanLabel = scan.phase === 'PROBING'
    ? probePlanLabel(scan.probe_angles_deg?.length || scan.total, recordedProbeSpacing(scan), scan.scan_point_count)
    : probePlanLabel(scanSettings?.point_count, scanSettings?.spacing_deg);
  const scanDescription = scanLabel || 'Probe the planned positions around the hit';
  setText($('mode-probe-btn').querySelector('.mode-desc'), scanDescription);
  $('mode-probe-btn').title = scanBlock || scanDescription;
  $('mode-probe-btn').querySelector('.mode-text').textContent = state.inspection.phase === 'PROBING' ? (probe.cycle_active && probe.cycle_phase === 'RETRACTING' ? 'Retracting…' : 'Probing the ground…') : 'Probe the ground';
  $('mode-probe-pill').textContent = state.inspection.phase === 'PROBING' ? 'RUNNING' : scanBlock ? 'BLOCKED' : 'READY';
  renderInspection();
  const inspectionStatus = state.inspection.phase !== 'IDLE';
  const scanStatus = $('inspection-status');
  scanStatus.hidden = !inspectionStatus;
  if (inspectionStatus) {
    if (['ERROR', 'STOPPED'].includes(state.inspection.phase)) {
      scanStatus.textContent = 'Last inspection: ' + scanStatus.textContent;
    }
    $('operation-help').hidden = !otherActionBlock || (state.inspection.active && owner && servoSafe && !drive.calibrating);
  } else {
    $('operation-help').hidden = false;
  }
  $('inspection-panel').hidden = !inspectionStatus;
  const depthInput = $('probe-cycle-target');
  depthInput.max = Number.isFinite(probe.max_depth_mm) && probe.max_depth_mm > 0 ? probe.max_depth_mm : 0;
  depthInput.disabled = !Number.isFinite(probe.max_depth_mm) || probe.max_depth_mm <= 0 || Boolean(probe.cycle_active);
  $('mode-sweep-btn').disabled = Boolean(sweepBlock);
  $('mode-sweep-btn').title = sweepBlock;
  $('sweep-speed').disabled = Boolean(sweepBlock);
  $('mode-sweep-pill').textContent = detector.sweep_enabled ? 'SWEEPING' : sweepBlock ? 'BLOCKED' : 'READY';
  $('mode-sweep-btn').querySelector('.mode-text').textContent = detector.sweep_enabled ? 'Stop searching' : 'Search for metal';
  const pumpActive = Boolean(state.pump_active || drive.pump_active);
  if (pendingPumpMark && state.marking?.last_shot_request === pendingPumpMark.id) {
    pendingPumpMark = null;
    showNotice('Shot commanded. Check the ground mark visually.');
  } else if (pendingPumpMark && performance.now() - pendingPumpMark.at > 3000) {
    pendingPumpMark = null;
    showNotice('No shot acknowledgement. Check the mission log before retrying.');
  }
  $('mode-mark-btn').disabled = Boolean(markBlock) || Boolean(pendingPumpMark);
  $('mode-mark-btn').title = markBlock || 'Run the saved marking shot.';
  $('mode-mark-pill').textContent = pumpActive ? 'COMMAND ACTIVE' : pendingPumpMark ? 'WAITING' : markBlock ? 'BLOCKED' : `${state.marking.settings.duration_s} s`;
  window.updateMarkingSettings?.();
  $('clear-target-log').disabled = !state.connected || !owner || !targetLog.length;
  $('mark-target-btn').disabled = !owner || !state.connected || robot.pose_fresh !== true || ![robot.x, robot.y, robot.yaw].every(Number.isFinite);
  $('mark-target-btn').title = $('mark-target-btn').disabled ? 'Fresh localization is required to log a pose.' : 'Record the current vehicle position.';
  $('probe-slider').disabled = Boolean(probeActionBlock(true)) || detector.sweep_enabled;
  $('probe-stop').disabled = !owner;

  checkProbeCycleState();

  $('speed').textContent = `${Math.abs(robot.speed_mps).toFixed(2)} m/s`;

  const sensor = detector.sensor;
  const range = detectorVoltageRange(sensor);
  const freshSignal = state.connected && detector.fresh !== false && sensor?.fresh === true && range;
  const volts = freshSignal ? detectorVolts(sensor.packet?.filtered_adc, sensor.reference_voltage) : null;
  const detectorLive = Number.isFinite(volts);
  $('signal').textContent = detectorLive ? volts.toFixed(4) : '—';
  const scalePercent = value => Math.max(0, Math.min(100, (value - range.min) / (range.max - range.min) * 100));
  $('meter-fill').style.width = `${detectorLive ? scalePercent(volts) : 0}%`;
  $('meter-fill').style.background = detectorColor(detector.signal_ratio);
  const mineVolts = detectorMineVolts(sensor, detector.threshold_ratio);
  $('threshold').hidden = !range || mineVolts === null;
  if (!$('threshold').hidden) $('threshold').style.left = `${scalePercent(mineVolts)}%`;
  $('threshold').title = `Mine level ${voltageLabel(mineVolts)}`;
  $('detector-voltage').textContent = voltageLabel(volts);
  const measuredAngle = state.safety?.simulated || state.actuators?.arm?.connected || (state.arm_servo?.connected && state.arm_servo?.active_role === 'arm');
  const fixture = measuredAngle && Number.isFinite(detector.fixture_angle_deg) ? detector.fixture_angle_deg.toFixed(1) + '°' : '—';
  $('detector-detail').textContent = `Fixture ${fixture} · Search ${Math.round(detector.sweep_speed_deg_s)}°/s · Filtered signal`;
  $('detector-status').textContent = detectorLive ? 'LIVE' : 'NO DATA';
  $('detector-status').className = `badge ${detectorLive ? 'safe' : 'neutral'}`;

  $('sweep-status').textContent = detector.sweep_enabled ? 'Sweeping' : '';
  setRange($('sweep-speed'), detector.sweep_speed_min, detector.sweep_speed_max, 'any');
  if (!activeSliders.has('sweep-speed')) $('sweep-speed').value = detector.sweep_speed_deg_s;
  $('sweep-speed-value').textContent = `${Math.round(detector.sweep_speed_deg_s)}°/s`;
  $('sweep-speed').title = `Configured maximum: ${detector.sweep_speed_max.toFixed(1)}°/s. Change in Setup → Arm. Actual speed also depends on acceleration and load.`;

  $('probe-depth').textContent = Number.isFinite(probe.depth_mm) ? `${Math.round(probe.depth_mm)} mm` : '—';
  $('probe-target').textContent = `${Math.round(probe.target_mm)} mm`;
  const contactLoad = probe.contact_zeroed ? probe.contact_load_percent : probe.pressure_ratio == null ? null : probe.pressure_ratio * 100;
  $('probe-pressure').textContent = Number.isFinite(contactLoad) ? `${contactLoad.toFixed(1)}%` : '—';
  $('probe-current').textContent = Number.isFinite(probe.current_a) ? `${probe.current_a.toFixed(3)} A` : '—';
  $('probe-current-change-label').textContent = probe.contact_zeroed ? 'Current change' : 'Current magnitude';
  $('probe-contact-rate').textContent = probe.contact_fresh ? `${probe.contact_rate_hz.toFixed(1)} Hz` : '—';
  $('probe-current-change').textContent = Number.isFinite(probe.current_change_ma) ? `${probe.current_change_ma.toFixed(1)} mA` : '—';
  $('probe-contact-peak').textContent = Number.isFinite(probe.contact_peak_percent) ? `${probe.contact_peak_percent.toFixed(1)}% / ${probe.current_peak_ma.toFixed(1)} mA` : '—';
  $('probe-position-error').textContent = Number.isFinite(probe.position_error_ticks) ? `${(probe.position_error_ticks * 360 / 4096).toFixed(2)}°` : '—';
  $('probe-zero-contact').disabled = !owner || !servoSafe || !probe.contact_can_zero;
  $('probe-zero-status').textContent = probe.contact_zeroed ? 'Zeroed · the chart shows change from zero' : 'Lift the probe clear to zero the motor load';
  setRange($('probe-slider'), 0, probe.max_depth_mm || 1, .1);
  if (!activeSliders.has('probe-slider')) $('probe-slider').value = probe.target_mm;
  if (!state.connected) drawProbeMotion(probe.depth_mm);
  $('probe-status').textContent = probe.fault ? 'FAULT' : probe.ready !== true ? (state.actuators?.probe?.connected === false ? 'OFFLINE' : probe.max_depth_mm > 0 ? 'WAITING' : 'TRAVEL NEEDED') : probe.cycle_active ? (probe.cycle_phase === 'DESCENDING' ? 'PROBING' : 'RETRACTING') : probe.holding ? 'HOLDING' : probe.depth_mm > 2 ? 'DEPLOYED' : 'STOWED';
  $('probe-status').title = probe.reason || '';
  $('probe-status').className = `badge ${probe.fault || (probe.cycle_active && probe.contact_detected) ? 'active' : probe.cycle_active ? 'active' : probe.depth_mm > 2 ? 'safe' : 'neutral'}`;

  const driveBadge = !state.connected ? 'OFFLINE' : state.safety?.mode === 'override' ? (state.safety.servo_allowed ? 'RC CONTROL' : 'DRIVE LOCKED')
    : !owner ? 'Watching only' : !safe ? 'DRIVE LOCKED' : drive.calibrating ? 'CALIBRATING' : drive.publishing ? 'ROS CONTROL' : 'READY';
  $('arm-state').textContent = driveBadge;
  $('arm-state').className = `badge ${owner && drive.armed ? 'active' : 'neutral'}`;

  const heading = ((robot.yaw * 180 / Math.PI % 360) + 360) % 360;
  $('position').textContent = state.connected && robot.pose_fresh === true && [robot.x, robot.y, robot.yaw].every(Number.isFinite)
    ? `X ${robot.x.toFixed(1)} · Y ${robot.y.toFixed(1)} · heading ${heading.toFixed(0)}°`
    : 'Position unavailable · check localization.';

  updateInputHint();
}

function deadmanHint() {
  return 'The RC transmitter switch decides who can drive.';
}

function padDisplayName(pad) {
  return pad.id.replace(/\([^)]*\)/g, '').replace(/\s+/g, ' ').trim();
}

function mappingLabel(kind) {
  if (kind === 'xbox360') return 'Xbox 360 / Linux';
  if (kind === 'standard') return 'standard gamepad';
  return kind || 'unknown';
}

function setLitEl(el, on, extraClass) {
  if (!el) return;
  el.classList.toggle('lit', Boolean(on));
  if (extraClass) el.classList.toggle(extraClass, Boolean(on));
}

function paintTriggerEl(el, value) {
  if (!el) return;
  const active = value > 0.08;
  el.classList.toggle('lit', active);
  el.style.fill = active ? `rgba(42, 165, 134, ${0.35 + value * 0.65})` : '';
}

function gamepadApiBlocked() {
  return typeof navigator.getGamepads !== 'function'
    || (typeof window.isSecureContext === 'boolean' && !window.isSecureContext);
}

function rosJoyLive() {
  return Boolean(state.drive.joy?.seen);
}

function driveBlockMessage() {
  if (state.connected && state.safety?.mode === 'override') {
    if (!state.safety.servo_allowed) return {text: state.safety.servo_reason || 'Waiting for PX4 arming state.', kind: 'blocked'};
    return {text: state.drive.rc?.fresh ? 'Physical RC controls the vehicle.' : 'RC input stale', kind: state.drive.rc?.fresh ? 'ready' : 'blocked'};
  }
  if (!state.connected) return {text: 'Waiting for vehicle telemetry.', kind: 'blocked'};
  if (!state.drive.you_control_owner) return {text: 'You are watching. Press Start driving to drive.', kind: 'blocked'};
  if (!state.safety?.allowed) return {text: state.safety?.reason || 'Waiting for PX4 safety status', kind: 'blocked'};
  if (state.drive.calibrating) return {text: 'Actuator calibration is active. Finish calibration to drive.', kind: 'blocked'};
  const padReady = Boolean(gamepad) || rosJoyLive();
  if (preferences.input === 'controller' && gamepadApiBlocked() && !rosJoyLive()) {
    return {
      text: 'Browser Gamepad API blocked. Plug the Xbox into the SVEA for ROS /joy, or open http://localhost:8080 / HTTPS.',
      kind: 'blocked',
    };
  }
  if (preferences.input === 'keyboard' && !cameraFocused) {
    return {text: 'Click the forward camera for WASD.', kind: 'blocked'};
  }
  if (preferences.input === 'controller' && !padReady) {
    return {text: 'No gamepad found. Plug it into this laptop or the car, then press a button.', kind: 'blocked'};
  }
  if (preferences.input === 'rc' && !state.drive.rc?.fresh) return {text: 'Waiting for fresh RC steering and throttle.', kind: 'blocked'};
  if (!state.drive.publishing) return {text: state.inspection?.active ? 'Driving will stop the current scan.' : 'Ready to drive.', kind: 'ready'};
  return {
    text: `ROS cmd_vel ${state.drive.cmd_linear_x.toFixed(2)} m/s · ${state.drive.cmd_angular_z.toFixed(2)} rad/s`,
    kind: 'ready',
  };
}

function setText(el, text) {
  if (el && el.textContent !== text) el.textContent = text;
}

function updateInputHint() {
  const override = state.connected && state.safety?.mode === 'override';
  $('drive-help').textContent = override ? 'Transmitter has direct drive authority.' : preferences.input === 'rc'
    ? 'Keep this dashboard focused with the transmitter switch in ROS mode.' : preferences.input === 'keyboard'
    ? 'Click camera to drive · Esc releases keys · hold Space to sweep' : deadmanHint();
  const keyboard = preferences.input === 'keyboard' && state.safety?.mode !== 'override';
  $('pad-visual').classList.toggle('hidden', keyboard);
  padEls.raw.classList.toggle('hidden', keyboard);
  const lock = padEls.inputLock;
  if (lock) {
    lock.classList.toggle('hidden', !keyboard);
    lock.classList.toggle('locked', keyboard && cameraFocused);
    setText(lock, cameraFocused ? 'KEYBOARD CAPTURED · ESC TO RELEASE' : 'CLICK CAMERA FOR WASD');
  }

  const rcOverride = override;
  const rc = state.connected && state.drive.rc || {};
  const showRc = rcOverride || preferences.input === 'rc';
  if (showRc) {
    setText(padEls.controller, rcOverride ? 'Physical RC transmitter' : 'RC through ROS');
    setText(padEls.mapping, `Steering CH${rc.steering_channel ?? 1} · throttle CH${rc.throttle_channel ?? 2}`);
  } else if (rosJoyLive() && !keyboard && !gamepad) {
    setText(padEls.controller, 'Xbox via ROS /joy (SVEA USB)');
    setText(padEls.mapping, 'LS steer · LT/RT throttle');
  } else if (gamepadApiBlocked() && !keyboard) {
    setText(padEls.controller, 'Browser Gamepad API blocked');
    setText(padEls.mapping,
      'Open http://localhost:8080 on this computer, or plug the Xbox into the SVEA so ROS /joy can drive.');
  } else if (keyboard) {
    setText(padEls.controller, cameraFocused
      ? 'WASD active'
      : 'Click the forward camera to use WASD');
    setText(padEls.mapping, `WASD steer/throttle · forward ${preferences.keyboardMaxThrottle}% · reverse ${preferences.keyboardReverseThrottle}% · Space sweeps`);
  } else if (gamepad) {
    setText(padEls.controller, padDisplayName(gamepad));
    setText(padEls.mapping,
      `${mappingLabel(resolveMapping(gamepad))} · LS steer · LT/RT throttle`);
  } else {
    setText(padEls.controller, 'No controller connected');
    setText(padEls.mapping,
      'Plug in the Xbox pad and press any button.');
  }

  if (padEls.raw) {
    if (showRc) {
      setText(padEls.raw, rc.fresh ? `RC PWM: ${(rc.channels || []).join(' ')}` : 'RC input stale');
    } else if (gamepad) {
      let axes = '';
      for (let i = 0; i < gamepad.axes.length; i++) {
        if (i) axes += ' ';
        axes += Number(gamepad.axes[i]).toFixed(2);
      }
      let buttons = '';
      for (let i = 0; i < gamepad.buttons.length; i++) buttons += gamepad.buttons[i].pressed ? '1' : '0';
      setText(padEls.raw, `${gamepad.mapping || 'no-mapping'} · axes ${axes} · btns ${buttons}`);
    } else if (rosJoyLive()) {
      const joy = state.drive.joy;
      setText(padEls.raw,
        `ROS /joy · age ${joy.age_ms ?? '--'} ms · LT ${Number(joy.lt || 0).toFixed(2)} RT ${Number(joy.rt || 0).toFixed(2)}`);
    } else {
      let seen = 0;
      if (navigator.getGamepads) {
        const pads = navigator.getGamepads();
        for (let i = 0; i < pads.length; i++) if (pads[i]) seen += 1;
      }
      setText(padEls.raw, gamepadApiBlocked()
        ? 'Gamepad API unavailable in this origin. Waiting for ROS /joy…'
        : `Pads seen: ${seen}. Press a button.`);
    }
  }

  const visual = showRc ? {
    stickX: rc.fresh ? rc.steering : 0, stickY: 0,
    lt: rc.fresh ? Math.max(0, -rc.throttle) : 0, rt: rc.fresh ? Math.max(0, rc.throttle) : 0,
  } : (!gamepad && rosJoyLive()) ? {
    stickX: state.drive.joy.stick_x || 0,
    stickY: state.drive.joy.stick_y || 0,
    lt: state.drive.joy.lt || 0,
    rt: state.drive.joy.rt || 0,
    lb: Boolean(state.drive.joy.lb),
    rb: Boolean(state.drive.joy.rb),
    a: Boolean(state.drive.joy.a),
    b: Boolean(state.drive.joy.b),
    x: Boolean(state.drive.joy.x),
    y: Boolean(state.drive.joy.y),
  } : lastInput;

  if (padEls.ls) {
    padEls.ls.setAttribute('transform', `translate(${visual.stickX * 14}, ${visual.stickY * 14})`);
    padEls.ls.classList.toggle('lit', Math.hypot(visual.stickX, visual.stickY) > 0.12);
  }
  paintTriggerEl(padEls.lt, visual.lt);
  paintTriggerEl(padEls.rt, visual.rt);
  setLitEl(padEls.lb, visual.lb, 'deadman');
  setLitEl(padEls.rb, visual.rb, 'deadman');
  setLitEl(padEls.a, visual.a);
  setLitEl(padEls.b, visual.b);
  setLitEl(padEls.x, visual.x);
  setLitEl(padEls.y, visual.y);

  const block = driveBlockMessage();
  if (padEls.driveBlock) {
    setText(padEls.driveBlock, block.text);
    if (padEls.driveBlock.className !== block.kind) padEls.driveBlock.className = block.kind;
  }
}

/* ---------------------------------------------------------------- Input --- */

function buttonValue(button) {
  return button?.value ?? 0;
}

function deadzone(value) {
  return Math.abs(value) < 0.12 ? 0 : value;
}

function finite(value) {
  return Number.isFinite(value) ? value : 0;
}

function looksLikeXbox(id) {
  return /xbox|x-box|xinput/i.test(id);
}

function resolveMapping(pad) {
  const override = preferences.mapping;
  if (override === 'standard' || override === 'xbox360') return override;
  if (pad.mapping === 'standard') return 'standard';
  if (looksLikeXbox(pad.id)) return 'xbox360';
  return 'standard';
}

function syncTriggerState(pad) {
  if (!pad || pad.id !== lastPadId) {
    triggerSeen.lt = false;
    triggerSeen.rt = false;
    lastPadId = pad?.id ?? null;
  }
}

// Linux xpad often reports LT/RT as -1..1, but the axis stays at 0 until the
// first press. Treat that unused 0 as released; after motion, map to 0..1.
function axisTrigger(key, value) {
  if (value == null || Number.isNaN(value)) return 0;
  if (!triggerSeen[key]) {
    if (value === 0) return 0;
    triggerSeen[key] = true;
  }
  return Math.max(0, Math.min(1, (value + 1) / 2));
}

function readTriggers(pad, kind, target) {
  if (kind === 'standard') {
    // Standard axes 2/3 are the right stick, never trigger fallbacks.
    target.lt = buttonValue(pad.buttons[6]);
    target.rt = buttonValue(pad.buttons[7]);
    return;
  }
  // Raw Xbox buttons 6/7 are Back/Start, not LT/RT.
  target.lt = axisTrigger('lt', pad.axes[2]);
  target.rt = axisTrigger('rt', pad.axes[5]);
}

function zeroInput(target) {
  target.linear = 0;
  target.angular = 0;
  target.deadman = false;
  target.steer = 0;
  target.throttle = 0;
  target.mapping = '';
  target.stickX = 0;
  target.stickY = 0;
  target.lt = 0;
  target.rt = 0;
  target.lb = false;
  target.rb = false;
  target.a = false;
  target.b = false;
  target.x = false;
  target.y = false;
}

function readGamepad(target) {
  const pad = gamepad;
  const kind = resolveMapping(pad);
  syncTriggerState(pad);
  const stickX = pad.axes[0] ?? 0;
  const stickY = pad.axes[1] ?? 0;
  const steering = deadzone(stickX);
  readTriggers(pad, kind, target);
  const throttle = target.rt - target.lt;
  const lb = Boolean(pad.buttons[4]?.pressed);
  const rb = Boolean(pad.buttons[5]?.pressed);
  target.linear = throttle * MAX_LINEAR_MPS;
  // Browser axes are positive right; ROS yaw is positive left.
  target.angular = -steering * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100;
  target.deadman = Math.abs(throttle) > 0 || steering !== 0;
  target.steer = steering;
  target.throttle = throttle;
  target.mapping = kind;
  target.stickX = stickX;
  target.stickY = stickY;
  target.lb = lb;
  target.rb = rb;
  target.a = Boolean(pad.buttons[0]?.pressed);
  target.b = Boolean(pad.buttons[1]?.pressed);
  target.x = Boolean(pad.buttons[2]?.pressed);
  target.y = Boolean(pad.buttons[3]?.pressed);
}

function keyboardVelocity(throttle) {
  const limit = throttle < 0 ? preferences.keyboardReverseThrottle : preferences.keyboardMaxThrottle;
  return throttle * (state.drive.max_velocity_mps ?? MAX_LINEAR_MPS) * limit / 100;
}

function readKeyboard(target = {}) {
  const throttle = (keys.has('KeyW') ? 1 : 0) - (keys.has('KeyS') ? 1 : 0);
  const steer = (keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0);
  const deadman = throttle !== 0 || steer !== 0;
  target.linear = keyboardVelocity(throttle);
  target.angular = -steer * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100;
  target.deadman = deadman;
  target.steer = steer;
  target.throttle = throttle;
  target.mapping = 'keyboard';
  target.stickX = steer;
  target.stickY = -throttle;
  target.lt = keys.has('KeyS') ? 1 : 0;
  target.rt = keys.has('KeyW') ? 1 : 0;
  target.lb = false;
  target.rb = false;
  target.a = false;
  target.b = false;
  target.x = false;
  target.y = false;
  return target;
}

function padActivity(pad) {
  let sum = 0;
  const buttons = pad.buttons;
  for (let i = 0; i < buttons.length; i++) {
    const item = buttons[i];
    sum += item.pressed ? 1 : (item.value || 0);
  }
  const axes = pad.axes;
  for (let i = 0; i < axes.length; i++) {
    const value = Math.abs(axes[i]);
    if (value > 0.2) sum += value;
  }
  return sum;
}

function pickGamepad() {
  const pads = visiblePads();
  if (!pads) return null;
  let best = null;
  let bestScore = -1;
  let previous = null;
  let xbox = null;
  for (let i = 0; i < pads.length; i++) {
    const pad = pads[i];
    if (!pad) continue;
    const score = padActivity(pad);
    if (score > bestScore) {
      best = pad;
      bestScore = score;
    }
    if (lastPadId && pad.id === lastPadId) previous = pad;
    if (!xbox && looksLikeXbox(pad.id)) xbox = pad;
  }
  if (best && bestScore > 0.2) return best;
  return previous || xbox || best;
}

function sendDriveIfNeeded(target, now) {
  if (!state.drive.you_control_owner) return;
  const linear = Math.round(finite(target.linear) * 1e4) / 1e4;
  const angular = Math.round(finite(target.angular) * 1e4) / 1e4;
  const deadman = Boolean(target.deadman);
  const changed =
    Math.abs(linear - lastSentDrive.linear) > DRIVE_EPSILON
    || Math.abs(angular - lastSentDrive.angular) > DRIVE_EPSILON
    || deadman !== lastSentDrive.deadman;
  const idle = !deadman && Math.abs(linear) <= DRIVE_EPSILON && Math.abs(angular) <= DRIVE_EPSILON;
  const since = now - lastDriveSent;
  if (changed) {
    if (since < DRIVE_MIN_INTERVAL_MS) return;
  } else if (idle || since < DRIVE_KEEPALIVE_MS) {
    return;
  }
  lastDriveSent = now;
  lastSentDrive.linear = linear;
  lastSentDrive.angular = angular;
  lastSentDrive.deadman = deadman;
  sendDrive(linear, angular, deadman);
}

function inputLoop() {
  requestAnimationFrame(inputLoop);
  const now = performance.now();
  const pads = visiblePads();
  gamepad = pads.find(pad => pad.index === selectedPadIndex && pad.id === selectedPadId) ?? (preferences.input === 'controller' ? pickGamepad() : null);

  if (cameraFocused && !keyboardCaptured()) releaseKeyboardCapture();
  if (keyboardCaptured()) {
    readKeyboard(lastInput);
    rampKeyboard(lastInput, now);
  } else if (preferences.input === 'controller' && gamepad) {
    const stamp = gamepad.timestamp || 0;
    if (!stamp || stamp !== lastPadTimestamp || gamepad.id !== lastPadId) {
      lastPadTimestamp = stamp;
      readGamepad(lastInput);
    }
  } else if (preferences.input === 'wheel' && gamepad) {
    zeroInput(lastInput);
    Object.assign(lastInput, readWheel(gamepad, preferences.wheel), {mapping: 'wheel'});
  } else {
    zeroInput(lastInput);
    syncTriggerState(null);
    prevA = false;
    lastPadTimestamp = -1;
  }

  renderInputs(pads, lastInput);
  const canDrive = state.connected && state.drive.armed && state.safety?.allowed && state.drive.you_control_owner
    && !state.drive.calibrating && !document.hidden && document.hasFocus();
  if (!canDrive) resetKeyboardRamp(now);
  if (preferences.input === 'rc') {
    const enabled = canDrive && !drivingPending && state.drive.input === 'rc' && state.drive.rc?.fresh;
    if (enabled && now - lastRcEnable >= DRIVE_KEEPALIVE_MS) {
      send({type: 'rc_drive_enable', enabled: true});
      lastRcEnable = now;
    } else if (!enabled) stopRcInput();
  }
  if (preferences.input !== 'rc' && !drivingPending && !state.drive.calibrating) sendDriveIfNeeded(canDrive && lastInput.deadman ? lastInput : {linear: 0, angular: 0, deadman: false}, now);

  if (now - lastPadUi >= PAD_UI_PERIOD_MS) {
    lastPadUi = now;
    updateInputHint();
  }

  if (noticeExpiry && now > noticeExpiry) {
    noticeExpiry = 0;
    $('notice').classList.add('hidden');
  }
}

function stopLocalInput() {
  cameraFocused = false;
  clearHeldInput();
}

function clearHeldInput() {
  stopKeyboardSweep();
  keys.clear();
  resetKeyboardRamp();
  zeroInput(lastInput);
  lastPadTimestamp = -1;
  activeSliders.clear();
}

$('take-control').onclick = () => send({type: 'take_control'});
$('stop-controls').onclick = () => { stopInput(); send({type: 'estop'}); pendingPumpMark = null; showNotice('Stop requested. Press Start driving again to resume.'); };
$('probe-zero-contact').onclick = () => send({type: 'probe_zero_contact'});
$('probe-stop').onclick = () => send({type: 'arm_servo', action: 'stop', role: 'probe'});

for (const [id, type, field] of [['probe-slider', 'probe_target', 'depth_mm'], ['sweep-speed', 'sweep_speed', 'deg_s']]) {
  const commit = event => {
    if (state.connected && state.drive.you_control_owner) send({type, [field]: Number(event.target.value)});
  };
  $(id).addEventListener('pointerdown', () => activeSliders.add(id));
  $(id).addEventListener('keydown', () => activeSliders.add(id));
  $(id).addEventListener('blur', () => activeSliders.delete(id));
  $(id).addEventListener('pointercancel', () => activeSliders.delete(id));
  $(id).oninput = throttle(70, commit);
  // Always deliver the released value, even inside the throttle window.
  $(id).onchange = event => { activeSliders.delete(id); commit(event); };
}
if ($('detector-align-probe')) $('detector-align-probe').onclick = alignProbeToPeak;
if ($('start-probing')) $('start-probing').onclick = startProbeCycle;
if ($('mode-probe-btn')) $('mode-probe-btn').onclick = () => startInspection('probe_scan');
if ($('mode-sweep-btn')) $('mode-sweep-btn').onclick = () => send({type: 'sweep_enabled', enabled: !state.detector.sweep_enabled});
if ($('mode-align-btn')) $('mode-align-btn').onclick = () => startInspection('confirm');
$('inspection-stop').onclick = () => send({type: 'inspection', action: 'stop'});
if ($('mode-mark-btn')) $('mode-mark-btn').onclick = triggerPumpMark;
if ($('mark-target-btn')) $('mark-target-btn').onclick = markCurrentPose;
if ($('export-target-log')) $('export-target-log').onclick = exportTargetLogCsv;
if ($('clear-target-log')) $('clear-target-log').onclick = () => { if (state.drive.you_control_owner && targetLog.length && confirm('Clear the mission log on the car for every viewer?')) send({type:'clear_log'}); };

let displaySignature = '';
function applyDisplaySettings(value) {
  const owner = state.connected && state.drive.you_control_owner && Boolean(value);
  const key = JSON.stringify([value, owner]);
  if (key === displaySignature) return;
  displaySignature = key;
  if (value) Object.assign(preferences, structuredClone(value));
  savePreferences();
  for (const input of document.querySelectorAll('#wheel-mapping input')) {
    const v = preferences.wheel[input.dataset.field];
    if (input.type === 'checkbox') input.checked = v;
    else input.value = v;
    input.disabled = !owner;
  }
  for (const id of ['search-alert-enabled','search-alert-level','search-alert-sound','steering-sensitivity','pad-mapping']) {
    $(id).disabled = !owner || (id === 'pad-mapping' && preferences.input !== 'controller') ||
      (['search-alert-level','search-alert-sound'].includes(id) && !preferences.searchAlert.enabled);
  }
}

function storeDisplaySettings() {
  if (!state.connected || !state.drive.you_control_owner) return;
  const {mapping, sensitivity, wheel, searchAlert} = preferences;
  send({type:'settings', section:'display', value:{mapping, sensitivity, wheel, searchAlert}});
}

function savePreferences() {

  $('drive-control-source').value = preferences.input;
  $('keyboard-max-throttle').value = preferences.keyboardMaxThrottle;
  $('keyboard-reverse-throttle').value = preferences.keyboardReverseThrottle;
  $('pad-mapping').value = preferences.mapping;
  $('steering-sensitivity').value = preferences.sensitivity;
  $('sensitivity-value').textContent = `${preferences.sensitivity}%`;
  $('pad-mapping').disabled = !state.drive.you_control_owner || preferences.input !== 'controller';
  const alertSettings = preferences.searchAlert;
  $('search-alert-enabled').checked = alertSettings.enabled;
  $('search-alert-sound').checked = alertSettings.sound;
  $('search-alert-level').disabled = !alertSettings.enabled;
  $('search-alert-sound').disabled = !alertSettings.enabled;
  renderSearchAlertLevel();
  updateInputHint();
}

$('search-alert-level').onchange = event => {
  const field = event.target;
  if (!field.reportValidity()) return;
  preferences.searchAlert.volts = field.valueAsNumber;
  delete preferences.searchAlert.level;
  renderSearchAlertLevel();
  storeDisplaySettings();
};
$('detector-alert-input').onchange = event => {
  const quick = event.target, field = $('search-alert-level');
  if (!quick.reportValidity() || field.disabled) return;
  field.value = quick.value;
  field.onchange({target: field});
};
$('search-alert-level').onblur = () => {
  const volts = searchAlertVolts();
  $('search-alert-level').value = Number.isFinite(volts) ? Number(volts.toFixed(6)) : '';
};
$('search-alert-enabled').onchange = event => {
  preferences.searchAlert.enabled = event.target.checked;
  storeDisplaySettings();
  savePreferences();
};
$('search-alert-sound').onchange = event => {
  preferences.searchAlert.sound = event.target.checked;
  storeDisplaySettings();
  savePreferences();
  if (event.target.checked) playSearchAlertSound(); // let the operator hear it
};
$('search-alert-dismiss').onclick = event => {
  event.stopPropagation();
  searchAlertDismissedAt = performance.now();
  $('search-alert').hidden = true;
};

$('settings').onclick = () => {
  if (preferences.input === 'keyboard') releaseKeyboardCapture();
  $('settings-dialog').showModal();
};
$('offline-settings').onclick = $('settings').onclick;
function updateDrivingSettings(value) {
  if (!value) return;
  const changed = preferences.input !== value.input || preferences.keyboardMaxThrottle !== value.forward_percent
    || preferences.keyboardReverseThrottle !== value.reverse_percent;
  if (changed) {
    stopLocalInput();
    lastPadTimestamp = -1;
    preferences.input = value.input;
    preferences.keyboardMaxThrottle = value.forward_percent;
    preferences.keyboardReverseThrottle = value.reverse_percent;
    $('pad-mapping').disabled = !state.drive.you_control_owner || preferences.input !== 'controller';
  }
  if (drivingPending && Object.keys(drivingPending).every(key => value[key] === drivingPending[key])) {
    clearTimeout(drivingSaveTimer);
    drivingPending = null;
    $('driving-feedback').textContent = 'Saved on car';
  }
  if (changed || !drivingPending) {
    $('drive-control-source').value = value.input;
    if (document.activeElement !== $('keyboard-max-throttle')) $('keyboard-max-throttle').value = value.forward_percent;
    if (document.activeElement !== $('keyboard-reverse-throttle')) $('keyboard-reverse-throttle').value = value.reverse_percent;
  }
}

function failDriveSave(message) {
  clearTimeout(drivingSaveTimer);
  drivingPending = null;
  $('driving-feedback').textContent = message;
  showNotice(message, true);
  updateDrivingSettings(state.settings?.driving);
  updateHud();
}

function changeDriveSetting(key, value) {
  if (!state.connected || !state.drive.you_control_owner || !state.settings?.driving || drivingPending) return;
  stopInput();
  drivingPending = {...state.settings.driving, [key]: value};
  drivingSaveTimer = setTimeout(() => failDriveSave('Drive settings were not confirmed by the car.'), 4500);
  send({type: 'settings', section: 'driving', value: drivingPending});
  $('driving-feedback').textContent = 'Saving on car…';
  updateHud();
}

$('drive-control-source').onchange = event => changeDriveSetting('input', event.target.value);
for (const [id, key] of [['keyboard-max-throttle', 'forward_percent'], ['keyboard-reverse-throttle', 'reverse_percent']]) {
  $(id).onchange = event => {
    if (event.target.reportValidity()) changeDriveSetting(key, event.target.valueAsNumber);
  };
}
$('steering-sensitivity').oninput = event => {
  preferences.sensitivity = Number(event.target.value);
  lastPadTimestamp = -1;
  savePreferences();
};
$('steering-sensitivity').onchange = storeDisplaySettings;
$('pad-mapping').onchange = event => {
  stopInput();
  preferences.mapping = event.target.value;
  storeDisplaySettings();
  triggerSeen.lt = false;
  triggerSeen.rt = false;
  savePreferences();
};

function keyboardCaptured() {
  return preferences.input === 'keyboard' && cameraFocused && !$('settings-dialog').open
    && !document.hidden && document.hasFocus()
    && cameraPanel.contains(document.activeElement);
}

function stopKeyboardSweep() {
  if (!keyboardSweep) return;
  keyboardSweep = false;
  if (state.connected && state.drive.you_control_owner) send({type: 'arm_servo', action: 'stop', role: 'arm'});
}

function releaseKeyboardCapture() {
  const captured = cameraFocused;
  stopLocalInput();
  if (captured && state.connected && state.drive.you_control_owner) sendDrive(0, 0, false);
  updateInputHint();
}

function activateKeyboardInput() {
  if (preferences.input !== 'keyboard' || $('settings-dialog').open || cameraFocused) return;
  stopLocalInput();
  cameraFocused = true;
  updateInputHint();
}

cameraPanel.addEventListener('pointerdown', event => {
  if (preferences.input !== 'keyboard') return;
  event.preventDefault();
  cameraPanel.focus({preventScroll: true});
  activateKeyboardInput();
});
cameraPanel.addEventListener('focusin', activateKeyboardInput);
cameraPanel.addEventListener('focusout', event => {
  if (!cameraPanel.contains(event.relatedTarget)) releaseKeyboardCapture();
});

document.addEventListener('pointerdown', event => {
  if (cameraFocused && !cameraPanel.contains(event.target)) releaseKeyboardCapture();
}, true);

document.addEventListener('keydown', event => {
  if (!keyboardCaptured()) return;
  if (event.code === 'Space') event.preventDefault();
  if (event.ctrlKey || event.metaKey || event.altKey) return;
  if (event.code === 'Escape') {
    event.preventDefault();
    releaseKeyboardCapture();
    document.activeElement.blur();
    return;
  }
  if (!['KeyW', 'KeyA', 'KeyS', 'KeyD', 'Space'].includes(event.code)) return;
  event.preventDefault();
  if (event.repeat || keys.has(event.code)) return;
  keys.add(event.code);
  if (event.code !== 'Space') return;
  const blocked = operationBlock() || armSetupBlock();
  if (blocked) { showNotice(blocked); return; }
  if (state.detector.sweep_enabled) return;
  keyboardSweep = true;
  send({type: 'sweep_enabled', enabled: true});
});

document.addEventListener('keyup', event => {
  if (!['KeyW', 'KeyA', 'KeyS', 'KeyD', 'Space'].includes(event.code)) return;
  if (keyboardCaptured()) event.preventDefault();
  keys.delete(event.code);
  if (event.code === 'Space') stopKeyboardSweep();
});

function resetKeyboardRamp(now = performance.now()) {
  keyboardRamp.throttle = keyboardRamp.steer = 0;
  keyboardRamp.updatedAt = now;
}

function rampKeyboard(target, now) {
  const dt = Math.max(0, Math.min(.05, (now - keyboardRamp.updatedAt) / 1000));
  keyboardRamp.updatedAt = now;
  if (!target.deadman) {
    resetKeyboardRamp(now);
    target.linear = target.angular = 0;
    return;
  }
  const approach = (value, desired, rise, fall) => {
    const slowing = desired === 0 || (value !== 0 && Math.sign(value) !== Math.sign(desired));
    const step = (slowing ? fall : rise) * dt;
    return value + Math.max(-step, Math.min(step, desired - value));
  };
  keyboardRamp.throttle = approach(keyboardRamp.throttle, target.throttle, 1.6, 2.8);
  keyboardRamp.steer = approach(keyboardRamp.steer, target.steer, 3.2, 4.8);
  target.linear = keyboardVelocity(keyboardRamp.throttle);
  target.angular = -keyboardRamp.steer * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100;
}

window.addEventListener('gamepadconnected', () => {
  showNotice('Controller found. RC authority is required to drive.');
});

/* --------------------------------------------------------------- Canvas --- */

function resize(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const width = Math.floor(canvas.clientWidth * ratio);
  const height = Math.floor(canvas.clientHeight * ratio);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return context;
}

// World to rover body frame: forward is along the rover's heading, side is to
// its right, which is the direction screen x grows in project().
function worldToCamera(x, y, pose = state.robot) {
  const dx = x - pose.x;
  const dy = y - pose.y;
  const yaw = pose.yaw;
  return {
    forward: Math.cos(yaw) * dx + Math.sin(yaw) * dy,
    side: Math.sin(yaw) * dx - Math.cos(yaw) * dy,
  };
}

function project(point, width, height) {
  if (point.forward < 0.35) return null;
  const horizon = height * 0.31;
  const scale = height * 0.72;
  return {
    x: width / 2 + point.side / point.forward * scale,
    y: horizon + scale / point.forward,
  };
}

function drawSegment(context, from, to) {
  if (!from || !to) return;
  context.beginPath();
  context.moveTo(from.x, from.y);
  context.lineTo(to.x, to.y);
  context.stroke();
}

function drawForwardView(pose = state.robot) {
  if (forward.classList.contains('hidden')) return;
  const context = resize(forward);
  const width = forward.clientWidth;
  const height = forward.clientHeight;
  const horizon = height * 0.31;
  const {x: roverX} = pose;

  context.clearRect(0, 0, width, height);
  context.fillStyle = '#4f8995';
  context.fillRect(0, 0, width, horizon);
  context.fillStyle = '#31492c';
  context.fillRect(0, horizon, width, height - horizon);

  // Cross-lane rungs every metre ahead of the rover along +X.
  context.strokeStyle = '#7b8e64';
  context.lineWidth = 1;
  for (let distance = 1; distance < CAMERA_RANGE_M; distance += 1) {
    const left = project(worldToCamera(roverX + distance, 5, pose), width, height);
    const right = project(worldToCamera(roverX + distance, -5, pose), width, height);
    drawSegment(context, left, right);
  }

  // Lane edges at constant Y, running along +X.
  context.strokeStyle = '#e2d29a';
  context.lineWidth = 3;
  for (const laneY of [-LANE_HALF_WIDTH_M, LANE_HALF_WIDTH_M]) {
    const start = project(worldToCamera(roverX + 0.4, laneY, pose), width, height);
    const end = project(worldToCamera(roverX + 28, laneY, pose), width, height);
    drawSegment(context, start, end);
  }

  for (const detection of state.detections) {
    const point = project(worldToCamera(detection.x, detection.y, pose), width, height);
    if (!point) continue;
    context.strokeStyle = '#d75b4e';
    context.lineWidth = 2;
    context.beginPath();
    context.arc(point.x, point.y - 8, 7, 0, Math.PI * 2);
    context.stroke();
    context.fillStyle = '#fff4e6';
    context.font = '700 11px system-ui';
    context.fillText('MARKED', point.x + 10, point.y - 6);
  }

  for (const t of targetLog) {
    if (!Number.isFinite(t.x) || !Number.isFinite(t.y)) continue;
    const point = project(worldToCamera(t.x, t.y, pose), width, height);
    if (!point) continue;
    context.fillStyle = t.statusType === 'contact' ? '#df6251' : t.statusType === 'clear' ? '#22885a' : '#e5c07b';
    context.beginPath();
    context.arc(point.x, point.y - 8, 7, 0, Math.PI * 2);
    context.fill();
    context.strokeStyle = '#fff';
    context.lineWidth = 1.5;
    context.stroke();
    context.fillStyle = '#fff4e6';
    context.font = '700 11px system-ui';
    context.fillText(`${t.id} ${t.status}`, point.x + 10, point.y - 6);
  }

  // Rover nose in the near field.
  context.fillStyle = '#18282a';
  context.beginPath();
  context.moveTo(width * 0.32, height);
  context.lineTo(width * 0.68, height);
  context.lineTo(width * 0.59, height * 0.84);
  context.lineTo(width * 0.41, height * 0.84);
  context.closePath();
  context.fill();

}

function drawMap(pose = state.robot) {
  const context = resize(map);
  const width = map.clientWidth;
  const height = map.clientHeight;
  const crossHalf = 6;
  const scale = Math.min(width / (LANE_END_M - LANE_START_M), height / (2 * crossHalf));
  // +X runs left to right, +Y runs up the screen.
  const centreX = (LANE_START_M + LANE_END_M) / 2;
  const toScreen = (x, y) => ({x: width / 2 + (x - centreX) * scale, y: height / 2 - y * scale});
  const leftX = Math.floor(centreX - width / (2 * scale));
  const rightX = Math.ceil(centreX + width / (2 * scale));

  context.clearRect(0, 0, width, height);
  context.fillStyle = '#101f20';
  context.fillRect(0, 0, width, height);

  context.strokeStyle = '#244244';
  context.lineWidth = 1;
  for (let x = leftX; x <= rightX; x += 1) {
    drawSegment(context, toScreen(x, -crossHalf), toScreen(x, crossHalf));
  }
  for (let y = -crossHalf; y <= crossHalf; y += 1) {
    drawSegment(context, toScreen(leftX, y), toScreen(rightX, y));
  }

  const laneTopLeft = toScreen(LANE_START_M, LANE_HALF_WIDTH_M);
  const laneBottomRight = toScreen(LANE_END_M, -LANE_HALF_WIDTH_M);
  const laneWidth = laneBottomRight.x - laneTopLeft.x;
  const laneHeight = laneBottomRight.y - laneTopLeft.y;
  context.fillStyle = '#1e4d43';
  context.fillRect(laneTopLeft.x, laneTopLeft.y, laneWidth, laneHeight);
  context.strokeStyle = '#d4ce8a';
  context.lineWidth = 2;
  context.strokeRect(laneTopLeft.x, laneTopLeft.y, laneWidth, laneHeight);

  // Search alerts: hollow amber diamonds where the operator was warned.
  context.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--warn').trim() || '#f0b44c';
  context.lineWidth = 1.5;


  // Detections mark where the rover stood when the signal crossed threshold.
  for (const detection of state.detections) {
    const point = toScreen(detection.x, detection.y);
    context.strokeStyle = '#df6251';
    context.lineWidth = 2;
    context.beginPath();
    context.arc(point.x, point.y, 5, 0, Math.PI * 2);
    context.stroke();
  }

  // Verified & marked targets from targetLog
  for (const t of targetLog) {
    if (!Number.isFinite(t.x) || !Number.isFinite(t.y)) continue;
    const point = toScreen(t.x, t.y);
    context.fillStyle = t.statusType === 'contact' ? '#df6251' : t.statusType === 'clear' ? '#22885a' : '#e5c07b';
    context.beginPath();
    context.arc(point.x, point.y, 6, 0, Math.PI * 2);
    context.fill();
    context.strokeStyle = '#fff';
    context.lineWidth = 1.5;
    context.stroke();
    context.fillStyle = '#fff';
    context.font = 'bold 9px system-ui';
    context.fillText(t.id, point.x + 8, point.y + 3);
  }

  if (!state.connected || state.robot.pose_fresh !== true || ![pose.x, pose.y, pose.yaw].every(Number.isFinite)) return;
  const rover = toScreen(pose.x, pose.y);
  context.save();
  context.translate(rover.x, rover.y);
  // Screen y is flipped relative to world y, so a counter-clockwise world yaw
  // is a clockwise canvas rotation.
  context.rotate(-pose.yaw);
  context.fillStyle = '#dff3df';
  context.beginPath();
  context.moveTo(10, 0);
  context.lineTo(-8, 7);
  context.lineTo(-8, -7);
  context.closePath();
  context.fill();
  context.restore();

}

let probePressureChart = null;
const contactAxes = {load: {max: 10, changed: 0}, current: {max: 50, changed: 0}};
function contactScale(axis, peak, minimum) {
  const now = performance.now();
  const desired = Math.max(minimum, Math.ceil(peak * 1.2 / minimum) * minimum);
  if (desired > axis.max || (desired < axis.max / 2 && now - axis.changed > 5000)) {
    axis.max = desired > axis.max ? desired : Math.max(desired, axis.max / 2);
    axis.changed = now;
  }
  return axis.max;
}
// Reuse Chart.js layout between telemetry updates. Only the plotted traces
// scroll every display frame; axes/legends stay fixed. Clip before translating
// so old samples never paint over labels. Stagger the two dashboard layouts.
Chart.register({id: 'operatorScroll', beforeDatasetsDraw(chart) {
  const shift = chart.$scrollPixels || 0;
  if (!shift) return;
  const {left, top, right, bottom} = chart.chartArea;
  chart.ctx.save();
  chart.ctx.beginPath(); chart.ctx.rect(left, top, right-left, bottom-top); chart.ctx.clip();
  chart.ctx.translate(-shift, 0);
}, afterDatasetsDraw(chart) { if (chart.$scrollPixels) chart.ctx.restore(); }});
function scrollTimeChart(chart, now, phase = 0) {
  const bucket = Math.floor((now + phase) / 50);
  if (chart.$frameBucket !== bucket) {
    chart.$frameBucket = bucket;
    chart.$renderedAt = now;
    chart.$scrollPixels = 0;
    return false;
  }
  const axis = chart.scales.x;
  chart.$scrollPixels = axis ? (now-chart.$renderedAt)/1000 * axis.width/(axis.max-axis.min) : 0;
  chart.draw();
  return true;
}

let pressureSource = null, pressureReceivedAt = 0;
let manualProbeChart = null;

function drawManualProbeHistory(frameTime = performance.now(), animate = false) {
  const canvas = $('manual-probe-history');
  if (!canvas.getClientRects().length) return;
  if (animate && manualProbeChart && scrollTimeChart(manualProbeChart, frameTime, 10)) return;
  const signal = window.operatorProbeVoltageHistory?.() ?? {data:[], limit:null, status:'Contact settings unavailable.'};
  const probe = state.probe;
  // Both car histories share Unix timestamps. Older snapshots give an age at
  // receipt, which still lets each measured encoder sample age off the plot.
  const anchor = Number.isFinite(probe.history_time_s) && Number.isFinite(signal.time)
    ? probe.history_time_s-signal.time : -(frameTime-pressureReceivedAt)/1000;
  const depths = [];
  if (state.connected && Array.isArray(probe.contact_samples)) {
    for (const sample of probe.contact_samples) {
      if (!Number.isFinite(sample.x)) continue;
      const x = anchor + sample.x;
      if (x < -30 || x > 0) continue;
      if (depths.length && x-depths.at(-1).x > .3) depths.push({x:x-.001,y:null});
      depths.push({x, y:Number.isFinite(sample.depth_mm) ? sample.depth_mm : null});
    }
  }
  if (!manualProbeChart) {
    const theme = getComputedStyle(document.documentElement);
    const color = token => theme.getPropertyValue(token).trim();
    manualProbeChart = new Chart(canvas, {
      type:'line',
      data:{datasets:[
        {label:'Filtered contact', yAxisID:'volts', borderColor:color('--accent'), borderWidth:2, pointRadius:0, spanGaps:false, data:[]},
        {label:'Saved limit', yAxisID:'volts', borderColor:color('--warn'), borderWidth:1, borderDash:[5,4], pointRadius:0, data:[]},
        {label:'Encoder depth', yAxisID:'depth', borderColor:color('--text'), borderWidth:1.5, pointRadius:0, spanGaps:false, data:[]},
      ]},
      options:{responsive:true,maintainAspectRatio:false,parsing:false,animation:false,
        plugins:{decimation:{enabled:true,algorithm:'min-max'},legend:{labels:{color:color('--muted'),boxWidth:10,font:{size:10}}},
          tooltip:{callbacks:{label:({dataset,parsed})=>dataset.label + ': ' +
            (dataset.yAxisID === 'depth' ? parsed.y.toFixed(1) + ' mm' : parsed.y.toFixed(4) + ' V')}}},
        scales:{
          x:{type:'linear',min:-30,max:0,ticks:{maxTicksLimit:4,callback:v=>v===0?'now':v+'s'}},
          volts:{type:'linear',position:'left',title:{display:true,text:'Contact (V)'},ticks:{maxTicksLimit:4}},
          depth:{type:'linear',position:'right',title:{display:true,text:'Depth (mm)'},beginAtZero:true,ticks:{maxTicksLimit:4},grid:{drawOnChartArea:false}},
        }},
    });
  }
  const [volts, limit, depth] = manualProbeChart.data.datasets;
  volts.label = signal.name ? `Filtered ${signal.name}` : 'Filtered contact';
  setSampleTrace(volts, signal.data, signal.gap ?? .5);
  limit.label = signal.enabled === false ? 'Saved limit (off)' : 'Saved limit';
  limit.data = Number.isFinite(signal.limit) ? [{x:-30,y:signal.limit},{x:0,y:signal.limit}] : [];
  // A gap or null sample breaks the line; no target or interpolated depth is plotted.
  setSampleTrace(depth, depths, .3);
  const axis = manualProbeChart.options.scales.volts;
  axis.min = signal.bounds?.min;
  axis.max = signal.bounds?.max;
  const last = depths.at(-1);
  const depthStatus = last && last.x >= -.5 && Number.isFinite(last.y) ? '' : 'Depth unavailable.';
  const status = [signal.status, depthStatus].filter(Boolean).join(' ');
  setText($('manual-probe-status'), status);
  $('manual-probe-status').hidden = !status;
  manualProbeChart.$scrollPixels = 0;
  manualProbeChart.$renderedAt = frameTime;
  manualProbeChart.update('none');
}

function drawPressureHistory(now = performance.now(), animate = false) {
  if (!$('pressure-history').getClientRects().length) return;
  const source = state.probe.contact_samples ?? state.probe.pressure_samples ?? state.probe.pressure_history;
  if (source !== pressureSource) { pressureSource = source; pressureReceivedAt = now - Math.max(0, state.probe.history_age_s ?? 0) * 1000; }
  if (animate && probePressureChart && scrollTimeChart(probePressureChart, now, 25)) return;
  const elapsed = (now - pressureReceivedAt) / 1000;
  const probe = state.probe;
  if (!probePressureChart) probePressureChart = new Chart($('pressure-history'), {
    type: 'line',
    data: {datasets: [
      {label: 'Load (%)', yAxisID: 'y', data: [], borderColor: '#57d3a8', borderWidth: 2, pointRadius: 0, spanGaps: false},
      {label: 'Current (mA)', yAxisID: 'current', data: [], borderColor: '#ffb86b', borderWidth: 2, pointRadius: 0, spanGaps: false}]},
    options: {responsive: true, maintainAspectRatio: false, animation: false, parsing: false,
      plugins: {decimation: {enabled: true, algorithm: 'min-max'}, legend: {display: true, labels: {color: '#b7c4d8', boxWidth: 10, font: {size: 10}}}},
      scales: {x: {type: 'linear', min: -30, max: 0, ticks: {color: '#b7c4d8', callback: value => Math.abs(value)}},
        y: {min: 0, max: 10, ticks: {color: '#57d3a8'}},
        current: {position: 'right', min: 0, max: 50, grid: {drawOnChartArea: false}, ticks: {color: '#ffb86b'}}}},
  });
  const samples = probe.contact_samples ?? (probe.pressure_samples ?? (probe.pressure_history || []).map((ratio, i, all) =>
    ({x: (i - all.length + 1) * .2, y: ratio * 100}))).map(p => ({x: p.x, load: p.y, current: null}));
  ['load', 'current'].forEach((field, index) => {
    const points = [];
    samples.forEach((point, i) => {
      if (i && point.x - samples[i - 1].x > .3) points.push({x: point.x - elapsed - .001, y: null});
      points.push({x: point.x - elapsed, y: point[field]});
    });
    probePressureChart.data.datasets[index].data = points;
    const peak = Math.max(0, ...points.map(p => p.y || 0), (index ? probe.current_peak_ma : probe.contact_peak_percent) || 0);
    probePressureChart.options.scales[index ? 'current' : 'y'].max = contactScale(contactAxes[field], peak, index ? 50 : 10);
  });
  probePressureChart.$scrollPixels = 0;
  probePressureChart.$renderedAt = now;
  probePressureChart.update('none');
}

// Fixture angles are counter-clockwise in the world, which is counter-clockwise
// on screen too once the canvas y flip is accounted for.
function beamScreenAngle(degrees) {
  return -Math.PI / 2 - degrees * Math.PI / 180;
}

function radarGeometry(width, height, minimum, maximum) {
  const angles = [minimum, maximum, 0];
  for (let angle = Math.ceil(minimum / 90) * 90; angle <= maximum; angle += 90) angles.push(angle);
  const xs = [0, ...angles.map(angle => Math.cos(beamScreenAngle(angle)))];
  const ys = [0, ...angles.map(angle => Math.sin(beamScreenAngle(angle)))];
  const left = Math.min(...xs), right = Math.max(...xs);
  const top = Math.min(...ys), bottom = Math.max(...ys);
  const radius = Math.max(1, Math.min((width - 20) / (right - left || 1), (height - 20) / (bottom - top || 1)));
  return {radius, cx: (width - radius * (right + left)) / 2,
    cy: (height - radius * (bottom + top)) / 2};
}

function displayDetectorAngle(now) {
  // One telemetry interval of display delay allows interpolation without
  // predicting motion. Raw plots, safety gates and commands use real samples.
  const at = now - 50;
  for (let i = detectorAngles.length - 1; i > 0; --i) {
    const a = detectorAngles[i-1], b = detectorAngles[i];
    if (a.at <= at && at <= b.at && b.at > a.at && b.at-a.at <= 250 &&
        Number.isFinite(a.angle) && Number.isFinite(b.angle)) {
      return a.angle + (b.angle-a.angle)*(at-a.at)/(b.at-a.at);
    }
    if (b.at < at) break;
  }
  return state.detector.fixture_angle_deg;
}
function drawRadar(now = performance.now()) {
  const canvas = $('radar-view');
  const context = resize(canvas);
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const {beam, beam_half_angle_deg: half} = state.detector;
  const minimum = state.detector.beam_min_angle_deg ?? -half;
  const maximum = state.detector.beam_max_angle_deg ?? half;
  const {cx, cy, radius} = radarGeometry(width, height, minimum, maximum);

  context.clearRect(0, 0, width, height);
  context.strokeStyle = '#265148';
  context.lineWidth = 1;
  for (const factor of [0.33, 0.66, 1]) {
    context.beginPath();
    context.arc(cx, cy, radius * factor, beamScreenAngle(maximum), beamScreenAngle(minimum));
    context.stroke();
  }
  for (const edge of [minimum, 0, maximum]) {
    const angle = beamScreenAngle(edge);
    drawSegment(context, {x: cx, y: cy}, {x: cx + Math.cos(angle) * radius, y: cy + Math.sin(angle) * radius});
  }

  const offset = probeOffsetDeg();
  const unreachableLow = offset >= 0 ? minimum : Math.max(minimum, maximum + offset);
  const unreachableHigh = offset >= 0 ? Math.min(maximum, minimum + offset) : maximum;
  if (unreachableHigh > unreachableLow) {
    const from = beamScreenAngle(unreachableHigh);
    const to = beamScreenAngle(unreachableLow);
    const boundary = offset >= 0 ? from : to;
    context.save();
    context.fillStyle = 'rgba(230, 80, 50, 0.12)';
    context.beginPath();
    context.moveTo(cx, cy);
    context.arc(cx, cy, radius, from, to);
    context.closePath();
    context.fill();
    context.setLineDash([3, 3]);
    context.strokeStyle = 'rgba(230, 80, 50, 0.5)';
    context.lineWidth = 1.5;
    drawSegment(context, {x: cx, y: cy}, {x: cx + Math.cos(boundary) * radius, y: cy + Math.sin(boundary) * radius});
    context.restore();
  }

  // Trace bins span the calibrated sector, including asymmetric endpoints.
  context.lineWidth = 8;
  for (let index = 0; index < beam.length - 1; index++) {
    const ratio = (beam[index] + beam[index + 1]) / 2;
    const step = (maximum - minimum) / (beam.length - 1);
    const from = beamScreenAngle(minimum + (index + 1) * step);
    const to = beamScreenAngle(minimum + index * step);
    context.strokeStyle = detectorColor(ratio);
    context.beginPath();
    context.arc(cx, cy, radius * 0.84, from, to);
    context.stroke();
  }

  // Active detector coil arm (solid green)
  const currentAngle = displayDetectorAngle(now);
  const angle = beamScreenAngle(currentAngle);
  context.strokeStyle = '#4de0a8';
  context.lineWidth = 2;
  drawSegment(context, {x: cx, y: cy}, {x: cx + Math.cos(angle) * radius, y: cy + Math.sin(angle) * radius});

  const probeArmAngle = currentAngle + offset;
  if (probeArmAngle >= minimum && probeArmAngle <= maximum + 5) {
    const probeScreen = beamScreenAngle(probeArmAngle);
    context.save();
    context.strokeStyle = '#72d6b4';
    context.lineWidth = 1.5;
    context.setLineDash([3, 3]);
    drawSegment(context, {x: cx, y: cy}, {x: cx + Math.cos(probeScreen) * radius * 0.9, y: cy + Math.sin(probeScreen) * radius * 0.9});
    context.beginPath();
    context.arc(cx + Math.cos(probeScreen) * radius * 0.9, cy + Math.sin(probeScreen) * radius * 0.9, 3, 0, Math.PI * 2);
    context.fillStyle = '#72d6b4';
    context.fill();
    context.restore();
  }

  // Latest metal detection peak target reticle
  const align = getProbeAlignment();
  if (align.hasPeak && align.peakAngle >= minimum && align.peakAngle <= maximum) {
    const peakScreen = beamScreenAngle(align.peakAngle);
    const px = cx + Math.cos(peakScreen) * radius * 0.84;
    const py = cy + Math.sin(peakScreen) * radius * 0.84;
    context.save();
    context.strokeStyle = align.reachable ? '#f59e0b' : '#ef4444';
    context.lineWidth = 2;
    context.beginPath();
    context.arc(px, py, 6, 0, Math.PI * 2);
    context.stroke();
    context.beginPath();
    context.moveTo(px - 8, py); context.lineTo(px + 8, py);
    context.moveTo(px, py - 8); context.lineTo(px, py + 8);
    context.stroke();
    context.restore();
  }

  if (state.detector.detected) {
    context.fillStyle = '#e5b54a';
    context.beginPath();
    context.arc(cx, cy - radius * 0.58, 4 + state.detector.signal_ratio * 5, 0, Math.PI * 2);
    context.fill();
  }

}

// Browser input acquisition and diagnostics. No device output reports are sent.
function visiblePads() {
  try { return navigator.getGamepads ? Array.from(navigator.getGamepads()).filter(Boolean) : []; }
  catch { return []; }
}

function readWheel(pad, mapping) {
  const zero = {linear: 0, angular: 0, deadman: false};
  const indices = ['steering', 'throttle', 'brake'];
  if (!indices.every(key => Number.isInteger(mapping[key]) && mapping[key] >= 0 &&
      Number.isFinite(pad.axes[mapping[key]])) ||
      new Set(indices.map(key => mapping[key])).size !== 3 ||
      !Number.isFinite(mapping.deadzone) || mapping.deadzone < 0 || mapping.deadzone >= 1) return zero;
  const pedal = (key, inverted) => Math.max(0, Math.min(1, (pad.axes[mapping[key]] * (inverted ? -1 : 1) + 1) / 2));
  const raw = pad.axes[mapping.steering] * (mapping.invertSteering ? -1 : 1);
  const steering = Math.sign(raw) * Math.max(0, Math.min(1, (Math.abs(raw) - mapping.deadzone) / (1 - mapping.deadzone)));
  return {linear: (pedal('throttle', mapping.invertThrottle) - pedal('brake', mapping.invertBrake)) * MAX_LINEAR_MPS,
    angular: -steering * MAX_ANGULAR_RAD_S * preferences.sensitivity / 100,
    deadman: true};
}

function stopRcInput() {
  if (lastRcEnable !== -Infinity) send({type: 'rc_drive_enable', enabled: false});
  lastRcEnable = -Infinity;
}

function stopInput() {
  stopRcInput();
  releaseKeyboardCapture();
  if (state.drive.you_control_owner) {
    sendDrive(0, 0, false);
  }
}

function stopCameraStream(slot) {
  cameraGeneration[slot] += 1;
  clearTimeout(cameraRetries[slot]);
  clearTimeout(cameraWatchdogs[slot]);
  cameraRetries[slot] = cameraWatchdogs[slot] = null;
  cameraTopics[slot] = null;
  cameraSources[slot] = null;
  cameraStreams[slot]?.getTracks().forEach(track => track.stop());
  cameraStreams[slot] = null;
  const video = slot === 'forward' ? $('forward-video') : $('aux-video');
  video.srcObject = null;
  const networkImage = slot === 'forward' ? $('forward-network-camera') : $('aux-network-camera');
  networkImage.onload = networkImage.onerror = null;
  networkImage.removeAttribute('src');
}

async function startCamera(slot, source, force = false) {
  const topic = source.startsWith('raspberry:') ? state.cameras?.[source.slice(10)]?.topic : null;
  if (!force && cameraSources[slot] === source && cameraTopics[slot] === topic) return;
  stopCameraStream(slot);
  cameraSources[slot] = source;
  cameraTopics[slot] = topic;
  cameraErrors[slot] = '';
  const generation = cameraGeneration[slot];
  const current = () => generation === cameraGeneration[slot];
  updateCameraVisibility();
  if (source === 'virtual' || source === 'off') return;
  if (source.startsWith('raspberry:')) {
    const networkImage = slot === 'forward' ? $('forward-network-camera') : $('aux-network-camera');
    const started = performance.now();
    const retry = reason => {
      if (!current()) return;
      cameraErrors[slot] = reason;
      $('camera-source-status').textContent = 'Camera stream unavailable. Retrying automatically…';
      updateCameraLatency();
      clearTimeout(cameraWatchdogs[slot]);
      cameraWatchdogs[slot] = null;
      clearTimeout(cameraRetries[slot]);
      cameraRetries[slot] = setTimeout(() => {
        if (!current()) return;
        cameraRetries[slot] = null;
        startCamera(slot, source, true);
      }, 1500);
    };
    networkImage.onerror = () => retry('STREAM OFFLINE · retrying');
    networkImage.onload = () => {
      if (!current()) return;
      if (cameraRetries[slot] === null) cameraErrors[slot] = '';
      updateCameraLatency();
    };
    const checkStream = () => {
      if (!current()) return;
      if (!networkImage.naturalWidth && performance.now() - started >= 5000) {
        retry('NO CAMERA FRAMES · retrying');
        return;
      }
      if (networkCameraStale(source)) {
        retry('CAMERA SOURCE STALE · retrying');
        return;
      }
      cameraWatchdogs[slot] = setTimeout(checkStream, 1000);
    };
    networkImage.src = `/camera/${encodeURIComponent(source.slice(10))}?t=${Date.now()}-${generation}`;
    cameraWatchdogs[slot] = setTimeout(checkStream, 1000);
    return;
  }
  try {
    if (!navigator.mediaDevices?.getUserMedia) throw Error('Laptop cameras need HTTPS or localhost.');
    const stream = await navigator.mediaDevices.getUserMedia({
      audio: false,
      video: {deviceId: {exact: source}, width: {ideal: 640}, height: {ideal: 480}, frameRate: {ideal: 30}},
    });
    if (!current()) {
      stream.getTracks().forEach(track => track.stop());
      return;
    }
    cameraStreams[slot] = stream;
    const video = slot === 'forward' ? $('forward-video') : $('aux-video');
    video.srcObject = stream;
    await video.play();
    if (!current()) return;
    stream.getVideoTracks().forEach(track => track.addEventListener('ended', () => {
      if (current()) startCamera(slot, source, true);
    }));
    $('camera-source-status').textContent = 'Camera previews are live on this browser.';
  } catch (error) {
    if (!current()) return;
    stopCameraStream(slot);
    cameraErrors[slot] = 'LOCAL CAMERA UNAVAILABLE';
    $('camera-source-status').textContent = `Could not open camera: ${error.message}`;
  }
  updateCameraVisibility();
}

function updateCameraVisibility() {
  const forwardIsNetwork = preferences.cameras.forward.startsWith('raspberry:');
  const forwardIsVirtual = preferences.cameras.forward === 'virtual';
  forward.classList.toggle('hidden', !forwardIsVirtual);
  $('forward-video').classList.toggle('hidden', forwardIsVirtual || forwardIsNetwork);
  $('forward-network-camera').classList.toggle('hidden', !forwardIsNetwork);
  $('forward-camera-label').textContent = forwardIsVirtual ? 'VIRTUAL FORWARD CAMERA' : forwardIsNetwork ? 'RASPBERRY PI CAMERA' : 'LOCAL CAMERA';
  $('forward-camera-latency').classList.toggle('hidden', !forwardIsNetwork && !cameraErrors.forward);
  const auxiliaryIsNetwork = preferences.cameras.auxiliary.startsWith('raspberry:');
  const auxiliaryIsLocal = preferences.cameras.auxiliary !== 'off' && Boolean(cameraStreams.auxiliary);
  $('aux-video').classList.toggle('hidden', !auxiliaryIsLocal || auxiliaryIsNetwork);
  $('aux-network-camera').classList.toggle('hidden', !auxiliaryIsNetwork);
  $('aux-camera-label').classList.toggle('hidden', !auxiliaryIsNetwork && !auxiliaryIsLocal);
  $('aux-camera-latency').classList.toggle('hidden', !auxiliaryIsNetwork && !cameraErrors.auxiliary);
  updateCameraLatency();
  applyCameraRotations();
}

function updateCameraLatency() {
  const render = (element, camera, slot) => {
    if (cameraErrors[slot]) {
      element.textContent = cameraErrors[slot];
      element.classList.add('stale');
      return;
    }
    if (!camera) {
      element.textContent = `NO FRAMES · LINK ${linkLatencyMs ?? '--'} ms`;
      element.classList.add('stale');
      return;
    }
    const age = Math.max(0, Math.round(camera.frame_age_ms));
    const fps = Number(camera.fps || 0).toFixed(1);
    element.textContent = `${fps} SOURCE FPS · SOURCE AGE ${age} ms · RTT ${linkLatencyMs ?? '--'} ms`;
    element.classList.toggle('stale', age > 500);
  };
  const selected = source => source.startsWith('raspberry:') ? state.cameras?.[source.slice(10)] : null;
  render($('forward-camera-latency'), selected(preferences.cameras.forward), 'forward');
  render($('aux-camera-latency'), selected(preferences.cameras.auxiliary), 'auxiliary');
}

function rebuildCameraOptions() {
  for (const [slot, id, fallback, label] of [
    ['forward', 'forward-camera-source', 'virtual', 'Virtual forward camera'],
    ['auxiliary', 'aux-camera-source', 'off', 'Off'],
  ]) {
    const select = $(id);
    const selected = preferences.cameras[slot];
    select.replaceChildren(new Option(label, fallback));
    Object.entries(state.cameras || {}).forEach(([id, camera]) =>
      select.add(new Option(`Raspberry Pi · ${camera.label}`, `raspberry:${id}`)));
    localCameras.filter(camera => camera.deviceId).forEach((camera, index) =>
      select.add(new Option(camera.label || `Laptop camera ${index + 1}`, camera.deviceId)));
    if (![...select.options].some(option => option.value === selected)) {
      select.add(new Option(`Unavailable · ${selected.startsWith('raspberry:') ? selected.slice(10) : 'saved laptop camera'}`, selected));
    }
    select.value = selected;
  }
}

function updateNetworkCameraOptions(force = false) {
  const signature = JSON.stringify(Object.entries(state.cameras || {}).map(([id, camera]) => [id, camera.topic, camera.label]));
  if (signature === networkCameraSignature && !force) return;
  if (signature !== networkCameraSignature) rebuildCameraOptions();
  networkCameraSignature = signature;
  if (!autoForwardPiDone && preferences.cameras.forward === 'virtual') {
    const firstPi = Object.keys(state.cameras || {})[0];
    if (firstPi) {
      autoForwardPiDone = true;
      preferences.cameras.forward = `raspberry:${firstPi}`;
      savePreferences();
      rebuildCameraOptions();
    }
  }
  for (const slot of ['forward', 'auxiliary']) {
    if (preferences.cameras[slot].startsWith('raspberry:')) startCamera(slot, preferences.cameras[slot], force);
  }
}

function networkCameraStale(source) {
  const camera = state.cameras?.[source.slice(10)];
  const telemetryAge = Math.max(0, performance.now() - lastTelemetryAt);
  return state.connected && telemetryAge < TELEMETRY_TIMEOUT_MS && Number.isFinite(camera?.frame_age_ms)
    && camera.frame_age_ms + telemetryAge > 5000;
}

function recoverNetworkCameras(force = false) {
  for (const slot of ['forward', 'auxiliary']) {
    const source = preferences.cameras[slot];
    if (!source.startsWith('raspberry:')) continue;
    const image = slot === 'forward' ? $('forward-network-camera') : $('aux-network-camera');
    if (force || cameraErrors[slot] || !image.naturalWidth || networkCameraStale(source)) startCamera(slot, source, true);
  }
}

window.addEventListener('offline', () => { cameraWasOffline = true; });
window.addEventListener('online', () => {
  if (!cameraWasOffline) return;
  cameraWasOffline = false;
  recoverNetworkCameras(true);
});

async function refreshCameraDevices(requestPermission = false) {
  const refresh = ++cameraDeviceRefresh;
  rebuildCameraOptions();
  if (!navigator.mediaDevices?.enumerateDevices) {
    $('refresh-cameras').disabled = true;
    $('camera-source-status').textContent = 'Raspberry Pi cameras work here. Laptop cameras need HTTPS or localhost.';
    return;
  }
  let permissionStream;
  try {
    if (requestPermission) {
      permissionStream = await navigator.mediaDevices.getUserMedia({video: true, audio: false});
      // Release the temporary capture before opening selected cameras.
      permissionStream.getTracks().forEach(track => track.stop());
    }
    const devices = await navigator.mediaDevices.enumerateDevices();
    if (refresh !== cameraDeviceRefresh) return;
    localCameras = devices.filter(device => device.kind === 'videoinput');
    rebuildCameraOptions();
    $('camera-source-status').textContent = `${Object.keys(state.cameras || {}).length} Raspberry Pi camera source(s) · ${localCameras.length} laptop camera(s).`;
    for (const slot of ['forward', 'auxiliary']) {
      const source = preferences.cameras[slot];
      if (localCameras.some(camera => camera.deviceId === source && camera.label)) await startCamera(slot, source);
    }
  } catch (error) {
    if (refresh === cameraDeviceRefresh) $('camera-source-status').textContent = `Camera permission failed: ${error.message}`;
  } finally {
    permissionStream?.getTracks().forEach(track => track.stop());
  }
}

function selectedCameraElement(slot) {
  const source = preferences.cameras[slot];
  if (source === 'off') return null;
  if (source === 'virtual') return forward;
  return $(slot === 'forward'
    ? (source.startsWith('raspberry:') ? 'forward-network-camera' : 'forward-video')
    : (source.startsWith('raspberry:') ? 'aux-network-camera' : 'aux-video'));
}

function applyCameraRotations() {
  for (const slot of ['forward', 'auxiliary']) {
    const angle = preferences.cameraRotation[slot];
    const elements = slot === 'forward' ? [forward, $('forward-video'), $('forward-network-camera')]
      : [$('aux-video'), $('aux-network-camera')];
    for (const element of elements) {
      const w = element.clientWidth, h = element.clientHeight;
      const scale = angle % 180 && w && h ? Math.min(w / h, h / w) : 1;
      element.style.transform = `rotate(${angle}deg) scale(${scale})`;
    }
    $(slot + '-rotation').textContent = `${angle}°`;
    $(slot + '-rotate').disabled = preferences.cameras[slot] === 'off';
  }
}

function cameraDetails(slot) {
  const source = preferences.cameras[slot];
  const element = selectedCameraElement(slot);
  const network = source.startsWith('raspberry:');
  const camera = network ? state.cameras?.[source.slice(10)] : null;
  const track = cameraStreams[slot]?.getVideoTracks()[0];
  const settings = track?.getSettings?.() || {};
  const virtual = source === 'virtual';
  const width = element?.videoWidth || element?.naturalWidth || (virtual ? element.width : 0) || camera?.width || settings.width || 0;
  const height = element?.videoHeight || element?.naturalHeight || (virtual ? element.height : 0) || camera?.height || settings.height || 0;
  const age = camera ? Math.round(camera.frame_age_ms + Math.max(0, performance.now() - lastTelemetryAt)) : null;
  let status = source === 'off' ? 'Off' : virtual ? 'Simulation' : 'Waiting';
  if (network && camera && state.connected) status = age > 1000 ? 'Stale' : element?.naturalWidth ? 'Live' : 'Waiting';
  if (!network && track?.readyState === 'live') status = 'Live';
  if (network && !state.connected) status = 'Disconnected';
  if (cameraErrors[slot]) status = 'Unavailable';
  const drawable = virtual || (network ? Boolean(element?.naturalWidth) : element?.readyState >= 2);
  return {element, width, height, status, drawable,
    fps: network ? (camera ? `${Number(camera.fps).toFixed(1)} fps (source)` : '—')
      : virtual ? '30 fps target' : settings.frameRate ? `${Number(settings.frameRate).toFixed(1)} fps (configured)` : '—',
    age: age === null ? '—' : `${age} ms`,
    kind: network ? 'Raspberry Pi · MJPEG' : virtual ? 'Virtual camera' : source === 'off' ? 'None' : 'Browser camera',
    detail: network ? camera?.topic || source.slice(10) : virtual ? 'Rendered simulation · no physical camera' : track?.label || (source === 'off' ? 'Select a camera to enable the inset.' : 'Waiting for camera permission or a connected device.'),
  };
}

function renderCameraSettings() {
  if (!$('settings-dialog').open || $('settings-cameras').hidden) return;
  for (const slot of ['forward', 'auxiliary']) {
    const details = cameraDetails(slot);
    $(slot + '-preview-state').textContent = details.status;
    $(slot + '-preview-state').dataset.state = details.status.toLowerCase();
    $(slot + '-dimensions').textContent = details.width && details.height ? `${details.width} × ${details.height} px` : '—';
    $(slot + '-fps').textContent = details.fps;
    $(slot + '-age').textContent = details.age;
    $(slot + '-source-kind').textContent = details.kind;
    $(slot + '-source-detail').textContent = details.detail;
    $(slot + '-camera-retry').disabled = ['off', 'virtual'].includes(preferences.cameras[slot]);
    const canvas = $(slot + '-settings-preview');
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const empty = $(slot + '-preview-empty');
    empty.textContent = cameraErrors[slot] || (details.status === 'Off' ? 'Inset camera is off' : `${details.status} · waiting for frames`);
    empty.hidden = details.drawable && !['Unavailable', 'Disconnected', 'Stale'].includes(details.status);
    if (!empty.hidden || !details.width || !details.height) continue;
    const angle = preferences.cameraRotation[slot];
    const sideways = angle % 180 !== 0;
    const scale = Math.min(canvas.width / (sideways ? details.height : details.width), canvas.height / (sideways ? details.width : details.height));
    ctx.save();
    ctx.translate(canvas.width / 2, canvas.height / 2);
    ctx.rotate(angle * Math.PI / 180);
    try {
      ctx.drawImage(details.element, -details.width * scale / 2, -details.height * scale / 2, details.width * scale, details.height * scale);
    } catch { empty.hidden = false; }
    ctx.restore();
  }
}

function initializeCameraSettings() {
  for (const slot of ['forward', 'auxiliary']) {
    $(slot + '-rotate').onclick = () => {
      preferences.cameraRotation[slot] = (preferences.cameraRotation[slot] + 90) % 360;
      window.saveCameraView?.(slot);
      savePreferences();
      applyCameraRotations();
      renderCameraSettings();
    };
    $(slot + '-camera-retry').onclick = () => startCamera(slot, preferences.cameras[slot], true);
  }
  new ResizeObserver(applyCameraRotations).observe($('camera-panel'));
  function previewLoop() {
    if (!document.hidden) {
      renderCameraSettings();
    }
    requestAnimationFrame(previewLoop);
  }
  requestAnimationFrame(previewLoop);
}

function armSetupForm({section, fields, save, status, describe, invalid, applies, preview}) {
  const inputs = Object.entries(fields).map(([key, id]) => [key, $(id)]);
  const button = $(save), feedback = $(status);
  let dirty = false, pending = null, message = '';

  function render() {
    const saved = state.settings?.[section];
    const available = state.connected && inputs.every(([key]) => Number.isFinite(saved?.[key]));
    const owner = state.connected && state.drive.you_control_owner;
    if (!owner) {
      pending = null;
      dirty = false;
      message = '';
    } else if (pending) {
      if (available && inputs.every(([key]) => pending.value[key] === saved[key])) {
        pending = null;
        dirty = false;
        message = 'Saved on car. ' + applies;
      } else if (performance.now() - pending.at > 4000) {
        pending = null;
        message = 'No save confirmation. Check the car settings and retry.';
      }
    }
    for (const [key, input] of inputs) {
      const value = available ? String(saved[key]) : '';
      if (!dirty && input.value !== value) input.value = value;
      input.readOnly = !owner || !available || Boolean(pending);
      if (input.hasAttribute('data-positive'))
        input.setCustomValidity(input.valueAsNumber <= 0 ? 'Enter a speed greater than zero.' : '');
    }
    const valid = inputs.every(([, input]) => Number.isFinite(input.valueAsNumber) && input.validity.valid);
    const changed = available && inputs.some(([key, input]) => input.valueAsNumber !== saved[key]);
    button.disabled = !owner || !available || Boolean(pending) || !valid || !changed;
    feedback.textContent = !available ? 'Waiting for car settings.'
      : !owner ? 'Start driving to edit.'
      : pending ? 'Saving…' : message
      || (!valid ? invalid : changed ? `Unsaved change. The car uses ${describe(saved)}.` : applies);
    preview?.();
  }

  for (const [, input] of inputs) input.oninput = () => {
    dirty = true;
    message = '';
    render();
  };
  button.onclick = () => {
    if (button.disabled || !inputs.every(([, input]) => input.reportValidity())) return;
    const value = Object.fromEntries(inputs.map(([key, input]) => [key, input.valueAsNumber]));
    pending = {value, at:performance.now()};
    send({type:'settings', section, value});
    render();
  };
  return {render, result(reply) {
    if (pending && !reply.ok) {
      pending = null;
      message = reply.message || 'Save rejected.';
    }
    render();
  }};
}

function showScanCoverage() {
  const count = $('arm-scan-count').valueAsNumber, spacing = $('arm-scan-spacing').valueAsNumber;
  let description = '';
  if (Number.isInteger(count) && count >= 1 && count <= 31 && spacing >= .1 && spacing <= 180) {
    const edge = Number(((count - 1) * spacing / 2).toFixed(6));
    if (count === 1) description = 'One point at the target bearing.';
    else {
      const centre = count % 2 ? 'Includes the target bearing.'
        : `Nearest points are ±${Number((spacing / 2).toFixed(6))}° from the target.`;
      description = `Requested span −${edge}° to +${edge}° from the target. ${centre}`;
    }
  }
  const arm = state.actuators?.arm;
  const calibration = arm?.calibration ?? arm?.arm_calibration;
  if (description && arm?.connected && Number.isFinite(arm.fixture_angle_offset_deg) &&
      ['minimum', 'center', 'maximum'].every(k => Number.isFinite(calibration?.[k]))) {
    const offset = arm.fixture_angle_offset_deg + probeOffsetDeg();
    const right = (calibration.minimum - calibration.center) * 360/4096 + offset;
    const left = (calibration.maximum - calibration.center) * 360/4096 + offset;
    description += ` Saved probe bearing range: ${right.toFixed(1)}° to ${left.toFixed(1)}°.`;
    const target = state.inspection?.confirmed_angle_deg;
    if (Number.isFinite(target)) {
      const clip = clippedProbeEdges([right, left], target, count, spacing);
      if (target < right || target > left) description += ' The confirmed target is outside this range.';
      else if (clip) description += ` At this target the scan reaches the ${clip}.`;
    }
  }
  const output = $('arm-scan-coverage');
  if (output.value !== description) output.value = description;
}

const armSetupForms = new Map([
  ['probe_alignment', armSetupForm({section:'probe_alignment', fields:{offset_deg:'arm-probe-offset'},
    save:'arm-probe-offset-save', status:'arm-probe-offset-status', describe:saved => `${saved.offset_deg}°`,
    invalid:'Enter an offset from −180° to 180°.', applies:'Applies to the next aim or probe scan.'})],
  ['probe_stroke', armSetupForm({section:'probe_stroke',
    fields:{down_mm_s:'probe-down-speed', retract_mm_s:'probe-retract-speed', home_load_percent:'probe-servo-home-load'},
    save:'probe-stroke-save', status:'probe-stroke-status',
    describe:saved => `${saved.down_mm_s} mm/s down, ${saved.retract_mm_s} mm/s up and ${saved.home_load_percent}% home load`,
    invalid:'Enter positive speeds and a home load from 10% to 60%.', applies:'Applies to the next probe movement.'})],
  ['probe_scan', armSetupForm({section:'probe_scan', fields:{point_count:'arm-scan-count', spacing_deg:'arm-scan-spacing'},
    save:'arm-scan-save', status:'arm-scan-status', describe:saved => `${saved.point_count} points at ${saved.spacing_deg}° spacing`,
    invalid:'Enter 1–31 whole points and spacing from 0.1° to 180°.', applies:'Applies to the next probe scan.', preview:showScanCoverage})],
]);

function updateAlignmentSettings() {
  armSetupForms.forEach(form => form.render());
}
let metalPending = null;
let pendingPasses = null;
let metalFeedback = '';
let metalCalibrationKey = '';
let metalPacketKey = '';
const metalPackets = [];

function detectorVolts(count, reference) {
  return Number.isFinite(count) && Number.isFinite(reference) && reference > 0 ? count * reference / 255 : null;
}

function detectorVoltageRange(sensor) {
  if (!sensor || !Number.isFinite(sensor.reference_voltage) || sensor.reference_voltage <= 0) return null;
  const fullScale = sensor.input_range_volts;
  if (fullScale == null) return {min:0, max:sensor.reference_voltage};
  return Number.isFinite(fullScale) && fullScale > 0 ? {min:-fullScale, max:fullScale} : null;
}

function detectorMineVolts(sensor, ratio) {
  if (!Number.isFinite(sensor?.baseline_adc) || !Number.isFinite(sensor?.full_response_adc) || !Number.isFinite(ratio)) return null;
  return detectorVolts(sensor.baseline_adc + (sensor.full_response_adc - sensor.baseline_adc) * ratio, sensor.reference_voltage);
}

function voltageLabel(value, digits = 4) {
  return Number.isFinite(value) ? `${value.toFixed(digits)} V` : '— V';
}

function updateMetalSettings() {
  const sensor = state.detector.sensor || {};
  const available = state.connected && sensor.available;
  const fresh = available && sensor.fresh && state.detector.fresh !== false;
  const owner = state.connected && state.drive.you_control_owner;
  const passes = state.settings?.inspection?.passes;
  if (pendingPasses !== null && (!owner || !passes)) {
    pendingPasses = null;
    $('confirm-passes-feedback').textContent = 'Waiting for car settings.';
  }
  if (pendingPasses !== null && pendingPasses === passes) {
    pendingPasses = null;
    $('confirm-passes-feedback').textContent = 'Saved on car.';
  }
  $('confirm-passes').disabled = !owner || !passes || pendingPasses !== null;
  $('confirm-passes').value = pendingPasses ?? passes ?? '';
  const reference = sensor.reference_voltage;
  const range = detectorVoltageRange(sensor);
  $('metal-status').textContent = !state.connected ? 'DISCONNECTED' : !available
    ? (state.detector.fresh ? 'NO SENSOR DATA' : 'NO DRIVER') : !fresh ? 'STALE' : range ? 'LIVE' : 'NO VOLTAGE REFERENCE';
  $('metal-status').className = `badge ${fresh && range ? 'safe' : 'neutral'}`;
  const rawVolts = fresh && range ? detectorVolts(sensor.packet?.precise_adc ?? sensor.packet?.amplitude_adc, reference) : null;
  const filteredVolts = fresh && range ? detectorVolts(sensor.packet?.filtered_adc, reference) : null;
  const baselineVolts = detectorVolts(sensor.baseline_adc, reference);
  const mineVolts = detectorMineVolts(sensor, state.detector.threshold_ratio);
  $('metal-raw').textContent = voltageLabel(rawVolts, 6);
  $('metal-voltage').textContent = voltageLabel(filteredVolts, 6);
  $('metal-response').textContent = voltageLabel(filteredVolts !== null && baselineVolts !== null ? filteredVolts - baselineVolts : null, 6);
  $('metal-rate').textContent = available ? `${sensor.rate_hz ?? 0} Hz` : '-- Hz';
  $('metal-age').textContent = sensor.age_ms == null ? '-- ms' : `${sensor.age_ms} ms`;
  $('metal-port').textContent = sensor.port || '--';
  $('metal-counts').textContent = `${sensor.received ?? 0} / ${sensor.invalid_lines ?? 0}`;
  $('metal-sequence').textContent = sensor.packet ? `${sensor.packet.seq} / ${sensor.packet.uptime_ms} ms` : '--';
  $('metal-error').textContent = !available ? 'Detector input unavailable' : sensor.error || (fresh ? 'Receiving detector readings' : 'No fresh readings. Check the ADC connection in Setup → Sensor board.');
  $('metal-voltage-note').textContent = range ? `Input range ${range.min} to ${range.max} V. Levels are saved on the vehicle.` : 'Waiting for detector voltage reference.';
  for (const id of ['metal-baseline', 'metal-full']) {
    $(id).min = range?.min ?? 0;
    $(id).max = range?.max ?? 0;
  }
  const key = JSON.stringify([baselineVolts, mineVolts, range]);
  if (key !== metalCalibrationKey) {
    metalCalibrationKey = key;
    $('metal-baseline').value = range && baselineVolts !== null ? baselineVolts.toFixed(6) : '';
    $('metal-full').value = range && mineVolts !== null ? mineVolts.toFixed(6) : '';
    $('metal-launch').value = range ? `Baseline: ${voltageLabel(baselineVolts, 6)}
Mine level: ${voltageLabel(mineVolts, 6)}
Input range: ${range.min} to ${range.max} V` : 'Voltage reference unavailable.';
  }
  if (metalPending && sensor.command_result?.request_id === metalPending.id) {
    metalFeedback = sensor.command_result.message;
    metalPending = null;
  } else if (metalPending && performance.now() - metalPending.at > 3000) {
    metalFeedback = 'No acknowledgement from detector driver.';
    metalPending = null;
  }
  const allowed = owner && !metalPending;
  $('metal-zero').disabled = !allowed || !fresh || !range;
  $('metal-apply').disabled = !allowed || !range;
  $('metal-reset').disabled = !allowed;
  $('metal-control').hidden = owner;
  $('metal-control').disabled = !state.connected;
  $('metal-feedback').textContent = !state.connected ? 'Connect to save calibration.'
    : !owner ? 'Start driving to save calibration.' : metalPending ? 'Saving…'
    : !range ? 'Waiting for detector voltage reference.'
    : metalFeedback || (fresh ? 'Ready' : 'Manual levels can be saved. Measurement needs fresh readings.');
  const packetKey = JSON.stringify(sensor.packet);
  if (fresh && sensor.packet && packetKey !== metalPacketKey) {
    metalPacketKey = packetKey;
    if ($('metal-debug-live').checked) {
      metalPackets.push(`#${sensor.packet.seq ?? '—'} · unfiltered ${voltageLabel(rawVolts, 6)} · filtered ${voltageLabel(filteredVolts, 6)}`);
      if (metalPackets.length > 50) metalPackets.shift();
      $('metal-log').value = metalPackets.join('\n');
      $('metal-log').scrollTop = $('metal-log').scrollHeight;
    }
  }
  $('metal-export').disabled = !range || !(sensor.history?.length);
  if ($('settings-dialog').open && !$('settings-detector').hidden) drawMetalHistory();
}

let metalChart = null;
let detectorChart = null;
// Draws the operator's search-alert level as a dashed line (not a dataset).
const searchAlertLinePlugin = {
  id: 'searchAlertLine',
  afterDatasetsDraw(chart) {
    const volts = preferences.searchAlert.enabled ? searchAlertVolts() : NaN;
    const y = chart.scales.y;
    if (!Number.isFinite(volts) || !y || volts < y.min || volts > y.max) return;
    const {ctx, chartArea} = chart;
    const py = y.getPixelForValue(volts);
    const color = getComputedStyle(document.documentElement).getPropertyValue('--warn').trim() || '#f0b44c';
    ctx.save();
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 1;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    ctx.moveTo(chartArea.left, py);
    ctx.lineTo(chartArea.right, py);
    ctx.stroke();
    ctx.font = '600 10px system-ui, sans-serif';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'bottom';
    ctx.fillText(`Alert ${volts.toFixed(4)} V`, chartArea.right - 4, py - 2);
    ctx.restore();
  },
};
let detectorHistoryAt = 0;
let detectorDiagnosticsChart = null;
const detectorAngles = [];

function recordDetectorAngle(now) {
  const angle = state.detector.fixture_angle_deg;
  const available = state.safety?.simulated || state.actuators?.arm?.connected || (state.arm_servo?.connected && state.arm_servo?.active_role === 'arm');
  detectorAngles.push({at: now, angle: available && Number.isFinite(angle) ? angle : null});
  while (detectorAngles.length && now - detectorAngles[0].at > 30000) detectorAngles.shift();
}

// Use the receiver's full two-minute history to scale the visible trace.
const SIGNAL_LOOKBACK_S = 120;

function signalBounds(low, high, minSpan, domainLow = -Infinity, domainHigh = Infinity) {
  if (!Number.isFinite(low) || !Number.isFinite(high)) return {};
  const span = Math.min(domainHigh - domainLow, Math.max(minSpan, 1.2 * (high - low)));
  const min = Math.min(domainHigh - span, Math.max(domainLow, (high + low - span) / 2));
  return {min, max: min + span};
}

function detectorAxis(sensor, elapsed) {
  const range = detectorVoltageRange(sensor);
  if (!range) return {min:0, max:1};
  let low = Infinity, high = -Infinity;
  for (const sample of sensor.history || []) {
    const volts = detectorVolts(sample.filtered_adc, sensor.reference_voltage);
    if (sample.age_s + elapsed > SIGNAL_LOOKBACK_S || volts === null) continue;
    low = Math.min(low, volts);
    high = Math.max(high, volts);
  }
  return Number.isFinite(low) ? signalBounds(low, high, (range.max - range.min) / 100, range.min, range.max) : range;
}

// Operator-chosen axis for the detector charts (view only, kept in this browser).
const detectorAxisPref = (() => {
  try { return JSON.parse(localStorage.getItem('pom.detectorAxis')) || {auto: true}; } catch { return {auto: true}; }
})();
function saveDetectorAxisPref() {
  try { localStorage.setItem('pom.detectorAxis', JSON.stringify(detectorAxisPref)); } catch {}
}
function viewDetectorAxis(sensor, elapsed) {
  const auto = detectorAxis(sensor, elapsed);
  const {min, max} = detectorAxisPref;
  if (detectorAxisPref.auto || !Number.isFinite(min) || !Number.isFinite(max) || max <= min) return auto;
  return {min, max};
}
function renderDetectorAxisControls(sensor = state.detector.sensor || {}) {
  const auto = detectorAxisPref.auto;
  $('detector-autoscale').checked = auto;
  for (const id of ['detector-axis-min', 'detector-axis-max']) {
    const input = $(id);
    input.disabled = auto;
    if (auto || document.activeElement !== input) {
      const axis = auto ? detectorAxis(sensor, 0) : detectorAxisPref;
      const value = id.endsWith('min') ? axis.min : axis.max;
      input.value = Number.isFinite(value) ? Number(value.toFixed(2)) : '';
    }
  }
}
function redrawDetectorCharts() {
  renderDetectorAxisControls();
  drawDetectorHistory();
  drawDetectorDiagnostics();
}
$('detector-autoscale').onchange = event => {
  detectorAxisPref.auto = event.target.checked;
  if (!detectorAxisPref.auto && !Number.isFinite(detectorAxisPref.min)) {
    const axis = detectorAxis(state.detector.sensor || {}, 0);
    detectorAxisPref.min = axis.min; detectorAxisPref.max = axis.max;
  }
  saveDetectorAxisPref();
  redrawDetectorCharts();
};
for (const id of ['detector-axis-min', 'detector-axis-max']) $(id).onchange = event => {
  const value = event.target.valueAsNumber;
  if (!Number.isFinite(value)) return;
  detectorAxisPref[id.endsWith('min') ? 'min' : 'max'] = value;
  saveDetectorAxisPref();
  redrawDetectorCharts();
};

function setSampleTrace(dataset, points, gap) {
  // Preserve gaps on a slow link. Isolated readings need a visible dot even
  // when neither neighbour is close enough to draw a line segment.
  let radii;
  for (let i = 0; i < points.length; i++) {
    if (!Number.isFinite(points[i].y)) continue;
    if ((i && Number.isFinite(points[i-1].y) && points[i].x - points[i-1].x <= gap) ||
        (i + 1 < points.length && Number.isFinite(points[i+1].y) && points[i+1].x - points[i].x <= gap)) continue;
    radii ??= Array(points.length).fill(0);
    radii[i] = 2;
  }
  dataset.data = points;
  dataset.pointRadius = radii || 0;
}

function detectorPlotVisible(id, chart) {
  const holder = $(id).parentElement;
  if (!holder.clientWidth || !holder.clientHeight) return false;
  if (chart && (!chart.width || !chart.height)) chart.resize();
  return true;
}

function drawDetectorHistory(now = performance.now(), animate = false) {
  if (!detectorPlotVisible('detector-history', detectorChart)) return;
  if (animate && detectorChart && scrollTimeChart(detectorChart, now)) return;
  const sensor = state.detector.sensor || {};
  const elapsed = Math.max(0, (now - detectorHistoryAt) / 1000);
  const reference = sensor.reference_voltage;
  const samples = detectorVoltageRange(sensor)
    ? (sensor.history || []).filter(p => p.age_s + elapsed <= 30 && Number.isFinite(p.filtered_adc)) : [];
  if (!detectorChart) detectorChart = new Chart($('detector-history'), {
    type: 'line',
    plugins: [searchAlertLinePlugin],
    data: {datasets: [{label: 'Filtered voltage (V)', borderColor: '#6edcc7', borderWidth: 2, data: [], pointRadius: 0, spanGaps: .25}]},
    options: {responsive: true, maintainAspectRatio: false, animation: false, parsing: false,
      plugins: {decimation: {enabled: true, algorithm: 'min-max'}, legend: {display: false}},
      scales: {
        x: {type: 'linear', min: -30, max: 0, ticks: {color: '#bad0ca', maxRotation: 0, count: 4, callback: v => v === 0 ? 'now' : `${v}s`}, grid: {color: '#24404a'}},
        y: {title: {display: true, text: 'Volts', color: '#bad0ca'}, ticks: {color: '#bad0ca', maxTicksLimit: 4}, grid: {color: '#24404a'}},
      }},
  });
  setSampleTrace(detectorChart.data.datasets[0], samples.map(p => ({x: -p.age_s - elapsed, y: p.filtered_adc * reference / 255})), .25);
  Object.assign(detectorChart.options.scales.y, viewDetectorAxis(sensor, elapsed));
  if (detectorAxisPref.auto) renderDetectorAxisControls(sensor);
  detectorChart.$scrollPixels = 0;
  detectorChart.$renderedAt = now;
  detectorChart.update('none');
}

// Other overlays can have different sample rates. Hover selects the nearest
// measured detector sample in time, regardless of which y-axis is under it.
function nearestDetectorSample(chart, event) {
  const samples = chart.data.datasets[0].data;
  if (!samples.length || !chart.isDatasetVisible(0)) return [];
  const x = chart.scales.x.getValueForPixel(event.x);
  let low = 0, high = samples.length - 1;
  while (low < high) {
    const mid = Math.floor((low + high) / 2);
    if (samples[mid].x < x) low = mid+1;
    else high = mid;
  }
  const index = low && x-samples[low-1].x < samples[low].x-x ? low-1 : low;
  const element = chart.getDatasetMeta(0).data[index];
  return element ? [{element, datasetIndex: 0, index}] : [];
}

function detectorUtc(seconds) {
  if (!Number.isFinite(seconds)) return '';
  const date = new Date(seconds * 1000);
  return Number.isFinite(date.getTime()) ? date.toISOString() : '';
}

function detectorVoltagePoints(samples, reference, elapsed) {
  return samples.map(sample => ({
    x:-sample.age_s-elapsed, y:detectorVolts(sample.filtered_adc, reference),
    angle_deg:sample.angle_deg, timestamp_s:sample.timestamp_s,
  }));
}

Chart.Interaction.modes.detectorSample = nearestDetectorSample;
const detectorSampleTooltip = {
  mode: 'detectorSample', intersect: false,
  callbacks: {
    title: items => {
      const point = items[0]?.raw;
      if (!point) return '';
      const utc = detectorUtc(point.timestamp_s);
      return utc ? utc.slice(11,23) + ' UTC' : (-point.x).toFixed(2) + ' s ago';
    },
    label: ({raw}) => [
      'Filtered: ' + voltageLabel(raw.y, 6),
      'Fixture angle: ' + (Number.isFinite(raw.angle_deg) ? raw.angle_deg.toFixed(2) + '°' : 'unavailable'),
    ],
  },
};

function drawDetectorDiagnostics(now = performance.now(), animate = false) {
  if (!detectorPlotVisible('detector-diagnostics-history', detectorDiagnosticsChart)) return;
  if (animate && detectorDiagnosticsChart && scrollTimeChart(detectorDiagnosticsChart, now)) return;
  const sensor = state.detector.sensor || {};
  const elapsed = Math.max(0, (now - detectorHistoryAt) / 1000);
  const samples = detectorVoltageRange(sensor) ? (sensor.history || []).filter(p => p.age_s + elapsed <= 30 && Number.isFinite(p.filtered_adc)) : [];
  const zero = detectorVoltageRange(sensor) && detectorVolts(sensor.baseline_adc, sensor.reference_voltage);
  const trigger = detectorVoltageRange(sensor) && detectorMineVolts(sensor, state.detector.threshold_ratio);
  if (!detectorDiagnosticsChart) {
    detectorDiagnosticsChart = new Chart($('detector-diagnostics-history'), {
      type: 'line',
      data: {datasets: [
        {label: 'Filtered voltage (V)', yAxisID: 'volts', borderColor: '#6edcc7', borderWidth: 1.5, pointRadius: 0, data: []},
        {label: 'Angle (deg)', yAxisID: 'angle', borderColor: '#b9a0ef', borderWidth: 1.5, data: []},
        {label: 'Baseline (V)', yAxisID: 'volts', borderColor: '#efbb63', borderDash: [4, 4], borderWidth: 1, data: []},
        {label: 'Mine level (V)', yAxisID: 'volts', borderColor: '#ed7770', borderDash: [4, 4], borderWidth: 1, data: []},
      ]},
      options: {responsive: true, maintainAspectRatio: false, animation: false, parsing: false,
        elements: {point: {radius: 0, hitRadius: 6}, line: {spanGaps: .25}},
        plugins: {decimation: {enabled: true, algorithm: 'min-max'}, legend: {labels: {color: '#a4b5bb', boxWidth: 10, font: {size: 9}}},
          tooltip: detectorSampleTooltip},
        scales: {
          x: {type: 'linear', min: -30, max: 0, ticks: {color: '#a4b5bb', count: 3, maxRotation: 0,
            callback: v => v === 0 ? 'now' : `${v}s`}, grid: {color: '#2b3d46'}},
          volts: {position: 'left', title: {display:true,text:'Volts'}, ticks: {color: '#6edcc7', maxTicksLimit: 4}, grid: {color: '#2b3d46'}},
          angle: {position: 'right', suggestedMin: -1, suggestedMax: 1,
            ticks: {color: '#b9a0ef', maxTicksLimit: 4, callback: v => `${v}\u00b0`}, grid: {drawOnChartArea: false}},
        }},
    });
  }
  const readings = detectorVoltagePoints(samples, sensor.reference_voltage, elapsed);
  setSampleTrace(detectorDiagnosticsChart.data.datasets[0], readings, .25);
  while (detectorAngles.length && now - detectorAngles[0].at > 30000) detectorAngles.shift();
  detectorDiagnosticsChart.data.datasets[1].data = samples.some(p => Object.hasOwn(p, 'angle_deg'))
    ? readings.map(p => ({x:p.x, y:Number.isFinite(p.angle_deg) ? p.angle_deg : null}))
    : detectorAngles.map(p => ({x:(p.at-now)/1000, y:p.angle}));
  for (const [index, value] of [[2, zero], [3, trigger]]) {
    detectorDiagnosticsChart.data.datasets[index].data = Number.isFinite(value) ? [{x: -30, y: value}, {x: 0, y: value}] : [];
  }
  Object.assign(detectorDiagnosticsChart.options.scales.volts, viewDetectorAxis(sensor, elapsed));
  detectorDiagnosticsChart.$scrollPixels = 0;
  detectorDiagnosticsChart.$renderedAt = now;
  detectorDiagnosticsChart.update('none');
}

function drawMetalHistory(animate = false) {
  if (animate === true && metalChart && scrollTimeChart(metalChart, performance.now())) return;
  const canvas = $('metal-history');
  if (!canvas.parentElement.clientWidth || !canvas.parentElement.clientHeight) return;
  const seconds = Number($('metal-time-range').value) || 30;
  const elapsed = Math.max(0, (performance.now()-detectorHistoryAt)/1000);
  const sensor = state.detector.sensor || {};
  const reference = sensor.reference_voltage;
  const range = detectorVoltageRange(sensor);
  const samples = range ? (sensor.history || []).filter(sample => sample.age_s + elapsed <= seconds && Number.isFinite(sample.filtered_adc)) : [];
  const values = samples.map(sample => detectorVolts(sample.filtered_adc, reference));
  const baseline = range && detectorVolts(sensor.baseline_adc, reference);
  const trigger = range && detectorMineVolts(sensor, state.detector.threshold_ratio);
  const {min: low, max: high} = $('metal-autoscale').checked || !range ? detectorAxis(sensor, elapsed) : range;
  if (!metalChart) {
    metalChart = new Chart(canvas, {
      type: 'line',
      data: {datasets: [
        {label: 'Filtered detector (V)', data: [], borderColor: '#6edcc7', borderWidth: 2,
          pointRadius: 0, pointHitRadius: 8, spanGaps: 0.5},
        {label: 'Baseline (V)', data: [], borderColor: '#efbb63', borderWidth: 1,
          borderDash: [5, 4], pointRadius: 0},
        {label: 'Mine level (V)', data: [], borderColor: '#ed7770', borderWidth: 1,
          borderDash: [5, 4], pointRadius: 0},
        {label: 'Search alert (V)', data: [], borderColor: '#f0b44c', borderWidth: 1,
          borderDash: [2, 3], pointRadius: 0},
      ]},
      options: {
        responsive: true, maintainAspectRatio: false, animation: false, parsing: false,
        plugins: {legend: {display: true, labels: {color: '#a4b5bb', boxWidth: 16}}, tooltip: detectorSampleTooltip},
        scales: {
          x: {type: 'linear', min: -30, max: 0, grid: {color: '#2b3d46'},
            ticks: {color: '#a4b5bb', maxRotation: 0, count: 4,
              callback: value => value === 0 ? 'now' : `${value}s`}},
          y: {title:{display:true,text:'Volts'}, min: low, max: high, grid: {color: '#2b3d46'},
            ticks: {color: '#a4b5bb', callback: value => Number(value).toFixed(4), maxTicksLimit: 6}},
        },
      },
    });
  }
  setSampleTrace(metalChart.data.datasets[0], detectorVoltagePoints(samples, reference, elapsed), .5);
  metalChart.data.datasets[1].data = Number.isFinite(baseline)
    ? [{x: -seconds, y: baseline}, {x: 0, y: baseline}] : [];
  metalChart.data.datasets[2].data = Number.isFinite(trigger)
    ? [{x: -seconds, y: trigger}, {x: 0, y: trigger}] : [];
  const alertVolts = preferences.searchAlert.enabled ? searchAlertVolts() : NaN;
  metalChart.data.datasets[3].data = Number.isFinite(alertVolts) && alertVolts >= low && alertVolts <= high
    ? [{x: -seconds, y: alertVolts}, {x: 0, y: alertVolts}] : [];
  metalChart.options.scales.x.min = -seconds;
  const preview = range && window.operatorDetectorFilterPreview?.(seconds);
  let lower = low, upper = high;
  if (preview) {
    Object.assign(metalChart.data.datasets[4] ??= {}, preview);
    if ($('metal-autoscale').checked) for (const point of preview.data) {
      if (!Number.isFinite(point.y)) continue;
      lower = Math.min(lower, point.y);
      upper = Math.max(upper, point.y);
    }
  } else if (metalChart.data.datasets.length > 4) metalChart.data.datasets.length = 4;
  metalChart.options.scales.y.min = lower;
  metalChart.options.scales.y.max = upper;
  metalChart.resize();
  metalChart.$scrollPixels = 0;
  metalChart.$renderedAt = performance.now();
  metalChart.update('none');
  canvas.setAttribute('aria-label', `Filtered metal detector voltage over the last ${seconds} seconds`);
  $('metal-range').textContent = values.length
    ? `Min ${Math.min(...values).toFixed(6)} V · Max ${Math.max(...values).toFixed(6)} V · Peak-to-peak ${(Math.max(...values) - Math.min(...values)).toFixed(6)} V`
    : range ? `No filtered readings in the last ${seconds} seconds` : 'Voltage reference unavailable.';
}

function detectorColor(signalRatio) {
  const fraction = Math.max(0, Math.min(1, signalRatio / Math.max(state.detector.threshold_ratio, 0.001)));
  return `hsl(${(1 - fraction) * 120} 78% 55%)`;
}

function initializeMetalSettings() {
  const command = action => {
    const id = `metal-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`;
    const payload = {type: 'detector_calibrate', action, request_id: id};
    if (action === 'apply') {
      if (!$('metal-baseline').value || !$('metal-full').value) { metalFeedback = 'Enter a baseline and mine level in volts.'; updateMetalSettings(); return; }
      const reference = state.detector.sensor?.reference_voltage;
      const range = detectorVoltageRange(state.detector.sensor);
      if (!range) { metalFeedback = 'Detector voltage reference is unavailable.'; updateMetalSettings(); return; }
      const baseline = Number($('metal-baseline').value), mine = Number($('metal-full').value);
      if (![baseline, mine].every(v => Number.isFinite(v) && v >= range.min && v <= range.max) || baseline === mine) {
        metalFeedback = `Baseline and mine level must be different values between ${range.min} and ${range.max} V.`; updateMetalSettings(); return;
      }
      payload.baseline_adc = baseline / reference * 255;
      payload.full_response_adc = mine / reference * 255;
    }
    metalPending = {id, at: performance.now()};
    send(payload); updateMetalSettings();
  };
  $('confirm-passes').onchange = () => {
    if ($('confirm-passes').disabled) return;
    pendingPasses = Number($('confirm-passes').value);
    $('confirm-passes-feedback').textContent = 'Saving…';
    send({type:'settings', section:'inspection', value:{passes:pendingPasses}});
    updateMetalSettings();
  };
  $('metal-zero').onclick = () => command('zero');
  $('metal-apply').onclick = () => command('apply');
  $('metal-reset').onclick = () => command('reset');
  $('metal-control').onclick = () => send({type: 'take_control'});
  $('metal-autoscale').onchange = drawMetalHistory;
  $('metal-time-range').onchange = drawMetalHistory;
  $('metal-export').onclick = () => {
    const sensor = state.detector.sensor;
    if (!detectorVoltageRange(sensor) || !sensor.history?.length) return;
    const rows = ['age_seconds,timestamp_utc,unfiltered_voltage_v,filtered_voltage_v,fixture_angle_deg',
      ...sensor.history.map(sample => {
        const age = Number.isFinite(sample.age_s) ? sample.age_s + Math.max(0, (performance.now() - detectorHistoryAt) / 1000) : '';
        return [age, detectorUtc(sample.timestamp_s),
          detectorVolts(sample.adc, sensor.reference_voltage), detectorVolts(sample.filtered_adc, sensor.reference_voltage),
          Number.isFinite(sample.angle_deg) ? sample.angle_deg : ''].join(',');
      })];
    const url = URL.createObjectURL(new Blob([rows.join('\n')], {type: 'text/csv'}));
    const link = document.createElement('a'); link.href = url; link.download = 'detector-readings.csv'; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
}

function initializeInputs() {
  initializeArmSettings();
  initializeMetalSettings();
  const tabs = [...document.querySelectorAll('[data-settings-tab]')];
  tabs.forEach((tab, index) => {
    tab.tabIndex = tab.getAttribute('aria-selected') === 'true' ? 0 : -1;
    tab.onkeydown = event => {
      const next = event.key === 'Home' ? 0 : event.key === 'End' ? tabs.length - 1
        : ['ArrowRight', 'ArrowDown'].includes(event.key) ? (index + 1) % tabs.length : ['ArrowLeft', 'ArrowUp'].includes(event.key) ? (index + tabs.length - 1) % tabs.length : null;
      if (next === null) return;
      event.preventDefault(); tabs[next].click(); tabs[next].focus();
    };
    tab.onclick = () => {
      stopArmMove();
      document.querySelectorAll('[data-settings-tab]').forEach(candidate => {
        const active = candidate === tab;
        candidate.classList.toggle('active', active);
        candidate.setAttribute('aria-selected', String(active));
        candidate.tabIndex = active ? 0 : -1;
      });
      document.querySelectorAll('[data-settings-panel]').forEach(panel => {
        const active = panel.dataset.settingsPanel === tab.dataset.settingsTab;
        panel.classList.toggle('active', active);
        panel.hidden = !active;
      });
      updateMetalSettings();
    };
  });
  document.querySelectorAll('[data-open-settings]').forEach(button => {
    button.onclick = () => { $('settings').click(); document.querySelector(`[data-settings-tab="${button.dataset.openSettings}"]`).click(); };
  });
  for (const [key, label, checkbox] of [
    ['steering', 'Steering axis'], ['throttle', 'Forward pedal axis'], ['brake', 'Reverse pedal axis'],
    ['deadzone', 'Steering deadzone'],
    ['invertSteering', 'Invert steering', true], ['invertThrottle', 'Invert forward pedal', true],
    ['invertBrake', 'Invert reverse pedal', true],
  ]) {
    const wrapper = document.createElement('label');
    wrapper.textContent = label;
    const input = document.createElement('input');
    input.type = checkbox ? 'checkbox' : 'number';
    if (checkbox) input.checked = Boolean(preferences.wheel[key]);
    else {
      input.min = 0; input.max = key === 'deadzone' ? 0.9 : 63;
      input.step = key === 'deadzone' ? 0.01 : 1;
      input.value = preferences.wheel[key];
    }
    input.onchange = () => {
      stopInput();
      if (!input.reportValidity() || (input.type === 'number' && !input.value)) return;
      preferences.wheel[key] = checkbox ? input.checked : input.value === '' ? null : Number(input.value);
      storeDisplaySettings();
      savePreferences();
    };
    input.dataset.field = key;
    wrapper.append(input); $('wheel-mapping').append(wrapper);
  }
  for (const code of ['KeyW', 'KeyA', 'KeyS', 'KeyD', 'Space']) {
    const key = document.createElement('kbd'); key.dataset.code = code;
    key.textContent = code.replace('Key', ''); $('key-preview').append(key);
  }
  $('input-device').onchange = event => {
    stopInput();
    const pad = visiblePads().find(p => String(p.index) === event.target.value);
    selectedPadIndex = pad?.index ?? null; selectedPadId = pad?.id ?? null;
  };
  $('keyboard-test').onclick = () => {
    $('settings-dialog').addEventListener('close', () => {
      cameraPanel.focus({preventScroll: true});
      if (!cameraFocused) activateKeyboardInput();
    }, {once: true});
    $('settings-dialog').close();
  };
  window.addEventListener('blur', stopInput);
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopInput(); else recoverNetworkCameras(); });
  initializeCameraSettings();
  $('refresh-cameras').onclick = () => refreshCameraDevices(true);
  $('forward-camera-source').onchange = async event => {
    stopInput();
    preferences.cameras.forward = event.target.value;
    savePreferences();
    window.saveCameraView?.('forward');
    await startCamera('forward', event.target.value);
  };
  $('aux-camera-source').onchange = async event => {
    preferences.cameras.auxiliary = event.target.value;
    savePreferences();
    window.saveCameraView?.('auxiliary');
    await startCamera('auxiliary', event.target.value);
  };
  navigator.mediaDevices?.addEventListener?.('devicechange', () => refreshCameraDevices(false));
  refreshCameraDevices(false);
}

let deviceSignature = '';
let lastInputHint = 0;
function renderInputs(pads, target) {
  if (performance.now() - lastInputHint > 100) {
    updateInputHint();
    lastInputHint = performance.now();
  }
  if (!$('settings-dialog').open || $('settings-input').hidden) return;
  const keyboard = preferences.input === 'keyboard';
  const rcInput = preferences.input === 'rc';
  $('device-settings').classList.toggle('hidden', keyboard || rcInput);
  $('rc-drive-settings').classList.toggle('hidden', !rcInput);
  if (rcInput) {
    const rc = state.drive.rc;
    $('rc-drive-input').textContent = rc?.fresh ? `Steering CH${rc.steering_channel}: ${Math.round(rc.steering * 100)}% · throttle CH${rc.throttle_channel}: ${Math.round(rc.throttle * 100)}%` : 'RC channels missing or stale';
  }
  $('wheel-settings').classList.toggle('hidden', preferences.input !== 'wheel');
  $('keyboard-test').classList.toggle('hidden', !keyboard);
  $('key-preview').classList.toggle('hidden', !keyboard);
  $('browser-status').textContent = `Secure context: ${window.isSecureContext ? 'yes' : 'no — open HTTPS or localhost'} · Gamepad API: ${navigator.getGamepads ? 'available' : 'unavailable'} · ${pads.length} device(s) visible`;
  const signature = JSON.stringify(pads.map(p => [p.index, p.id]));
  if (signature !== deviceSignature) {
    deviceSignature = signature;
    $('input-device').replaceChildren(new Option('Select a connected device', ''));
    pads.forEach(p => $('input-device').add(new Option(`${p.index}: ${p.id}`, String(p.index))));
    $('input-device').value = gamepad ? String(gamepad.index) : '';
  }
  $('device-info').textContent = gamepad
    ? `${gamepad.id} · ${gamepad.mapping || 'non-standard'} · ${gamepad.axes.length} axes · ${gamepad.buttons.length} buttons${preferences.input === 'controller' ? ` · ${mappingLabel(resolveMapping(gamepad))}` : ''}`
    : 'No device selected. Press a device button, then select it above.';
  document.querySelectorAll('[data-code]').forEach(el => el.classList.toggle('pressed', keys.has(el.dataset.code)));
  const rc = state.drive.rc;
  const preview = rcInput ? {
    linear: rc?.fresh ? rc.throttle * (state.drive.max_velocity_mps ?? MAX_LINEAR_MPS) : 0,
    angular: rc?.fresh ? -rc.steering * (state.drive.max_yaw_rate_rad_s ?? MAX_ANGULAR_RAD_S) : 0,
    deadman: false,
  } : target;
  $('steering-preview').setAttribute('transform', `rotate(${-preview.angular / MAX_ANGULAR_RAD_S * 100} 80 80)`);
  $('mapped-input').textContent = `Forward / reverse: ${preview.linear.toFixed(3)} m/s\nSteering: ${preview.angular.toFixed(3)} rad/s\nDeadman: ${preview.deadman ? 'HELD' : 'released'}\nDrive output: ${state.drive.publishing ? `${state.drive.cmd_linear_x.toFixed(2)} m/s · ${state.drive.cmd_angular_z.toFixed(2)} rad/s` : driveBlockMessage().text}`;
  const axes = keyboard ? [] : gamepad?.axes ?? [];
  const buttons = keyboard ? [] : gamepad?.buttons ?? [];
  const axisRows = $('raw-axes'), buttonRows = $('raw-buttons');
  if (axisRows.children.length !== axes.length) {
    axisRows.replaceChildren(...axes.map(() => {
      const label=document.createElement('label'), text=document.createTextNode('');
      const meter=document.createElement('meter'); meter.min=-1; meter.max=1;
      label.append(text,meter); return label;
    }));
  }
  axes.forEach((value,index) => {
    const row=axisRows.children[index], text=`Axis ${index}  ${value.toFixed(3)}`;
    if (row.firstChild.nodeValue !== text) row.firstChild.nodeValue=text;
    row.lastChild.value=value;
  });
  if (buttonRows.children.length !== buttons.length)
    buttonRows.replaceChildren(...buttons.map(() => document.createElement('span')));
  buttons.forEach((button,index) => {
    const item=buttonRows.children[index], text=`B${index} ${button.value.toFixed(2)}`;
    item.classList.toggle('pressed',button.pressed);
    if (item.textContent !== text) item.textContent=text;
  });
}

initializeInputs();
savePreferences();
updateCameraVisibility();
setConnection(false, 'Connecting to operator stack');
connect();
inputLoop();
// Render at the display refresh rate; telemetry and motion command cadence
// remain independent. Never invent measurements to fill display frames.
function drawDashboard(now) {
  if (!document.hidden) {
    const motion = displayMotion(now);
    updateSearchAlert(now);
    drawForwardView(motion.robot);
    drawRadar(now);
    if (!map.hidden && map.clientWidth && map.clientHeight) drawMap(motion.robot);
    drawProbeMotion(motion.depth);
    drawPressureHistory(now, true);
    drawDetectorHistory(now, true);
    drawDetectorDiagnostics(now, true);
    window.operatorADCFrame?.();
    servoSettings.forEach(panel => panel.draw());
    if ($('settings-dialog').open && !$('settings-detector').hidden) drawMetalHistory(true);
  }
  requestAnimationFrame(drawDashboard);
}
requestAnimationFrame(drawDashboard);
