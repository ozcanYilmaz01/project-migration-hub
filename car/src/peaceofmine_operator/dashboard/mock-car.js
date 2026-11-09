// Preview-only pretend car. Loaded only with ?mock=1 and never served by the
// car (operator_web_node.py does not allow-list this file). It replaces the
// /ws WebSocket with a local source that emits `state` messages shaped like
// operator_control_node.py + web_bridge.py telemetry.
(() => {
  const RealWebSocket = window.WebSocket;
  const started = performance.now();
  const detectorHistory = [];
  const probeHistory = [];
  let lastProbeHistoryAt = -Infinity;
  const mock = {owner: false, armed: false, sweep: false, linear: 0, angular: 0, pose: {x: 0, y: 0, yaw: 0},
    inspection: {phase: 'IDLE', active: false, message: '', completed: 0, total: 0, confirmed_angle_deg: null, points: []},
    phaseAt: 0, armAngle: 0, depth: 0, cycle: false, calibrating: false, homing: 0, homed: true};
  window.__mockCar = mock; // preview-only handle for scripted checks
  const capture = {device:'auto', width:640, height:480, framerate:30, pixel_format:'mjpeg2rgb'};
  mock.settings = {
    gnss: {max_wait_s: 5, target_accuracy_m: 1},
    inspection:{passes:2}, probe_alignment:{offset_deg:30}, probe_scan:{point_count:7,spacing_deg:10},
    probe_stroke:{down_mm_s:5,retract_mm_s:5,home_load_percent:10},
    marking:{duration_s:2, idle_percent:0, active_percent:100, manual_percent:0},
    saved:true, driving:{input:'controller',forward_percent:100,reverse_percent:100},
    display:{mapping:'auto',sensitivity:100,wheel:{steering:0,throttle:1,brake:2,deadman:0,invertSteering:false,
      invertThrottle:true,invertBrake:true,deadzone:.05},searchAlert:{enabled:true,level:60,sound:true}},
    cameras:{forward:{source:'virtual',rotation:0},auxiliary:{source:'raspberry:camera_probe',rotation:0},
      capture:{forward:{...capture},auxiliary:{...capture}}}
  };
  mock.fieldEntries = [];
  function previewLog(event) {
    const mux = adcConfig.channels.find(ch => ch.enabled && ['detector','both'].includes(routes[ch.mux]))?.mux;
    mock.fieldEntries.unshift({id:String(++logSequence), timestamp:Date.now()/1000,
      x:mock.pose.x, y:mock.pose.y, heading_deg:mock.pose.yaw*180/Math.PI,
      detector_volts:adcFiltered.get(mux) ?? null, ...event});
  }
  let logSequence = 0;
  const pump = {mode:'idle', until:0, output:0, lastRequest:null};
  function stopPump() {
    pump.mode = 'idle';
    pump.until = 0;
    pump.output = mock.owner && mock.armed ? mock.settings.marking.idle_percent : 0;
  }

  // The preview compresses confirmation passes to two seconds each.
  function stepInspection(t) {
    const i = mock.inspection;
    const dt = t - mock.phaseAt - 1; // First second is the automatic home.
    if (i.active && dt < 0) {
      i.message = i.phase === 'CONFIRMING' ? 'Homing probe before confirmation.' : 'Homing probe before scan.';
      mock.homing = 1;
      return;
    }
    if (mock.homing && i.active) {
      mock.depth = 0;
      mock.homed = true;
      mock.homing = 0;
      i.message = '';
    }
    i.probe_offset_deg = mock.settings.probe_alignment.offset_deg;
    i.probe_reachable = Number.isFinite(i.confirmed_angle_deg) && Math.abs(i.confirmed_angle_deg - i.probe_offset_deg) <= 45;
    if (i.phase === 'CONFIRMING') {
      i.progress = Math.min(i.total, dt / 2);
      i.completed = Math.floor(i.progress);
      if (i.completed >= i.total) Object.assign(i, {phase: 'CONFIRMED', active: false, message: '', confirmed_angle_deg: 6, probe_reachable: Math.abs(6-i.probe_offset_deg) <= 45, progress:i.total});
    } else if (i.phase === 'PROBING') {
      const point = i.points.length;
      const stroke = mock.settings.probe_stroke;
      mock.cycle = true;
      i.probe_goal_id = `preview-${mock.phaseAt}-${point}`;
      if (!mock.pointResult) {
        mock.cyclePhase = 'DESCENDING';
        mock.depth = Math.min(i.probe_target_depth_mm, Math.max(0, t - mock.pointAt) * stroke.down_mm_s);
        const contact = mockContact();
        const hit = contact.detected === true && mock.depth < i.probe_target_depth_mm;
        if (!hit && mock.depth < i.probe_target_depth_mm) return;
        mock.pointResult = {at:t, depth:mock.depth, status:hit ? 'CONTACT' : contact.fresh ? 'CLEAR' : 'UNKNOWN',
          contact_depth_mm:hit ? mock.depth : null, contact_volts:contact.filtered_volts};
      }
      mock.cyclePhase = 'RETRACTING';
      const result = mock.pointResult;
      mock.depth = Math.max(0, result.depth - (t - result.at) * stroke.retract_mm_s);
      if (mock.depth > 0) return;
      i.points.push({angle_deg:i.probe_angles_deg[point], status:result.status, contact_depth_mm:result.contact_depth_mm});
      previewLog({status:result.status === 'CONTACT' ? 'CONTACT TRIGGER' : 'PROBE ' + result.status,
        statusType:result.status.toLowerCase(), contact_volts:result.contact_volts, contact_depth_mm:result.contact_depth_mm});
      i.completed = i.progress = i.points.length;
      mock.pointAt = t;
      mock.pointResult = null;
      if (i.completed === i.total) {
        mock.cycle = false;
        i.probe_goal_id = null;
        const hits = i.points.filter(p => p.status === 'CONTACT').length;
        Object.assign(i, {phase:'COMPLETE', active:false, message:`Probe scan complete · ${hits} of ${i.total} positions had contact.`});
      }
    }
  }

  function detectorVoltage(t) {
    const angle = mock.sweep ? 40 * Math.sin(t * 1.2) : mock.armAngle;
    const nearTarget = Math.exp(-((mock.pose.x - 8) ** 2) / 2);
    return (40 + 120 * nearTarget * Math.cos(angle * Math.PI / 180) ** 2 + 4 * Math.sin(t * 7)) * 3.3 / 255;
  }
  function telemetry() {
    const t = (performance.now() - started) / 1000;
    const angle = mock.sweep ? 40 * Math.sin(t * 1.2) : mock.armAngle;
    const assigned = adcConfig.channels.find(channel => channel.enabled && ['detector','both'].includes(routes[channel.mux]));
    const rawVolts = assigned?.mux === 6 ? detectorVoltage(t) : .2;
    const raw = rawVolts * 255 / 3.3;
    const filtered = (adcFiltered.get(assigned?.mux) ?? rawVolts) * 255 / 3.3;
    detectorHistory.push({at:t, adc:raw, filtered_adc:filtered, angle_deg:angle, timestamp_s:Date.now()/1000});
    while (detectorHistory.length && t - detectorHistory[0].at > 120) detectorHistory.shift();
    const dt = 0.1;
    mock.pose.yaw += mock.angular * dt;
    mock.pose.x += mock.linear * Math.cos(mock.pose.yaw) * dt;
    mock.pose.y += mock.linear * Math.sin(mock.pose.yaw) * dt;
    const ratio = Math.max(0, Math.min(1, (filtered - 40) / 120));
    stepInspection(t);
    if (!mock.owner || pump.mode === 'idle' || performance.now() >= pump.until) stopPump();
    const load = 100 * (mock.cycle ? 0.08 + mock.depth / 150 + 0.02 * Math.random() : 0.05 + 0.01 * Math.random());
    probeHistory.push({at:t, load, depth_mm:mock.homed ? mock.depth : null});
    while (probeHistory.length && t - probeHistory[0].at > 30) probeHistory.shift();
    const includeProbeHistory = t-lastProbeHistoryAt >= .2;
    if (includeProbeHistory) lastProbeHistoryAt = t;
    return {
      type: 'state',
      marking:{settings:{...mock.settings.marking},mode:pump.mode,output_percent:pump.output,last_shot_request:pump.lastRequest},
      pump_active:pump.mode !== 'idle',
      safety: {allowed: true, servo_allowed: true, mode: 'simulation', simulated: true, reason: 'Pretend car (preview)'},
      drive: {armed: mock.armed, px4_armed: null, input: mock.settings.driving.input, rc: {fresh: false}, deadman: false, publishing: mock.owner && (mock.linear !== 0 || mock.angular !== 0),
        command_source: 'browser', joy: {seen: false}, cmd_linear_x: mock.linear, cmd_angular_z: mock.angular,
        cmd_vel_topic: 'cmd_vel', timeout_ms: 300, client_count: 1, you_control_owner: mock.owner,
        control_owner_present: mock.owner, calibrating: mock.calibrating, max_velocity_mps: 0.8, max_yaw_rate_rad_s: 1.4, pump_active: pump.mode !== 'idle'},
      robot: {...mock.pose, speed_mps: Math.abs(mock.linear), stationary: mock.linear === 0, odometry_fresh: true, pose_fresh: true},
      detector: {latest_peak: Number.isFinite(mock.inspection.confirmed_angle_deg)
          ? {angle_deg:mock.inspection.confirmed_angle_deg, signal:1, fresh:true} : null,
        signal_ratio: ratio, threshold_ratio: 1, detected: ratio >= 1, fresh: true, fixture_angle_deg: angle,
        sweep_enabled: mock.sweep, sweep_speed_deg_s: 36, sweep_speed_min: 5, sweep_speed_max: 180, beam_half_angle_deg: 45,
        beam_min_angle_deg: -45, beam_max_angle_deg: 45, beam: [],
        sensor: {available: true, fresh: true, age_ms: 20, rate_hz: 50, received: Math.round(t * 50), invalid_lines: 0,
          reference_voltage: 3.3, input_range_volts:[6.144,4.096,2.048,1.024,.512,.256][adcConfig.pga], baseline_adc: 40, packet: {filtered_adc: filtered, precise_adc: raw, amplitude_adc: Math.round(raw)}, full_response_adc: 160, port: '/dev/ttyMOCK',
          history_age_s: 0, history: detectorHistory.map(p => ({age_s: +(t - p.at).toFixed(3), adc: p.adc, filtered_adc: p.filtered_adc, angle_deg: p.angle_deg, timestamp_s: p.timestamp_s}))}},
      probe: {depth_mm: mock.depth, target_mm: mock.cycle ? 60 : 0, max_depth_mm: 110, pressure_ratio: mock.cycle ? 0.1 + mock.depth / 200 : 0.05,
        ...(includeProbeHistory ? {contact_samples:probeHistory.map(p => ({x:p.at-t, load:p.load, current:null, depth_mm:p.depth_mm})),
          history_time_s:(performance.timeOrigin+started)/1000+t, history_age_s:0} : {}), fault: false, ready: true, active: mock.cycle, cycle_active: mock.cycle, cycle_phase: mock.cyclePhase, cycle_goal_id: mock.inspection.probe_goal_id,
        contact_detected: mock.cycle && mock.pointResult?.status === 'CONTACT',
        contact_depth_mm: mock.cycle ? mock.pointResult?.contact_depth_mm : null, contact_volts: mock.cycle ? mock.pointResult?.contact_volts : null, holding: false,
        adc_contact_state: mockContact()},
      power: {battery: {available: true, present: true, stale: false, power_w: 38 + 6 * Math.sin(t / 3) + mock.linear * 40,
          current_a: 3.2, voltage_v: 11.9, remaining_pct: 72, temperature_c: 31, cell_delta_v: 0.012},
        history: Array.from({length: 30}, (_, i) => ({age_s: i * 2, watts: 38 + 6 * Math.sin((t - i * 2) / 3)})),
        esc: {available: true, stale: false, motors: [{id: 1, online: true, power_w: 12, current_a: 1, temperature_c: 34, rpm: 0, faults: 0}]}},
      actuators: {arm: {connected: true, arm_calibration: true, servo_id: 1, voltage: 11.8, current_a: 0.12, temperature_c: 33},
        probe: {connected: true, servo_id: 2, serial_connected: true, position: 2048, goal_position: 2048, torque: false, reverse: false, position_mode: 'multi-turn',
          homing: mock.homing > 0, home_position: mock.homed ? 2048 : null, home_status: mock.homed ? 'Homed at top' : mock.homing ? 'Homing…' : 'Not homed', probe: {calibrated: true, max_depth_mm: 110}}},
      settings: structuredClone(mock.settings),
      inspection: structuredClone(mock.inspection),
      cameras: {camera_probe: {label: 'camera / probe', topic: '/camera/probe/camera_info', fps: 15, frame_age_ms: 60, width: 944, height: 704}},
    };
  }

  class MockSocket extends EventTarget {
    constructor(url) {
      super();
      this.url = url; this.readyState = 0;
      setTimeout(() => {
        this.readyState = 1; this.onopen?.({});
        this.timer = setInterval(() => this.emit(telemetry()), 100);
      }, 300);
    }
    emit(message) { if (this.readyState === 1) this.onmessage?.({data: JSON.stringify(message)}); }
    send(raw) {
      let m; try { m = JSON.parse(raw); } catch { return; }
      if (m.type === 'latency_ping') return setTimeout(() => this.emit({type: 'latency_pong', probe_id: m.probe_id}), 18);
      if (m.type === 'take_control') { mock.owner = true; mock.armed = true; stopPump(); return; }
      if (m.type === 'release_control' || m.type === 'estop') { mock.owner = mock.armed = false; mock.linear = mock.angular = 0; mock.sweep = false; stopPump(); return; }
      if (!mock.owner) return;
      if (m.type === 'pump_mark' && !mock.calibrating && !mock.inspection.active && !mock.cycle && pump.mode === 'idle') {
        const settings = mock.settings.marking, fix = mock.fieldFix();
        Object.assign(pump, {mode:'shot', output:settings.active_percent,
          until:performance.now() + settings.duration_s * 1000, lastRequest:m.request_id});
        const within = fix && Number.isFinite(fix.horizontal_accuracy_m) && fix.horizontal_accuracy_m <= mock.settings.gnss.target_accuracy_m;
        previewLog({id:m.request_id || String(Date.now()), timestamp:Date.now()/1000,
          kind:'pump_mark', status:'SHOT COMMANDED', statusType:'marked', ...settings, ...fix,
          x:mock.pose.x, y:mock.pose.y, heading_deg:mock.pose.yaw * 180 / Math.PI,
          gps_status:within ? 'within_target' : fix ? 'uncertain' : 'unavailable'});
        return;
      }
      if (m.type === 'pump_manual') {
        const now = performance.now();
        if (m.action === 'start' && pump.mode === 'idle')
          Object.assign(pump, {mode:'manual', output:m.percent, until:now+250});
        if (pump.mode === 'manual' && m.action === 'renew' && now < pump.until) pump.until = now+250;
        if (pump.mode === 'manual' && m.action === 'stop') stopPump();
        return;
      }
      if (m.type === 'adc' && m.action === 'settings') {
        Object.assign(adcConfig, m.settings.config);
        routes = m.settings.routes;
        probeTrigger = m.settings.probe_trigger;
        adcFiltered.clear();
        resetContact();
        adcRevision++;
        this.emit({type:'adc_result',ok:true,message:'Applied in preview.'});
        return;
      }
      if (m.type === 'settings' && ['driving','display','cameras','gnss','marking','inspection','probe_alignment','probe_scan','probe_stroke'].includes(m.section)) {
        mock.settings[m.section] = m.value;
        this.emit({type:'settings_result',section:m.section,ok:true,message:'Saved in preview.'});
        return;
      }
      if (m.type === 'clear_log') mock.fieldEntries.length = 0;
      if (m.type === 'log_pose') previewLog({status:'POSE LOGGED'});
      if (m.type === 'drive' && mock.owner && !mock.calibrating) {
        mock.linear = m.deadman ? m.linear_x : 0;
        mock.angular = m.deadman ? m.angular_z : 0;
        if ((mock.linear || mock.angular) && (mock.inspection.active || mock.inspection.confirmed_angle_deg != null)) {
          if (mock.inspection.phase === 'PROBING') mock.cycle = false;
          Object.assign(mock.inspection, {phase:'STOPPED', active:false, confirmed_angle_deg:null, message:'Vehicle moved. Confirm the target again before probing.'});
        }
        return;
      }
      if (m.type === 'align_probe' && !mock.inspection.active) {
        const angle = m.target_bearing_deg - mock.settings.probe_alignment.offset_deg;
        if (Number.isFinite(angle) && Math.abs(angle) <= 45) {
          mock.armAngle = angle;
          mock.sweep = false;
        }
        return;
      }
      if (m.type === 'sweep_enabled') { mock.sweep = !!m.enabled; return; }
      if (m.type === 'calibration_begin' || m.type === 'calibration_end') { stopPump(); mock.calibrating = m.type === 'calibration_begin'; mock.homing = 0; return; }
      if (m.type === 'arm_servo' && m.role === 'probe' && m.action === 'home' && m.held) { mock.calibrating = true; if (++mock.homing > 15) mock.homed = true; return; }
      if (m.type === 'arm_servo' && m.role === 'probe' && m.action === 'home_release') { mock.homed = true; mock.homing = 0; return; }
      if (m.type === 'arm_servo' && m.action === 'stop') { mock.homing = 0; return; }
      if (m.type === 'inspection') {
        const t = (performance.now() - started) / 1000;
        if (m.action === 'stop') Object.assign(mock.inspection, {phase: 'STOPPED', active: false, message: 'Inspection stopped.'}), mock.cycle = false, mock.depth = 0;
        if (m.action === 'confirm') Object.assign(mock.inspection, {phase: 'CONFIRMING', active: true, total: mock.settings.inspection.passes, completed: 0, progress: 0, points: [], probe_angles_deg: [], probe_target_depth_mm: null, probe_target_bearing_deg:null, probe_arm_center_deg:null, scan_offset_deg:null, scan_point_count:null, scan_spacing_deg:null, message: ''});
        if (m.action === 'probe_scan') {
          const bearing = mock.inspection.confirmed_angle_deg, offset = mock.settings.probe_alignment.offset_deg;
          const center = bearing - offset;
          if (!Number.isFinite(bearing) || Math.abs(center) > 45) return;
          const {point_count, spacing_deg} = mock.settings.probe_scan;
          const angles = [...new Set(Array.from({length:point_count}, (_, i) => {
            const angle = center + (i - (point_count - 1) / 2) * spacing_deg;
            return Number(Math.max(-45, Math.min(45, angle)).toFixed(4));
          }))];
          mock.pointResult = null;
          mock.pointAt = t + 1;
          Object.assign(mock.inspection, {phase:'PROBING', active:true, total:angles.length, completed:0, progress:0,
            points:[], probe_angles_deg:angles.map(angle => angle+offset), probe_target_bearing_deg:bearing,
            scan_point_count:point_count, scan_spacing_deg:spacing_deg,
            probe_arm_center_deg:center, scan_offset_deg:offset, probe_target_depth_mm:m.depth_mm, message:''});
        }
        mock.homing = mock.inspection.active ? 1 : 0;
        mock.phaseAt = t;
      }
    }
    close() { clearInterval(this.timer); if (this.readyState === 3) return; this.readyState = 3; this.onclose?.({}); }
  }
  for (const k of ['CONNECTING', 'OPEN', 'CLOSING', 'CLOSED']) MockSocket[k] = RealWebSocket[k];

  // /api/adc: ADS1115 sample batches (detector on MUX 6, probe contact on MUX 4).
  const adcConfig = {bus: 1, address: 72, rate: 128, pga: 1, mode: 'single', alert: 'disabled', polarity: 0, latch: false,
    queue: 1, low: -1000, high: 1000, interval_ms: 0, filter_ms: 100,
    channels: Array.from({length: 8}, (_, i) => ({mux: i, enabled: i === 4 || i === 6, multiplier: 1, offset: 0}))};
  let adcRevision = 1;
  let routes = ['none','none','none','none','probe','none','detector','none'];
  let probeTrigger = {enabled:true, mux:4, level:1, direction:'above', hysteresis:0, debounce_ms:0};
  const adcFiltered = new Map();
  let contactDetected = false, candidate = null, candidateSince = 0;
  function resetContact() { contactDetected = false; candidate = null; }
  function sampleContact(scaled, t) {
    const sign = probeTrigger.direction === 'above' ? 1 : -1;
    const next = sign*(scaled-probeTrigger.level) >= (contactDetected ? -probeTrigger.hysteresis : 0);
    if (next !== candidate) { candidate = next; candidateSince = t; }
    if ((t-candidateSince)*1000 >= probeTrigger.debounce_ms) contactDetected = next;
  }
  function mockContact() {
    const channel = adcConfig.channels[probeTrigger.mux], filtered = adcFiltered.get(probeTrigger.mux);
    const fresh = probeTrigger.enabled && channel.enabled && Number.isFinite(filtered);
    return {...probeTrigger, fresh, filtered_volts:fresh ? filtered : null,
      value:fresh ? filtered*channel.multiplier+channel.offset : null, detected:fresh ? contactDetected : null};
  }
  let adcSeq = 0, adcLast = performance.now() / 1000;
  const fieldTrail = [];
  mock.fieldFix = () => ({latitude: 59.35 + mock.pose.y / 111320, longitude: 18.07 + mock.pose.x / 56750,
    horizontal_accuracy_m: .4, satellites: 16, fix: 'Preview fix'});
  const realFetch = window.fetch.bind(window);
  window.fetch = (input, init) => {
    const url = String(input?.url ?? input);
    if (url.startsWith('/api/field')) {
      const fix = mock.fieldFix();
      const last = fieldTrail.at(-1);
      if (fix && (!last || last[1] !== fix.latitude || last[2] !== fix.longitude)) {
        fieldTrail.push([Date.now()/1000, fix.latitude, fix.longitude]);
        if (fieldTrail.length > 900) fieldTrail.shift();
      }
      const revision = JSON.stringify(mock.fieldEntries.map(entry => [entry.id, entry.gps_status]));
      return Promise.resolve(Response.json({device:'Preview', receiver:fix?.fix || 'No fix', fix,
        trail:fieldTrail, entries:mock.fieldEntries, revision}));
    }
    if (!url.startsWith('/api/adc')) return realFetch(input, init);
    const now = performance.now() / 1000, samples = [];
    for (let t = adcLast + 0.02; t <= now; t += 0.02) {
      const volts = detectorVoltage(t - started / 1000);
      // A physical fixture presses the preview probe sensor on the third stroke.
      const pressing = mock.inspection.phase === 'PROBING' && mock.inspection.completed === 2 &&
        mock.depth >= mock.inspection.probe_target_depth_mm * .4;
      for (const ch of adcConfig.channels.filter(ch => ch.enabled)) {
        const value = ch.mux === 6 ? volts : (ch.mux === 4 && pressing ? 1.5 : .2);
        const old = adcFiltered.get(ch.mux);
        const filtered = old == null || !adcConfig.filter_ms ? value : old - Math.expm1(-20 / adcConfig.filter_ms) * (value-old);
        adcFiltered.set(ch.mux, filtered);
        if (ch.mux === probeTrigger.mux) sampleContact(filtered*ch.multiplier+ch.offset, t);
        samples.push({seq:++adcSeq, time:performance.timeOrigin/1000+t, mux:ch.mux, raw:Math.round(value*8000), volts:value, scaled:value*ch.multiplier+ch.offset,
          filtered_volts:filtered, filtered_scaled:filtered*ch.multiplier+ch.offset, clipped:false});
      }
      adcLast = t;
    }
    const body = {session: 'mock', revision: adcRevision, applied: adcRevision, running: true, connected: true, demo: true, error: null,
      registers: {config: 0x8583}, now:performance.timeOrigin/1000+now, sequence: adcSeq, config: adcConfig,
      routes,
      probe_trigger: probeTrigger,
      probe_contact: mockContact(),
      samples};
    return Promise.resolve(new Response(JSON.stringify(body), {headers: {'Content-Type': 'application/json'}}));
  };

  window.WebSocket = function (url, protocols) {
    return /\/ws$/.test(String(url)) ? new MockSocket(url) : new RealWebSocket(url, protocols);
  };
  Object.assign(window.WebSocket, {CONNECTING: 0, OPEN: 1, CLOSING: 2, CLOSED: 3});
})();
