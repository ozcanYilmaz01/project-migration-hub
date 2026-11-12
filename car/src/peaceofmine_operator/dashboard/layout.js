// Presentation only: dock tabs, phase-aware camera focus and the header
// battery chip. Reads app.js globals (`state`); never sends commands.
(() => {
  const $ = id => document.getElementById(id);
  const tabs = [...document.querySelectorAll('[data-dock-tab]')];
  const panes = [...document.querySelectorAll('[data-dock-pane]')];

  /* ---------------------------------------------------------- Dock tabs --- */
  const workspace = document.querySelector('.workspace');
  const expandChart = $('detector-expand');
  function expandDetector(expanded) {
    workspace.classList.toggle('detector-expanded', expanded);
    expandChart.setAttribute('aria-pressed', String(expanded));
    expandChart.textContent = expanded ? 'Restore chart' : 'Expand chart';
    window.dispatchEvent(new Event('resize'));
  }
  expandChart.addEventListener('click', () => expandDetector(!workspace.classList.contains('detector-expanded')));
  document.addEventListener('keydown', event => {
    if (event.key === 'Escape' && !document.querySelector('dialog[open]') && workspace.classList.contains('detector-expanded')) {
      expandDetector(false);
      expandChart.focus();
    }
  });
  function selectTab(name, focus = false) {
    if (name !== 'detector' && workspace.classList.contains('detector-expanded')) expandDetector(false);
    tabs.forEach(tab => {
      const on = tab.dataset.dockTab === name;
      tab.classList.toggle('active', on);
      tab.setAttribute('aria-selected', String(on));
      tab.tabIndex = on ? 0 : -1;
      if (on && focus) tab.focus();
    });
    panes.forEach(pane => { pane.hidden = pane.dataset.dockPane !== name; });
    window.dispatchEvent(new Event('resize')); // charts size themselves on show
  }
  tabs.forEach((tab, i) => {
    tab.addEventListener('click', () => selectTab(tab.dataset.dockTab));
    tab.addEventListener('keydown', event => {
      const step = {ArrowRight: 1, ArrowLeft: -1}[event.key];
      if (!step) return;
      event.preventDefault();
      selectTab(tabs[(i + step + tabs.length) % tabs.length].dataset.dockTab, true);
    });
  });
  selectTab('overview');

  /* ------------------------------------------- Setup cross-links --- */
  // Jump to another Setup section without reopening the window.
  document.querySelectorAll('[data-goto-settings]').forEach(link => {
    link.addEventListener('click', () => document.querySelector(`[data-settings-tab="${link.dataset.gotoSettings}"]`)?.click());
  });

  /* ------------------------------------------------------------- Tips ---
   * Any element with data-tip gets one shared floating tooltip on hover or
   * keyboard focus. <span class="tip"> is the small (i) used next to titles.
   */
  const tooltip = document.createElement('div');
  tooltip.className = 'tooltip';
  tooltip.id = 'tooltip';
  tooltip.setAttribute('role', 'tooltip');
  tooltip.hidden = true;
  let tipTarget = null;
  document.querySelectorAll('[data-tip]').forEach(el => {
    if (el.classList.contains('tip')) {
      el.tabIndex = 0;
      el.setAttribute('role', 'img');
      el.setAttribute('aria-label', el.dataset.tip);
    } else {
      el.setAttribute('aria-description', el.dataset.tip);
    }
  });
  function showTip(el) {
    tipTarget = el;
    tooltip.textContent = el.dataset.tip;
    // a modal dialog sits in the top layer, so the tooltip must live inside it
    const host = el.closest('dialog[open]') || document.body;
    if (tooltip.parentElement !== host) host.append(tooltip);
    tooltip.hidden = false;
    const r = el.getBoundingClientRect();
    const t = tooltip.getBoundingClientRect();
    const margin = 8;
    let top = r.bottom + 6;
    if (top + t.height > innerHeight - margin) top = r.top - t.height - 6;
    const left = Math.min(Math.max(margin, r.left + r.width / 2 - t.width / 2), innerWidth - t.width - margin);
    tooltip.style.top = `${Math.max(margin, top)}px`;
    tooltip.style.left = `${left}px`;
  }
  function hideTip(el) {
    if (el && el !== tipTarget) return;
    tipTarget = null;
    tooltip.hidden = true;
  }
  document.addEventListener('mouseover', event => {
    const el = event.target.closest?.('[data-tip]');
    if (el && el !== tipTarget) showTip(el);
    else if (!el && tipTarget && document.activeElement !== tipTarget) hideTip();
  });
  document.addEventListener('focusin', event => { const el = event.target.closest?.('[data-tip]'); if (el) showTip(el); else hideTip(); });
  document.addEventListener('focusout', event => hideTip(event.target.closest?.('[data-tip]')));
  document.addEventListener('scroll', () => hideTip(), true);
  document.addEventListener('keydown', event => { if (event.key === 'Escape') hideTip(); });
  document.addEventListener('close', () => hideTip(), true);

  /* ------------------------------------------------------- Mission stage ---
   * Drive & search: forward camera large, probe camera small, detector chart.
   * Probe (confirmed target, scan, or a probe cycle): probe camera large and
   * probe load in the dock. Changes only on stage transitions, so a manual
   * swap or tab choice stays until the mission moves on.
   */
  const stageEl = $('camera-stage');
  let stage = null;
  let manualFocus = null;
  function missionStage() {
    const phase = state.inspection?.phase || 'IDLE';
    if (state.probe?.cycle_active || ['CONFIRMED', 'PROBING', 'COMPLETE'].includes(phase)) return 'probe';
    if (phase === 'CONFIRMING') return 'confirm';
    return 'drive';
  }
  // The probe camera stays a corner thumbnail; only a click enlarges it.
  function applyFocus() {
    document.body.dataset.stage = stage;
    stageEl.dataset.focus = 'forward';
    const large = manualFocus === 'probe' && probeCameraShown();
    if ((stageEl.dataset.probeSize === 'large') === large) return;
    stageEl.dataset.probeSize = large ? 'large' : 'small';
    window.dispatchEvent(new Event('resize'));
  }
  function setProbeLarge(on) { manualFocus = on ? 'probe' : null; applyFocus(); }
  $('probe-camera-panel').addEventListener('click', event => {
    if (event.target.closest('button, a, input, select')) return;
    setProbeLarge(stageEl.dataset.probeSize !== 'large');
  });
  document.addEventListener('keydown', event => {
    if (event.key === 'Escape' && stageEl.dataset.probeSize === 'large' && !document.querySelector('dialog[open]')) setProbeLarge(false);
  });

  /* ------------------------------------------------------- Probe camera --- */
  function probeCameraShown() {
    const video = $('aux-video'), image = $('aux-network-camera');
    return (!video.classList.contains('hidden') && video.readyState >= 2) ||
      (!image.classList.contains('hidden') && image.naturalWidth > 0 && !cameraErrors.auxiliary);
  }

  /* ------------------------------------------------------ Battery chip --- */
  const chip = $('power-chip');
  chip.addEventListener('click', () => { setDetails(true); selectTab('drive'); });
  function renderChip() {
    const battery = state.power?.battery || {};
    const live = state.connected && battery.available && !battery.stale && battery.present;
    const pct = battery.remaining_pct, watts = battery.power_w;
    $('power-chip-pct').textContent = live && Number.isFinite(pct) ? `${Math.round(pct)}%` : '—';
    $('power-chip-w').textContent = live && Number.isFinite(watts) ? `${watts.toFixed(0)} W` : 'No battery data';
    chip.dataset.level = !live || !Number.isFinite(pct) ? 'none' : pct < 20 ? 'low' : pct < 40 ? 'mid' : 'ok';
  }

  /* ------------------------------------------- Ready-to-drive checklist --- */
  // Reads the hidden status badges that app.js keeps up to date.
  const remoteHelp = {
    override: "Flip the remote's switch to ROS to allow driving",
    kill: "Release the kill switch on the remote",
    rc_lost: "Turn on the remote and keep it in range",
    ros_disarmed: "Arm the car on the remote",
  };
  function renderReady() {
    const connected = !!state.connected;
    const remote = connected && $('rc-safety').classList.contains('ready');
    const owner = connected && $('control-owner').classList.contains('safe');
    const items = {connected, remote, owner};
    for (const el of document.querySelectorAll('[data-ready-item]')) el.dataset.ok = String(items[el.dataset.readyItem]);
    const next = !connected ? 'Waiting for the car to connect…'
      : !remote ? (remoteHelp[state.safety?.mode] || "Flip the remote's switch to ROS to allow driving")
      : !owner ? ($('take-control').textContent.includes('Take over') ? 'Someone else is driving. Press Take over driving' : 'Press Start driving')
      : 'Ready to drive';
    const el = $('ready-next');
    if (el.textContent !== next) el.textContent = next;
    el.dataset.ok = String(connected && remote && owner);
  }

  /* ------------------------------------------------- Guided mission flow --- */
  // Shows only the current step. Reads the enabled state app.js sets; sends nothing.
  const steps = ['sweep', 'align', 'probe', 'mark'];
  const explain = {
    sweep: "Drive slowly. The arm swings the detector and we'll alert you when it finds metal.",
    align: "Stop over the hit. The arm sweeps it a few more times to make sure it's real.",
    probe: "The probe gently pushes into the ground at a few spots around the hit.",
    mark: "The pump sprays a mark on the ground so the spot can be found again.",
  };
  let pickedStep = null, autoStep = null, hitSeen = false, lastSweep = false;
  function sweepRestarted() { // sweeping again after a hit means the operator went back to searching
    const on = !!state.detector?.sweep_enabled, restarted = on && !lastSweep;
    lastSweep = on;
    return restarted && !document.body.dataset.searchAlert;
  }
  for (const dot of document.querySelectorAll('[data-mission-step]'))
    dot.addEventListener('click', () => { pickedStep = dot.dataset.missionStep; if (pickedStep === 'sweep') hitSeen = false; renderMission(); });
  $('mission-open-settings').addEventListener('click', () => $('settings').click());
  function missionAutoStep() {
    const phase = state.inspection?.phase || 'IDLE';
    if (phase === 'COMPLETE') return 'mark';
    if (state.probe?.cycle_active || ['CONFIRMED', 'PROBING'].includes(phase)) return 'probe';
    if (phase === 'CONFIRMING') return 'align';
    // A search alert (set by app.js on <body>) means metal was found: move on to Check.
    if (sweepRestarted()) hitSeen = false;
    if (document.body.dataset.searchAlert) hitSeen = true;
    return hitSeen ? 'align' : 'sweep';
  }
  function missionSentence(step, btn) {
    const pill = $(`mode-${step}-pill`).textContent.trim();
    if (btn.disabled) {
      const reason = (btn.title || '').trim() || 'Not available yet.';
      return {text: reason, kind: 'blocked', settings: /set ?up|calibrat|configure|settings/i.test(reason)};
    }
    if (step === 'sweep') return state.detector?.sweep_enabled ? {text: 'Searching… press the button again to stop.', kind: 'running'} : {text: 'Ready when you are.', kind: 'ready'};
    if (step === 'align') return state.inspection?.phase === 'CONFIRMING' ? {text: `Checking pass ${pill}…`, kind: 'running'} : {text: 'Ready. Stop the car over the hit first.', kind: 'ready'};
    if (step === 'probe') return pill === 'RUNNING' ? {text: 'Probing… keep the car still.', kind: 'running'} : {text: 'Ready. Keep the car still while it probes.', kind: 'ready'};
    return pill === 'COMMAND ACTIVE' ? {text: 'Marking…', kind: 'running'} : {text: `Ready. The pump runs for ${pill.replace(' s', ' seconds')}.`, kind: 'ready'};
  }
  function renderMission() {
    const auto = missionAutoStep();
    if (auto !== autoStep) { autoStep = auto; pickedStep = null; } // a finished step moves the flow on
    const current = pickedStep || auto;
    const list = $('mission-steps');
    if (list.dataset.current !== current) list.dataset.current = current;
    const index = steps.indexOf(current), autoIndex = steps.indexOf(auto);
    for (const dot of document.querySelectorAll('[data-mission-step]')) {
      const i = steps.indexOf(dot.dataset.missionStep), li = dot.parentElement;
      li.dataset.current = String(i === index);
      li.dataset.done = String(i < autoIndex);
      dot.setAttribute('aria-current', i === index ? 'step' : 'false');
    }
    const btn = $(`mode-${current}-btn`);
    if ($('mission-explain').textContent !== explain[current]) $('mission-explain').textContent = explain[current];
    const next = missionSentence(current, btn);
    const out = $('mission-next');
    if ($('mission-next-text').textContent !== next.text) $('mission-next-text').textContent = next.text;
    out.dataset.kind = next.kind;
    $('mission-open-settings').hidden = !next.settings;
  }

  /* ------------------------------------------------------ Details drawer --- */
  const detailsBtn = $('details-toggle');
  const DETAILS_KEY = 'pom.detailsOpen';
  function setDetails(open) {
    workspace.classList.toggle('details-open', open);
    detailsBtn.textContent = open ? 'Details ▾' : 'Details ▸';
    detailsBtn.setAttribute('aria-expanded', String(open));
    try { localStorage.setItem(DETAILS_KEY, open ? '1' : '0'); } catch (_) { /* storage blocked */ }
  }
  let detailsOpen = false;
  try { detailsOpen = localStorage.getItem(DETAILS_KEY) === '1'; } catch (_) { /* storage blocked */ }
  setDetails(detailsOpen);
  detailsBtn.addEventListener('click', () => setDetails(!workspace.classList.contains('details-open')));
  $('search-alert-signal').addEventListener('click', () => { setDetails(true); selectTab('detector'); });

  /* Words first; the raw numbers app.js writes stay hidden and move into the tip. */
  function setWords(key, text, tip, level) {
    for (const el of document.querySelectorAll(`[data-words-for="${key}"]`)) {
      if (el.textContent !== text) el.textContent = text;
      if (el.dataset.tip !== tip) el.dataset.tip = tip;
      el.dataset.level = level || '';
    }
  }
  function renderWords() {
    const raw = $('detector-voltage').textContent.trim();
    const volts = parseFloat($('signal').textContent);
    const alertLevel = parseFloat($('search-alert-level').value);
    if (!state.connected || !Number.isFinite(volts)) setWords('detector-voltage', 'No signal yet', 'No detector reading', '');
    else {
      const ratio = Number.isFinite(alertLevel) && alertLevel > 0 ? volts / alertLevel : 0;
      const [text, level] = ratio >= 1 ? ['Strong signal', 'strong'] : ratio >= .8 ? ['Getting stronger', ''] : ['Quiet · no metal', 'good'];
      const tip = `${raw} now${Number.isFinite(alertLevel) ? ` · alert at ${alertLevel} V` : ''}`;
      setWords('detector-voltage', text, tip, level);
      setWords('signal', text, tip, level);
    }
    if (!state.connected || !Number.isFinite(volts)) setWords('signal', 'No signal yet', 'No detector reading', '');
    const probe = state.probe || {};
    const depth = Number.isFinite(probe.depth_mm) ? Math.round(probe.depth_mm) : null;
    const probeTip = `Depth ${depth ?? '—'} mm · target ${Number.isFinite(probe.target_mm) ? Math.round(probe.target_mm) : '—'} mm`;
    setWords('probe', !state.connected ? 'Waiting for the probe'
      : probe.contact_detected && Number.isFinite(probe.contact_depth_mm) ? `Contact at ${Math.round(probe.contact_depth_mm)} mm`
      : probe.cycle_active ? `Probing · ${depth ?? 0} mm deep`
      : depth ? `Probe at ${depth} mm` : 'Probe up · no contact', probeTip, probe.contact_detected ? 'strong' : '');
    const b = state.power?.battery || {};
    const live = state.connected && b.available && !b.stale && b.present;
    const pct = b.remaining_pct;
    const battTip = ['battery-charge', 'battery-watts', 'battery-voltage', 'battery-current', 'battery-temperature', 'battery-cells']
      .map(id => `${{'battery-charge': 'Charge', 'battery-watts': 'Draw', 'battery-voltage': 'Voltage', 'battery-current': 'Current', 'battery-temperature': 'Temperature', 'battery-cells': 'Cell spread'}[id]} ${$(id).textContent}`).join(' · ');
    setWords('battery', !live || !Number.isFinite(pct) ? 'No battery data' : pct < 20 ? 'Battery low · charge soon' : pct < 40 ? 'Battery half empty' : 'Battery good',
      battTip, !live || !Number.isFinite(pct) ? '' : pct < 20 ? 'low' : pct < 40 ? 'strong' : 'good');
  }

  /* Stop probe on the main screen only while the probe moves; Advanced nav opens for its active tab. */
  function renderProbeLive() {
    const p = state.probe || {};
    const moving = !!p.cycle_active || (Number.isFinite(p.depth_mm) && Number.isFinite(p.target_mm) && Math.abs(p.depth_mm - p.target_mm) > 1);
    const live = $('probe-live');
    const show = state.connected && (moving || !$('probe-cycle-alert').classList.contains('hidden'));
    if (live.hidden === show) live.hidden = !show;
    const adv = $('setup-nav-advanced');
    if (!adv.open && adv.querySelector('.settings-tab.active')) adv.open = true;
  }

  const fullTextIds = ['controller', 'controller-mapping', 'drive-block', 'drive-help', 'detector-detail', 'position', 'connection', 'ready-next'];
  /* Mission panel extras: signal meter (alert line at 80%) and the three newest found targets. */
  function renderExtras() {
    const volts = parseFloat($('signal').textContent), alertLevel = parseFloat($('search-alert-level').value);
    const ok = state.connected && Number.isFinite(volts) && alertLevel > 0;
    const ratio = ok ? volts / alertLevel : 0;
    $('mission-meter-fill').style.width = Math.min(100, ratio * 80) + '%';
    $('mission-meter-fill').dataset.level = ratio >= 1 ? 'strong' : ratio >= .8 ? 'near' : '';
    $('meter-word').textContent = !ok ? 'No signal yet' : ratio >= 1 ? 'Metal!' : ratio >= .8 ? 'Getting stronger' : 'Quiet';
    const rows = [...document.querySelectorAll('#target-log-body tr:not(.empty-row)')].slice(0, 3);
    const key = rows.map(r => r.textContent).join('|');
    const list = $('recent-targets-list');
    if (list.dataset.key === key) return;
    list.dataset.key = key;
    list.replaceChildren(...(rows.length ? rows.map(r => {
      const c = r.cells, li = document.createElement('li');
      const id = document.createElement('strong'); id.textContent = c[0]?.textContent || '';
      const res = document.createElement('span'); res.textContent = c[5]?.textContent || '';
      const time = document.createElement('small'); time.textContent = (c[6]?.textContent || '').split(', ').pop();
      li.append(id, res, time); return li;
    }) : [Object.assign(document.createElement('li'), {className: 'recent-empty', textContent: 'None yet'})]));
  }
  $('recent-targets-all').addEventListener('click', () => { setDetails(true); selectTab('log'); });

  function tick() {
    renderExtras();
    const next = missionStage();
    if (next !== stage) {
      const entering = stage !== null;
      stage = next;
      if (entering) selectTab(stage === 'probe' ? 'probe' : stage === 'confirm' ? 'detector' : 'overview');
    }
    const off = preferences.cameras.auxiliary === 'off';
    stageEl.classList.toggle('single-camera', off);
    $('camera-swap').hidden = off;
    if (off) manualFocus = null;
    applyFocus();
    $('probe-camera-empty').hidden = probeCameraShown();
    keyHint.hidden = !$('input-lock').classList.contains('locked');
    renderChip();
    renderReady();
    renderMission();
    renderWords();
    renderProbeLive();
    // one-line status texts are cut with an ellipsis; keep the full text on hover
    for (const id of fullTextIds) { const el = $(id); if (el && el.title !== el.textContent) el.title = el.textContent; }
    for (const el of document.querySelectorAll('.readouts dd')) { if (el.title !== el.textContent) el.title = el.textContent; }
  }

  /* ------------------------------------------------------ Welcome guide --- */
  const guideSteps = [
    ['Get ready', 'Turn on the car and flip the remote\'s switch to ROS. Then press Start driving at the top.'],
    ['Drive and search', 'Click the camera and drive with W A S D. Press Search for metal and drive slowly.'],
    ['Check and mark', 'When the metal alert appears, stop. Follow the steps on the right to check, probe and mark the spot.'],
  ];
  const guide = $('welcome');
  let guideStep = 0;
  function renderGuide() {
    const [title, text] = guideSteps[guideStep];
    $('welcome-title').textContent = title;
    $('welcome-text').textContent = text;
    guide.querySelectorAll('[data-art]').forEach(svg => { svg.hidden = svg.dataset.art !== String(guideStep + 1); });
    guide.querySelectorAll('.welcome-dots li').forEach((li, i) => { li.dataset.on = i === guideStep; });
    $('welcome-back').hidden = guideStep === 0;
    $('welcome-next').textContent = guideStep === guideSteps.length - 1 ? 'Start' : 'Next';
  }
  function closeGuide() {
    try { localStorage.setItem('pom.welcomeSeen', '1'); } catch (_) { /* storage blocked */ }
    if (guide.open) guide.close();
  }
  function openGuide() { guideStep = 0; renderGuide(); if (!guide.open) guide.showModal(); }
  $('help-open').addEventListener('click', openGuide);
  $('welcome-skip').addEventListener('click', closeGuide);
  $('welcome-back').addEventListener('click', () => { guideStep = Math.max(0, guideStep - 1); renderGuide(); });
  $('welcome-next').addEventListener('click', () => {
    if (guideStep >= guideSteps.length - 1) { closeGuide(); return; }
    guideStep += 1; renderGuide();
  });
  guide.addEventListener('cancel', () => closeGuide());
  let seen = false;
  try { seen = localStorage.getItem('pom.welcomeSeen') === '1'; } catch (_) { seen = false; }
  if (!seen) openGuide();

  /* ------------------------------------------------------ WASD key hint --- */
  const keyHint = $('key-hint');
  function lightKey(event, on) {
    const k = keyHint.querySelector(`[data-key="${(event.key || '').toLowerCase()}"]`);
    if (k) k.dataset.on = on;
  }
  document.addEventListener('keydown', e => lightKey(e, true));
  document.addEventListener('keyup', e => lightKey(e, false));
  window.addEventListener('blur', () => keyHint.querySelectorAll('kbd').forEach(k => { k.dataset.on = false; }));

  /* ---------------------------------------------- Frozen video / link lost ---
   * Read-only: uses state.connected, state.cameras[].frame_age_ms, lastTelemetryAt
   * and the forward <video> playback time from app.js. */
  const fwdVideo = $('forward-video'), fwdImage = $('forward-network-camera');
  let videoTime = -1, videoChangedAt = performance.now(), lostAt = null;
  function forwardFrozen() {
    if (!state.connected) return false;
    if (!fwdVideo.classList.contains('hidden')) {
      if (fwdVideo.currentTime !== videoTime) { videoTime = fwdVideo.currentTime; videoChangedAt = performance.now(); }
      return fwdVideo.readyState >= 2 && performance.now() - videoChangedAt > 1000;
    }
    const source = preferences.cameras?.forward || '';
    if (!fwdImage.classList.contains('hidden') && source.startsWith('raspberry:')) {
      const age = state.cameras?.[source.slice(10)]?.frame_age_ms;
      if (!Number.isFinite(age)) return false;
      return age + Math.max(0, performance.now() - lastTelemetryAt) > 1000;
    }
    return false; // simulated view is drawn locally
  }
  function renderLink() {
    const frozen = forwardFrozen();
    $('video-frozen').hidden = !frozen;
    $('camera-panel').classList.toggle('is-frozen', frozen);
    if (state.connected) lostAt = null; else if (lostAt === null) lostAt = performance.now();
    const lost = lostAt !== null && performance.now() - lostAt > 1000;
    $('link-lost').hidden = !lost;
    document.body.dataset.link = lost ? 'lost' : 'ok';
    if (lost) {
      const secs = Math.floor((performance.now() - lostAt) / 1000);
      $('link-lost-time').textContent = secs < 60 ? `${secs} s ago` : `${Math.floor(secs / 60)} min ${secs % 60} s ago`;
    }
  }
  setInterval(renderLink, 200);
  tick();
  setInterval(tick, 250);
})();
