/* PMB3 marking controls. The operator node owns both output timeouts. */
(() => {
  const fields = {duration_s: $('marking-duration'), idle_percent: $('marking-idle'),
    active_percent: $('marking-active'), manual_percent: $('marking-manual')};
  const hold = $('marking-hold');
  let signature = '', renewal = null, saving = false;

  function release() {
    if (renewal === null) return;
    clearInterval(renewal);
    renewal = null;
    send({type: 'pump_manual', action: 'stop'});
    updateMarkingSettings();
  }

  function manualBlock() {
    if (!state.connected) return 'Connect to the operator stack.';
    if (!state.drive.you_control_owner) return 'Take control to test the pump.';
    const authority = pumpAuthorityBlock();
    if (authority) return authority;
    if (state.marking.mode === 'shot') return 'Wait for the timed shot to finish.';
    return '';
  }

  window.updateMarkingSettings = () => {
    const settings = state.marking?.settings;
    const key = JSON.stringify(settings);
    if (settings && signature !== key) {
      signature = key;
      for (const [name, input] of Object.entries(fields)) input.value = settings[name];
    }
    const owner = state.connected && state.drive.you_control_owner;
    const blocked = manualBlock();
    if (blocked || $('settings-marking').hidden || !$('settings-dialog').open) release();
    $('marking-take-control').disabled = !state.connected || owner;
    $('marking-save').disabled = !owner || !settings || !state.settings?.marking || saving || renewal !== null;
    $('marking-output').textContent = state.connected && Number.isFinite(state.marking?.output_percent)
      ? `${state.marking.output_percent}%` : '—';
    $('marking-status').textContent = blocked || (state.marking.mode === 'manual' ? 'Manual test active' : 'Ready');
    hold.disabled = Boolean(blocked);
    hold.title = blocked;
    hold.textContent = renewal === null ? 'Hold to test' : 'Release to stop';
    if (!state.connected) saving = false;
  };

  function start() {
    if (renewal !== null || hold.disabled || !fields.manual_percent.reportValidity()) return;
    const percent = fields.manual_percent.valueAsNumber;
    if (!Number.isFinite(percent)) return;
    send({type: 'pump_manual', action: 'start', percent});
    renewal = setInterval(() => {
      if (manualBlock() || $('settings-marking').hidden || !$('settings-dialog').open) return release();
      send({type: 'pump_manual', action: 'renew'});
    }, 80);
    updateMarkingSettings();
  }
  hold.onpointerdown = event => {
    if (event.button !== 0 || !event.isPrimary) return;
    event.preventDefault();
    hold.setPointerCapture(event.pointerId);
    start();
  };
  hold.onpointerup = hold.onpointercancel = hold.onlostpointercapture = hold.onblur = release;
  hold.onkeydown = event => {
    if (![' ', 'Enter'].includes(event.key)) return;
    event.preventDefault();
    if (!event.repeat) start();
  };
  hold.onkeyup = event => { if ([' ', 'Enter'].includes(event.key)) release(); };
  window.addEventListener('blur', release);
  window.addEventListener('pagehide', release);
  document.addEventListener('visibilitychange', () => { if (document.hidden) release(); });
  $('settings-dialog').addEventListener('close', release);
  document.querySelector('.setup-nav').addEventListener('click', release);
  $('marking-take-control').onclick = () => send({type: 'take_control'});
  $('marking-save').onclick = () => {
    const value = {};
    for (const [name, input] of Object.entries(fields)) {
      if (!input.reportValidity()) return;
      value[name] = input.valueAsNumber;
    }
    saving = true;
    $('marking-feedback').textContent = 'Saving…';
    send({type: 'settings', section: 'marking', value});
    updateMarkingSettings();
  };
  window.markingSettingsResult = message => {
    saving = false;
    $('marking-feedback').textContent = message.message || (message.ok ? 'Saved on car.' : 'Settings rejected.');
    updateMarkingSettings();
  };
  updateMarkingSettings();
})();
