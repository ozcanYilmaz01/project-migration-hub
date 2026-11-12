/* Car-owned GNSS trail and mark log. Tiles are optional. */
(() => {
  const el = id => document.getElementById(id);
  const format = (value, digits) => Number.isFinite(value) ? value.toFixed(digits) : '—';
  const hasPosition = fix => Number.isFinite(fix?.latitude) && Number.isFinite(fix?.longitude);
  const style = getComputedStyle(document.documentElement);
  const color = token => style.getPropertyValue(token).trim();
  let entries = [], revision = '', field = {}, configKey;

  function fieldMap(id, followId) {
    const container = el(id), followButton = el(followId), caption = el(id + '-status');
    let map, trace, car, errorCircle, marks, trailKey, markKey;
    let following = true, tilesOffline = false;

    function follow(enabled) {
      following = enabled;
      followButton.setAttribute('aria-pressed', String(enabled));
      if (enabled && map && hasPosition(field.fix)) map.panTo([field.fix.latitude, field.fix.longitude], {animate: false});
    }
    followButton.onclick = () => follow(!following);
    function status() {
      const accuracy = field.fix?.horizontal_accuracy_m;
      const detail = [field.receiver || 'Waiting for field recorder'];
      if (hasPosition(field.fix)) detail.push(Number.isFinite(accuracy) ? 'est. error ' + format(accuracy, 2) + ' m' : 'accuracy unknown');
      if (tilesOffline) detail.push('Base tiles offline');
      const label = detail.join(' · ');
      caption.textContent = label;
      caption.title = label;
    }
    function initialize(point) {
      if (!window.L || !point) return;
      map = L.map(container, {attributionControl: true, scrollWheelZoom: false}).setView(point, 19);
      L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxNativeZoom: 19, maxZoom: 22,
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>',
      }).on('tileerror', () => { tilesOffline = true; status(); })
        .on('tileload', () => { tilesOffline = false; status(); }).addTo(map);
      trace = L.polyline([], {color: color('--accent'), weight: 3}).addTo(map);
      marks = L.layerGroup().addTo(map);
      map.on('dragstart', () => follow(false));
      el(id + '-empty').hidden = true;
    }
    function draw() {
      status();
      if (!container.clientWidth || !container.clientHeight) return;
      const fix = field.fix, trail = field.trail || [];
      const point = hasPosition(fix) ? [fix.latitude, fix.longitude] : null;
      const savedMark = entries.find(item => item.kind === 'pump_mark' && hasPosition(item));
      if (!map) initialize(point || trail.at(-1)?.slice(1) || (savedMark && [savedMark.latitude, savedMark.longitude]));
      followButton.disabled = !point || !map;
      if (!map) return;
      // Split at acquisition gaps so disconnected segments do not imply travel.
      const key = JSON.stringify([trail.length, trail[0], trail.at(-1)]);
      if (key !== trailKey) {
        const segments = [];
        let previous;
        for (const [t, lat, lon] of trail) {
          if (!previous || t - previous > 35) segments.push([]);
          segments.at(-1).push([lat, lon]);
          previous = t;
        }
        trace.setLatLngs(segments);
        trailKey = key;
      }
      const last = point || trail.at(-1)?.slice(1);
      if (last) {
        if (!car) car = L.circleMarker(last, {radius: 5, fillOpacity: 1, weight: 2}).addTo(map);
        car.setLatLng(last).setStyle({color: color('--bg-sunken'), fillColor: color(point ? '--accent' : '--muted')});
      }
      const radius = fix?.horizontal_accuracy_m;
      if (point && Number.isFinite(radius)) {
        if (!errorCircle) errorCircle = L.circle(point, {radius, color: color('--accent'), weight: 1, fillOpacity: .1}).addTo(map);
        errorCircle.setLatLng(point).setRadius(radius).bringToBack();
      } else if (errorCircle) { errorCircle.remove(); errorCircle = null; }
      if (!last && car) { car.remove(); car = null; }
      if (point && following) map.panTo(point, {animate: false});
      if (markKey !== revision) {
        marks.clearLayers();
        // Every found target with a position goes on the map, numbered oldest first.
        entries.forEach((entry, index) => {
          if (!hasPosition(entry)) return;
          const number = entries.length - index, mark = entry.kind === 'pump_mark';
          const label = document.createElement('span');
          const error = Number.isFinite(entry.horizontal_accuracy_m) ? ' · est. error ' + format(entry.horizontal_accuracy_m, 2) + ' m' : ' · accuracy unknown';
          const gps = {within_target: 'within target', uncertain: 'uncertain', pending: 'waiting for accuracy'}[entry.gps_status] || entry.gps_status;
          label.textContent = 'Target ' + number + ' · ' + (mark ? 'ground marked' : (entry.status || 'logged').toLowerCase()) + (gps ? ' · ' + gps : '') + error;
          L.circleMarker([entry.latitude, entry.longitude], {color: color(mark ? '--bg-sunken' : '--warn'), fillColor: color('--warn'), weight: 2, radius: mark ? 7 : 6, fillOpacity: mark ? 1 : .35})
            .bindTooltip(String(number), {permanent: true, direction: 'top', className: 'target-pin'})
            .bindPopup(label).addTo(marks);
        });
        markKey = revision;
      }
    }
    new ResizeObserver(() => {
      if (container.clientWidth && container.clientHeight) {
        map?.invalidateSize({pan: false});
        draw();
      }
    }).observe(container);
    return draw;
  }

  const drawField = fieldMap('gnss-map', 'map-follow');
  const drawPreview = fieldMap('gnss-preview', 'gnss-preview-follow');
  el('map-mode').onchange = () => {
    const local = el('map-mode').value === 'local';
    el('map-view').hidden = !local;
    el('gnss-map').hidden = el('map-follow').hidden = el('gnss-map-status').hidden = local;
    if (!local) drawField();
  };

  function setup() {
    const policy = state.settings?.gnss;
    const key = JSON.stringify(policy);
    if (policy && configKey !== key) {
      el('gnss-wait').value = policy.max_wait_s;
      el('gnss-target').value = policy.target_accuracy_m;
    }
    configKey = key;
    el('gnss-save').disabled = !policy || !state.connected || !state.drive.you_control_owner;
    const editable = !el('gnss-save').disabled;
    el('gnss-wait').disabled = el('gnss-target').disabled = !editable;
    const label = field.fix?.fix || (field.receiver === 'Stale fix' ? 'STALE' : window.operatorFieldAvailable === false ? 'OFFLINE' : 'WAITING');
    el('gnss-fix-label').textContent = label;
    el('gnss-device').textContent = field.device || '—';
    el('gnss-sats').textContent = field.fix?.satellites ?? '—';
    el('gnss-lat').textContent = format(field.fix?.latitude, 7);
    el('gnss-lon').textContent = format(field.fix?.longitude, 7);
    el('gnss-accuracy').textContent = Number.isFinite(field.fix?.horizontal_accuracy_m) ? format(field.fix.horizontal_accuracy_m, 2) + ' m' : '—';
  }
  el('gnss-save').onclick = () => {
    const wait = el('gnss-wait'), target = el('gnss-target');
    if (!wait.reportValidity() || !target.reportValidity() || !state.connected || !state.drive.you_control_owner) return;
    send({type: 'settings', section: 'gnss', value: {max_wait_s: wait.valueAsNumber, target_accuracy_m: target.valueAsNumber}});
    el('gnss-feedback').textContent = 'Saving GNSS settings…';
  };
  window.operatorGnssResult = message => { el('gnss-feedback').textContent = message.message || (message.ok ? 'Saved' : 'Settings rejected'); };
  async function poll() {
    try {
      const query = new URLSearchParams({revision});
      const previous = field.trail || [];
      if (previous.length) query.set('trail_after', previous.at(-1)[0]);
      const response = await fetch('/api/field?' + query, {cache: 'no-store', signal: AbortSignal.timeout(4000)});
      if (!response.ok) throw Error();
      field = await response.json();
      if (Number.isFinite(field.trail_from)) field.trail = previous.filter(p => p[0] >= field.trail_from).concat(field.trail);
      window.operatorFieldAvailable = field.receiver !== 'Field recorder offline';
      if (Array.isArray(field.entries)) {
        entries = field.entries;
        revision = field.revision;
        window.operatorFieldLog?.(entries);
      }
    } catch {
      window.operatorFieldAvailable = false;
      field = {...field, receiver: 'Field recorder offline', fix: null};
    }
    try { drawField(); drawPreview(); setup(); } finally { setTimeout(poll, 1000); }
  }
  poll();
})();
