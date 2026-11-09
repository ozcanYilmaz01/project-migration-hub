"""Exercise the actual map, geographic markers and GNSS Setup without external tiles."""
from pathlib import Path
import re
import shutil
import subprocess
from tempfile import TemporaryDirectory
import unittest

DASHBOARD = Path(__file__).resolve().parents[1] / 'dashboard'


@unittest.skipUnless(shutil.which('chromium'), 'requires Chromium')
class BrowserFieldTest(unittest.TestCase):
    def test_field(self):
        page = (DASHBOARD / 'index.html').read_text().split('  <script>')[0]
        page = re.sub(r'<link rel="stylesheet" href="assets/([^"]+)">', lambda match:
                      '<style>' + (DASHBOARD / match[1]).read_text() + '</style>', page)
        page += '<script>' + (DASHBOARD / 'leaflet.js').read_text() + '</script>'
        page += """<script>
        // Render the full Leaflet vector map, while keeping every tile request offline.
        L.TileLayer.prototype.createTile = () => document.createElement('canvas');
        const state = {settings:{gnss:{target_accuracy_m:1,max_wait_s:5}}, connected:true, drive:{you_control_owner:false}};
        const logs = []; window.operatorFieldLog = rows => logs.push(rows);
        let sent; const send = message => { sent = message; };
        let next, data = {fix:null,trail:[],receiver:'Waiting for receiver',entries:[],revision:'a'};
        window.fetch = async () => Response.json(data);
        let ready; const initialized = new Promise(resolve => ready=resolve);
        window.setTimeout = (fn, delay) => { if (delay === 1000) { next=fn; ready(); } };
        </script>"""
        page += '<script>' + (DASHBOARD / 'field-map.js').read_text() + '</script>'
        page += """<script>window.addEventListener('load', async () => {
          const el = id => document.getElementById(id);
          const check = (ok, message) => { if (!ok) throw Error(message); };
          try {
            await initialized;
            check(!el('gnss-map-empty').hidden && el('map-follow').disabled, 'initial missing fix');
            const dialog = el('settings-dialog'), preview = el('gnss-preview'), panel = el('settings-gnss');
            dialog.showModal();
            for (const item of dialog.querySelectorAll('[data-settings-panel]')) {
              item.hidden = item !== panel;
              item.classList.toggle('active', item === panel);
            }
            await next();
            check(!el('gnss-preview-empty').hidden && el('gnss-preview-follow').disabled, 'preview initially has no fix');
            check(window.operatorFieldAvailable === true, 'missing receiver does not imply unavailable recorder');
            data = {fix:{latitude:0,longitude:0,horizontal_accuracy_m:.42,fix:'RTK float',satellites:20},
              trail:[[1,0,0],[2,0,.00001]],device:'/dev/ttyAMA0',receiver:'RTK float',revision:'b',
              entries:[{id:'m',kind:'pump_mark',latitude:0,longitude:0,gps_status:'uncertain',horizontal_accuracy_m:4}]};
            await next();
            check(el('gnss-map-empty').hidden && !el('map-follow').disabled, 'first zero-coordinate fix initializes map');
            check(el('gnss-map').querySelectorAll('.leaflet-overlay-pane svg path').length >= 4, 'trail, car, radius and ground mark are drawn');
            check(el('gnss-accuracy').textContent==='0.42 m' && logs.at(-1)[0].id==='m', 'accuracy and shared log');
            check(preview.querySelectorAll('.leaflet-overlay-pane svg path').length >= 4 && el('gnss-preview-empty').hidden, 'Setup shows fix, error circle, trail and mark');
            preview.querySelector('.leaflet-overlay-pane svg path:last-child').dispatchEvent(new MouseEvent('click', {bubbles:true}));
            check(preview.querySelector('.leaflet-popup-content')?.textContent.includes('uncertain'), 'Setup mark popup includes accuracy');
            el('gnss-preview-follow').click();
            check(el('gnss-preview-follow').getAttribute('aria-pressed')==='false' && el('map-follow').getAttribute('aria-pressed')==='true', 'map controls are independent');
            check(el('gnss-save').disabled && el('gnss-wait').disabled, 'spectator');
            state.drive.you_control_owner=true; await next(); el('gnss-wait').value=0; el('gnss-save').click();
            check(sent?.section==='gnss' && sent.value.max_wait_s===0, 'zero wait submitted through settings');
            el('map-follow').click(); check(el('map-follow').getAttribute('aria-pressed')==='false', 'follow toggle');
            el('map-mode').value='local'; el('map-mode').onchange(); check(!el('map-view').hidden && el('gnss-map').hidden,'local grid fallback');
            data={fix:null,trail:data.trail,receiver:'Stale fix'}; await next();
            check(el('gnss-map-status').textContent==='Stale fix' && el('gnss-accuracy').textContent==='—','stale position');
            check(el('gnss-fix-label').textContent==='STALE' && preview.querySelectorAll('.leaflet-overlay-pane svg path').length===3, 'preview retains trail and marks and removes stale uncertainty');
            dialog.close(); await next(); dialog.showModal(); await next();
            check(preview.querySelectorAll('.leaflet-map-pane').length===1, 'reopening Setup reuses the map instance');
            window.fetch=async () => {throw Error('offline');}; await next();
            check(window.operatorFieldAvailable===false && el('gnss-preview-status').textContent.includes('offline'), 'lost recorder stays distinct from no GNSS fix');
            document.body.textContent='FIELD PASSED';
          } catch (error) { document.body.textContent='FIELD FAILED '+error.stack; }
        });</script>"""
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'field.html'
            path.write_text(page)
            process = subprocess.run(['chromium', '--headless', '--no-sandbox', '--disable-gpu',
                                      '--user-data-dir=' + directory + '/profile', '--virtual-time-budget=3000',
                                      '--dump-dom', path.as_uri()], capture_output=True, text=True, timeout=60)
        body = process.stdout.split('<body>')[-1]
        self.assertIn('FIELD PASSED</body>', body, body[-1500:] + process.stderr[-1000:])
