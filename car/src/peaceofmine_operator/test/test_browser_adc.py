"""Exercise real ADS1115 settings and Chart.js overlays in Chromium."""
import copy
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest
from peaceofmine_operator.ads1115 import DEFAULTS
from peaceofmine_operator.camera_settings import DEFAULTS as CAMERA_DEFAULTS

DASHBOARD = Path(__file__).resolve().parents[1] / 'dashboard'


@unittest.skipUnless(shutil.which('chromium'), 'Chromium required')
class BrowserADCTest(unittest.TestCase):
    def test_settings_routing_and_sample_batches(self):
        config = copy.deepcopy(DEFAULTS)
        for channel in config['channels']:
            channel['enabled'] = channel['mux'] in (4, 6)
        value = dict(config=config, routes=['none']*4+['probe','none','detector','none'],
                     revision=1, applied=1, running=True, connected=True, demo=True,
                     error=None, registers={'config': 0x8583}, now=100, sequence=2,
                     probe_trigger=dict(enabled=False,mux=4,level=1.,direction='above',hysteresis=0.,debounce_ms=0.),
                     samples=[dict(seq=1, time=99.9, mux=6, raw=100, volts=.1, scaled=2, filtered_scaled=2, filtered_volts=.1, clipped=False),
                              dict(seq=2, time=99.9, mux=4, raw=200, volts=.2, scaled=3, filtered_scaled=3, filtered_volts=.2, clipped=False)])
        setup = '''
window.requestAnimationFrame = () => 0;
window.setInterval = () => 0;
window.setTimeout = (fn) => { if(fn.name === 'poll') window.nextPoll = fn; return 0; };
window.WebSocket = class {static OPEN=1; readyState=1; send(data) {window.sent.push(JSON.parse(data));}};
window.sent=[];
window.fetch = async () => ({ok:true, json:async () => window.adcResponse});
'''
        checks = '''
const assert = (v,label) => {if(!v) throw Error(label);};
for(let i=0;i<10;i++) await Promise.resolve();
state.connected=true; state.drive.you_control_owner=true;
$('probe-diagnostics').open=true;
$('settings').click(); document.querySelector('[data-settings-tab="adc"]').click();
await window.nextPoll();
assert($('adc-status').textContent.includes('DEMO'), 'demo is explicit: '+$('adc-status').textContent+' '+$('adc-error').textContent);
const boardChart=Chart.getChart('adc-history');
assert($('adc-quantity').value==='filtered_volts' && boardChart.data.datasets.every(ds=>ds.label.includes('filtered (V)') && !ds.label.includes('unfiltered')), 'filtered voltage alone is the default');
let exportBlob;
const makeObjectURL=URL.createObjectURL, clickLink=HTMLAnchorElement.prototype.click;
URL.createObjectURL=blob=>{exportBlob=blob;return 'blob:csv-test';};HTMLAnchorElement.prototype.click=()=>{};
$('adc-csv').click();assert((await exportBlob.text()).split('\\n')[0]==='time,channel,filtered_volts','normal CSV contains only filtered physical voltage');
$('adc-quantity').value='comparison';$('adc-csv').click();
assert((await exportBlob.text()).split('\\n')[0]==='time,channel,filtered_volts,unfiltered_volts','comparison CSV adds explicitly named unfiltered voltage only');
URL.createObjectURL=makeObjectURL;HTMLAnchorElement.prototype.click=clickLink;
$('adc-quantity').value='filtered_volts';

assert(document.querySelectorAll('[data-adc-channel]').length === 8, 'all mux inputs');
assert(!document.querySelector('[data-adc-channel] select') && $('settings-contact').contains($('adc-trigger-mux')) && $('settings-detector').contains($('metal-adc-input')), 'sensor tabs own their assignments');
assert($('adc-trigger-mux').value==='4', 'default probe trigger uses A0');
assert($('adc-readings').textContent.includes('Live'), 'live readings');
const adcField=key=>document.querySelector('[data-adc-field="'+key+'"]');
const setAdcField=(key,value)=>{const input=adcField(key);input.value=String(value);input.dispatchEvent(new Event('input'));};
assert(adcField('low').valueAsNumber===-.125 && adcField('high').valueAsNumber===.125,'legacy ±1000 comparator register counts import as signed volts');
setAdcField('low',-.10004);setAdcField('high',.20021);
$('adc-apply').click();
assert(sent.at(-1).settings.config.low===-800 && sent.at(-1).settings.config.high===1602 && $('adc-high-volts').textContent.includes('0.20025 V'), 'entered signed voltages quantize to nearest code');
operatorADCResult({ok:true});setAdcField('pga',5);
assert(adcField('low').valueAsNumber===-.10004 && adcField('high').valueAsNumber===.20021,'changing PGA preserves requested volts');
setAdcField('low',-.256);setAdcField('high',.2559921875);$('adc-apply').click();
assert(sent.at(-1).settings.config.low===-32768 && sent.at(-1).settings.config.high===32767,'signed comparator range maps both asymmetric endpoints');
operatorADCResult({ok:true});const comparatorBlocked=sent.length;
setAdcField('high',.256);$('adc-apply').click();
assert(sent.length===comparatorBlocked && $('adc-feedback').textContent.includes('V'),'positive full-scale overflow rejected in volts');
setAdcField('low',-.2561);setAdcField('high',.2);$('adc-apply').click();
assert(sent.length===comparatorBlocked,'negative overflow rejected');
setAdcField('alert','window');setAdcField('low',.1);setAdcField('high',.100001);$('adc-apply').click();
assert(sent.length===comparatorBlocked && $('adc-feedback').textContent.includes('rounded'),'ordering tested after register quantization');
$('adc-revert').click();assert(adcField('low').valueAsNumber===-.125 && adcField('pga').value==='1','reload restores volts using the saved PGA');
const draftField=document.querySelector('[data-adc-field="rate"]');
draftField.value='64'; await window.nextPoll();
assert(draftField.value==='64', 'poll preserves edits');
$('adc-apply').click();
assert(sent.at(-1).settings.config.rate===64, 'settings sent');
assert(sent.at(-1).settings.routes[6]==='detector' && sent.at(-1).settings.routes[4]==='probe', 'A2 detector and A0 probe routing');
operatorADCResult({ok:true});
document.querySelectorAll('[data-dock-pane]').forEach(pane => { pane.hidden = false; }); // charts live in dock tabs
state.probe.history_time_s=100;
state.probe.contact_samples=[{x:-1.5,depth_mm:2},{x:-.1,depth_mm:11},{x:-.05,depth_mm:null},{x:0,depth_mm:13}];
drawManualProbeHistory(); drawPressureHistory(); drawDetectorDiagnostics(1000);
const [voltageTrace, savedLimit, measuredDepth] = manualProbeChart.data.datasets;
assert(voltageTrace.label.includes('A0') && voltageTrace.data[0].y===.2,'Manual probe plots selected filtered physical volts');
assert(manualProbeChart.options.scales.volts.title.text.includes('(V)') && manualProbeChart.options.scales.depth.title.text.includes('(mm)') && manualProbeChart.data.datasets.length===3,'Manual probe has one voltage axis and one depth axis');
assert(probePressureChart.data.datasets.length===2,'optional motor chart contains only load and current');
assert(savedLimit.data[0].y===1 && savedLimit.label.includes('off'),'saved disabled threshold remains identified and visible');
assert(measuredDepth.data.some((p,i,a)=>i && i+1<a.length && p.y===null) && measuredDepth.data.at(-1).y===13,'measured encoder history includes gaps and never uses requested depth');
assert(Math.abs(measuredDepth.data.find(p=>p.y===11).x-voltageTrace.data[0].x)<.015,'ADC and encoder points with the same car Unix time align on the chart');
const routedAxis=manualProbeChart.scales.volts;
assert(routedAxis.min<=.2 && routedAxis.max>=1,'physical voltage axis includes the saved threshold');
adcResponse={...adcResponse,revision:2,applied:2,routes:['none','none','none','none','both','none','none','none'],sequence:3,
 samples:[{seq:3,time:100,mux:4,raw:300,volts:.3,scaled:7,filtered_scaled:7,filtered_volts:.3,clipped:true}]};
await window.nextPoll(); drawManualProbeHistory(); drawDetectorDiagnostics(2000);
assert(manualProbeChart.data.datasets.length===3, 'old routing removed');
assert(voltageTrace.data.length===1, 'revision clears old samples');
assert(voltageTrace.data[0].y===.3, 'new voltage samples arrive');
assert($('adc-readings').textContent.includes('CLIPPED'), 'clipping shown');
adcResponse={...adcResponse,probe_trigger:{enabled:true,mux:4,level:6,direction:'above'},
 probe_contact:{enabled:true,fresh:true,detected:true,mux:4,value:7,level:6,direction:'above'},samples:[]};
await window.nextPoll(); drawManualProbeHistory();
assert(savedLimit.label==='Saved limit', 'probe threshold plotted');
assert(savedLimit.data[0].y===6, 'threshold uses voltage axis with unit scaling');
assert($('probe-adc-contact').textContent.includes('CONTACT'), 'contact indication');
adcResponse={...adcResponse,probe_contact:{...adcResponse.probe_contact,fresh:false,detected:null}};
await window.nextPoll();
assert($('probe-adc-contact').textContent.includes('Waiting for contact') && !$('probe-adc-contact').textContent.includes('CONTACT'),'stale sensor never reports contact');
adcResponse={...adcResponse,now:100.02,sequence:4,samples:[{seq:4,time:100.02,mux:4,raw:400,volts:.4,scaled:9,filtered_scaled:7.42,filtered_volts:.321,clipped:false}]};
await window.nextPoll();drawManualProbeHistory();drawDetectorDiagnostics(2100);
const filteredExpected=.321;
const filterField=document.querySelector('[data-adc-field="filter_ms"]');
filterField.value='250';filterField.dispatchEvent(new Event('input'));
assert($('adc-quantity').value==='filtered_volts' && boardChart.data.datasets.some(ds=>ds.label.includes('preview')) && !boardChart.data.datasets.some(ds=>ds.label.includes('unfiltered')), 'editing smoothing previews its proposed voltage without enabling unfiltered traces');
assert($('metal-smoothing').value === '250', 'sensor board and detector share one smoothing draft');
document.querySelector('[data-settings-tab="detector"]').click();
state.detector.sensor = {...state.detector.sensor, reference_voltage:3.3, baseline_adc:20, full_response_adc:200,
  history:[{age_s:.1,filtered_adc:22},{age_s:0,filtered_adc:23}]};
drawMetalHistory();
const smoothingLine = metalChart.data.datasets.find(d=>d.label==='Preview (250 ms)');
assert(smoothingLine?.data.some(p=>Number.isFinite(p.y)), 'metal chart previews draft smoothing from raw samples of the assigned input');
$('metal-smoothing').value='0'; $('metal-smoothing').dispatchEvent(new Event('input'));
const unsmoothed = metalChart.data.datasets.at(-1);
assert(unsmoothed.label==='Preview (0 ms)' && unsmoothed.data.at(-1).y===.4, 'zero smoothing previews the unfiltered sensor voltage');
$('metal-smoothing-save').click();
assert(sent.at(-1).type==='adc' && sent.at(-1).settings.config.filter_ms===0 && sent.at(-1).settings.config.rate===adcResponse.config.rate, 'saving smoothing preserves the applied board configuration and does not save other drafts');
operatorADCResult({ok:false,message:'Save rejected'});
assert($('metal-smoothing-feedback').textContent === 'Save rejected' && $('metal-smoothing').value==='0', 'rejected smoothing save retains the draft');
state.settings={inspection:{passes:2}}; updateMetalSettings();
const passSelect=$('confirm-passes');
assert([...passSelect.options].map(o=>o.value).join(',')==='2,4,6,8' && passSelect.value==='2', 'allowed confirmation counts reflect shared settings');
passSelect.value='6'; passSelect.dispatchEvent(new Event('change'));
assert(sent.at(-1).type==='settings' && sent.at(-1).section==='inspection' && sent.at(-1).value.passes===6 && passSelect.disabled, 'pass count persists on car and waits for acknowledgement');
state.settings.inspection.passes=6; updateMetalSettings();
assert(passSelect.value==='6' && !passSelect.disabled && $('confirm-passes-feedback').textContent.includes('Saved'), 'applied count reported by car');
passSelect.value='8'; passSelect.dispatchEvent(new Event('change'));
socket.onmessage({data:JSON.stringify({type:'settings_result',section:'inspection',ok:false,message:'Save denied'})});
assert(passSelect.value==='6' && !passSelect.disabled && $('confirm-passes-feedback').textContent==='Save denied','rejected confirmation setting restores applied value');
state.drive.you_control_owner=false;updateMetalSettings();await window.nextPoll();
assert(passSelect.disabled && $('metal-smoothing-save').disabled,'spectator cannot save confirmation or smoothing settings');
state.drive.you_control_owner=true;updateMetalSettings();await window.nextPoll();
document.querySelector('[data-settings-tab="adc"]').click();
filterField.value='250';filterField.dispatchEvent(new Event('input'));
assert($('adc-filter-status').textContent.includes('Unsaved preview: 250 ms'),'draft and applied filters are identified');
filterField.value='100';filterField.dispatchEvent(new Event('input'));
assert(voltageTrace.data.at(-1).y===filteredExpected, 'probe plot uses filtered physical voltage independently of scale');
const adcRow=$('adc-readings').rows[0];
assert(adcRow.cells[1].textContent==='0.321000' && adcRow.cells.length===4 && !adcRow.textContent.includes('0.400000'), 'default table displays filtered volts and filtered statistics');
assert([...$('adc-quantity').options].map(o=>o.value).join(',')==='filtered_volts,comparison', 'unfiltered voltage is an explicit comparison');
$('adc-quantity').value='filtered_volts';boardChart.$frameBucket=null;operatorADCFrame();
assert(boardChart.data.datasets.every(dataset=>dataset.label.endsWith('filtered (V)')) && boardChart.data.datasets[0].data.at(-1).y===filteredExpected,'filtered-only board view uses physical sample');
assert(boardChart.options.scales.ads1115.title.text==='Voltage (V)','board chart axis identifies volts');
$('adc-quantity').value='comparison';boardChart.$frameBucket=null;operatorADCFrame();
assert(boardChart.data.datasets.some(d=>d.label.includes('unfiltered (V)') && d.data.at(-1)?.y===.4), 'explicit comparison includes the unfiltered voltage');
adcResponse={...adcResponse,now:101,sequence:5,samples:[{seq:5,time:101,mux:4,raw:500,volts:.5,scaled:4,filtered_scaled:4,filtered_volts:.5,clipped:false}]};
await window.nextPoll();drawManualProbeHistory();
assert(voltageTrace.data.at(-1).y===.5,'filtered probe volts recover after sensor gap');
const updateChart = Chart.prototype.update;
let failContactChart = true;
Chart.prototype.update = function(...args) {
  if (this.canvas?.id === 'contact-history' && failContactChart) {
    failContactChart = false;
    throw Error('Interrupted chart initialization');
  }
  return updateChart.apply(this, args);
};
document.querySelector('[data-settings-tab="contact"]').click();
await window.nextPoll();
assert(!Chart.getChart('contact-history') && $('contact-chart-error').textContent.includes('Interrupted chart initialization'), 'failed initialization releases the registered canvas');
assert($('adc-status').textContent.includes('SAMPLING') && $('adc-error').textContent === '', 'render failure preserves ADC connection');
assert($('contact-live-volts').textContent === '0.5000 V' && $('adc-trigger-state').textContent.startsWith('Applied · ') && $('adc-trigger-state').textContent.includes('clear <'), 'live sensor and threshold comparison remain available while the chart retries');
Chart.prototype.update = updateChart;
const realNow = performance.now.bind(performance);
Object.defineProperty(performance,'now',{value:() => realNow()+1500,configurable:true});
operatorADCFrame();
delete performance.now;
assert($('contact-chart-error').hidden, 'chart recovers after a transient render failure');
const hInput = $('adc-trigger-hysteresis');
const oldWidth = hInput.value;
hInput.value='50'; hInput.dispatchEvent(new Event('input'));
assert($('probe-contact-save').disabled && hInput.validationMessage.includes('below'), 'impossible voltage hysteresis rejected');
assert(!$('contact-draft-preview').hidden && $('contact-draft-preview').textContent.startsWith('Draft ·') && $('adc-trigger-state').textContent.startsWith('Applied ·'), 'draft comparison stays separate from the car decision');
hInput.value=oldWidth; hInput.dispatchEvent(new Event('input')); $('contact-revert').click();
const contactChart = Chart.getChart('contact-history');
assert(contactChart, 'contact chart created: '+JSON.stringify({open:$('settings-dialog').open,hidden:$('settings-contact').hidden,error:$('adc-error').textContent,canvases:Object.values(Chart.instances).map(c=>c.canvas?.id)}));
assert(contactChart.data.datasets[0].label.includes('filtered V') && contactChart.data.datasets[0].data.at(-1).y === .5, 'calibration plots car-filtered voltage on selected input');
assert(contactChart.data.datasets.at(-1).data.at(-1).y === 6, 'calibration charts voltage threshold');
$('probe-contact-volts').value='0.22';$('probe-contact-volts').dispatchEvent(new Event('input'));
$('probe-contact-direction').value='below';$('probe-contact-direction').dispatchEvent(new Event('change'));
await window.nextPoll();
drawManualProbeHistory(); assert(savedLimit.data[0].y===6,'main chart ignores unsaved Setup contact limit');
assert($('probe-contact-volts').value==='0.22' && operatorContactBlock().includes('Save'),'contact voltage draft survives telemetry');
state.probe={ready:true,fault:false,depth_mm:0,max_depth_mm:120,active:false,holding:false,cycle_active:false,adc_contact_state:{enabled:true,fresh:true,detected:false}};
state.drive.armed=true;state.drive.calibrating=false;state.safety={servo_allowed:true,allowed:true};state.detector.sweep_enabled=false;
const beforeDirtyProbe=sent.length;startProbeCycle();
assert(sent.length===beforeDirtyProbe && $('notice').textContent.includes('Save'),'manual probe respects visible unsaved contact limit');
$('probe-contact-save').click();
assert(sent.at(-1).type==='adc' && sent.at(-1).settings.probe_trigger.enabled && sent.at(-1).settings.probe_trigger.level===.22 && sent.at(-1).settings.probe_trigger.direction==='below','main contact limit saves through ADC settings and enables detection');
assert($('probe-contact-volts').disabled && operatorContactBlock().includes('saving'),'pending contact save is explicit');
operatorADCResult({ok:true});
adcResponse={...adcResponse,revision:3,applied:3,probe_trigger:{...adcResponse.probe_trigger,enabled:true,level:.22,direction:'below'},samples:[]};
await window.nextPoll();
assert(!$('probe-contact-volts').disabled && Number($('probe-contact-volts').value)===.22 && !operatorContactBlock(),'saved main contact voltage restored');
const negativeConfig=structuredClone(adcResponse.config);negativeConfig.channels[4].multiplier=-2;negativeConfig.channels[4].offset=1;
adcResponse={...adcResponse,revision:4,applied:4,config:negativeConfig,probe_trigger:{...adcResponse.probe_trigger,level:.56,direction:'above',hysteresis:.04},samples:[]};
await window.nextPoll();
drawManualProbeHistory();
assert(Math.abs(savedLimit.data[0].y-.22)<1e-9 && savedLimit.yAxisID==='volts','negative scale and offset convert legacy threshold to physical volts');
adcResponse={...adcResponse,probe_contact:{enabled:true, fresh:true, detected:false, mux:4, value:.5, filtered_volts:.0912}};
await window.nextPoll();assert($('probe-adc-contact').textContent.includes('0.0912 V') && $('probe-map-sensor').textContent.includes('0.0912 V'),'live labels prefer explicit filtered voltage');
adcResponse={...adcResponse,probe_contact:{...adcResponse.probe_contact,filtered_volts:null}};
await window.nextPoll();assert($('probe-adc-contact').textContent.includes('Waiting for contact'),'missing explicit voltage is never replaced with a scaled reading');
assert(Math.abs(Number($('probe-contact-volts').value)-.22)<1e-8 && $('probe-contact-direction').value==='below','negative scaling restores physical volts and comparison direction');
$('probe-contact-volts').value='.23';$('probe-contact-volts').dispatchEvent(new Event('input'));
$('probe-contact-direction').value='above';$('probe-contact-direction').dispatchEvent(new Event('change'));$('probe-contact-save').click();
assert(Math.abs(sent.at(-1).settings.probe_trigger.level-.54)<1e-10 && sent.at(-1).settings.probe_trigger.direction==='below','negative scaling saves opposite scaled comparison correctly');
operatorADCResult({ok:false,message:'Save failed'});
assert(Number($('probe-contact-volts').value)===.23 && $('probe-contact-feedback').textContent.includes('Save failed') && operatorContactBlock().includes('Save'),'failed contact save retains draft and blocks unsaved probing');

$('contact-revert').click();
assert(!$('settings-adc').contains($('adc-trigger-enabled')) && !$('probe-task').contains($('probe-contact-direction')),'contact editors have one owner');
$('adc-trigger-mux').value='6'; $('adc-trigger-mux').dispatchEvent(new Event('change')); $('probe-contact-save').click();
assert(sent.at(-1).settings.probe_trigger.mux===6 && sent.at(-1).settings.routes[6]==='probe' && sent.at(-1).settings.routes[4]==='detector','changing contact input keeps detector assignment');
operatorADCResult({ok:false,message:'Test rollback'}); $('contact-revert').click();
document.querySelector('[data-settings-tab="detector"]').click();
$('metal-adc-input').value='6'; $('metal-adc-input').dispatchEvent(new Event('change')); $('metal-adc-save').click();
assert(sent.at(-1).settings.routes[6]==='detector' && sent.at(-1).settings.routes[4]==='probe' && sent.at(-1).settings.probe_trigger.mux===4,'detector input editor preserves probe assignment');
operatorADCResult({ok:false,message:'Test rollback'});
document.querySelector('[data-settings-tab="adc"]').click();
const triggerBeforeApply=structuredClone(adcResponse.probe_trigger);
const contactScaling=document.querySelector('[data-adc-channel="4"]').querySelectorAll('input');
$('adc-revert').click();contactScaling[1].value='3';contactScaling[2].value='-.25';$('adc-apply').click();
const remapped=sent.at(-1).settings.probe_trigger;
assert(Math.abs(remapped.level-.41)<1e-9 && Math.abs(remapped.hysteresis-.06)<1e-9 && remapped.direction==='below','board gain and offset edits preserve the saved physical voltage, hysteresis and direction');
operatorADCResult({ok:true});contactScaling[1].value='0';const beforeZero=sent.length;$('adc-apply').click();
assert(sent.length===beforeZero && $('adc-feedback').textContent.includes('nonzero multiplier'),'assigned contact input rejects a noninvertible calibration');
$('adc-revert').click();$('adc-apply').click();
assert(sent.at(-1).settings.probe_trigger.level===triggerBeforeApply.level && sent.at(-1).settings.probe_trigger.direction===triggerBeforeApply.direction,'advanced ADC saves preserve the canonical contact threshold: '+JSON.stringify({last:sent.at(-1),feedback:$('adc-feedback').textContent,disabled:$('adc-apply').disabled,owner:state.drive.you_control_owner,connected:state.connected}));
operatorADCResult({ok:true});
const importedADC={config:{...structuredClone(adcResponse.config),pga:0,low:-12345,high:9876},routes:adcResponse.routes,probe_trigger:{...adcResponse.probe_trigger,level:.67,direction:'above'}};
Object.defineProperty($('adc-file'),'files',{configurable:true,value:[{text:async()=>JSON.stringify(importedADC)}]});
await $('adc-file').onchange();
assert(Math.abs(adcField('low').valueAsNumber-(-12345*6.144/32768))<1e-10,'legacy count-based JSON is displayed using its own PGA');
$('adc-apply').click();
assert(sent.at(-1).settings.config.pga===0 && sent.at(-1).settings.config.low===-12345 && sent.at(-1).settings.config.high===9876,'saving imported signed thresholds reproduces the exact register values');
assert(sent.at(-1).settings.probe_trigger.level===.67 && sent.at(-1).settings.probe_trigger.direction==='above','import preserves threshold and direction without duplicate fields');
operatorADCResult({ok:true});
$('adc-revert').click();$('adc-apply').click();
assert(sent.at(-1).settings.probe_trigger.level===adcResponse.probe_trigger.level && sent.at(-1).settings.probe_trigger.direction===adcResponse.probe_trigger.direction,'revert drops imported threshold override');
operatorADCResult({ok:true});
$('adc-revert').click(); assert(document.querySelector('[data-adc-channel="4"] td:last-child').textContent.includes('Metal detector + probe contact'), 'revert loads shared settings');
state.settings={cameras:structuredClone(cameraDefaults),saved:true};
state.settings.cameras.forward.rotation=90;
operatorSettingsState(state.settings);
assert(preferences.cameraRotation.forward===90, 'robot camera rotation restored');
$('forward-capture-width').value='800'; $('camera-save-shared').click();
assert(sent.at(-1).type==='settings' && sent.at(-1).value.capture.forward.width===800, 'camera settings go through ROS gateway');
preferences.cameras.forward='laptop-private-id';
state.settings.cameras.forward.rotation=180; operatorSettingsState(state.settings);
assert(preferences.cameras.forward==='laptop-private-id', 'laptop device selection remains local');
$('camera-save-shared').click();
assert(sent.at(-1).value.forward.source==='virtual', 'laptop ID never stored on robot');
state.drive.you_control_owner=false; operatorSettingsState(state.settings);
assert($('camera-save-shared').disabled, 'spectator cannot save robot defaults');
await window.nextPoll();
assert($('adc-apply').disabled && !$('adc-run'), 'spectators cannot configure; acquisition has no start/stop button');
assert($('adc-status').textContent.includes('SAMPLING'), 'spectators still receive live ADC readings');
state.drive.you_control_owner=true;await window.nextPoll();
$('metal-smoothing').value='175';$('metal-smoothing').dispatchEvent(new Event('input'));
$('metal-smoothing-save').click();operatorADCResult({ok:true});
adcResponse={...adcResponse,revision:5,applied:5,config:{...adcResponse.config,filter_ms:175}};
await window.nextPoll();
assert($('metal-smoothing').value==='175' && filterField.value==='175' && $('metal-smoothing-feedback').textContent==='Applied: 175 ms' && $('metal-smoothing-save').disabled, 'both panels reflect confirmed applied smoothing');
assert(operatorDetectorFilterPreview(30)===null,'draft preview disappears after save');
window.fetch=async()=>{throw Error('offline');}; await window.nextPoll();
assert($('adc-status').textContent==='DISCONNECTED', 'network failure surfaced');
state.connected=false;drawManualProbeHistory();
assert($('manual-probe-status').textContent.includes('offline') && $('manual-probe-status').textContent.includes('Depth unavailable') && measuredDepth.data.length===0,'mission chart identifies offline voltage and missing depth');
assert(document.querySelector('.probe-diagnostics-panel').contains($('manual-probe-history')) && !$('probe-diagnostics').contains($('manual-probe-history')), 'chart sits directly beneath Manual probe outside motor diagnostics');
document.body.textContent='ADC BROWSER TESTS PASSED';
'''
        html = re.sub(r'<script src="/?assets/[^"]+"></script>', '', (DASHBOARD/'index.html').read_text())
        html = re.sub(r'<link rel="stylesheet" href="/?assets/style.css">', lambda _:  '<style>'+(DASHBOARD/'style.css').read_text()+'</style>', html)
        scripts = setup + 'window.adcResponse='+json.dumps(value)+';\nwindow.cameraDefaults='+json.dumps(CAMERA_DEFAULTS)+';\n'
        scripts += '\n'.join((DASHBOARD/name).read_text() for name in ('chart-4.4.8.umd.js','keydrown-1.3.0.js','actuator-settings.js','app.js','adc.js','camera-config.js'))
        html += '<script>(async()=>{try{'+scripts+checks+"}catch(e){document.body.textContent='FAILED: '+e.stack;}})();</script>"
        with tempfile.TemporaryDirectory() as tmp:
            page=Path(tmp)/'test.html'; page.write_text(html)
            result=subprocess.run(['chromium','--headless','--no-sandbox','--disable-gpu',
                                   '--user-data-dir='+tmp+'/profile','--virtual-time-budget=1000','--dump-dom',page.as_uri()],
                                  capture_output=True,text=True,timeout=60)
        self.assertIn('ADC BROWSER TESTS PASSED</body>',re.sub(r'<head>.*?</head>', '', result.stdout, flags=re.S),re.sub(r'<head>.*?</head>', '', result.stdout, flags=re.S)+result.stderr[-2000:])
