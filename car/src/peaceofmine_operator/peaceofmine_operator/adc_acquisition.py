"""Operator-owned ADS1115 acquisition and persistent plot routing."""
import copy
import json
import math
import threading
import time
from pathlib import Path
from . import configuration
from .detector import DetectorLowPass
from collections import deque
from itertools import takewhile
from .ads1115 import ADS1115, DEFAULTS, INPUTS, RANGES, config_word, validate


class Acquisition:
    def __init__(self, demo=False, config=None):
        self.demo = demo
        self.config = validate(config or copy.deepcopy(DEFAULTS))
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.wake_event = threading.Event()
        self.running = False
        self.filters = {}
        self.revision = 1
        self.applied = 0
        self.connected = False
        self.error = None
        self.registers = {}
        self.samples = deque(maxlen=30000)
        self.sequence = 0
        self.device = None
        self.thread = threading.Thread(target=self.work, daemon=True)

    def update(self, value):
        config = validate(value)
        with self.lock:
            self.config = config
            self.revision += 1
            self.samples.clear()
            self.wake_event.set()

    def run(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('running must be a boolean')
        with self.lock:
            self.running = enabled
            self.wake_event.set()
            if not enabled and self.demo and 'config' in self.registers:
                self.registers['config'] |= 0x8103
            if not enabled and self.device:
                try:
                    self.device.idle()
                    self.registers = self.device.registers()
                except (OSError, TimeoutError) as exc:
                    self.error = str(exc)
                    self.disconnect()
                    raise

    def snapshot(self, after=0):
        with self.lock:
            samples = list(takewhile(lambda s: s['seq'] > after, reversed(self.samples)))
            samples.reverse()
            dropped = max(0, (samples[0]['seq'] - after - 1)) if samples and after else 0
            return dict(demo=self.demo, config=copy.deepcopy(self.config),
                        revision=self.revision, applied=self.applied,
                        connected=self.connected, running=self.running, error=self.error,
                        registers=self.registers.copy(), samples=samples[:5000],
                        sequence=self.sequence, dropped=dropped, now=time.time(), inputs=INPUTS)

    def disconnect(self):
        if self.device:
            try:
                self.device.close()
            except (OSError, TimeoutError):
                pass
            self.device = None
        self.filters.clear()
        self.connected = False
        self.applied = 0

    def work(self):
        index = 0
        next_register_read = 0
        while not self.stop_event.is_set():
            if not self.running:
                self.wake_event.wait(0.05)
                self.wake_event.clear()
                continue
            delay = 0.05
            with self.lock:
                c = self.config
                try:
                    if not self.running:
                        continue
                    if self.applied != self.revision:
                        self.disconnect()
                        if not self.demo:
                            self.device = ADS1115(c['bus'], c['address'])
                            self.registers = self.device.configure(c)
                        else:
                            lo, hi = (0, 32768) if c['alert'] == 'ready' else (c['low'] & 65535, c['high'] & 65535)
                            mux = next((ch['mux'] for ch in c['channels'] if ch['enabled']), 4)
                            self.registers = dict(config=config_word(c, mux, idle=True) | 0x8000, low=lo, high=hi)
                        self.applied = self.revision
                        self.connected = True
                        self.error = None
                        index = 0
                        next_register_read = 0
                    channels = [ch for ch in c['channels'] if ch['enabled']]
                    if self.running and channels:
                        ch = channels[index % len(channels)]
                        mux = ch['mux']
                        if self.demo:
                            time.sleep(1 / c['rate'])
                            voltage = 0.12 * math.sin(time.monotonic() * (0.7 + mux * 0.13))
                            voltage += (mux - 3) * 0.35 if mux >= 4 else 0
                            raw = max(-32768, min(32767, round(voltage * 32768 / RANGES[c['pga']])))
                            self.registers['config'] = config_word(c, mux) | (0x8000 if c['mode'] == 'single' else 0)
                        else:
                            raw = self.device.sample(c, mux)
                            # Diagnostic reads must not consume three bus transfers per sample.
                            if time.monotonic() >= next_register_read:
                                self.registers = self.device.registers()
                                next_register_read = time.monotonic() + 1
                        self.registers['conversion'] = raw & 65535
                        volts = raw * RANGES[c['pga']] / 32768
                        self.sequence += 1
                        if mux not in self.filters:
                            self.filters[mux] = DetectorLowPass(c['filter_ms'] / 1000.)
                        filtered = self.filters[mux].update(volts, time.monotonic())
                        self.samples.append(dict(seq=self.sequence, time=time.time(), mux=mux,
                                                 raw=raw, volts=volts, filtered_volts=filtered,
                                                 scaled=volts * ch['multiplier'] + ch['offset'],
                                                 filtered_scaled=filtered * ch['multiplier'] + ch['offset'],
                                                 clipped=raw in (-32768, 32767)))
                        index += 1
                        delay = c['interval_ms'] / 1000 if index % len(channels) == 0 else 0
                except (OSError, TimeoutError, ImportError) as exc:
                    self.error = f'{type(exc).__name__}: {exc}'
                    self.disconnect()
                    delay = 1
            self.wake_event.wait(max(0.0001, delay))
            self.wake_event.clear()
        with self.lock:
            self.disconnect()

    def close(self):
        self.stop_event.set()
        self.wake_event.set()
        self.thread.join(timeout=3)


def contact_trigger(settings):
    """The saved trigger mux is the channel the probe physically monitored."""
    if 'probe_trigger' in settings:
        return settings['probe_trigger']
    # Exports predating contact detection had plot routing only.
    assigned = [mux for mux, role in enumerate(settings.get('routes', [])) if role in ('probe', 'both')]
    return dict(enabled=False, mux=assigned[0] if len(assigned) == 1 else 4, level=1.0, direction='above')


def unreachable_hysteresis(trigger, config):
    """The latch must be able to cross its release threshold on this ADC input."""
    h = trigger.get('hysteresis', 0.)
    if not h:
        return False
    mux = trigger['mux']
    gain = config['channels'][mux]['multiplier']
    offset = config['channels'][mux]['offset']
    full = RANGES[config['pga']]
    low_v = 0. if mux >= 4 else -full
    high_v = full * 32767 / 32768
    low, high = sorted((low_v*gain+offset, high_v*gain+offset))
    room = trigger['level']-low if trigger['direction']=='above' else high-trigger['level']
    return h >= min(high-low, room)


def saved_contact_trigger(settings):
    trigger = contact_trigger(settings)
    if unreachable_hysteresis(trigger, settings.get('config') or DEFAULTS):
        return dict(trigger, hysteresis=0.)
    return trigger


class OperatorADC:
    def __init__(self, path, demo=False, initial_settings=None):
        self.path = Path(path).expanduser()
        self.probe_trigger = dict(enabled=False, mux=4, level=1.0, direction='above')
        self.routes = ['none'] * 8
        self.routes[4], self.routes[6] = 'probe', 'detector'
        config = copy.deepcopy(DEFAULTS)
        for ch in config['channels']:
            ch['enabled'] = ch['mux'] in (4, 6)
        self.section = 'adc_simulation' if demo else 'adc'
        self.load_error = self.contact_warning = None
        try:
            saved = configuration.load(str(self.path))
            section = 'adc_simulation' if demo else 'adc'
            value = saved.get(section, initial_settings)
            if value is not None:
                config, self.routes, self.probe_trigger = self.validate(value, restoring=True)
                self.contact_warning = self.recovery_notice(value, self.probe_trigger)
        except FileNotFoundError:
            pass
        except (ValueError, TypeError, OSError) as exc:
            self.load_error = f'Saved ADC settings could not be loaded: {exc}'
        self.acquisition = Acquisition(demo, config)
        self.acquisition.run(self.load_error is None)
        self.acquisition.thread.start()

    @staticmethod
    def validate(value, restoring=False):
        if not isinstance(value, dict) or set(value) not in ({'config', 'routes'}, {'config', 'routes', 'probe_trigger'}):
            raise ValueError('Expected config and routes')
        config = validate(value['config'])
        routes = value['routes']
        if not isinstance(routes, list) or len(routes) != 8 or any(
                route not in ('none', 'detector', 'probe', 'both') for route in routes):
            raise ValueError('Each input needs a valid plot destination')
        trigger = contact_trigger(value)
        if not isinstance(trigger, dict) or not {'enabled', 'mux', 'level', 'direction'} <= set(trigger) or set(trigger) - {'enabled','mux','level','direction','hysteresis','debounce_ms'}:
            raise ValueError('Invalid probe trigger settings')
        if (type(trigger['enabled']) is not bool or type(trigger['mux']) is not int
                or trigger['mux'] not in range(8) or trigger['direction'] not in ('above', 'below')
                or type(trigger['level']) not in (int, float) or not math.isfinite(trigger['level'])
                or abs(trigger['level']) > 1e12):
            raise ValueError('Invalid probe trigger level or input')
        if trigger['enabled'] and not config['channels'][trigger['mux']]['enabled']:
            raise ValueError('Enable the selected trigger input')
        for key, maximum in (('hysteresis',1e12), ('debounce_ms',10000)):
            value = trigger.get(key,0.)
            if type(value) not in (int,float) or not math.isfinite(value) or not 0 <= value <= maximum:
                raise ValueError('Invalid trigger hysteresis or debounce')
        if unreachable_hysteresis(trigger, config):
            if not restoring:
                raise ValueError('Choose a smaller hysteresis so the contact input can clear within its voltage range.')
            trigger = dict(trigger, hysteresis=0.)
        # Keep legacy plot exports and consumers compatible. Derive the probe
        # role from its physical trigger input, preserving all detector routes.
        normalized = []
        for mux, role in enumerate(routes):
            detector = role in ('detector', 'both')
            if mux == trigger['mux']:
                normalized.append('both' if detector else 'probe')
            else:
                normalized.append('detector' if detector else 'none')
        return config, normalized, dict(trigger)

    @staticmethod
    def recovery_notice(saved, effective):
        if contact_trigger(saved).get('hysteresis', 0.) != effective.get('hysteresis', 0.):
            return 'Saved hysteresis is outside the input range. Applying 0 V until contact settings are saved.'
        return None

    def command(self, payload, persist=True, restoring=False):
        if payload.get('action') == 'settings':
            config, routes, trigger = self.validate(payload.get('settings'), restoring=restoring)
            with self.acquisition.lock:
                if persist:
                    configuration.save_section(str(self.path), self.section, dict(
                        config=config, routes=routes, probe_trigger=trigger))
                self.acquisition.update(config)
                self.routes = routes
                self.probe_trigger = trigger
                self.load_error = None
                self.contact_warning = self.recovery_notice(payload['settings'], trigger)
                self.acquisition.run(True)
        elif payload.get('action') == 'run':
            raise ValueError('ADC acquisition runs automatically; start/stop commands are not supported')
        else:
            raise ValueError('Unknown ADC action')

    def snapshot(self, after=0):
        with self.acquisition.lock:
            return {**self.acquisition.snapshot(after), 'routes': list(self.routes),
                    'load_error': self.load_error, 'contact_warning': self.contact_warning, 'probe_trigger': dict(self.probe_trigger),
                    'probe_contact': dict(enabled=self.probe_trigger['enabled'], fresh=False, detected=None)}

    def close(self):
        self.acquisition.close()
