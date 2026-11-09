"""Probe-owned threshold evaluation, with hysteresis, debounce, and freshness checks."""
import math
import time


class AdcContact:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.config = dict(enabled=False, mux=4, level=1., direction='above', hysteresis=0., debounce_ms=0.)
        self.value = self.filtered_volts = None
        self.received = 0.
        self.detected = False
        self.candidate = None
        self.since = 0.
        self.session = None
        self.sequence = -1
        self.max_age = .5

    def configure(self, config, max_age=.5):
        if config != self.config:
            self.received = 0.
            self.candidate = None
            self.detected = False
        self.config = dict(config)
        self.max_age = max(.5, max_age)

    def sample(self, value, sequence, session, filtered_volts=None):
        if session != self.session:
            self.session, self.sequence = session, -1
            self.detected, self.candidate = False, None
        if sequence <= self.sequence or not math.isfinite(value) or (filtered_volts is not None and not math.isfinite(filtered_volts)):
            return
        now = self.clock()
        if now-self.received > self.max_age:
            self.detected, self.candidate = False, None
        self.sequence, self.value, self.received = sequence, value, now
        self.filtered_volts = filtered_volts

        config = self.config
        sign = 1 if config.get('direction', 'above') == 'above' else -1
        distance = sign*(value-config.get('level', 1.0))
        state = distance >= (-config.get('hysteresis',0.) if self.detected else 0.)
        if state != self.candidate:
            self.candidate, self.since = state, now
        if now-self.since >= config.get('debounce_ms',0.)/1000:
            self.detected = state

    def snapshot(self):
        valid = self.config.get('enabled', False) and self.value is not None and self.clock()-self.received < self.max_age
        is_hit = self.detected if valid else None
        return dict(enabled=self.config.get('enabled', False), fresh=valid, detected=is_hit,
            filtered_volts=self.filtered_volts if valid else None,
            value=self.value if valid else None, mux=self.config.get('mux', 4), level=self.config.get('level', 1.0),
            direction=self.config.get('direction', 'above'))
