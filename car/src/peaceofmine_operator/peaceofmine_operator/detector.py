"""Bounded detector history and freshness for dashboard telemetry."""

from collections import deque
import math
import time


def calibration_values(value):
    # The saved format expresses voltage in units of reference_voltage / 255.
    # ADS1115 differential channels can be negative or exceed that reference.
    base, full, ref = (value.get(k) for k in ('baseline_adc', 'full_response_adc', 'reference_voltage'))
    if (not all(type(v) in (float, int) and math.isfinite(v) for v in (base, full, ref))
            or not 0 < ref <= 5.5 or base == full or max(abs(base), abs(full)) * ref / 255 > 6.144 + 1e-9):
        raise ValueError('Invalid detector calibration')
    return base, full, ref


class DetectorLowPass:
    def __init__(self, time_constant=.1):
        self.time_constant = time_constant
        self.reset()

    def reset(self):
        self.value = self.timestamp = None

    def update(self, value, timestamp):
        if not math.isfinite(value) or not math.isfinite(timestamp):
            raise ValueError('Detector sample and timestamp must be finite')
        if self.timestamp is not None and timestamp <= self.timestamp:
            return self.value
        if self.timestamp is None or timestamp-self.timestamp > .5 or self.time_constant == 0:
            self.value = value
        else:
            self.value += -math.expm1(-(timestamp-self.timestamp)/self.time_constant)*(value-self.value)
        self.timestamp = timestamp
        return self.value


class DetectorTelemetry:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.state = {}
        self.updated = None
        # Retain by elapsed time: 600 samples cover only 12 seconds at 50 Hz.
        self.history = deque()
        self.history_seconds = 120
        self.last_packet = None

    def update(self, value):
        self.state = value
        self.updated = self.clock()
        self._expire_history(self.updated)
        packet = value.get('packet')
        if isinstance(packet, dict) and value.get('fresh'):
            key = (packet.get('seq'), packet.get('uptime_ms'))
            raw = packet.get('precise_adc', packet.get('amplitude_adc'))
            filtered = packet.get('filtered_adc', raw)
            if key != self.last_packet and all(type(v) in (int,float) and math.isfinite(v) for v in (raw,filtered)):
                self.history.append((self.updated, raw, filtered))
                self.last_packet = key

    def record_aligned(self, at, raw, filtered, angle, unix_time):
        self._expire_history(self.clock())
        self.history.append((at, raw, filtered, angle, unix_time))

    def _expire_history(self, now):
        while self.history and now - self.history[0][0] > self.history_seconds:
            self.history.popleft()

    def recent_points(self, seconds):
        cutoff = self.clock() - seconds
        points = []
        for point in reversed(self.history):
            if point[0] < cutoff:
                break
            points.append(point)
        points.reverse()
        return points

    def snapshot(self, include_history=True):
        now = self.clock()
        self._expire_history(now)
        available = self.updated is not None and now - self.updated < 1.0
        age = self.state.get('age_ms')
        if age is not None and self.updated is not None:
            age += round((now - self.updated) * 1000)
        state = {**self.state, 'available': available,
                'fresh': available and self.state.get('fresh', False), 'age_ms': age}
        if include_history:
            state['history'] = [dict(age_s=round(now-p[0], 3), adc=p[1], filtered_adc=p[2],
                                     **({'angle_deg': p[3], 'timestamp_s': p[4]} if len(p) > 3 else {}))
                                for p in self.history]
        return state


def fixture_angle_at(history, stamp, max_gap=.2):
    """Interpolate measured fixture positions within one continuous encoder segment."""
    for index in range(len(history) - 1, -1, -1):
        before, angle = history[index]
        if before == stamp:
            return angle
        if before < stamp:
            if index+1 < len(history):
                after, next_angle = history[index+1]
                if after-before <= max_gap:
                    return angle + (next_angle-angle) * (stamp-before) / (after-before)
            break
    return None
