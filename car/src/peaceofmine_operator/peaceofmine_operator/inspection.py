"""Confirm a detector target over opposing sweeps, then probe around it."""
from collections import deque
from itertools import combinations
from statistics import median
import math
import time
from . import probe_alignment, probe_scan
from .sweep_arc import sweep_arc



DEFAULTS = {'passes': 2}


def validate(value):
    passes = value.get('passes') if isinstance(value, dict) else None
    if type(passes) is not int or passes not in (2, 4, 6, 8):
        raise ValueError('Choose 2, 4, 6 or 8 confirmation passes')
    return {'passes': passes}


def matching_arcs(passes):
    """Find the largest majority with a common footprint and both sweep directions."""
    valid = [arc for arc in passes if 'angle_deg' in arc]
    best_overlap = 0.
    for count in range(len(valid), len(passes)//2, -1):
        matches = []
        for group in combinations(valid, count):
            if len({p['direction'] for p in group}) != 2:
                continue
            common = min(p['right_deg'] for p in group) - max(p['left_deg'] for p in group)
            span = max(p['right_deg'] for p in group) - min(p['left_deg'] for p in group)
            overlap = max(0., common) / span
            best_overlap = max(best_overlap, overlap)
            if overlap >= .5:
                matches.append((overlap, group))
        if matches:
            overlap, group = max(matches, key=lambda match: match[0])
            return group, overlap
    return (), best_overlap


class InspectionWorkflow:
    def __init__(self, execute, cancel, clear_peak, clock=time.monotonic):
        self.execute, self.cancel_io, self.clear_peak, self.clock = execute, cancel, clear_peak, clock
        self.probe_offset = probe_alignment.DEFAULTS['offset_deg']
        self.generation = 0
        self.phase, self.message = 'IDLE', 'Search while driving, then confirm a detection.'
        self.confirmed_angle = None
        self.lower = self.upper = None
        self.completed = self.total = 0
        self.progress = 0.
        self.confirm_speed = None
        self.pass_from = self.pass_to = None
        self.points, self.passes = [], []
        self.probe_angles = []
        self.probe_depth = None
        self.probe_arm_center = None
        self.scan_offset = None
        self.scan_pattern = None
        self.arc_overlap = None
        self.queue = deque()
        self.pending = None
        self.samples = []
        self.pass_polarity = None
        self.pass_started = 0.
        self.angle = None
        self.wait_started = self.clock()

    @property
    def active(self):
        return self.phase in ('CONFIRMING', 'PROBING')

    def snapshot(self):
        return dict(phase=self.phase, active=self.active, message=self.message,
                    completed=self.completed, total=self.total, progress=self.progress,
                    confirmed_angle_deg=self.confirmed_angle, arc_overlap_ratio=self.arc_overlap,
                    probe_angles_deg=list(self.probe_angles), probe_target_depth_mm=self.probe_depth,
                    probe_offset_deg=self.probe_offset, scan_offset_deg=self.scan_offset,
                    scan_point_count=self.scan_pattern['point_count'] if self.scan_pattern else None,
                    scan_spacing_deg=self.scan_pattern['spacing_deg'] if self.scan_pattern else None,
                    probe_arm_center_deg=self.probe_arm_center,
                    probe_target_bearing_deg=(self.probe_arm_center + self.scan_offset
                                              if self.probe_arm_center is not None else None),
                    probe_reachable=self.confirmed_angle is not None and self.lower <= self.confirmed_angle - self.probe_offset <= self.upper, points=list(self.points), passes=list(self.passes))

    def confirm(self, lower, upper, speed, passes=2, *, home_probe=False):
        validate({'passes': passes})
        if not all(math.isfinite(v) for v in (lower, upper)) or lower >= upper:
            raise ValueError('Calibrated arm limits are required')
        if type(speed) not in (int, float) or not math.isfinite(speed) or speed <= 0:
            raise ValueError('Current search sweep speed is unavailable or invalid')
        self._begin('CONFIRMING')
        self.arc_overlap = None
        self.confirm_speed = speed
        self.progress = None
        self.lower, self.upper = lower, upper
        self.confirmed_angle = None
        self.passes, self.points = [], []
        self.probe_angles = []
        self.probe_depth = None
        self.probe_arm_center = None
        self.scan_offset = None
        self.scan_pattern = None
        self.total = passes
        if home_probe:
            self.queue.append(('home', 0.))
        self.queue.append(('start', lower))
        self.queue.extend(('pass', upper if index % 2 == 0 else lower) for index in range(passes))
        self.message = 'Homing probe before confirmation.' if home_probe else 'Moving to sweep start.'

    def probe(self, lower, upper, depth, pattern=None):
        if self.confirmed_angle is None:
            raise ValueError('Confirm a detection before starting the probe scan')
        if not all(math.isfinite(v) for v in (lower, upper, depth)) or lower >= upper or depth <= 0:
            raise ValueError('Valid arm limits and maximum probe depth are required')
        center = self.confirmed_angle - self.probe_offset
        if not lower <= center <= upper:
            raise ValueError('Confirmed target is outside probe reach. Reposition the rover and confirm again.')
        pattern = probe_scan.validate(probe_scan.DEFAULTS if pattern is None else pattern)
        count, spacing = pattern['point_count'], pattern['spacing_deg']
        midpoint = (count - 1) / 2
        angles = sorted({round(max(lower, min(upper, center + (index - midpoint) * spacing)), 4)
                         for index in range(count)})
        self._begin('PROBING')
        self.lower, self.upper = lower, upper
        self.points = []
        self.scan_pattern = pattern
        self.scan_offset = self.probe_offset
        self.probe_angles = [angle + self.scan_offset for angle in angles]
        self.probe_depth = depth
        self.probe_arm_center = center
        self.total = len(angles)
        self.queue.append(('home', 0.))
        for angle in angles:
            self.queue.extend([('position', angle), ('probe', depth)])
        self.message = 'Homing probe before scan.'

    def _begin(self, phase):
        if self.active:
            raise ValueError('Stop the current inspection first')
        self.generation += 1
        self.phase, self.completed = phase, 0
        self.progress = 0.
        self.pass_from = self.pass_to = None
        self.queue.clear()
        self.pending = None
        self.samples = []
        self.pass_polarity = None
        self.wait_started = self.clock()

    def stop(self, message='Inspection stopped', error=False):
        if not self.active and self.confirmed_angle is None:
            return
        active = self.active
        self.generation += 1
        self.queue.clear()
        self.pending = None
        # A failed probe stroke doesn't move the rover or erase the detector arc.
        # Driving, recalibration, or a stop request still invalidates the target.
        if not (error and self.phase == 'PROBING'):
            self.confirmed_angle = None
        self.phase, self.message = ('ERROR' if error else 'STOPPED'), message
        if active:
            self.cancel_io()

    def record_detector(self, sample):
        if self.pending != 'pass' or not sample or not sample.get('fresh'):
            return
        readings = sample.get('sweep_samples')
        if readings is None:  # Legacy and simulated detectors publish individual readings.
            if not sample.get('angle_fresh'):
                return
            readings = [(sample.get('sample_time'), sample.get('angle_deg'),
                         sample.get('filtered_volts', sample.get('volts')))]
        for stamp, angle, voltage in readings:
            if (not all(type(v) in (float, int) and math.isfinite(v) for v in (stamp, angle, voltage))
                    or stamp < self.pass_started or (self.samples and stamp <= self.samples[-1]['sample_time'])):
                continue
            polarity = sample.get('response_direction')
            if polarity not in (-1, 1):
                self.stop('Detector response direction unavailable', error=True)
                return
            if self.pass_polarity is not None and polarity != self.pass_polarity:
                self.stop('Detector response direction changed; confirm again', error=True)
                return
            self.pass_polarity = polarity
            self.samples.append(dict(angle_deg=angle, volts=voltage, sample_time=stamp))
            fraction = (angle-self.pass_from)/(self.pass_to-self.pass_from)
            self.progress = max(self.progress, self.completed+max(0., min(.999, fraction)))

    def tick(self, idle, sample=None):
        if sample:
            self.record_detector(sample)
        if not self.active:
            return
        if self.pending is not None:
            return
        if not idle:
            if self.clock()-self.wait_started > 2:
                self.stop('Servo did not release after stopping; check the driver.', error=True)
            return
        if not self.queue:
            return
        kind, target = self.queue.popleft()
        self.pending = kind
        if kind == 'pass':
            self.clear_peak()
            self.pass_started = self.clock()
            self.samples, self.pass_polarity = [], None
            self.pass_from = self.lower if self.completed % 2 == 0 else self.upper
            self.pass_to = target
            self.progress = float(self.completed)
            self.message = f'Confirmation pass {self.completed+1} of {self.total} at {self.confirm_speed:g}°/s.'
        elif kind == 'start':
            self.message = f'Moving to sweep start at {self.confirm_speed:g}°/s.'
        elif kind == 'position':
            self.angle = target
            self.message = f'Position {self.completed+1} of {self.total}: moving to {target+self.scan_offset:.1f}°.'
        elif kind == 'probe':
            self.message = f'Position {self.completed+1} of {self.total}: probing at {self.angle+self.scan_offset:.1f}°.'
        generation = self.generation

        def complete(result):
            if generation != self.generation:
                return
            self.pending = None
            self.wait_started = self.clock()
            if not result.get('success'):
                self.stop(result.get('message') or 'Actuator operation failed', error=True)
                return
            if kind == 'pass':
                try:
                    arc = sweep_arc(self.samples, self.pass_polarity)
                except ValueError as exc:
                    if self.total == 2:
                        self.stop(f'Pass {self.completed+1}: {exc}', error=True)
                        return
                    arc = {'reason': str(exc)}
                arc['direction'] = 1 if self.pass_to > self.pass_from else -1
                self.passes.append(arc)
                self.completed += 1
                if self.completed == self.total:
                    group, self.arc_overlap = matching_arcs(self.passes)
                    if not group:
                        self.stop(f'No majority of passes matched in both directions (best overlap {self.arc_overlap:.0%}). Repeat confirmation.', error=True)
                        return
                    # Equal weight per direction cancels opposing time-filter lag even
                    # when one noisy pass was discarded from a larger scan.
                    centers = [median(p['angle_deg'] for p in group if p['direction'] == direction)
                               for direction in (1, -1)]
                    self.confirmed_angle = sum(centers) / 2
                    self.phase = 'CONFIRMED'
                    self.message = (f'Arc centered at {self.confirmed_angle:.1f}° · {len(group)}/{self.total} '
                                    f'passes matched · {self.arc_overlap:.0%} overlap.')
                self.progress = float(self.completed)
            elif kind == 'probe':
                contact = result.get('contact') is True
                depth = result.get('contact_depth_mm') if contact else None
                if contact and (type(depth) not in (int,float) or not math.isfinite(depth) or depth < 0):
                    self.stop('Probe returned an invalid contact depth', error=True)
                    return
                self.points.append(dict(angle_deg=self.angle+self.scan_offset, status='CONTACT' if contact else 'CLEAR',
                                        contact_depth_mm=depth))
                self.completed += 1
                self.progress = float(self.completed)
                if not self.queue:
                    self.phase = 'COMPLETE'
                    hits = sum(p['status'] == 'CONTACT' for p in self.points)
                    self.message = f'Probe scan complete: contact at {hits} of {self.total} positions.'

        try:
            if kind in ('home', 'probe'):
                self.execute('probe', 'home' if kind == 'home' else 'probe_cycle', target, 0., complete)
            else:
                speed = self.confirm_speed if self.phase == 'CONFIRMING' else 5.
                self.execute('arm', 'angle', target, speed, complete)
        except (ValueError, RuntimeError) as exc:
            self.stop(str(exc), error=True)
