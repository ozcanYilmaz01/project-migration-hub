"""Shared settings for the PMB3 misc0 marking output."""
import math

DEFAULTS = dict(duration_s=2.0, idle_percent=0.0, active_percent=100.0, manual_percent=0.0)


def percent(value):
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 100:
        raise ValueError('Marking output must be between 0 and 100 percent.')
    return float(value)


def validate(value):
    if not isinstance(value, dict):
        raise ValueError('Expected marking settings.')
    duration = value.get('duration_s')
    if type(duration) not in (int, float) or not math.isfinite(duration) or not 0.1 <= duration <= 10:
        raise ValueError('Shot duration must be between 0.1 and 10 seconds.')
    return dict(duration_s=float(duration), **{
        key: percent(value.get(key)) for key in ('idle_percent', 'active_percent', 'manual_percent')})
