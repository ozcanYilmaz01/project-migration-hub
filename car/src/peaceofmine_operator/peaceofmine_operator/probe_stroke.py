"""Car settings for calibrated probe travel and automatic homing."""
import sys

DEFAULTS = dict(down_mm_s=5., retract_mm_s=5., home_load_percent=10.)


def validate(value):
    if not isinstance(value, dict):
        raise ValueError('Expected probe stroke settings')
    for key in ('down_mm_s', 'retract_mm_s'):
        speed = value.get(key)
        if type(speed) not in (int, float) or not 0 < speed <= sys.float_info.max:
            raise ValueError('Down and retract speeds must be positive mm/s')
    load = value.get('home_load_percent')
    if type(load) not in (int, float) or not 10 <= load <= 60:
        raise ValueError('Automatic home load must be between 10 and 60 percent')
    return {key: float(value[key]) for key in DEFAULTS}
