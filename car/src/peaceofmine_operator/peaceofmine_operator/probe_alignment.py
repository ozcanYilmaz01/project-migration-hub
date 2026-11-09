"""Angular mounting calibration. Probe bearing = arm angle + offset."""
import math


DEFAULTS = {'offset_deg': 30.0}


def validate(value):
    offset = value.get('offset_deg') if isinstance(value, dict) else None
    if type(offset) not in (int, float) or not math.isfinite(offset) or not -180 <= offset <= 180:
        raise ValueError('Enter a detector-to-probe offset between −180° and 180°')
    return {'offset_deg': float(offset)}
