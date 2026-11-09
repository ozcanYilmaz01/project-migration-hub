"""Car-owned probe scan pattern, centered on the confirmed detector bearing."""
import math


DEFAULTS = {'point_count': 7, 'spacing_deg': 10.0}


def validate(value):
    if not isinstance(value, dict):
        raise ValueError('Expected probe scan settings')
    count, spacing = value.get('point_count'), value.get('spacing_deg')
    if type(count) is not int or not 1 <= count <= 31:
        raise ValueError('Choose 1 to 31 probe points')
    if type(spacing) not in (int, float) or not math.isfinite(spacing) or not .1 <= spacing <= 180:
        raise ValueError('Choose a probe spacing between 0.1° and 180°')
    return {'point_count': count, 'spacing_deg': float(spacing)}
