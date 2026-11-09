"""Settings shared by all operator displays."""
import math

DEFAULTS = dict(mapping='auto', sensitivity=100,
                wheel=dict(steering=0, throttle=1, brake=2, deadman=0, invertSteering=False,
                           invertThrottle=True, invertBrake=True, deadzone=.05),
                searchAlert=dict(enabled=True, level=60, sound=True))


def validate(value):
    if not isinstance(value, dict) or value.get('mapping') not in ('auto', 'standard', 'xbox360'):
        raise ValueError('Invalid gamepad mapping')
    wheel, alert = value.get('wheel'), value.get('searchAlert')
    if not isinstance(wheel, dict) or not isinstance(alert, dict):
        raise ValueError('Missing wheel or search alert settings')
    # Old cars store a percentage of their detector calibration span. Keep its
    # meaning until an operator saves an absolute voltage.
    threshold = ('volts', -6.144, 6.144) if 'volts' in alert else ('level', 20, 100)
    for obj, key, lower, upper in [(value, 'sensitivity', 50, 150), (wheel, 'deadzone', 0, .9),
                                    (alert, *threshold)]:
        number = obj.get(key)
        if type(number) not in (int, float) or not math.isfinite(number) or not lower <= number <= upper:
            raise ValueError(f'Invalid {key}')
    for key in ('steering', 'throttle', 'brake', 'deadman'):
        if type(wheel.get(key)) is not int or not 0 <= wheel[key] <= 63:
            raise ValueError(f'Invalid wheel {key}')
    if any(type(wheel.get(k)) is not bool for k in ('invertSteering', 'invertThrottle', 'invertBrake')) or any(
            type(alert.get(k)) is not bool for k in ('enabled', 'sound')):
        raise ValueError('Invalid switch value')
    return dict(mapping=value['mapping'], sensitivity=value['sensitivity'],
                wheel={k: wheel[k] for k in DEFAULTS['wheel']}, searchAlert={k: alert[k] for k in ('enabled', 'sound', threshold[0])})
