"""Vehicle-wide drive input and directional keyboard throttle limits."""

DEFAULTS = dict(input='controller', forward_percent=100, reverse_percent=100)


def validate(value):
    if not isinstance(value, dict) or value.get('input') not in ('controller', 'keyboard', 'wheel', 'rc'):
        raise ValueError('Choose a supported drive input.')
    for key in ('forward_percent', 'reverse_percent'):
        if type(value.get(key)) is not int or not 0 <= value[key] <= 100:
            raise ValueError('Keyboard throttle limits must be whole percentages from 0 to 100.')
    return {key: value[key] for key in DEFAULTS}
