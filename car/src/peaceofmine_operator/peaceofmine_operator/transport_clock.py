"""Measure ROS command age across processes without relying on a stable wall clock on the car."""
import math
from pathlib import Path
import time

try:
    BOOT_ID = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
except OSError:
    BOOT_ID = ''


def _boot_time():
    return time.clock_gettime(time.CLOCK_BOOTTIME)


def transport_stamp():
    fields = {'stamp': time.time()}
    if BOOT_ID and hasattr(time, 'CLOCK_BOOTTIME'):
        fields.update(boot_id=BOOT_ID, boot_stamp=_boot_time())
    return fields


def transport_age(envelope):
    if BOOT_ID and hasattr(time, 'CLOCK_BOOTTIME') and envelope.get('boot_id') == BOOT_ID:
        now, stamp = _boot_time(), envelope.get('boot_stamp')
    else:  # Older senders and remote ROS hosts use the original wire format.
        now, stamp = time.time(), envelope.get('stamp')
    return now-stamp if type(stamp) in (float, int) and math.isfinite(stamp) else math.inf
