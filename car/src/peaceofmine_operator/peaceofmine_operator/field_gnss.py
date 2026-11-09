"""GNSS fix interpretation and durable, time-bounded geotags for mission events."""
import json
import math
import sqlite3


DEFAULTS = {'max_wait_s': 5.0, 'target_accuracy_m': 1.0}
FRESH_S = 2.0


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


def validate(value):
    if not isinstance(value, dict):
        raise ValueError('Expected GNSS mark settings')
    wait, target = value.get('max_wait_s'), value.get('target_accuracy_m')
    if not finite(wait) or not 0 <= wait <= 120:
        raise ValueError('Maximum wait must be between 0 and 120 seconds')
    if not finite(target) or not 0.01 <= target <= 100:
        raise ValueError('Target accuracy must be between 0.01 and 100 metres')
    return dict(max_wait_s=float(wait), target_accuracy_m=float(target))


def position(message):
    """Coordinates and horizontal accuracy from the same navigation epoch."""
    if message.identity == 'NAV-PVT':
        valid = bool(message.gnssFixOk and message.fixType in (2, 3, 4))
        quality = {1: 'RTK float', 2: 'RTK fixed'}.get(message.carrSoln, 'Differential' if message.diffSoln else 'GNSS fix')
        accuracy = message.hAcc / 1000
        satellites = message.numSV
    elif message.identity.endswith('GGA'):
        valid = message.quality in (1, 2, 4, 5)
        quality = {2: 'Differential', 4: 'RTK fixed', 5: 'RTK float'}.get(message.quality, 'GNSS fix')
        # GGA HDOP is dimensionless. Do not manufacture an accuracy in metres.
        accuracy = None
        satellites = message.numSV
    else:
        return None
    lat, lon = message.lat, message.lon
    valid = valid and finite(lat) and finite(lon) and abs(lat) <= 90 and abs(lon) <= 180
    return dict(latitude=lat if valid else None, longitude=lon if valid else None,
                horizontal_accuracy_m=accuracy if valid and finite(accuracy) and accuracy >= 0 else None,
                fix=quality if valid else 'No fix', satellites=satellites if finite(satellites) else None)


class FieldLog:
    """One SQLite writer. Pending rows retain their best candidate across a restart."""
    def __init__(self, path):
        self.db = sqlite3.connect(path)
        self.db.execute('CREATE TABLE IF NOT EXISTS events (id TEXT PRIMARY KEY, body TEXT NOT NULL, pending INTEGER NOT NULL)')
        self.db.execute('CREATE TABLE IF NOT EXISTS trail (t REAL, lat REAL, lon REAL)')
        # The rover may have moved while the recorder was stopped.
        self.refine(None, 0, False)
        row = self.db.execute('SELECT t,lat,lon FROM trail ORDER BY rowid DESC LIMIT 1').fetchone()
        self.last_point = row

    def entries(self):
        return [json.loads(row[0]) for row in self.db.execute('SELECT body FROM events ORDER BY rowid DESC')]

    def clear(self):
        with self.db:
            self.db.execute('DELETE FROM events')

    def trail(self):
        return self.db.execute('SELECT t,lat,lon FROM trail ORDER BY rowid').fetchall()

    def trace(self, fix, now):
        if fix is None or fix['latitude'] is None:
            return
        lat, lon = fix['latitude'], fix['longitude']
        if self.last_point:
            t, y, x = self.last_point
            if now - t < 1 or (now - t < 30 and abs(y-lat) < 1e-6 and abs(x-lon) < 1e-6):
                return
        with self.db:
            self.db.execute('INSERT INTO trail VALUES (?,?,?)', (now, lat, lon))
            self.db.execute('DELETE FROM trail WHERE rowid <= (SELECT MAX(rowid)-900 FROM trail)')
        self.last_point = now, lat, lon

    @staticmethod
    def candidate(event, fix):
        if not fix or fix['latitude'] is None:
            return
        accuracy, previous = fix['horizontal_accuracy_m'], event.get('horizontal_accuracy_m')
        if event.get('latitude') is None or (accuracy is not None and (previous is None or accuracy < previous)):
            event.update({k: fix[k] for k in ('latitude', 'longitude', 'horizontal_accuracy_m', 'fix')})

    @staticmethod
    def resolve(event, finished):
        accuracy = event.get('horizontal_accuracy_m')
        if accuracy is not None and accuracy <= event['target_accuracy_m']:
            return 'within_target'
        if finished:
            return 'uncertain' if event.get('latitude') is not None else 'unavailable'
        return 'pending'

    def record(self, payload, mark, fix, settings, now):
        event = dict(payload)
        if not isinstance(event.get('id'), str) or not event['id'] or not finite(event.get('timestamp')):
            raise ValueError('Mission event requires an ID and timestamp')
        # Buffered or replayed events must not inherit a later rover position.
        if abs(now - event['timestamp']) <= FRESH_S:
            self.candidate(event, fix)
        if mark:
            event.update(settings, status='SHOT COMMANDED', statusType='marked', kind='pump_mark')
            event['gps_status'] = self.resolve(event, now >= event['timestamp'] + settings['max_wait_s'])
        with self.db:
            inserted = self.db.execute('INSERT OR IGNORE INTO events VALUES (?,?,?)',
                                       (event['id'], json.dumps(event, allow_nan=False), event.get('gps_status') == 'pending'))
        return inserted.rowcount == 1

    def refine(self, fix, now, stationary):
        changed = False
        with self.db:
            for ident, body in self.db.execute('SELECT id,body FROM events WHERE pending=1').fetchall():
                event = json.loads(body)
                expired = now >= event['timestamp'] + event['max_wait_s']
                if stationary and not expired:
                    self.candidate(event, fix)
                event['gps_status'] = self.resolve(event, expired or not stationary)
                updated = json.dumps(event, allow_nan=False)
                if updated != body:
                    self.db.execute('UPDATE events SET body=?,pending=? WHERE id=?',
                                    (updated, event['gps_status'] == 'pending', ident))
                    changed = True
        return changed
