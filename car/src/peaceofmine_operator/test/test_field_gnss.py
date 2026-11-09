from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from peaceofmine_operator.field_gnss import DEFAULTS, FieldLog, position, validate


def fix(accuracy, lat=0.0):
    return dict(latitude=lat, longitude=0., horizontal_accuracy_m=accuracy, fix='GNSS fix')


class FieldTest(unittest.TestCase):
    def setUp(self):
        self.tmp = TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'field.sqlite3'
        self.log = FieldLog(self.path)
        self.addCleanup(lambda: self.log.db.close())
        self.shot = dict(id='m1', timestamp=100., duration_s=2, x=None)

    def test_limits(self):
        self.assertEqual(validate(DEFAULTS), DEFAULTS)
        for wait, target in ((-1, 1), (121, 1), (True, 1), (0, 0), (0, 101), (0, float('nan'))):
            with self.assertRaises(ValueError):
                validate(dict(max_wait_s=wait, target_accuracy_m=target))

    def test_accuracy_is_in_metres(self):
        pvt = SimpleNamespace(identity='NAV-PVT', lat=0., lon=0., numSV=10, fixType=3,
                              gnssFixOk=1, carrSoln=2, diffSoln=1, hAcc=35)
        self.assertEqual(position(pvt), dict(latitude=0., longitude=0., satellites=10, fix='RTK fixed', horizontal_accuracy_m=.035))
        pvt.gnssFixOk = 0
        self.assertIsNone(position(pvt)['latitude'])
        gga = SimpleNamespace(identity='GNGGA', quality=1, lat=0., lon=0., numSV=10, HDOP=.4)
        self.assertIsNone(position(gga)['horizontal_accuracy_m'])

    def test_wait_chooses_best_and_duplicate_shot_is_idempotent(self):
        self.log.record(self.shot, True, fix(8), DEFAULTS, 100)
        self.log.refine(fix(2), 101, True)
        self.log.refine(fix(4), 102, True)
        self.assertEqual(self.log.entries()[0]['horizontal_accuracy_m'], 2)
        self.log.refine(fix(.5), 103, True)
        self.assertFalse(self.log.record(self.shot, True, fix(10), DEFAULTS, 104))
        self.assertEqual(self.log.entries()[0]['gps_status'], 'within_target')

    def test_timeout_unavailable_and_moving_uses_old_position(self):
        self.log.record(self.shot, True, None, DEFAULTS, 100)
        self.log.refine(fix(.1), 106, True)
        self.assertEqual(self.log.entries()[0]['gps_status'], 'unavailable')
        self.log.record(dict(self.shot, id='m2'), True, fix(None), DEFAULTS, 100)
        self.log.refine(fix(.1, 50), 101, False)
        event = self.log.entries()[0]
        self.assertEqual((event['gps_status'], event['latitude']), ('uncertain', 0.))

    def test_replayed_event_does_not_inherit_current_position(self):
        self.log.record(self.shot, True, fix(.4), DEFAULTS, 800)
        self.assertEqual(self.log.entries()[0]['gps_status'], 'unavailable')
        self.assertNotIn('latitude', self.log.entries()[0])

    def test_restart_retains_mark_trail_and_manual_event(self):
        self.log.record(self.shot, True, fix(6), DEFAULTS, 100)
        self.log.record(dict(id='pose', timestamp=101., status='POSE LOGGED'), False, None, DEFAULTS, 101)
        self.log.trace(fix(1), 102)
        self.log.db.close()
        self.log = FieldLog(self.path)
        manual, mark = self.log.entries()
        self.assertEqual(manual['status'], 'POSE LOGGED')
        self.assertEqual((mark['gps_status'],mark['horizontal_accuracy_m']), ('uncertain', 6))
        self.assertEqual(self.log.trail(), [(102.,0.,0.)])
        self.log.clear()
        self.assertEqual(self.log.entries(), [])
        self.assertEqual(len(self.log.trail()), 1)
