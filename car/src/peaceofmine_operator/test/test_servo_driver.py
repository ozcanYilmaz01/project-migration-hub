"""Driver safety invariants independent of controller or browser behavior."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from peaceofmine_interfaces.msg import ServoCommand
from peaceofmine_operator.servo_runtime import ServoRuntime, AUTONOMOUS_POSITION
from peaceofmine_operator.simulated_servo_bus import SimulatedServoBus


class DriverTest(unittest.TestCase):
    def setUp(self):
        self.now=1.
        self.allowed=True
        self.runtime=ServoRuntime(SimulatedServoBus(),[1,2],lambda:self.allowed,lambda ident:(14,4),clock=lambda:self.now)

    def command(self,**changes):
        values=dict(owner='arm',operation_id='one',sequence=1,servo_id=1,mode=1,
                    target=2000,minimum=0,maximum=4095,speed=14,acceleration=4,lead=114)
        values.update(changes)
        return ServoCommand(**values)

    def tearDown(self):
        self.runtime.close()

    def test_watchdog_latches_even_if_controller_returns(self):
        self.runtime.submit(self.command())
        self.runtime.tick()
        self.assertTrue(self.runtime.bus.values[1][24])
        self.now+=.3
        self.runtime.enforce()
        self.assertFalse(self.runtime.bus.values[1][24])
        with self.assertRaisesRegex(ValueError,'retired'):
            self.runtime.submit(self.command(sequence=2))
        self.runtime.submit(self.command(operation_id='explicit-new-start',sequence=3))
        self.assertIsNotNone(self.runtime.active)

    def test_auto_pause_cuts_torque_and_needs_a_post_pause_challenge(self):
        self.runtime.submit(self.command(mode=AUTONOMOUS_POSITION))
        self.runtime.tick()
        self.now += .28
        self.runtime.enforce()
        self.assertFalse(self.runtime.bus.values[1][24])
        self.assertEqual(self.runtime.outcomes[1][1], 'paused')
        with self.assertRaisesRegex(ValueError, 'after torque'):
            self.runtime.submit(self.command(mode=AUTONOMOUS_POSITION,sequence=2), challenge_at=self.now-.01)
        self.now += .05
        self.runtime.submit(self.command(mode=AUTONOMOUS_POSITION,sequence=3), challenge_at=self.now)
        self.runtime.tick()
        self.assertTrue(self.runtime.bus.values[1][24])
        self.now += .76
        self.runtime.enforce()
        self.assertIsNone(self.runtime.active)
        with self.assertRaisesRegex(ValueError, 'retired'):
            self.runtime.submit(self.command(mode=AUTONOMOUS_POSITION,sequence=4), challenge_at=self.now)

    def test_auto_pause_retired_by_stop_and_permission_loss(self):
        for reason in ('stop', 'rc'):
            self.allowed = True
            self.runtime.submit(self.command(mode=AUTONOMOUS_POSITION,operation_id=reason))
            self.now += .26
            self.runtime.enforce()
            if reason == 'stop':
                self.runtime.submit(self.command(mode=0,operation_id=reason))
            else:
                self.allowed=False
                self.runtime.enforce()
            self.allowed=True
            with self.assertRaisesRegex(ValueError, 'retired'):
                self.runtime.submit(self.command(mode=AUTONOMOUS_POSITION,operation_id=reason,sequence=2), challenge_at=self.now)

    def test_duplicate_and_queued_challenge_do_not_abort_or_renew_live_operation(self):
        from test_arm_node import module
        driver = module('servo_driver_node')
        node = SimpleNamespace(runtime=self.runtime, session='session', reason='Connected', challenge_times={1:self.now})
        msg = self.command(session='session',challenge=1,sequence=5)
        with patch.object(driver.time,'monotonic',side_effect=lambda:self.now):
            driver.ServoDriver.command(node,msg)
            self.now += .1
            driver.ServoDriver.command(node,msg)
            driver.ServoDriver.command(node,self.command(session='session',challenge=0,sequence=6))
        self.assertIsNotNone(self.runtime.active)
        self.assertEqual(self.runtime.renewed,self.now-.1)
        self.assertEqual(node.reason,'Connected')
        self.now += .16
        self.runtime.enforce()
        self.assertIsNone(self.runtime.active)

    def test_permission_loss_retires_operation(self):
        self.runtime.submit(self.command())
        self.allowed=False
        self.runtime.enforce()
        self.allowed=True
        with self.assertRaises(ValueError):self.runtime.submit(self.command(sequence=2))

    def test_slow_telemetry_cannot_enable_torque_after_command_expires(self):
        motor = self.runtime.motors[1]
        snapshot = motor.snapshot
        def delayed_snapshot():
            self.now += .26
            return snapshot()
        motor.snapshot = delayed_snapshot
        self.runtime.submit(self.command())
        self.runtime.tick()
        self.assertFalse(self.runtime.bus.values[1][24])
        self.assertIsNone(self.runtime.active)
        self.assertIn('Command watchdog', self.runtime.outcomes[1][2])

    def test_all_motion_modes_enforce_speed_limit(self):
        for mode in (1,2):
            for speed in (0,15,1023):
                with self.subTest(mode=mode,speed=speed),self.assertRaises(ValueError):
                    self.runtime.submit(self.command(mode=mode,speed=speed))
        self.assertIsNone(self.runtime.active)

    def test_exclusive_owner_and_foreign_stop(self):
        self.runtime.submit(self.command())
        with self.assertRaises(ValueError):
            self.runtime.submit(self.command(owner='probe',servo_id=2))
        self.runtime.submit(self.command(owner='probe',mode=0))
        self.assertIsNotNone(self.runtime.active)
        self.runtime.submit(self.command(mode=0))
        self.assertIsNone(self.runtime.active)

    def test_duplicate_and_reordered_commands_cannot_renew(self):
        self.runtime.submit(self.command(sequence=3))
        for sequence in (3,2):
            with self.assertRaises(ValueError):self.runtime.submit(self.command(sequence=sequence))
        self.now+=.3
        self.runtime.enforce()
        self.assertIsNone(self.runtime.active)

    def test_probe_can_move_inward_from_just_beyond_home(self):
        bus = self.runtime.bus
        bus.positions[2] = -198  # Released two ticks past the calibrated top.
        self.runtime.submit(self.command(servo_id=2, target=-1000, minimum=-1500, maximum=-200))
        with patch.object(bus, 'write', wraps=bus.write) as writes:
            self.runtime.tick()
        goals = [call.args[2] for call in writes.call_args_list if call.args[:2] == (2, 30)]
        self.assertEqual(goals[0], -200)
        self.assertTrue(all(-312 <= goal <= -200 for goal in goals))
        self.assertEqual(self.runtime.outcomes[2][1], 'running')
        self.assertTrue(bus.values[2][24])

    def test_unreachable_range_aborts_only_that_operation(self):
        self.runtime.submit(self.command(servo_id=2, target=-1000, minimum=-1500, maximum=-500))
        self.runtime.tick()
        operation, outcome, reason = self.runtime.outcomes[2]
        self.assertEqual((operation, outcome), ('one', 'aborted'))
        self.assertIn('encoder at -200 ticks', reason)
        self.assertFalse(self.runtime.bus.values[2][24])
        self.runtime.submit(self.command(operation_id='next'))
        self.runtime.tick()
        self.assertTrue(self.runtime.bus.values[1][24])

    def test_invalid_bounds_rejected_before_torque(self):
        for changes in (dict(target=5000),dict(minimum=-1),dict(maximum=5000),dict(load_stop_percent=float('nan'))):
            with self.assertRaises(ValueError):self.runtime.submit(self.command(**changes))
        self.assertFalse(self.runtime.bus.values[1][24])

    def test_protective_load_stop_is_latched(self):
        self.runtime.submit(self.command(load_stop_percent=1.))
        self.runtime.tick()
        self.assertIsNone(self.runtime.active)
        self.assertEqual(self.runtime.outcomes[1][1],'load_stop')
        with self.assertRaises(ValueError):self.runtime.submit(self.command(sequence=2))
