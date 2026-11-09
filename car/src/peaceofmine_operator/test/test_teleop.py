"""Run with a sourced ROS overlay on an isolated ROS_DOMAIN_ID."""

import importlib.util
from pathlib import Path
import unittest
import json
from unittest.mock import patch

import rclpy
from sensor_msgs.msg import CameraInfo, Joy
from std_msgs.msg import Float32, String
from peaceofmine_operator.actuator_protocol import command_dict

spec = importlib.util.spec_from_file_location(
    'operator_control', Path(__file__).parents[1] / 'scripts/operator_control_node.py')
gateway_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gateway_module)


class TeleopTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-r', '__ns:=/teleop_test'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = gateway_module.OperatorControl()
        self.node._rc_safety.simulation = True
        self.owner = object()
        self.clock = patch.object(gateway_module.time, 'monotonic', return_value=100.0)
        self.now = self.clock.start()
        self.node._odometry_at = 100.0
        self.node._motion_speed = 0.0
        self.output = patch.object(self.node, '_publish_twist').start()

    def tearDown(self):
        patch.stopall()
        self.node.destroy_node()

    def command(self, kind, **kwargs):
        return self.node.handle_command(self.owner, dict(type=kind, **kwargs))

    def arm(self):
        self.command('take_control')
        self.command('arm')

    def test_refresh_and_steal_preserve_completed_confirmation(self):
        self.command('take_control')
        workflow = self.node._inspection
        workflow.phase = 'CONFIRMED'
        workflow.confirmed_angle, workflow.lower, workflow.upper = 12., -90., 90.
        self.node.release(self.owner)
        self.assertEqual(self.node.snapshot()['inspection']['confirmed_angle_deg'], 12.)
        self.command('take_control')
        self.node.handle_command('second', {'type':'take_control'})
        self.assertEqual(self.node.snapshot()['inspection']['phase'], 'CONFIRMED')
        self.assertFalse(self.node.owns_lease(self.owner))
        self.assertTrue(self.node.owns_lease('second'))

    def test_delayed_transport_stops_only_the_same_owner_and_work(self):
        def deliver(client, seq, payload, age=0., event='command'):
            self.node._transport_command(String(data=json.dumps(dict(client_id=client, sequence=seq,
                stamp=gateway_module.time.time()-age, payload=payload, event=event))))
        deliver('a',1,{},event='heartbeat')
        deliver('a',2,dict(type='take_control'))
        with patch.object(self.node._result_pub,'publish') as result, patch.object(self.node,'_actuator_command') as actuator:
            actuator.return_value=None
            deliver('a',3,{},age=2.,event='heartbeat')
            deliver('a',4,dict(type='drive',deadman=True,linear_x=.8),age=2.)
            result.assert_not_called()
            deliver('a',6,dict(type='arm_servo',action='home',role='probe',held=True,request_id='start'))
            actuator.reset_mock()
            deliver('a',5,dict(type='arm_servo',action='stop',role='probe'),age=1.)
            actuator.assert_not_called()
            deliver('a',7,dict(type='arm_servo',action='stop',role='probe'),age=1.)
            self.assertEqual(actuator.call_args.args[0]['action'],'stop')
            deliver('a',8,dict(type='probe_target',request_id='expired-click'),age=2.)
            self.assertEqual(json.loads(result.call_args.args[0].data)['request_id'],'expired-click')
            deliver('b',9,{},event='heartbeat')
            deliver('b',10,dict(type='take_control'))
            deliver('a',11,dict(type='estop'),age=1.)
            self.assertTrue(self.node.owns_lease('b'))
            deliver('b',12,dict(type='estop'),age=1.)
            self.assertFalse(self.node.owns_lease('b'))

    def test_delayed_inspection_and_pump_releases_are_scoped(self):
        self.node._lease='a'
        self.node._client_seen['a']=100.
        with patch.object(self.node,'handle_command',return_value=None) as command:
            for kind, start in [('pump_manual','start'), ('inspection','confirm')]:
                def deliver(seq, action, age):
                    self.node._transport_command(String(data=json.dumps(dict(client_id='a',sequence=seq,
                        stamp=gateway_module.time.time()-age, payload=dict(type=kind,action=action)))))
                deliver(5,start,0.)
                command.reset_mock()
                deliver(4,'stop',1.)
                command.assert_not_called()
                deliver(6,'stop',1.)
                command.assert_called_once_with('a',dict(type=kind,action='stop'))

    def test_detector_calibration_requires_control_and_fresh_sensor(self):
        self.node._odometry_at = 0.
        command = dict(action='apply', request_id='trigger', baseline_adc=20, full_response_adc=150)
        self.assertIsNotNone(self.command('detector_calibrate', **command))
        self.command('take_control')
        with patch.object(self.node._detector_telemetry, 'snapshot', return_value={'fresh': True}), \
                patch.object(self.node._detector_command_pub, 'publish') as publish:
            self.assertIsNone(self.command('detector_calibrate', **command))
            self.assertEqual(json.loads(publish.call_args.args[0].data), command)

    def test_manual_detector_calibration_needs_no_fresh_reading(self):
        self.command('take_control')
        with patch.object(self.node._detector_telemetry,'snapshot',return_value={'fresh':False}), patch.object(self.node._detector_command_pub,'publish') as publish:
            self.assertIsNone(self.command('detector_calibrate',action='apply',baseline_adc=125.7,full_response_adc=126.2))
            self.assertEqual(json.loads(publish.call_args.args[0].data)['full_response_adc'],126.2)
            self.assertIsNotNone(self.command('detector_calibrate',action='zero'))

    def test_browser_and_ros_joy_drive_through_payload_activity(self):
        from peaceofmine_interfaces.msg import ActuatorStatus
        self.arm()
        self.node._probe_state_cb(ActuatorStatus(probe_cycle_active=True, depth_mm=40.))
        self.node._pump_active = True
        self.assertIsNone(self.command('drive', linear_x=.4, deadman=True))
        self.output.assert_called_with(.4, 0.)
        self.node._joy_cb(Joy(axes=[0., 0., 1., 0., 0., -1.], buttons=[0]*11))
        self.output.assert_called_with(.8, 0.)
        self.node._emit_cmd_vel()
        self.output.assert_called_with(.8, 0.)
        self.assertTrue(self.node.snapshot()['drive']['publishing'])

    def test_mark_is_independent_of_odometry_and_probe_setup_but_rc_still_stops_it(self):
        from test_rc_safety import rc, state
        from peaceofmine_operator.marking_settings import DEFAULTS
        self.node._rc_safety.simulation=False
        self.node._state_cb(state())
        self.node._rc_cb(rc())
        self.node._marking_settings_cb(String(data=json.dumps(dict(marking=DEFAULTS))))
        self.command('take_control')
        self.node._odometry_at=0.
        self.node._probe_state={}
        self.assertIsNone(self.command('pump_mark'))
        self.node._drive_watchdog()
        self.assertTrue(self.node._pump_active)
        self.node._rc_cb(rc(2000000,pwm=2000))
        self.assertFalse(self.node._pump_active)
        self.assertFalse(self.node._armed)

    def test_align_angle_survives_typed_command(self):
        from peaceofmine_interfaces.msg import ActuatorStatus
        self.arm()
        self.node._probe_state_cb(ActuatorStatus(probe_ready=True, depth_mm=0.))
        with patch.object(self.node._arm_command_pub, 'publish') as publish:
            self.assertIsNone(self.command('align_probe', angle_deg=-15.))
            self.assertEqual(command_dict(publish.call_args.args[0])['angle_deg'], -15.)

    def test_shutdown_stops_outputs_and_rejects_new_control(self):
        self.arm()
        self.command('drive', linear_x=.4, deadman=True)
        with patch.object(self.node._permission_pub, 'publish') as permission, \
                patch.object(self.node._arm_command_pub, 'publish') as servo:
            self.node.begin_shutdown()
            self.output.assert_called_with(0.0, 0.0)
            self.assertFalse(permission.call_args.args[0].data)
            self.assertEqual(command_dict(servo.call_args.args[0]), {'action':'stop'})
            self.assertIsNotNone(self.command('take_control'))
            self.assertIsNotNone(self.command('arm'))
            self.node._drive_watchdog()
            self.assertFalse(permission.call_args.args[0].data)
            self.assertFalse(self.node._armed)

    def test_servo_commands_work_in_rc_override_while_drive_is_blocked(self):
        from mavros_msgs.msg import State, RCIn
        self.node._rc_safety.simulation = False
        message = State(connected=True, armed=True, system_status=4)
        message.header.stamp.sec = 100
        self.node._state_cb(message)
        remote = RCIn(channels=[1700, 1800, 1500, 1500, 1500], rssi=100)
        remote.header.stamp.sec = 100
        self.node._rc_cb(remote)
        self.arm()
        self.assertTrue(self.node._armed)
        with patch.object(self.node._arm_command_pub, 'publish') as servo, \
                patch.object(self.node._permission_pub, 'publish') as permission:
            self.assertIsNone(self.command('arm_servo', action='move', point='center', held=True))
            self.assertTrue(servo.called)
            self.node._drive_watchdog()
            self.assertTrue(permission.call_args.args[0].data)
            self.assertIsNotNone(self.command('drive', linear_x=.4, deadman=True))
            remote.channels[4] = 2000
            remote.header.stamp.sec = 101
            self.node._rc_cb(remote)
            self.assertFalse(self.node._armed)
            self.assertFalse(permission.call_args.args[0].data)
            self.assertIsNotNone(self.command('arm'))

    def test_only_lease_owner_can_actuate_or_calibrate(self):
        self.command('take_control')
        spectator = object()
        self.node._detector_telemetry.update(dict(fresh=True))
        with patch.object(self.node._sweep_enabled_pub, 'publish') as sweep, \
                patch.object(self.node._detector_command_pub, 'publish') as detector:
            for payload in (dict(type='sweep_enabled', enabled=True), dict(type='sweep_enabled', enabled=False),
                            dict(type='detector_calibrate', action='zero'), dict(type='probe_target', depth_mm=20),
                            dict(type='arm_servo', action='stop'), dict(type='drive', linear_x=.4)):
                self.assertIn('Spectator', self.node.handle_command(spectator, payload)['message'])
            sweep.assert_not_called()
            detector.assert_not_called()
            self.node.handle_command(spectator, dict(type='take_control'))
            self.assertFalse(sweep.call_args.args[0].enabled)
            self.assertIs(self.node._lease, spectator)
            self.assertIn('Spectator', self.command('sweep_enabled', enabled=True)['message'])
            self.assertIsNone(self.node.handle_command(spectator, dict(type='detector_calibrate', action='zero', request_id='new-owner')))
            detector.assert_called_once()

    def test_rc_loss_stops_motion_and_recovery_requires_fresh_commands(self):
        try:
            from test_rc_safety import healthy
        except ImportError:
            from .test_rc_safety import healthy
        self.node._rc_safety.simulation = False
        healthy(self.node._rc_safety)
        self.arm()
        self.command('drive', linear_x=.4, deadman=True)
        self.assertTrue(self.node._armed)
        with patch.object(self.node._probe_command_pub, 'publish') as probe, patch.object(self.node._sweep_enabled_pub, 'publish') as sweep:
            from test_rc_safety import rc
            self.node._rc_cb(rc(2000000,pwm=2000))
            self.node._drive_watchdog()
            self.assertFalse(self.node._armed)
            self.output.assert_called_with(0.0, 0.0)
            self.assertFalse(sweep.call_args.args[0].enabled)
            for kind in ('arm', 'drive', 'probe_target', 'sweep_enabled', 'sweep_speed'):
                self.assertIsNotNone(self.command(kind, deadman=True))
            self.assertTrue(all(command_dict(call.args[0])['action'] == 'stop' for call in probe.call_args_list))
            self.node._rc_cb(rc(3000000))
            self.node._drive_watchdog()
            self.assertTrue(self.node._armed)
            self.assertFalse(self.node._deadman)
            self.assertEqual(self.node._command, (0.0, 0.0))

    def test_calibration_blocks_browser_and_ros_vehicle_input(self):
        self.command('take_control')
        self.command('arm', calibration=True)
        self.assertTrue(self.node._armed)
        self.assertIsNotNone(self.command('drive', linear_x=.4, deadman=True))
        self.assertIsNotNone(self.command('probe_target', depth_mm=20))
        self.node._joy_cb(Joy(axes=[0., 0., 1., 0., 0., -1.], buttons=[0] * 11))
        self.node._drive_watchdog()
        self.assertEqual(self.node._command, (0.0, 0.0))
        self.assertFalse(self.node.snapshot()['drive']['publishing'])
        self.command('disarm')
        self.assertFalse(self.node._calibrating)

    def test_browser_requires_lease_rc_permission_and_deadman(self):
        drive = dict(linear_x=0.4, angular_z=-0.2)
        self.assertIsNotNone(self.command('drive', **drive))
        self.command('take_control')
        self.assertTrue(self.node._armed)
        self.assertIsNone(self.command('drive', **drive))
        self.assertFalse(self.node.snapshot()['drive']['publishing'])
        self.command('drive', **dict(drive,deadman=True))
        self.output.assert_called_with(0.4, -0.2)
        self.command('drive', **dict(drive, deadman=False))
        self.output.assert_called_with(0.0, 0.0)

    def test_estop_releases_control_and_cannot_auto_resume(self):
        self.command('take_control')
        self.command('drive', linear_x=.4, deadman=True)
        self.command('estop')
        self.node._drive_watchdog()
        self.assertIsNone(self.node._lease)
        self.assertFalse(self.node._armed)
        self.assertIsNotNone(self.command('drive', linear_x=.4, deadman=True))
        self.command('take_control')
        self.assertTrue(self.node._armed)
        self.assertFalse(self.node._deadman)

    def test_probe_reverse_boolean_survives_browser_dispatch(self):
        self.command('take_control')
        with patch.object(self.node._probe_command_pub,'publish') as publish:
            for reverse in (False,True):
                self.assertIsNone(self.command('arm_servo',role='probe',action='configure_direction',enabled=reverse))
                command=command_dict(publish.call_args.args[0])
                self.assertIs(command['enabled'],reverse)
                self.assertEqual(command['action'],'configure_direction')

    def test_deliberate_drive_cancels_confirmation_and_neutral_input_preserves_it(self):
        from unittest.mock import Mock
        self.command('take_control')
        self.node._arm_state=dict(connected=True,torque=False,calibration=dict(minimum=1000,center=1500,maximum=2000))
        self.node._arm_state_at=100.
        self.node._probe_state={}
        self.node._inspection_detector=dict(fresh=True,latest_peak=None,sweep_speed_deg_s=36.)
        self.node._inspection_detector_at=100.
        self.node._inspection_clients={'arm':Mock(),'probe':Mock()}
        self.assertIsNone(self.command('inspection',action='confirm'))
        self.assertTrue(self.node._inspection.active)
        self.assertIsNone(self.command('drive', linear_x=0., deadman=True))
        self.assertTrue(self.node._inspection.active)
        self.assertIsNone(self.command('drive', linear_x=.5, deadman=True))
        self.output.assert_called_with(.5, 0.)
        self.assertEqual(self.node._inspection.phase, 'STOPPED')
        self.assertFalse(self.node._inspection.active)
        self.assertEqual(self.node._lease,self.owner)

    def test_manual_servo_enters_calibration_without_arm_command(self):
        self.command('take_control')
        self.command('drive', linear_x=.4, deadman=True)
        with patch.object(self.node._arm_command_pub, 'publish') as servo:
            self.assertIsNone(self.command('arm_servo', action='jog', direction=1, held=True))
            self.assertEqual(command_dict(servo.call_args.args[0])['action'], 'jog')
        self.assertTrue(self.node._calibrating)
        self.assertFalse(self.node._deadman)
        self.output.assert_called_with(0.0, 0.0)
        self.assertIsNotNone(self.command('drive', linear_x=.4, deadman=True))
        self.command('calibration_end')
        self.assertFalse(self.node._calibrating)
        self.assertTrue(self.node._armed)
        self.assertFalse(self.node._deadman)

    def test_sweep_with_rc_override_needs_no_ui_arm(self):
        from mavros_msgs.msg import RCIn
        from test_rc_safety import healthy
        self.node._rc_safety.simulation = False
        healthy(self.node._rc_safety)
        remote = RCIn(channels=[1750, 1250, 1500, 1500, 1500], rssi=100)
        remote.header.stamp.sec = 2000000
        self.node._rc_cb(remote)
        self.command('take_control')
        self.node._arm_state = dict(connected=True, calibration=dict(minimum=1000, center=2000, maximum=3000))
        self.node._arm_state_at = 100.0
        with patch.object(self.node._sweep_enabled_pub, 'publish') as sweep, patch.object(self.node._permission_pub, 'publish') as permission:
            self.assertIsNone(self.command('sweep_enabled', enabled=True))
            self.assertTrue(sweep.call_args.args[0].enabled)
            self.node._drive_watchdog()
            self.assertTrue(permission.call_args.args[0].data)
            self.assertIsNotNone(self.command('drive', linear_x=.4, deadman=True))
            self.assertIsNone(self.command('drive', linear_x=0., deadman=False))
            controls = self.node.snapshot()['drive']['rc']
            self.assertEqual((controls['steering'], controls['throttle']), (.5, -.5))
            remote.header.stamp.sec += 1
            remote.channels[4] = 2000
            self.node._rc_cb(remote)
            self.assertFalse(permission.call_args.args[0].data)
            self.assertFalse(sweep.call_args.args[0].enabled)
            self.assertIsNotNone(self.command('sweep_enabled', enabled=True))

    def test_car_enforces_both_keyboard_limits_and_selected_source(self):
        self.arm()
        self.node._drive_settings_cb(String(data=json.dumps(dict(
            driving=dict(input='keyboard', forward_percent=50, reverse_percent=25)))))
        self.command('drive', input='keyboard', deadman=True, linear_x=1.)
        self.output.assert_called_with(.4, 0.)
        self.command('drive', input='keyboard', deadman=True, linear_x=-1.)
        self.output.assert_called_with(-.2, 0.)
        self.assertIsNotNone(self.command('drive', input='wheel', deadman=True, linear_x=.8))
        self.node._joy_cb(Joy(axes=[0., 0., 1., 0., 0., -1.], buttons=[0] * 11))
        self.assertEqual(self.node._command, (-.2, 0.))

    def test_rc_source_requires_owner_presence_and_new_valid_samples(self):
        from test_rc_safety import rc, state
        self.node._rc_safety.simulation = False
        self.node._state_cb(state())
        self.node._drive_settings_cb(String(data=json.dumps(dict(
            driving=dict(input='rc', forward_percent=40, reverse_percent=20)))))
        stamp = 100
        def sample(position=1000):
            nonlocal stamp
            stamp += 1
            msg = rc(stamp, pwm=position)
            msg.channels[0], msg.channels[1] = 1750, 1250
            self.node._rc_cb(msg)
            return msg

        sample()
        self.assertIsNotNone(self.command('rc_drive_enable', enabled=True))
        self.command('take_control')
        sample()
        self.assertFalse(self.node.snapshot()['drive']['publishing'])
        self.node._probe_state = {'probe': {'cycle_active': True, 'depth_mm': 23.}}
        self.node._pump_active = True
        self.node._inspection.confirm(-45., 45., 30.)
        self.command('rc_drive_enable', enabled=True)
        self.assertTrue(self.node._inspection.active)
        msg = sample()
        self.assertFalse(self.node._inspection.active)
        self.output.assert_called_with(-.4, -.7)
        self.assertEqual(self.node.snapshot()['drive']['command_source'], 'rc')
        self.assertIsNotNone(self.command('drive', deadman=False))
        self.node._joy_cb(Joy(axes=[0., 0., 1., 0., 0., -1.], buttons=[0] * 11))
        self.assertEqual(self.node._command, (-.4, -.7))

        self.now.return_value += .26
        sample()
        self.output.assert_called_with(0., 0.)
        self.command('rc_drive_enable', enabled=True)
        msg = sample()
        self.now.return_value += .26
        self.command('rc_drive_enable', enabled=True)
        self.node._rc_cb(msg)  # Repeated PWM frame does not renew motion.
        self.node._drive_watchdog()
        self.output.assert_called_with(0., 0.)
        sample(1500)
        self.assertEqual(self.node.snapshot()['safety']['mode'], 'override')
        self.assertFalse(self.node.snapshot()['drive']['publishing'])
        sample(2000)
        self.assertFalse(self.node.snapshot()['drive']['armed'])
        sample()
        self.assertFalse(self.node.snapshot()['drive']['publishing'])
        self.command('rc_drive_enable', enabled=True)
        sample()
        self.command('calibration_begin')
        sample()
        self.output.assert_called_with(0., 0.)
        self.command('calibration_end')
        self.command('rc_drive_enable', enabled=True)
        sample()
        self.command('estop')
        sample()
        self.assertFalse(self.node.snapshot()['drive']['publishing'])

    def test_browser_timeout_disarm_and_disconnect_stop(self):
        for stop in ('timeout', 'disarm', 'disconnect'):
            with self.subTest(stop=stop):
                self.arm()
                self.command('drive', linear_x=0.4, angular_z=0.0, deadman=True)
                self.output.assert_called_with(0.4, 0.0)
                if stop == 'timeout':
                    self.now.return_value += 0.3
                    self.node._drive_watchdog()
                elif stop == 'disarm':
                    self.command('disarm')
                else:
                    self.node.release(self.owner)
                self.output.assert_called_with(0.0, 0.0)
                self.now.return_value += 1.0
                self.output.reset_mock()
                self.node._drive_watchdog()
                self.output.assert_not_called()

    def test_ros_xbox_triggers_and_back_start(self):
        self.arm()
        buttons = [0] * 11
        for lt, rt, velocity in ((1., 1., 0.), (1., -1., 0.8),
                                 (-1., 1., -0.8), (-1., -1., 0.)):
            with self.subTest(lt=lt, rt=rt):
                self.node._joy_cb(Joy(axes=[0., 0., lt, 0., 0., rt], buttons=buttons))
                self.assertAlmostEqual(self.node._joy['linear'], velocity)
                self.assertEqual(self.node._joy['deadman'], velocity != 0.)
        buttons[6] = buttons[7] = 1
        self.node._joy_cb(Joy(axes=[0., 0., 1., 0., 0., 1.], buttons=buttons))
        self.assertEqual(self.node._joy['linear'], 0.)
        self.assertFalse(self.node._joy['deadman'])

    def test_uninitialized_ros_triggers_are_released(self):
        parsed = self.node._parse_xbox_joy(Joy(axes=[0.] * 6, buttons=[0] * 11))
        self.assertEqual((parsed['lt'], parsed['rt']), (0., 0.))
        self.assertFalse(parsed['deadman'])


if __name__ == '__main__':
    unittest.main()
