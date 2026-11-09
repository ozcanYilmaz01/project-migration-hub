#!/usr/bin/env python3
"""ROS operator control: lease arbitration, safety gates and motion coordination."""
from __future__ import annotations
import json
import math
import uuid
import threading
import time
import uuid
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from mavros_msgs.msg import State, RCIn
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, String
from peaceofmine_operator import marking_settings
from peaceofmine_operator.rc_safety import RcSafety
from peaceofmine_operator.transport_clock import transport_age
from peaceofmine_operator import driving_settings, inspection, probe_alignment, probe_scan
from peaceofmine_operator.detector import DetectorTelemetry
from peaceofmine_operator.inspection import InspectionWorkflow
from peaceofmine_interfaces.action import ActuatorOperation
from peaceofmine_operator.node_runner import run_node
from peaceofmine_operator.actuator_protocol import command_message, status_dict
from peaceofmine_interfaces.msg import ActuatorCommand, ActuatorStatus

# Authoritative actuator limits. The dashboard reads these from telemetry so
# the browser never carries its own copy.
SWEEP_SPEED_MIN_DEG_S = 5.0
SWEEP_SPEED_MAX_DEG_S = 180.0

# After the drive command stops being valid, keep publishing zeros for this
# long and then go silent, so an idle gateway does not hold the downstream
# twist_consumer watchdog open or fight other cmd_vel publishers.
STOP_TAIL_S = 0.5


class OperatorControl(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('operator_control', **kwargs)
        self.declare_parameter('safety_simulation', False)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('odometry_topic', 'odometry/local')
        self.declare_parameter('velocity_topic', '')
        self.declare_parameter('fixture_sweep_enabled_topic', 'fixture/sweep_enabled')
        self.declare_parameter('fixture_sweep_speed_topic', 'fixture/sweep_speed_deg_s')
        self.declare_parameter('probe_fault_topic', 'probe/fault')
        self.declare_parameter('command_timeout_s', 0.25)
        self.declare_parameter('max_velocity_mps', 0.8)
        self.declare_parameter('max_yaw_rate_rad_s', 1.4)
        self.declare_parameter('probe_motion_speed_limit_mps', 0.03)
        self.declare_parameter('rc_steering_channel', 1)
        self.declare_parameter('rc_throttle_channel', 2)
        self.declare_parameter('fixture_angle_offset_deg', 0.)

        self._lock = threading.RLock()
        self._shutting_down = False
        self._rc_safety = RcSafety(bool(self.get_parameter('safety_simulation').value), clock=lambda: time.monotonic())
        self._arm_state = {'connected': False, 'reason': 'Arm driver not running'}
        self._arm_state_at = 0.0
        self._permission_pub = self.create_publisher(Bool, 'operator/actuation_enabled', 1)
        self._arm_command_pub = self.create_publisher(ActuatorCommand, 'arm/command', 10)
        self._probe_command_pub = self.create_publisher(ActuatorCommand, 'probe/command', 10)
        self._field_log = self.create_publisher(String, 'field/log', 10)
        self._field_clear = self.create_publisher(Bool, 'field/clear', 10)
        self._confirmation_passes = inspection.DEFAULTS['passes']
        self._probe_pattern = probe_scan.DEFAULTS
        self._alert_options = {'enabled': True, 'level': 60}
        self._search_alert = {'phase':'idle'}
        self.create_subscription(String, 'operator/settings', self._mission_settings_cb, 10)
        self._last_probe_log = None
        self._last_inspection_phase = 'IDLE'
        self.create_subscription(String, 'probe/web_state', self._log_probe_result, 10)
        self._probe_state = {}
        self._probe_state_at = 0.0
        self._selected_actuator = 'arm'
        self.create_subscription(ActuatorStatus, 'probe/state', self._probe_state_cb, 10)
        self.create_subscription(State, 'mavros/state', self._state_cb, 10)
        self.create_subscription(RCIn, 'mavros/rc/in', self._rc_cb, 10)
        self.create_subscription(ActuatorStatus, 'arm/state', self._arm_state_cb, 1)
        self._lease = None
        # Do not call this `_clients`: rclpy Node uses that private attribute
        # for ROS service clients while its executor spins.
        self._connected_clients = set()
        self._client_seen = {}
        self._motion_fence = {}
        self._calibrating = False
        self._armed = False
        self._deadman = False
        self._command = (0.0, 0.0)
        self._last_command = 0.0
        self._drive_input = driving_settings.DEFAULTS['input']
        self._keyboard_limits = (1., 1.)
        self._command_source = 'none'
        self._rc_enabled_until = 0.
        self.create_subscription(String, 'operator/settings', self._drive_settings_cb, 10)
        self._last_joy = 0.0
        self._joy_prev_a = False
        self._trigger_seen = {'lt': False, 'rt': False}
        self._joy = {
            'seen': False,
            'age_ms': None,
            'stick_x': 0.0,
            'stick_y': 0.0,
            'lt': 0.0,
            'rt': 0.0,
            'lb': False,
            'rb': False,
            'a': False,
            'b': False,
            'x': False,
            'y': False,
            'deadman': False,
        }
        self._stop_until = 0.0
        self._detector_telemetry = DetectorTelemetry()
        self._detector_command_pub = self.create_publisher(String, 'detector/command', 10)
        self.create_subscription(String, 'detector/state', self._detector_state_cb, 10)
        self._probe_fault = False
        self._pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._pose_at = 0.0
        self._speed = 0.0
        self._motion_speed = math.inf
        self._odometry_at = 0.0

        self._max_velocity = float(self.get_parameter('max_velocity_mps').value)
        self._max_yaw_rate = float(self.get_parameter('max_yaw_rate_rad_s').value)
        self._timeout = float(self.get_parameter('command_timeout_s').value)
        self._probe_speed_limit = float(self.get_parameter('probe_motion_speed_limit_mps').value)

        self._pump_active = False
        self._pump_mode = 'idle'
        self._pump_until = 0.
        self._pump_output = 0.
        self._marking = None
        self._last_shot_request = None
        self._mark_pub = self.create_publisher(String, 'field/mark', 10)
        self._pump_pub = self.create_publisher(Float32, 'actuation/pump_percent', 1)
        self.create_subscription(String, 'operator/settings', self._marking_settings_cb, 10)

        self._cmd_pub = self.create_publisher(Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self._sweep_enabled_pub = self.create_publisher(ActuatorCommand, 'arm/command', 1)
        self._sweep_speed_pub = self.create_publisher(Float32, str(self.get_parameter('fixture_sweep_speed_topic').value), 10)
        self.create_subscription(Odometry, str(self.get_parameter('odometry_topic').value), self._odometry_cb, 10)
        velocity_topic = str(self.get_parameter('velocity_topic').value)
        if velocity_topic:
            from rclpy.qos import qos_profile_sensor_data
            self.create_subscription(TwistWithCovarianceStamped, velocity_topic, self._velocity_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, str(self.get_parameter('probe_fault_topic').value), self._probe_fault_cb, 10)
        self.create_subscription(Joy, str(self.get_parameter('joy_topic').value), self._joy_cb, 10)
        self.create_timer(0.05, self._drive_watchdog)
        self._state_pub = self.create_publisher(String, 'operator/state', 10)
        self._result_pub = self.create_publisher(String, 'operator/result', 10)
        self._adc_command_pub = self.create_publisher(String, 'adc/command', 10)
        self._settings_command_pub = self.create_publisher(String, 'operator/settings/command', 10)
        self.create_subscription(String, 'operator/command', self._transport_command, 50)
        self._inspection_serial = 0
        self._inspection_goal = None
        self._inspection_detector, self._inspection_detector_at = {}, 0.
        self._inspection_clients = {role:ActionClient(self, ActuatorOperation, role+'/execute') for role in ('arm','probe')}
        self._inspection = InspectionWorkflow(self._inspection_execute, self._inspection_cancel, self._inspection_clear_peak, clock=lambda:time.monotonic())
        self.create_subscription(String, 'detector/control_state', self._inspection_detector_cb, 1)
        self.create_timer(.05, self.publish_state)

    def _mission_settings_cb(self, msg):
        try:
            settings = json.loads(msg.data)
            self._confirmation_passes = inspection.validate(settings.get('inspection', inspection.DEFAULTS))['passes']
            alignment = settings.get('probe_alignment', probe_alignment.DEFAULTS)
            self._inspection.probe_offset = probe_alignment.validate(alignment)['offset_deg']
            self._probe_pattern = probe_scan.validate(settings.get('probe_scan', probe_scan.DEFAULTS))
            self._alert_options = settings['display']['searchAlert']
        except (ValueError, KeyError, TypeError):
            pass

    def _update_search_alert(self, now):
        reading = self._inspection_detector
        volts = reading.get('latest_filtered_volts')
        base, full = reading.get('baseline_volts'), reading.get('full_response_volts')
        alert = self._search_alert
        live = (self._alert_options['enabled'] and now - self._inspection_detector_at < .5 and
                reading.get('fresh') and all(type(v) in (int, float) and math.isfinite(v) for v in (volts, base, full))
                and full != base and not self._calibrating and not self._inspection.active and not self._probe_active())
        if not live:
            if alert['phase'] in ('active', 'cooling'):
                alert['cleared_at'] = time.time()
            alert['phase'] = 'idle'
            return
        limit = self._alert_options.get('volts')
        if limit is None:
            limit = base + (full - base) * self._alert_options['level'] / 100
        direction = 1 if full > base else -1
        distance = (volts - limit) * direction
        rearm_margin = abs(full - base) * .05
        phase = alert['phase']
        if phase == 'idle' and distance >= 0:
            alert.update(phase='pending', since=now)
        elif phase == 'pending':
            if distance < 0:
                alert['phase'] = 'idle'
            elif now - alert['since'] >= .15:
                alert.update(phase='active', fired_at=time.time(), peak_volts=volts)
                self._log_field_event('SEARCH ALERT', 'alert')
        elif phase in ('active', 'cooling'):
            if (volts - alert['peak_volts']) * direction > 0:
                alert['peak_volts'] = volts
            if phase == 'active' and distance < -rearm_margin:
                alert.update(phase='cooling', since=now)
            elif phase == 'cooling':
                if distance >= 0:
                    alert['phase'] = 'active'
                elif now - alert['since'] >= 1.5:
                    alert.update(phase='idle', cleared_at=time.time())

    def _inspection_detector_cb(self, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                with self._lock:
                    self._inspection_detector = value
                    self._inspection_detector_at = time.monotonic()
                    self._inspection.record_detector(value)
        except (ValueError, TypeError):
            self._inspection_detector = {}

    def _inspection_clear_peak(self):
        self._detector_command_pub.publish(String(data=json.dumps(dict(action='clear_peak'))))

    def _inspection_cancel(self):
        self._inspection_serial += 1
        if self._inspection_goal is not None:
            try:
                self._inspection_goal.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'Inspection cancellation failed: {exc}')
            self._inspection_goal = None
        self._arm_command_pub.publish(command_message(dict(action='stop')))
        self._probe_command_pub.publish(command_message(dict(action='stop')))

    def _inspection_execute(self, role, operation, target, speed, complete):
        client = self._inspection_clients[role]
        if not client.server_is_ready():
            raise ValueError(f'{role.title()} action server unavailable')
        self._inspection_serial += 1
        serial = self._inspection_serial
        future = client.send_goal_async(ActuatorOperation.Goal(operation=operation, target=float(target), speed=float(speed)))

        def accepted(future):
            with self._lock:
                try:
                    goal = future.result()
                    if serial != self._inspection_serial:
                        if goal.accepted:
                            goal.cancel_goal_async()
                        return
                    if not goal.accepted:
                        complete(dict(success=False, message=f'{role.title()} rejected the operation; check its setup and connection.'))
                        return
                    self._inspection_goal = goal

                    def finished(result_future):
                        with self._lock:
                            if serial != self._inspection_serial:
                                return
                            self._inspection_goal = None
                            try:
                                result = result_future.result().result
                                complete(dict(success=result.success, message=result.message,
                                              contact=result.contact, contact_depth_mm=result.contact_depth_mm))
                            except Exception as exc:
                                complete(dict(success=False, message=str(exc)))

                    goal.get_result_async().add_done_callback(finished)
                except Exception as exc:
                    if serial == self._inspection_serial:
                        complete(dict(success=False, message=str(exc)))

        future.add_done_callback(accepted)

    def _inspection_tick(self, now):
        if not self._inspection.active:
            return
        step = self._inspection.pending
        next_step = step or (self._inspection.queue[0][0] if self._inspection.queue else None)
        needs_probe = self._inspection.phase == 'PROBING' or next_step == 'home'
        for role, last_update in (('arm', self._arm_state_at), ('probe', self._probe_state_at)):
            if (role == 'arm' or needs_probe) and now-last_update >= 1.:
                self._inspection.stop(f'{role.title()} telemetry timed out', error=True)
                return
        if step == 'pass' and (now-self._inspection_detector_at >= .5 or not self._inspection_detector.get('fresh')):
            self._inspection.stop('Detector readings lost during confirmation', error=True)
            return
        role = 'probe' if step in ('home', 'probe') else 'arm'
        if step and not self._inspection_clients[role].server_is_ready():
            self._inspection.stop(f'{role.title()} action server disconnected', error=True)
            return
        # The action reports the actual motor outcome. Between actions wait for
        # fresh, released servos so an old status cannot start the next move.
        arm_idle = (now-self._arm_state_at < .5 and self._arm_state.get('connected')
                    and not self._arm_state.get('torque') and self._arm_state.get('lifecycle') == 'IDLE')
        probe_idle = (now-self._probe_state_at < .5 and self._probe_state.get('connected')
                      and not self._probe_state.get('torque') and self._probe_state.get('lifecycle') == 'IDLE')
        idle = arm_idle and (probe_idle or (not needs_probe and not self._probe_state.get('connected')))
        self._inspection.tick(idle)

    def _start_inspection(self, payload):
        action = payload.get('action')
        if action not in ('confirm', 'probe_scan'):
            raise ValueError('Unknown inspection action')
        if self._inspection.active:
            raise ValueError('An inspection is already running')
        if self._calibrating:
            raise ValueError('Finish actuator calibration before inspecting')
        if self._probe_state.get('probe', {}).get('cycle_active') or self._pump_active:
            raise ValueError('Finish the active probe or marking operation first')
        calibration = self._arm_state.get('calibration')
        if time.monotonic()-self._arm_state_at >= .5 or not self._arm_state.get('connected') or not calibration:
            raise ValueError('Connect and calibrate the arm in Setup')
        offset = float(self.get_parameter('fixture_angle_offset_deg').value)
        lower = (calibration['minimum']-calibration['center'])*360/4096+offset
        upper = (calibration['maximum']-calibration['center'])*360/4096+offset
        if not self._inspection_clients['arm'].server_is_ready():
            raise ValueError('Arm action server unavailable')
        if action == 'confirm':
            if time.monotonic()-self._inspection_detector_at >= .5 or not self._inspection_detector.get('fresh'):
                raise ValueError('Fresh detector readings are required for confirmation')
            self._stop_motion_locked(for_inspection=True)
            self._inspection.confirm(lower, upper, self._inspection_detector.get('sweep_speed_deg_s'),
                                     self._confirmation_passes, home_probe=bool(self._probe_state.get('connected')))
        else:
            if self._inspection.confirmed_angle is None:
                raise ValueError('Confirm a detection before starting the probe scan')
            probe = self._probe_state.get('probe', {})
            depth = payload.get('depth_mm')
            if (type(depth) not in (int,float) or not math.isfinite(depth)
                    or not 0 < depth <= probe.get('max_depth_mm', 0)):
                raise ValueError('Choose a positive probe depth within its calibrated travel')
            if self._probe_fault:
                raise ValueError('Probe fault active')
            if time.monotonic()-self._probe_state_at >= .5 or not self._probe_state.get('connected'):
                raise ValueError('Waiting for probe encoder telemetry')
            if not probe.get('ready'):
                raise ValueError('Save the measured probe travel in Setup → Probe')
            if not self._inspection_clients['probe'].server_is_ready():
                raise ValueError('Probe action server unavailable')
            self._stop_motion_locked(stop_inspection=False, for_inspection=True)
            self._inspection.probe(lower, upper, depth, self._probe_pattern)
        self._stop_until = time.monotonic()+STOP_TAIL_S
        self._publish_twist(0., 0.)

    def _arm_state_cb(self, message):
        try:
            value = status_dict(message)
            if isinstance(value, dict):
                with self._lock:
                    self._arm_state = value
                    self._arm_state_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def _state_cb(self, message):
        with self._lock:
            self._rc_safety.update_state(message)
            self._enforce_safety_locked()
        self._emit_cmd_vel()

    def _drive_settings_cb(self, message):
        try:
            settings = driving_settings.validate(json.loads(message.data)['driving'])
            selected = settings['input']
            limits = (settings['reverse_percent'] / 100., settings['forward_percent'] / 100.)
        except (KeyError, TypeError, ValueError):
            return
        with self._lock:
            limits_changed = limits != self._keyboard_limits
            self._keyboard_limits = limits
            if selected == self._drive_input and (selected != 'keyboard' or not limits_changed):
                return
            self._drive_input = selected
            self._rc_enabled_until = 0.
            self._apply_drive_locked(0., 0., False)
        self._emit_cmd_vel()

    def _rc_controls(self):
        return self._rc_safety.controls(int(self.get_parameter('rc_steering_channel').value),
                                        int(self.get_parameter('rc_throttle_channel').value))

    def _rc_cb(self, message):
        with self._lock:
            updated = self._rc_safety.update_rc(message)
            safety = self._enforce_safety_locked()
            if updated and self._drive_input == 'rc':
                controls = self._rc_controls()
                if (self._armed and safety['allowed'] and controls['fresh'] and not self._calibrating
                        and time.monotonic() < self._rc_enabled_until):
                    steering, throttle = controls['steering'], controls['throttle']
                    self._apply_drive_locked(throttle * self._max_velocity, -steering * self._max_yaw_rate,
                                             bool(steering or throttle), 'rc')
                else:
                    self._apply_drive_locked(0., 0., False)
        self._emit_cmd_vel()

    def _disable_actuation_locked(self):
        self._armed = False
        self._permission_pub.publish(Bool(data=False))
        self._stop_motion_locked(stop_inspection=self._inspection.active)

    def _stop_motion_locked(self, stop_inspection=True, for_inspection=False):
        if stop_inspection:
            self._inspection.stop()
        self._calibrating = False
        self._rc_enabled_until = 0.
        self._deadman = False
        self._command = (0.0, 0.0)
        action = 'stop_manual' if for_inspection else 'stop'
        if not for_inspection:
            self._sweep_enabled_pub.publish(command_message(dict(action='sweep',enabled=False)))
        self._arm_command_pub.publish(command_message(dict(action=action)))
        self._probe_command_pub.publish(command_message(dict(action=action)))
        self._stop_pump(0.)

    def _marking_settings_cb(self, message):
        try:
            settings = json.loads(message.data)
            value = marking_settings.validate(settings['marking']) if isinstance(settings, dict) and not settings.get('error') else None
        except (ValueError, TypeError, KeyError):
            value = None
        with self._lock:
            if self._marking != value:
                self._stop_pump(0.)
                self._marking = value

    def _pump_allowed(self, safety):
        return bool(self._marking and self._lease is not None and self._armed and safety['servo_allowed']
                    and (self._rc_safety.simulation or self._rc_safety.px4_armed() is True))

    def _publish_pump(self, percent):
        self._pump_output = percent
        self._pump_pub.publish(Float32(data=percent))

    def _stop_pump(self, idle):
        self._pump_active = False
        self._pump_mode = 'idle'
        self._pump_until = 0.
        if self._pump_output != idle:
            self._publish_pump(idle)

    def _pump_tick(self, now, safety):
        allowed = self._pump_allowed(safety)
        idle = self._marking['idle_percent'] if allowed else 0.
        if self._pump_active and (not allowed or now >= self._pump_until):
            self._stop_pump(idle)
        self._publish_pump(self._pump_output if self._pump_active else idle)

    def _pump_command(self, payload, safety):
        manual = payload['type'] == 'pump_manual'
        action = payload.get('action') if manual else 'shot'
        if action == 'stop':
            if self._pump_mode == 'manual':
                self._stop_pump(self._marking['idle_percent'])
            return
        if not self._pump_allowed(safety):
            reason = safety.get('servo_reason') or 'Marking requires an armed PX4 and a live RC transmitter.'
            return dict(type='error', message=reason)
        if not manual and self._calibrating:
            return dict(type='error', message='Close Setup before marking.')
        if not manual and self._probe_active():
            return dict(type='error', message='Stop the active probe operation before marking.')
        now = time.monotonic()
        if action == 'renew':
            # A delayed heartbeat must never restart a released or expired hold.
            if self._pump_mode == 'manual' and now < self._pump_until:
                self._pump_until = now + .25
            return
        if manual and action != 'start':
            return dict(type='error', message='Unknown manual pump command.')
        if self._pump_active:
            return dict(type='error', message='Pump mark cycle already in progress.')
        if manual:
            try:
                output = marking_settings.percent(payload.get('percent'))
            except ValueError as exc:
                return dict(type='error', message=str(exc))
            duration = .25
        else:
            duration = self._marking['duration_s']
            output = self._marking['active_percent']
        self._pump_active = True
        self._pump_mode = 'manual' if manual else 'shot'
        self._pump_until = now + duration
        self._publish_pump(output)
        if not manual:
            request = payload.get('request_id')
            self._last_shot_request = request if isinstance(request, str) and len(request) <= 100 else None
            pose = self._pose if self._pose_at and now - self._pose_at < .5 else {}
            self._mark_pub.publish(String(data=json.dumps(dict(
                x=pose.get('x'), y=pose.get('y'), heading_deg=math.degrees(pose['yaw']) if pose else None,
                **self._log_detector_reading(),
                id=uuid.uuid4().hex, timestamp=time.time(), duration_s=duration,
                active_percent=output, idle_percent=self._marking['idle_percent']))))

    def begin_shutdown(self):
        """Revoke control before closing transports or invalidating ROS publishers."""
        with self._lock:
            self._shutting_down = True
            self._lease = None
            self._disable_actuation_locked()
            self._stop_until = time.monotonic() + STOP_TAIL_S
            self._publish_twist(0.0, 0.0)

    def _enforce_safety_locked(self):
        safety = self._rc_safety.snapshot()
        # Compatibility telemetry field: readiness is derived from RC and the
        # control lease, never a second dashboard arming latch.
        was_armed = self._armed
        seen = self._client_seen.get(self._lease)
        owner_live = self._lease is not None and (seen is None or time.monotonic()-seen <= .5)
        self._armed = bool(safety['servo_allowed'] and owner_live and not self._shutting_down)
        if was_armed and not self._armed:
            self._disable_actuation_locked()
        elif not safety['allowed']:
            self._rc_enabled_until = 0.
            self._deadman = False
            self._command = (0.0, 0.0)
        if (self._pump_active or self._pump_output) and not self._pump_allowed(safety):
            self._stop_pump(0.)
        return safety

    ## ROS callbacks ##

    def _odometry_cb(self, message: Odometry) -> None:
        quaternion = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                         1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))
        with self._lock:
            x, y = message.pose.pose.position.x, message.pose.pose.position.y
            if all(math.isfinite(v) for v in (x, y, yaw)):
                self._pose = {'x': x, 'y': y, 'yaw': yaw}
                self._pose_at = time.monotonic()
            else:
                self._pose_at = 0.0
        if not self.get_parameter('velocity_topic').value:
            self._velocity_cb(message)

    def _velocity_cb(self, message):
        with self._lock:
            twist = message.twist.twist
            values = (twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z)
            self._motion_speed = max(abs(v) for v in values) if all(math.isfinite(v) for v in values) else math.inf
            if self._inspection.confirmed_angle is not None and self._motion_speed > self._probe_speed_limit:
                self._inspection.stop('Vehicle moved; confirm the target again.')
            self._speed = twist.linear.x if math.isfinite(twist.linear.x) else None
            self._odometry_at = time.monotonic()

    def _stationary(self):
        return time.monotonic() - self._odometry_at < .5 and self._motion_speed <= self._probe_speed_limit

    def _probe_active(self):
        probe = self._probe_state.get('probe', {})
        return bool(probe.get('active') or probe.get('holding') or probe.get('cycle_active'))

    def _detector_state_cb(self, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                with self._lock:
                    self._detector_telemetry.update(value)
        except (ValueError, TypeError):
            pass

    def _probe_fault_cb(self, message: Bool) -> None:
        with self._lock:
            self._probe_fault = bool(message.data)

    def _axis_trigger(self, key: str, axes: Any, index: int) -> float:
        if index >= len(axes):
            return 0.0
        value = float(axes[index])
        if not self._trigger_seen[key]:
            if value == 0.0:
                return 0.0
            self._trigger_seen[key] = True
        # joy_node inverts SDL axes: released is +1, pressed is -1.
        return max(0.0, min(1.0, (1.0 - value) / 2.0))

    @staticmethod
    def _deadzone(value: float) -> float:
        return 0.0 if abs(value) < 0.12 else value

    @staticmethod
    def _finite(value: float) -> float:
        return value if math.isfinite(value) else 0.0

    def _parse_xbox_joy(self, message: Joy) -> dict[str, Any]:
        axes = message.axes
        buttons = message.buttons

        def pressed(index: int) -> bool:
            return index < len(buttons) and bool(buttons[index])

        # ROS Xbox buttons 6/7 are Back/Start, not browser triggers.
        left_trigger = self._axis_trigger('lt', axes, 2)
        right_trigger = self._axis_trigger('rt', axes, 5)
        stick_x = float(axes[0]) if axes else 0.0
        stick_y = float(axes[1]) if len(axes) > 1 else 0.0
        steering = self._deadzone(stick_x)
        throttle = right_trigger - left_trigger
        left_bumper = pressed(4)
        right_bumper = pressed(5)
        return {
            'seen': True,
            'age_ms': 0,
            'stick_x': stick_x,
            'stick_y': stick_y,
            'lt': left_trigger,
            'rt': right_trigger,
            'lb': left_bumper,
            'rb': right_bumper,
            'a': pressed(0),
            'b': pressed(1),
            'x': pressed(2),
            'y': pressed(3),
            'deadman': left_bumper or right_bumper or abs(throttle) > 0.12,
            'linear': throttle * self._max_velocity,
            'angular': steering * self._max_yaw_rate,
        }

    def _apply_drive_locked(self, linear: float, angular: float, deadman: bool, source='browser') -> None:
        self._command_source = source
        self._deadman = deadman
        reverse, forward = self._keyboard_limits if self._drive_input == 'keyboard' else (1., 1.)
        self._command = (
            max(-self._max_velocity * reverse, min(self._max_velocity * forward, self._finite(linear))),
            max(-self._max_yaw_rate, min(self._max_yaw_rate, self._finite(angular))),
        )
        if deadman and self._command != (0., 0.):
            self._inspection.stop('Vehicle moved. Confirm the target again before probing.')
        self._last_command = time.monotonic()

    def _publish_twist(self, linear: float, angular: float) -> None:
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._cmd_pub.publish(command)

    def _emit_cmd_vel(self) -> None:
        now = time.monotonic()
        with self._lock:
            safety = self._enforce_safety_locked()
            active = (self._armed and self._deadman and safety['allowed'] and not self._calibrating and self._lease is not None
                      and now - self._last_command <= self._timeout)
            if self._drive_input == 'rc' and now >= self._rc_enabled_until:
                self._deadman = False
                self._command = (0.0, 0.0)
                active = False
            if active:
                self._stop_until = now + STOP_TAIL_S
                linear, angular = self._command
            elif now < self._stop_until:
                linear, angular = 0.0, 0.0
            else:
                return
            self._publish_twist(linear, angular)

    def _joy_cb(self, message: Joy) -> None:
        with self._lock:
            parsed = self._parse_xbox_joy(message)
            self._joy = parsed
            self._last_joy = time.monotonic()
            if self._drive_input != 'controller' or self._lease is None or not self._armed or self._calibrating or not self._enforce_safety_locked()['allowed']:
                self._joy_prev_a = bool(parsed['a'])
                return
            self._joy_prev_a = bool(parsed['a'])
            self._apply_drive_locked(float(parsed['linear']), float(parsed['angular']), bool(parsed['deadman']), 'joy')
        self._emit_cmd_vel()

    def _joy_live(self, now: float) -> bool:
        return self._last_joy > 0.0 and now - self._last_joy < 0.4

    def _drive_watchdog(self) -> None:
        with self._lock:
            now = time.monotonic()
            for client, seen in list(self._client_seen.items()):
                if now - seen > 3.:
                    self.release(client)
                    self._client_seen.pop(client, None)
            safety = self._enforce_safety_locked()
            self._permission_pub.publish(Bool(data=bool(self._armed and safety['servo_allowed'] and self._lease is not None)))
            self._inspection_tick(now)
            self._pump_tick(now, safety)
        self._emit_cmd_vel()

    def _log_detector_reading(self, recent=True):
        detector = self._inspection_detector
        fresh = recent and detector.get('fresh') and time.monotonic()-self._inspection_detector_at < .5
        volts = detector.get('latest_filtered_volts', detector.get('filtered_volts')) if fresh else None
        ratio = detector.get('signal_ratio') if fresh else None
        return dict(detector_volts=volts if type(volts) in (int, float) and math.isfinite(volts) else None,
                    metal_signal_pct=round(ratio*100) if type(ratio) in (int, float) and math.isfinite(ratio) else None)

    def _log_field_event(self, status, kind='marked', depth=None, event_id=None, timestamp=None, contact_volts=None):
        recent = timestamp is None or 0 <= time.time() - timestamp < 1.
        pose_fresh = recent and self._pose_at > 0 and time.monotonic() - self._pose_at < .5
        value = dict(id=event_id or uuid.uuid4().hex, timestamp=timestamp or time.time(),
                     x=self._pose['x'] if pose_fresh else None, y=self._pose['y'] if pose_fresh else None,
                     heading_deg=round(math.degrees(self._pose['yaw']) % 360) if pose_fresh else None,
                     **self._log_detector_reading(recent), contact_volts=contact_volts,
                     contact_depth_mm=depth, status=status, statusType=kind)
        self._field_log.publish(String(data=json.dumps(value, allow_nan=False)))

    def _log_probe_result(self, message):
        try:
            result = json.loads(message.data).get('probe', {}).get('last_result') or {}
            stamp = result.get('timestamp')
            if type(stamp) not in (int,float) or not math.isfinite(stamp) or self._last_probe_log == stamp:
                return
            self._last_probe_log = stamp
            status = result.get('status', 'UNKNOWN')
            self._log_field_event('CONTACT TRIGGER' if result.get('contact') else 'PROBE ' + status,
                                  'contact' if result.get('contact') else 'clear' if status == 'CLEAR' else 'unknown', result.get('contact_depth_mm'),
                                  f'probe-{stamp}', stamp, contact_volts=result.get('contact_volts'))
        except (TypeError, ValueError, AttributeError):
            self.get_logger().warning('Invalid probe result')

    def _probe_state_cb(self, message):
        try:
            value = status_dict(message)
            if isinstance(value, dict):
                self._probe_state, self._probe_state_at = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    def _actuator_command(self, payload):
        role = payload.get('role', self._selected_actuator)
        if role not in ('arm', 'probe'):
            return {'type': 'error', 'message': 'Select arm or probe'}
        if payload.get('action') == 'select_role':
            self._selected_actuator = role
            return None
        if role == 'arm' and payload.get('action') in ('configure','select'):
            self._inspection.stop('Arm calibration changed; confirm the target again.')
        if role == 'arm' and payload.get('action') in ('jog', 'move', 'position', 'angle') and (self._probe_active() or self._pump_active):
            return {'type': 'error', 'message': 'Stop the active probe or marking operation before moving the arm.'}
        publisher = self._arm_command_pub if role == 'arm' else self._probe_command_pub
        publisher.publish(command_message({**payload, 'role':role}))
        return None

    ## Dashboard protocol ##

    def snapshot(self) -> dict[str, Any]:
        """Telemetry shared by every client, without the per-client lease flag."""
        now = time.monotonic()
        with self._lock:
            rc_enabled = self._drive_input != 'rc' or now < self._rc_enabled_until
            publishing = (rc_enabled and self._armed and self._deadman and self._rc_safety.snapshot()['allowed'] and not self._calibrating and self._lease is not None
                          and now - self._last_command <= self._timeout)
            joy_live = self._joy_live(now)
            joy = {**self._joy, 'seen': joy_live}
            if self._last_joy:
                joy['age_ms'] = round((now - self._last_joy) * 1000)
            inspection = self._inspection.snapshot()
            if self._inspection.pending == 'probe' and self._inspection_goal is not None:
                inspection['probe_goal_id'] = bytes(self._inspection_goal.goal_id.uuid).hex()
            return {
                'type': 'state',
                'safety': self._rc_safety.snapshot(),
                'selected_actuator': self._selected_actuator,
                'drive': {
                    'armed': self._armed,
                    'px4_armed': self._rc_safety.px4_armed(),
                    'px4_state_status': self._rc_safety.px4_state_status(),
                    'input': self._drive_input,
                    'rc': self._rc_controls(),
                    'calibrating': self._calibrating,
                    'deadman': self._deadman,
                    'publishing': publishing,
                    'command_source': self._command_source if publishing else 'none',
                    'joy': joy,
                    'cmd_linear_x': self._command[0] if publishing else 0.0,
                    'cmd_angular_z': self._command[1] if publishing else 0.0,
                    'cmd_vel_topic': 'cmd_vel',
                    'timeout_ms': round(self._timeout * 1000),
                    'client_count': len(self._connected_clients),
                    'control_owner_present': self._lease is not None,
                    'max_velocity_mps': self._max_velocity,
                    'max_yaw_rate_rad_s': self._max_yaw_rate,
                    'pump_active': self._pump_active,
                },
                'pump_active': self._pump_active,
                'marking': dict(settings=self._marking, mode=self._pump_mode,
                                output_percent=self._pump_output, last_shot_request=self._last_shot_request),
                'inspection': inspection,
                'search_alert': dict(self._search_alert, server_time=time.time()),
                'robot': {**self._pose, 'speed_mps': self._speed, 'stationary': self._stationary(),
                          'odometry_fresh': now - self._odometry_at < .5,
                          'pose_fresh': self._pose_at > 0 and now - self._pose_at < .5},

            }

    def connected_clients(self) -> list[str]:
        with self._lock:
            return list(self._connected_clients)

    def owns_lease(self, ws: str) -> bool:
        with self._lock:
            return self._lease == ws

    def connect_client(self, ws: str) -> None:
        with self._lock:
            self._connected_clients.add(ws)

    def release(self, ws: str) -> None:
        dropped = False
        with self._lock:
            self._connected_clients.discard(ws)
            if self._lease == ws:
                dropped = True
                self._disable_actuation_locked()
                self._lease = None
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)
                self._joy_prev_a = False
        if dropped:
            self._emit_cmd_vel()

    def handle_command(self, ws: str, payload: dict[str, Any]) -> dict[str, Any] | None:
        message_type = payload.get('type')
        error: dict[str, Any] | None = None
        flush_drive = False
        with self._lock:
            if self._shutting_down:
                return {'type': 'error', 'message': 'Operator control node is shutting down.'}
            safety = self._enforce_safety_locked()
            if message_type == 'take_control':
                # A new owner must send fresh motion/deadman commands.
                self._disable_actuation_locked()
                self._lease = ws
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)
                self._enforce_safety_locked()
                flush_drive = True
            elif self._lease != ws:
                error = {'type': 'error', 'message': 'Spectator mode: take control before sending commands.'}
            elif message_type == 'log_pose':
                if self._pose_at <= 0 or time.monotonic() - self._pose_at >= .5:
                    return {'type':'error', 'message':'Fresh localization is required to log a pose.'}
                self._log_field_event('POSE LOGGED')
            elif message_type == 'clear_log':
                self._field_clear.publish(Bool(data=True))
            elif message_type == 'inspection' and payload.get('action') == 'stop':
                if self._inspection.active:
                    self._stop_motion_locked()
                    self._stop_until = time.monotonic()+STOP_TAIL_S
                    flush_drive = True
            elif message_type in ('calibration_begin', 'calibration_end'):
                if message_type == 'calibration_end' and not self._calibrating:
                    return None
                self._stop_motion_locked(stop_inspection=self._inspection.active)
                self._calibrating = message_type == 'calibration_begin'
                flush_drive = True
            elif message_type in ('adc', 'settings'):
                next_scan_setting = message_type == 'settings' and payload.get('section') in ('inspection', 'probe_alignment', 'probe_scan', 'probe_stroke')
                if self._inspection.active and not next_scan_setting:
                    self._stop_motion_locked()
                publisher = self._adc_command_pub if message_type == 'adc' else self._settings_command_pub
                publisher.publish(String(data=json.dumps(payload)))
            elif message_type == 'probe_zero_contact':
                self._probe_command_pub.publish(command_message(dict(action='zero_contact', request_id=payload.get('request_id'))))
            elif message_type == 'detector_calibrate':
                if self._inspection.active:
                    self._stop_motion_locked()
                if payload.get('action') == 'zero' and not self._detector_telemetry.snapshot()['fresh']:
                    return {'type': 'error', 'message': 'Fresh detector readings required to measure a baseline.'}
                command = {key: payload[key] for key in ('action', 'request_id', 'baseline_adc', 'full_response_adc', 'reference_voltage') if key in payload}
                self._detector_command_pub.publish(String(data=json.dumps(command)))
            elif message_type == 'arm_servo' and payload.get('action') in ('select', 'select_role', 'reconnect', 'capture', 'configure', 'configure_motion', 'configure_direction', 'enable_multiturn', 'save_probe_extension', 'stop', 'home_release'):
                command = {key: payload[key] for key in ('action', 'request_id', 'hold_id', 'servo_id', 'role', 'point', 'minimum', 'center', 'maximum', 'max_speed_deg_s', 'acceleration_deg_s2', 'max_extension_mm', 'enabled') if key in payload}
                if self._inspection.active:
                    self._stop_motion_locked()
                return self._actuator_command(command)
            elif message_type == 'arm_servo' and payload.get('role') == 'probe' and payload.get('action') == 'home_release':
                return self._actuator_command(dict(action='home_release', role='probe', hold_id=payload.get('hold_id'),
                                                   request_id=payload.get('request_id')))
            elif message_type == 'pump_manual':
                return self._pump_command(payload, safety)
            elif message_type == 'rc_drive_enable':
                if self._drive_input == 'rc':
                    if (payload.get('enabled') is True and self._armed and safety['allowed']
                            and not self._calibrating):
                        self._rc_enabled_until = time.monotonic() + self._timeout
                    else:
                        self._rc_enabled_until = 0.
                        self._apply_drive_locked(0., 0., False)
                        flush_drive = True
            elif message_type == 'drive' and (self._drive_input == 'rc' or payload.get('input', self._drive_input) != self._drive_input):
                return {'type': 'error', 'message': 'The car has a different drive input selected.'}
            elif message_type == 'drive' and payload.get('deadman') is not True:
                self._apply_drive_locked(0., 0., False)
                flush_drive = True
            elif message_type == 'drive' and not safety['allowed']:
                return {'type': 'error', 'message': safety['reason']}
            elif message_type not in ('disarm', 'estop') and not safety['servo_allowed']:
                return {'type': 'error', 'message': self._rc_safety.servo_snapshot()['reason']}
            elif message_type == 'arm':
                # Older clients may still select calibration this way; this
                # message cannot override the RC gate.
                self._disable_actuation_locked()
                self._calibrating = payload.get('calibration') is True
                self._enforce_safety_locked()
                flush_drive = True
            elif message_type in ('disarm', 'estop'):
                # STOP stays stopped until control is explicitly taken again.
                self._lease = None
                self._disable_actuation_locked()
                self._armed = False
                self._deadman = False
                self._command = (0.0, 0.0)
                flush_drive = True
            elif not self._armed:
                return {'type':'error', 'message':'Take control and enable motion on the RC.'}
            elif self._calibrating and message_type in ('drive', 'probe_target', 'start_probe_cycle', 'probe_cycle', 'sweep_enabled', 'sweep_speed'):
                return {'type': 'error', 'message': 'Finish actuator calibration before using this control.'}
            elif message_type == 'inspection':
                self._start_inspection(payload)
            elif self._inspection.active and message_type in ('arm_servo','probe_target','start_probe_cycle','probe_cycle','sweep_enabled','sweep_speed','align_probe','pump_mark'):
                return {'type':'error', 'message':'Inspection running; stop it before using manual controls.'}
            elif message_type == 'drive':
                # A live ROS pad on the SVEA owns the sticks so the browser
                # Gamepad API is not required on http://<lan-ip>.
                if self._drive_input != 'controller' or not self._joy_live(time.monotonic()):
                    self._apply_drive_locked(
                            float(payload.get('linear_x', 0.0)),
                            float(payload.get('angular_z', 0.0)),
                            payload.get('deadman') is True,
                    )
                    flush_drive = True
            elif message_type == 'arm_servo':
                if payload.get('action') in ('jog', 'position', 'home', 'move') and not self._calibrating:
                    # Stop prior work without briefly revoking permission:
                    # that false heartbeat can otherwise race the new jog.
                    self._stop_motion_locked()
                    self._calibrating = True
                    self._enforce_safety_locked()
                    self._stop_until = time.monotonic() + STOP_TAIL_S
                    self._publish_twist(0.0, 0.0)
                command = {key: payload[key] for key in ('action', 'request_id', 'role', 'point', 'held', 'direction', 'position', 'speed_deg_s', 'hold_id', 'load_percent') if key in payload}
                return self._actuator_command(command)
            elif message_type in ('probe_target', 'start_probe_cycle', 'probe_cycle'):
                requested = float(payload.get('depth_mm', 80.0 if message_type != 'probe_target' else 0.0))
                if not math.isfinite(requested):
                    return {'type': 'error', 'message': 'Probe extension must be finite'}
                if self._pump_active:
                    error = {'type': 'error', 'message': 'Wait for ground marking to finish.'}
                elif self._probe_fault:
                    error = {'type': 'error', 'message': 'Probe motion is blocked by a probe fault.'}
                else:
                    probe = self._probe_state.get('probe', {})
                    if time.monotonic() - self._probe_state_at >= .5 or not probe.get('ready'):
                        return {'type': 'error', 'message': probe.get('reason', 'Home and calibrate probe first')}
                    if self._arm_state.get('sweeping'):
                        return {'type': 'error', 'message': 'Stop arm sweep before moving the probe'}
                    if not 0 <= requested <= probe['max_depth_mm']:
                        return {'type': 'error', 'message': 'Target exceeds saved maximum extension'}
                    if message_type in ('start_probe_cycle', 'probe_cycle'):
                        if requested <= 0:
                            return {'type': 'error', 'message': 'Probe cycle target depth must be greater than 0 mm'}
                        self._probe_command_pub.publish(command_message(dict(action='probe_cycle', depth_mm=requested, request_id=payload.get('request_id'))))
                    else:
                        self._probe_command_pub.publish(command_message(dict(action='target', depth_mm=requested, request_id=payload.get('request_id'))))
            elif message_type == 'sweep_enabled':
                if payload.get('enabled'):
                    self._inspection.stop('Searching; confirm the next detection before probing.')
                    self._log_field_event('SEARCH SWEEP')
                if payload.get('enabled') and (self._probe_active() or self._pump_active):
                    return {'type': 'error', 'message': 'Stop the active probe or marking operation before sweeping.'}
                if payload.get('enabled') and not self._rc_safety.simulation:
                    if time.monotonic() - self._arm_state_at >= 1 or not self._arm_state.get('connected'):
                        return {'type': 'error', 'message': 'Select a connected arm servo in Setup first.'}
                    if not self._arm_state.get('calibration'):
                        return {'type': 'error', 'message': 'Record and apply arm limits in Setup before sweeping.'}
                # Publish permission before the sweep request; separate topics may
                # still arrive out of order, so the periodic watchdog renews it.
                self._permission_pub.publish(Bool(data=True))
                self._sweep_enabled_pub.publish(command_message(dict(action='sweep',enabled=bool(payload.get('enabled',False)))))
            elif message_type == 'sweep_speed':
                maximum = self._arm_state.get('motion_speed_limit', SWEEP_SPEED_MAX_DEG_S / .684) * .684
                minimum = .684 if self._arm_state.get('connected') else SWEEP_SPEED_MIN_DEG_S
                speed = max(minimum, min(maximum, float(payload.get('deg_s', 60.0))))
                self._sweep_speed_pub.publish(Float32(data=speed))
            elif message_type == 'align_probe':
                if self._calibrating:
                    return {'type': 'error', 'message': 'Close Setup before aligning.'}
                if self._pump_active or self._probe_active():
                    return {'type': 'error', 'message': 'Stop the active probe or marking operation before aligning.'}
                if not self._rc_safety.simulation:
                    if time.monotonic() - self._arm_state_at >= 1 or not self._arm_state.get('connected'):
                        return {'type': 'error', 'message': 'Select a connected arm servo in Setup first.'}
                    if not self._arm_state.get('calibration'):
                        return {'type': 'error', 'message': 'Record and apply arm limits in Setup before positioning arm.'}
                if 'target_bearing_deg' in payload:
                    bearing = payload['target_bearing_deg']
                    if type(bearing) not in (float, int) or not math.isfinite(bearing):
                        return {'type': 'error', 'message': 'Target bearing must be finite'}
                    angle = bearing - self._inspection.probe_offset
                else:  # Older clients send the already-converted arm command.
                    angle = payload.get('angle_deg')
                    if type(angle) not in (float, int) or not math.isfinite(angle):
                        return {'type': 'error', 'message': 'Target angle must be finite'}
                self._permission_pub.publish(Bool(data=True))
                self._arm_command_pub.publish(command_message(dict(action='angle', angle_deg=angle, request_id=payload.get('request_id'))))
            elif message_type == 'pump_mark':
                return self._pump_command(payload, safety)
            else:
                error = {'type': 'error', 'message': 'Unsupported operator command.'}
        if flush_drive:
            self._emit_cmd_vel()
        return error

    def _command_scopes(self, kind, payload):
        if kind == 'inspection':
            return ('inspection',) if payload.get('action') == 'stop' else ('inspection', 'arm', 'probe')
        if kind == 'arm_servo' and payload.get('action') in ('stop','home_release','home','move','jog','position','angle'):
            return (str(payload.get('role', self._selected_actuator)),)
        if kind in ('start_probe_cycle', 'probe_cycle', 'probe_target'):
            return ('probe',)
        if kind in ('align_probe', 'sweep_enabled', 'sweep_speed'):
            return ('arm',)
        if kind in ('pump_manual', 'pump_mark'):
            return ('pump',)
        if kind in ('calibration_begin', 'calibration_end'):
            return ('arm', 'probe', 'inspection', 'pump')
        return ()

    def _transport_command(self, message):
        client = None
        payload = {}
        try:
            envelope = json.loads(message.data)
            client = envelope['client_id']
            if not isinstance(client, str) or not 1 <= len(client) <= 100:
                raise ValueError('Invalid client ID')
            payload = envelope['payload']
            if not isinstance(payload, dict):
                raise ValueError('Invalid command payload')
            event = envelope.get('event', 'command')
            if event == 'disconnect':
                # Socket IDs are unique. Even a delayed disconnect revokes
                # that socket's lease without disturbing a new owner.
                self.release(client)
                self._client_seen.pop(client, None)
                return
            age = transport_age(envelope)
            fresh = 0 <= age < .5
            if event == 'heartbeat':
                if fresh:
                    self.connect_client(client)
                    self._client_seen[client] = max(self._client_seen.get(client, -math.inf), time.monotonic() - age)
                return
            kind = payload.get('type')
            stream = kind in ('drive', 'rc_drive_enable') or (
                kind == 'arm_servo' and payload.get('held') and not payload.get('request_id')
            ) or (kind == 'pump_manual' and payload.get('action') == 'renew')
            stop = (kind in ('disarm', 'estop', 'calibration_end') or (kind in ('inspection', 'pump_manual') and payload.get('action') == 'stop')
                    or (kind == 'arm_servo' and payload.get('action') in ('stop', 'home_release')))
            # The writer sequence orders a client's starts, renewals and
            # releases even when the Pi clock steps. Legacy messages use
            # their send timestamp within that socket's lease.
            order = envelope.get('sequence', envelope.get('stamp'))
            if type(order) not in (float, int) or not math.isfinite(order):
                raise ValueError('Invalid command sequence')
            scopes = self._command_scopes(kind, payload)
            owner = self._lease == client
            if event != 'command':
                raise ValueError('Invalid operator event')
            if owner and kind == 'take_control' and order <= self._motion_fence.get('lease', -math.inf):
                return
            if owner and kind != 'take_control' and order <= max(
                    (self._motion_fence.get(scope, -math.inf) for scope in ('lease', *scopes))):
                return  # An ordered-out release/renewal cannot affect later work.
            owner_stop = owner and stop and math.isfinite(age) and age >= 0
            if not fresh and not owner_stop:
                if stream or stop:
                    return
                raise ValueError('Operator request expired before execution.')
            if client not in self._client_seen and not owner_stop:
                if stream or stop:
                    return
                raise ValueError('Client heartbeat required')
            if (stream or stop) and not owner:
                return
            result = self.handle_command(client, payload)
            if not result:
                if kind == 'take_control':
                    self._motion_fence = {'lease': order}
                elif not stream or stop:
                    for scope in scopes:
                        self._motion_fence[scope] = order
            if result and not stream:
                self._result_pub.publish(String(data=json.dumps(dict(
                    client_id=client, request_id=payload.get('request_id'), command_type=kind, **result))))
        except (ValueError, TypeError, KeyError) as exc:
            self._result_pub.publish(String(data=json.dumps(dict(
                type='error', client_id=client, command_type=payload.get('type') if isinstance(payload, dict) else None,
                request_id=payload.get('request_id') if isinstance(payload, dict) else None, message=str(exc)))))

    def destroy_node(self):
        self.begin_shutdown()
        for client in self._inspection_clients.values():
            client.destroy()
        return super().destroy_node()

    def publish_state(self):
        self._update_search_alert(time.monotonic())
        phase = self._inspection.phase
        if phase != self._last_inspection_phase:
            self._last_inspection_phase = phase
            if phase in ('CONFIRMED', 'COMPLETE', 'ERROR'):
                self._log_field_event(self._inspection.message)
        state = self.snapshot()
        state['control_owner'] = self._lease
        self._state_pub.publish(String(data=json.dumps(state, allow_nan=False)))


def main():
    run_node(OperatorControl)


if __name__ == '__main__':
    main()
