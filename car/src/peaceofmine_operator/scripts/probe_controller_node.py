#!/usr/bin/env python3
"""Probe homing, depth and contact policy over typed driver telemetry."""
import math
import time
from std_msgs.msg import Bool, Float32
from peaceofmine_interfaces.msg import AdcSamples, ContactState
from peaceofmine_interfaces.action import ActuatorOperation
from peaceofmine_operator.actuator_controller import ActuatorController
from peaceofmine_operator.adc_acquisition import saved_contact_trigger
from peaceofmine_operator.adc_contact import AdcContact
from peaceofmine_operator.probe_contact import ProbeContact
from peaceofmine_operator import configuration, probe_calibration, probe_stroke
from peaceofmine_operator.node_runner import run_node


class ProbeController(ActuatorController):
    role = 'probe'
    supported_actions = ('home', 'target', 'probe_cycle')

    def __init__(self, **kwargs):
        self.reverse = False
        self.extension_sign = -1
        self.home_position = None
        self.home_status = 'Waiting for probe encoder'
        self.home_servo_id = None
        self.home_range = None
        self.motion = None
        self.home_hold = None
        self.target = 0.
        self.started = 0.
        self.fault = False
        self.history = []
        self.encoder_stamp_ns = None
        self.encoder_at = None
        self.motion_timeout = 60.
        self.cycle_timeout = 60.
        self.action_probe_cycle = False
        self.cycle_active = False
        self.cycle_phase = 'IDLE'
        self.cycle_target_depth = 80.0
        self.contact_detected = False
        self.contact_depth_mm = None
        self.contact_volts = None
        self.last_probe_result = None
        super().__init__(**kwargs)
        self.extension = probe_calibration.load(self.config_file,self.simulation)
        self.apply_direction(self.saved.get('probe_direction', dict(reverse=False)))
        self.apply_probe_motion(self.saved.get('probe_motion',dict(max_speed_deg_s=self.motion_speed*.684,acceleration_deg_s2=34.332)))
        self.contact = AdcContact()
        self.load_contact = ProbeContact()
        self.hold_target = True
        self.adc_revision = 0
        self.reload_contact()
        self.depth_pub = self.create_publisher(Float32, 'probe/depth_mm', 10)
        self.pressure_pub = self.create_publisher(Float32, 'probe/pressure_ratio', 10)
        self.contact_pub = self.create_publisher(ContactState, 'probe/contact', 1)
        self.create_subscription(Bool, 'probe/fault', lambda msg:setattr(self, 'fault', msg.data), 1)
        self.create_subscription(AdcSamples, 'adc/filtered_samples', self.adc_cb, 10)
        self.create_timer(1., self.reload_contact)

    def apply_probe_motion(self, value):
        speed, acceleration=value.get('max_speed_deg_s'),value.get('acceleration_deg_s2')
        if not all(type(v) in (int,float) and math.isfinite(v) for v in (speed,acceleration)) or not .684 <= speed <= 699.732 or not 8.583 <= acceleration <= 2180.082:
            raise ValueError('Invalid probe motion limits')
        self.motion_speed=max(1,math.floor(speed/.684+1e-9))
        self.acceleration=max(1,math.floor(acceleration/8.583+1e-9))

    def apply_direction(self, value):
        reverse = probe_calibration.validate_direction(value)['reverse']
        if reverse != self.reverse:
            self.home_position = None
            self.home_status = 'Direction changed; waiting for encoder'
        self.reverse = reverse
        self.extension_sign = 1 if reverse else -1

    def apply_saved_settings(self):
        self.apply_direction(self.saved.get('probe_direction', dict(reverse=False)))
        self.extension=probe_calibration.load(self.config_file,self.simulation)
        if 'probe_motion' in self.saved:
            self.apply_probe_motion(self.saved['probe_motion'])

    def reload_contact(self):
        try:
            saved = configuration.load(self.config_file)
        except (ValueError,OSError) as exc:
            self.contact.received=0.
            self.reason=str(exc)
            return
        adc = saved.get('adc_simulation' if self.simulation else 'adc', {})
        config = adc.get('config', {})
        scan = sum(c['enabled'] for c in config.get('channels', []))/config.get('rate',128)
        self.contact.configure(saved_contact_trigger(adc),
            max(.5, config.get('interval_ms',0)/1000+3*scan))
        revision=saved.get('section_revisions',{}).get('adc_simulation' if self.simulation else 'adc',0)
        if revision != self.adc_revision:
            self.contact.received=0.
        self.adc_revision=revision

    def adc_cb(self, msg):
        if not msg.valid:
            self.contact.received=0.
            return
        if msg.config_revision != self.adc_revision:
            self.reload_contact()
        if msg.config_revision != self.adc_revision:
            return
        for sample in msg.samples:
            age=time.time()-(sample.stamp.sec+sample.stamp.nanosec/1e9)
            if sample.channel == self.contact.config['mux'] and 0 <= age <= self.contact.max_age:
                self.contact.sample(sample.scaled, sample.sequence, (msg.session, msg.config_revision), sample.volts)

    def interlock(self):
        return super().interlock() and not self.fault

    def calibration(self):
        return None

    def driver_cb(self, msg):
        super().driver_cb(msg)
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        # Convert the driver's sampling-batch stamp to the history's monotonic clock.
        # Keep the same stamp when a controller tick reuses the encoder packet.
        if not stamp_ns:
            # Legacy motor telemetry can still be shown at arrival time.
            self.encoder_stamp_ns, self.encoder_at = None, self.driver_at
        elif stamp_ns != self.encoder_stamp_ns:
            self.encoder_stamp_ns = stamp_ns
            age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            self.encoder_at = time.monotonic()-age if 0 <= age < .3 else None
        if self.connected():
            sample = self.telemetry
            bounds = (sample.minimum, sample.maximum)
            if self.home_position is None or bounds != self.home_range:
                if sample.minimum < sample.maximum and sample.minimum <= sample.position <= sample.maximum:
                    self.set_home(sample.position, 'Assuming probe is at the top · zero set')

    def set_home(self, position, status):
        self.home_position = position
        self.home_servo_id = self.ident
        self.home_range = (self.telemetry.minimum, self.telemetry.maximum)
        self.home_status = status

    def on_disconnect(self):
        if self.home_servo_id != self.ident:
            self.home_position = None
            self.home_status = 'Waiting for probe encoder'

    def stop(self, reason='Stopped; torque released', success=False):
        if self.cycle_active:
            self.cycle_active = False
            self.cycle_phase = 'IDLE'
            if not success:
                self.last_probe_result = dict(status='ABORTED', reason=reason, timestamp=time.time(), contact_volts=self.contact_volts)
        if self.motion == 'home' and not success:
            self.home_status = 'Homing stopped · previous zero retained' if self.home_position is not None else 'Waiting for probe encoder'
        self.motion = None
        super().stop(reason, success)

    def stroke_settings(self):
        saved = configuration.load(self.config_file)
        return probe_stroke.validate(saved.get('probe_stroke', probe_stroke.DEFAULTS))

    def stroke_speed(self, mm_per_second):
        ticks_per_mm = self.extension['travel_ticks'] / self.extension['max_extension_mm']
        units_per_mm_s = ticks_per_mm * (360 / (4096 * .684))
        return max(1, round(min(self.motion_speed, mm_per_second * units_per_mm_s)))

    @staticmethod
    def travel_seconds(ticks, speed):
        return abs(ticks) * 360 / (4096 * speed * .684)

    def start_home(self, hold, threshold, automatic=False):
        if hold == self.home_hold:
            if self.motion == 'home':
                self.command_at = time.monotonic()
            return
        stroke = self.stroke_settings()
        if automatic and threshold == 0:
            threshold = stroke['home_load_percent']
        if type(threshold) not in (int,float) or not math.isfinite(threshold) or not 10 <= threshold <= 60:
            raise ValueError('Home load threshold must be 10..60 percent')
        target = self.telemetry.minimum if self.reverse else self.telemetry.maximum
        calibrated = self.extension and self.extension['servo_id'] == self.ident
        speed = self.stroke_speed(stroke['retract_mm_s']) if calibrated else self.motion_speed
        self.start(target, self.telemetry.minimum, self.telemetry.maximum, speed=speed, load=threshold, autonomous=automatic)
        self.motion_timeout = max(60., 10+self.travel_seconds(self.extension['travel_ticks'], speed)) if calibrated else 60.
        self.home_hold = hold
        self.motion, self.started = 'home', time.monotonic()
        self.home_status, self.lifecycle = 'Seeking the top stop', 'HOMING'

    def start_probe_cycle(self, depth, speed=0.):
        self.reload_contact()
        calibration = self.extension
        if self.home_position is None or not calibration or calibration['servo_id'] != self.ident:
            raise ValueError('Probe scan needs an encoder zero and saved travel in Setup → Probe')
        if type(depth) not in (int, float) or not math.isfinite(depth) or not 0 < depth <= calibration['max_extension_mm']:
            raise ValueError(f"Target depth must be between 0 and {calibration['max_extension_mm']} mm")
        if not self.interlock():
            raise ValueError('Probe cycle requires RC permission, a connected probe and no fault')
        if round(self.home_position+self.extension_sign*depth/calibration['max_extension_mm']*calibration['travel_ticks']) == self.home_position:
            raise ValueError('Probe depth is smaller than one encoder tick')
        if self.cycle_active or self.operation:
            raise ValueError('Stop the current probe operation first')
        contact = self.contact.snapshot()
        if not contact['enabled'] or not contact['fresh']:
            raise ValueError('Enable a fresh ADC contact input before probing')
        self.cycle_target_depth = float(depth)
        self.at_cycle_depth = False
        self.contact_detected = bool(contact['detected'])
        self.contact_depth_mm = self.contact_volts = None
        if self.contact_detected:
            # The sensor was already active at the start of this downstroke.
            # Record the measured starting depth and never issue an extension.
            ticks = self.extension_sign*(self.telemetry.position-self.home_position)
            self.contact_depth_mm = round(max(0., ticks*calibration['max_extension_mm']/calibration['travel_ticks']), 1)
            self.contact_volts = contact.get('filtered_volts')
            if ticks <= 0:
                self.finish_cycle()
                return
            self.start_target(0., hold=False, speed=speed, autonomous=True)
            self.cycle_phase = 'RETRACTING'
            self.reason = f'Contact at {self.contact_depth_mm:.1f} mm; retracting'
        else:
            self.start_target(depth, hold=False, speed=speed, autonomous=True)
            self.cycle_phase = 'DESCENDING'
        self.cycle_active = True
        down_seconds = self.travel_seconds(self.operation.target-self.telemetry.position, self.operation.speed)
        return_seconds = self.travel_seconds(self.home_position-self.operation.target, self.return_speed)
        self.cycle_timeout = max(60., 10+down_seconds+return_seconds)

    def finish_cycle(self):
        status = 'CONTACT' if self.contact_detected else 'CLEAR'
        self.last_probe_result = dict(status=status, contact=self.contact_detected, contact_depth_mm=self.contact_depth_mm,
                                      contact_volts=self.contact_volts, target_depth_mm=self.cycle_target_depth, timestamp=time.time())
        self.cycle_active = False
        self.cycle_phase = 'IDLE'
        self.stop(f'Probe cycle complete: {status}', success=True)

    def start_target(self, depth, hold=True, speed=0., autonomous=False):
        calibration = self.extension
        if self.home_position is None or not calibration or calibration['servo_id'] != self.ident:
            raise ValueError('Depth targets need an encoder zero and saved travel in Setup → Probe')
        if type(depth) not in (int,float) or not math.isfinite(depth) or not 0 <= depth <= calibration['max_extension_mm']:
            raise ValueError('Probe depth outside calibrated range')
        low, high = sorted((self.home_position, self.home_position+self.extension_sign*calibration['travel_ticks']))
        if not self.telemetry.minimum <= low < high <= self.telemetry.maximum:
            raise ValueError('Probe travel exceeds encoder range')
        target = round(self.home_position+self.extension_sign*depth/calibration['max_extension_mm']*calibration['travel_ticks'])
        stroke = self.stroke_settings()
        # Snapshot both leg speeds for this operation. Positive action speeds
        # are an explicit mm/s override, still bounded by the motor limit.
        self.return_speed = self.stroke_speed(speed or stroke['retract_mm_s'])
        extending = self.extension_sign * (target - self.telemetry.position) > 0
        leg_speed = self.stroke_speed(speed or stroke['down_mm_s']) if extending else self.return_speed
        self.start(target, low, high, speed=leg_speed, autonomous=autonomous)
        self.motion_timeout = max(60., 10+self.travel_seconds(target-self.telemetry.position, leg_speed))
        self.arrival_tolerance = max(1, min(self.operation.tolerance, abs(target-self.telemetry.position)//4))
        self.target, self.motion, self.started = depth, 'target', time.monotonic()
        self.hold_target = hold

    def target_reached(self):
        return (self.telemetry.operation_id == self.operation.operation_id
                and self.telemetry.goal == self.operation.target
                and abs(self.telemetry.position-self.operation.target) <= self.arrival_tolerance
                and abs(self.telemetry.speed_deg_s) <= 3*.684)

    def special_command(self, action, command):
        if action == 'home_release':
            expired_hold = not self.operation and self.reason == 'Held command expired; release and start again'
            if self.home_hold and command.get('hold_id') == self.home_hold and (self.motion == 'home' or expired_hold):
                position = self.telemetry.position
                self.stop('Home hold released', success=True)
                self.set_home(position, 'Zero set on operator release')
        elif action == 'home':
            if command.get('held') is not True or not self.interlock():
                raise ValueError('Homing requires a held command, RC permission and no fault')
            hold = command.get('hold_id')
            if not isinstance(hold,str) or not hold:
                raise ValueError('Home requires a unique hold identifier')
            self.start_home(hold, command.get('load_percent'))
        elif action == 'target':
            self.start_target(command.get('depth_mm'))
        elif action == 'probe_cycle':
            target = command.get('target_depth_mm', command.get('depth_mm', 80.0))
            self.start_probe_cycle(target)
        elif action == 'save_probe_extension':
            if self.operation or self.telemetry.torque or self.home_position is None:
                raise ValueError('Wait for an encoder zero and release the motor before saving travel')
            value = probe_calibration.validate(dict(servo_id=self.ident, max_extension_mm=command.get('max_extension_mm'),
                travel_ticks=self.extension_sign*(self.telemetry.position-self.home_position)))
            self.save(command, 'probe', value, lambda:setattr(self,'extension',value))
            return True
        elif action == 'zero_contact':
            if not self.interlock():
                raise ValueError('Contact zero requires RC permission and a connected probe')
            self.load_contact.zero(time.monotonic())
            self.reason = 'Motor contact baseline zeroed for this session'
        elif action == 'configure_direction':
            if self.operation or self.telemetry.torque:
                raise ValueError('Stop the probe before reversing direction')
            value = probe_calibration.validate_direction(dict(reverse=command.get('enabled')))
            self.save(command, 'probe_direction', value, lambda:self.apply_direction(value))
            return True
        elif action == 'jog':
            if self.reverse and command.get('direction') in (-1, 1):
                command['direction'] = -command['direction']
            return False
        elif action == 'configure_motion':
            if self.operation or self.telemetry.torque:
                raise ValueError('Stop before changing probe limits')
            old=self.motion_speed,self.acceleration
            self.apply_probe_motion(command)
            self.motion_speed,self.acceleration=old
            value={k:command[k] for k in ('max_speed_deg_s','acceleration_deg_s2')}
            self.save(command,'probe_motion',value,lambda:self.apply_probe_motion(value))
            return True
        elif action in ('configure','capture'):
            raise ValueError('Use probe homing and extension calibration')
        else:
            return False
        self.reply(command, True)
        return True

    def start_action(self, request):
        self.action_probe_cycle = request.operation == 'probe_cycle'
        if not math.isfinite(request.speed) or request.speed < 0:
            raise ValueError('Probe action speed must be finite and nonnegative')
        if request.operation == 'probe_cycle':
            self.start_probe_cycle(request.target, speed=request.speed)
        elif request.operation == 'home':
            import uuid
            self.start_home(uuid.uuid4().hex, request.load_percent, automatic=True)
        else:
            self.start_target(request.target, hold=False, speed=request.speed)

    def action_result(self, success, message):
        result = self.last_probe_result if self.action_probe_cycle and success else None
        return ActuatorOperation.Result(success=success, message=message,
            contact=bool(result and result['contact']),
            contact_depth_mm=float(result['contact_depth_mm'] if result and result['contact'] else 0.))

    def operation_finished(self, outcome, message):
        if self.motion == 'home' and outcome == 'load_stop':
            position = self.telemetry.position
            self.stop('Home reached; torque released', success=True)
            self.set_home(position, 'Zero set at top load')
        else:
            self.stop(message or outcome)

    def update_behavior(self):
        if not self.operation:
            return
        if self.motion and time.monotonic()-self.started >= (self.cycle_timeout if self.cycle_active else self.motion_timeout):
            error_mm = abs(self.telemetry.position-self.operation.target)*self.extension['max_extension_mm']/self.extension['travel_ticks'] if self.extension else None
            reason = f'Probe {self.cycle_phase.lower() if self.cycle_active else self.motion} timed out'
            if error_mm is not None:
                reason += f': {error_mm:.1f} mm from target. Check travel and home calibration.'
            if self.cycle_active:
                self.cycle_active = False
                self.cycle_phase = 'IDLE'
                self.last_probe_result = dict(status='TIMEOUT', reason=reason, timestamp=time.time(), contact_volts=self.contact_volts)
            self.stop(reason)
            return

        if self.cycle_active:
            self.command_at = time.monotonic()
            if not self.interlock():
                self.cycle_active = False
                self.cycle_phase = 'IDLE'
                self.last_probe_result = dict(status='ABORTED', reason='RC permission or probe connection lost', timestamp=time.time(), contact_volts=self.contact_volts)
                self.stop('Probe cycle aborted: RC permission or probe connection lost')
                return

            ready = bool(self.connected() and self.extension and self.extension['servo_id']==self.ident and self.home_position is not None)
            current_depth = self.extension_sign*(self.telemetry.position-self.home_position)*self.extension['max_extension_mm']/self.extension['travel_ticks'] if ready else 0.0

            if self.cycle_phase == 'DESCENDING':
                contact = self.contact.snapshot()
                if not contact['enabled'] or not contact['fresh']:
                    self.stop('Contact input lost; probe result unknown')
                    return
                acknowledged = self.telemetry.operation_id == self.operation.operation_id
                remaining_ticks = self.extension_sign * (self.operation.target - self.telemetry.position)
                if acknowledged and remaining_ticks <= self.arrival_tolerance:
                    self.at_cycle_depth = True
                detected = contact['detected'] is True and not self.at_cycle_depth
                if detected or (self.target_reached()
                                and self.extension_sign*(self.telemetry.position-self.home_position) > 0):
                    self.contact_detected = detected
                    self.contact_volts = contact.get('filtered_volts')
                    self.contact_depth_mm = round(max(0.0, current_depth), 1) if detected else None
                    self.cycle_phase = 'RETRACTING'
                    self.reason = (f'Contact at {self.contact_depth_mm:.1f} mm; retracting' if detected
                                   else f'Target {self.cycle_target_depth:.0f} mm reached without contact; retracting')
                    self.target = 0.0
                    # Keep short return strokes precise; use the command's tolerance for full travel.
                    self.arrival_tolerance = max(1, min(self.operation.tolerance, abs(self.home_position-self.telemetry.position)//4))
                    self.operation.target = self.home_position
                    self.operation.speed = self.return_speed
                    return

            elif self.cycle_phase == 'RETRACTING':
                if self.target_reached():
                    self.finish_cycle()
                    return

        elif (self.contact.snapshot()['enabled'] and not self.contact.snapshot()['fresh']
                and self.extension_sign*(self.operation.target-self.telemetry.position) > 3):
            self.stop('Probe contact reading lost; extension stopped')
            return

        elif (self.contact.snapshot()['detected'] is True
                and self.extension_sign*(self.operation.target-self.telemetry.position) > 3):
            self.stop('Probe contact detected; extension stopped')
            return

        elif self.motion == 'target':
            self.command_at = time.monotonic()
            if self.target_reached():
                if self.hold_target:
                    self.lifecycle, self.reason = 'HOLDING', 'Holding requested depth; stop releases torque'
                else:
                    self.stop('Probe target reached; torque released', success=True)
        elif self.motion == 'home':
            if self.action_handle:
                self.command_at = time.monotonic()
            if abs(self.telemetry.position-self.operation.target) <= 3:
                position = self.telemetry.position
                self.stop('Reached upper encoder limit', success=True)
                self.set_home(position, 'Zero set at upper encoder limit')

    def behavior_state(self, include_history=True):
        now, wall_now = time.monotonic(), time.time()
        ready = bool(self.connected() and self.extension and self.extension['servo_id']==self.ident and self.home_position is not None)
        depth = self.extension_sign*(self.telemetry.position-self.home_position)*self.extension['max_extension_mm']/self.extension['travel_ticks'] if ready else None
        if depth is not None:
            self.depth_pub.publish(Float32(data=depth))
        ratio = min(1., abs(self.telemetry.load_percent)/100) if self.connected() else None
        if ratio is not None:
            self.pressure_pub.publish(Float32(data=ratio))
            self.history.append((now,ratio))
        self.history = [(t,v) for t,v in self.history if now-t<=30][-700:]
        contact = self.contact.snapshot()
        if isinstance(self.contact.session, tuple):
            contact['source_session'], contact['source_revision'] = self.contact.session
        msg = ContactState(valid=contact['fresh'], detected=contact['detected'] is True,
            channel=contact['mux'], value=float(contact['value'] or 0), threshold=float(contact['level']),
            reason='Fresh' if contact['fresh'] else 'Contact input disabled or stale')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.contact_pub.publish(msg)
        self.load_contact.update(dict(connected=self.connected(), active_role='probe',
            sample_time=self.encoder_stamp_ns or self.driver_at, sample_at=self.encoder_at,
            depth_mm=depth if self.encoder_stamp_ns else None, session=self.driver.session if self.driver else None,
            serial_port=self.driver.device if self.driver else '', servo_id=self.ident,
            load_percent=self.telemetry.load_percent if self.telemetry else None, current_a=self.telemetry.current_a if self.telemetry else None,
            position=self.telemetry.position if self.telemetry else None, goal_position=self.telemetry.goal if self.telemetry else None,
            torque=bool(self.telemetry and self.telemetry.torque), probe=dict(holding=self.lifecycle=='HOLDING')), now)
        return dict(reverse=self.reverse, sweep_acceleration_deg_s2=self.acceleration*8.583, home_position=self.home_position, home_status=self.home_status, homing=self.motion=='home',
            probe=dict(ready=ready, homed=self.home_position is not None, calibrated=bool(self.extension),
                max_depth_mm=self.extension['max_extension_mm'] if self.extension else None,
                depth_mm=depth, target_mm=self.target, active=self.motion=='target', moving=self.motion=='target' and self.lifecycle!='HOLDING', holding=self.lifecycle=='HOLDING',
                cycle_active=self.cycle_active, cycle_phase=self.cycle_phase, cycle_target_depth=self.cycle_target_depth,
                cycle_goal_id=(bytes(self.action_handle.goal_id.uuid).hex()
                               if self.cycle_active and self.action_probe_cycle and self.action_handle else None),
                cycle_started_at=self.started if self.cycle_active else None,
                contact_detected=self.contact_detected, contact_depth_mm=self.contact_depth_mm, contact_volts=self.contact_volts, last_result=self.last_probe_result,
                reason=self.reason, fault=self.fault, pressure_ratio=ratio, pressure_window_s=30, history_time_s=wall_now,
                **({'pressure_samples':[dict(x=t-now,y=v*100) for t,v in self.history]} if include_history else {}),
                **self.load_contact.snapshot(now, include_history), adc_contact=contact['detected'], adc_contact_fresh=contact['fresh'], adc_contact_state=contact))


def main():
    run_node(ProbeController)


if __name__ == '__main__':
    main()
