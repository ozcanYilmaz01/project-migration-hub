#!/usr/bin/env python3
"""Record mission events independently of browser connections and GNSS availability."""
import json
import math
from pathlib import Path
import threading
import time
import uuid

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String

from peaceofmine_operator import configuration
from peaceofmine_operator.field_gnss import DEFAULTS, FRESH_S, FieldLog, position, validate
from peaceofmine_operator.node_runner import run_node


class FieldRecorder(Node):
    def __init__(self):
        super().__init__('field_recorder')
        simulation = self.declare_parameter('simulation', False).value
        self.device = self.declare_parameter('gnss_device', '/dev/ttyAMA0').value
        self.baud = self.declare_parameter('gnss_baud', 115200).value
        config = self.declare_parameter('config_file', configuration.default_path()).value
        path = self.declare_parameter('field_log_path', '').value
        name = 'field-simulation.sqlite3' if simulation else 'field.sqlite3'
        self.log = FieldLog(path or Path(config).resolve().with_name(name))
        self.policy = dict(DEFAULTS)
        self.source = None
        self.port_state = 'Simulation' if simulation else 'Waiting for receiver'
        self.stationary = False
        self.motion_at = 0
        self.stop = threading.Event()
        self.reader = None
        self.state_pub = self.create_publisher(String, 'field/state', 1)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.log_pub = self.create_publisher(String, 'field/entries', retained)
        self.create_subscription(String, 'field/mark', lambda msg: self.record(msg, True), 10)
        self.create_subscription(String, 'field/log', lambda msg: self.record(msg, False), 20)
        self.create_subscription(Bool, 'field/clear', self.clear, 10)
        self.create_subscription(String, 'operator/settings', self.settings, 1)
        self.create_subscription(String, 'operator/state', self.motion, 1)
        self.create_timer(.5, self.tick)
        self.publish_entries()
        if self.declare_parameter('rtk_fix', False).value and not simulation:
            # The localization stack already owns the receiver in RTK mode.
            self.device = '/fix'
            self.create_subscription(NavSatFix, self.device, self.ros_fix, qos_profile_sensor_data)
        elif not simulation and self.device:
            self.reader = threading.Thread(target=self.read_receiver, daemon=True)
            self.reader.start()

    def ros_fix(self, msg):
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        valid = 0 <= age <= FRESH_S and msg.status.status >= 0 and abs(msg.latitude) <= 90 and abs(msg.longitude) <= 180
        ee, nn = msg.position_covariance[0], msg.position_covariance[4]
        known = msg.position_covariance_type != 0 and all(math.isfinite(n) and n >= 0 for n in (ee, nn))
        self.source = (time.monotonic() - max(0, age), dict(latitude=msg.latitude if valid else None,
            longitude=msg.longitude if valid else None, horizontal_accuracy_m=math.sqrt(ee + nn) if valid and known else None,
            fix='GNSS fix' if valid else 'No fix', satellites=None))

    def read_receiver(self):
        # Keep all serial I/O (including reconnects) outside the ROS executor.
        try:
            import serial
            from pyubx2 import UBXReader
        except ImportError:
            self.port_state = 'GNSS reader unavailable'
            return
        while not self.stop.is_set():
            try:
                with serial.Serial(self.device, self.baud, timeout=.4, exclusive=True) as port:
                    reader = UBXReader(port)
                    self.port_state = 'Waiting for GNSS data'
                    last_pvt = -float('inf')
                    while not self.stop.is_set():
                        _, msg = reader.read()
                        now = time.monotonic()
                        if msg is None:
                            continue
                        if msg.identity == 'NAV-PVT':
                            last_pvt = now
                        elif now - last_pvt < FRESH_S:
                            continue
                        fix = position(msg)
                        if fix is not None:
                            self.source = (now, fix)
            except (OSError, ValueError, AttributeError) as exc:
                self.source = None
                self.port_state = f'Receiver unavailable · {exc}'
                self.stop.wait(2)

    def settings(self, msg):
        try:
            self.policy = validate(json.loads(msg.data)['gnss'])
        except (KeyError, TypeError, ValueError):
            pass

    def motion(self, msg):
        state = json.loads(msg.data)
        robot, drive = state.get('robot', {}), state.get('drive', {})
        self.motion_at = time.monotonic()
        # With no wheel odometry, the control node still reports its own drive
        # commands. Direct transmitter control cannot supply that guarantee.
        drive_idle = not drive.get('cmd_linear_x') and not drive.get('cmd_angular_z')
        ros_mode = state.get('safety', {}).get('mode') in ('ros', 'simulation')
        self.stationary = drive_idle and ros_mode and (not robot.get('odometry_fresh') or robot.get('stationary') is True)
        if not self.stationary and getattr(self, 'waiting', False):
            if self.log.refine(None, time.time(), False):
                self.publish_entries()

    def current(self):
        sample = self.source
        if sample and time.monotonic() - sample[0] <= FRESH_S:
            return sample[1]
        return None

    def publish_entries(self):
        entries = self.log.entries()
        self.waiting = any(item.get('gps_status') == 'pending' for item in entries)
        self.log_pub.publish(String(data=json.dumps(dict(revision=uuid.uuid4().hex, entries=entries))))

    def record(self, msg, mark):
        try:
            payload = json.loads(msg.data)
            if self.log.record(payload, mark, self.current(), self.policy, time.time()):
                self.publish_entries()
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid mission event: {exc}')

    def clear(self, msg):
        if msg.data:
            self.log.clear()
            self.publish_entries()

    def tick(self):
        fix = self.current()
        now = time.time()
        stationary = self.stationary and time.monotonic() - self.motion_at < 1
        if self.waiting and self.log.refine(fix, now, stationary):
            self.publish_entries()
        self.log.trace(fix, now)
        state = dict(receiver=fix['fix'] if fix else ('Stale fix' if self.source else self.port_state),
                     device=self.device, fix=fix, trail=self.log.trail())
        self.state_pub.publish(String(data=json.dumps(state, allow_nan=False)))

    def destroy_node(self):
        self.stop.set()
        if self.reader:
            self.reader.join(timeout=1)
        self.log.db.close()
        super().destroy_node()


if __name__ == '__main__':
    run_node(FieldRecorder)
