"""Exercise receiver input, mark events, shared settings, and web bridge across ROS."""
import importlib.util
import json
import os
from pathlib import Path
import pty
import tempfile
import time
import unittest

try:
    import rclpy
    import serial
    from pyubx2 import UBXReader
    from std_msgs.msg import String
    from peaceofmine_operator.web_bridge import OperatorWebBridge
except ImportError:
    rclpy = None


@unittest.skipUnless(rclpy, 'requires the ROS workspace with GNSS dependencies')
class FieldTransportTest(unittest.TestCase):
    def test_gga_receiver_to_bridge_and_durable_log(self):
        script = Path(__file__).resolve().parents[1] / 'scripts/field_recorder_node.py'
        spec = importlib.util.spec_from_file_location('field_recorder_node', script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        with tempfile.TemporaryDirectory() as directory:
            args = ['--ros-args', '-r', '__ns:=/field_test_' + str(os.getpid()), '-p',
                    'gnss_device:=' + os.ttyname(slave), '-p', 'field_log_path:=' + directory + '/field.db']
            rclpy.init(args=args)
            try:
                receiver = module.FieldRecorder()
                bridge = OperatorWebBridge()
                executor = rclpy.executors.SingleThreadedExecutor()
                executor.add_node(receiver)
                executor.add_node(bridge)
                marks = bridge.create_publisher(String, 'field/mark', 10)
                settings = bridge.create_publisher(String, 'operator/settings', 10)

                def wait(check):
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=.03)
                        if check():
                            return
                    self.fail('Timed out waiting for field telemetry')

                try:
                    wait(lambda: marks.get_subscription_count() and settings.get_subscription_count() >= 2)
                    settings.publish(String(data=json.dumps(dict(gnss=dict(max_wait_s=0, target_accuracy_m=.5)))))
                    wait(lambda: receiver.policy['max_wait_s'] == 0)
                    sentence = b'GNGGA,120000.00,5920.0000,N,01800.0000,E,1,12,0.8,42.0,M,0.0,M,,'
                    checksum = 0
                    for byte in sentence:
                        checksum ^= byte
                    os.write(master, b'$' + sentence + ('*%02X\r\n' % checksum).encode())
                    wait(lambda: (bridge.field_snapshot(None) or {}).get('fix'))
                    marks.publish(String(data=json.dumps(dict(id='example', timestamp=time.time()))))
                    wait(lambda: (bridge.field_snapshot(None) or {}).get('entries'))
                    result = bridge.field_snapshot(None)
                    entry = result['entries'][0]
                    self.assertEqual(entry['gps_status'], 'uncertain')
                    self.assertEqual(entry['status'], 'SHOT COMMANDED')
                    self.assertAlmostEqual(entry['latitude'], 59 + 20/60)
                    self.assertTrue(result['trail'])
                    self.assertEqual(bridge.field_snapshot(None, result['trail'][-1][0])['trail'], [])
                    self.assertNotIn('entries', bridge.field_snapshot(result['revision']))
                    wait(lambda: bridge.field_snapshot(None)['fix'] is None)
                    self.assertEqual(bridge.field_snapshot(None)['receiver'], 'Stale fix')
                finally:
                    executor.shutdown()
                    bridge.destroy_node()
                    receiver.destroy_node()
            finally:
                rclpy.try_shutdown()
