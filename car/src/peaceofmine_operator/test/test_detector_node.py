import json
import unittest
from unittest.mock import Mock, patch
import time
import rclpy
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from test_arm_node import module


class DetectorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = module('detector_node').DetectorNode()
        self.node.publisher.publish = Mock()

    def tearDown(self):
        self.node.destroy_node()

    def test_signed_volts_survive_detection_history_and_control(self):
        from peaceofmine_operator.detector import calibration_values
        self.node.baseline, self.node.full_response, self.node.reference_voltage = calibration_values(
            dict(baseline_adc=0., full_response_adc=-51., reference_voltage=5.))
        now = time.monotonic()
        self.node.control_pub.publish = Mock()
        self.node.raw_pub.publish = Mock()
        self.node.process_adc_sample(-.97, -.95, 1, now, time.time())
        self.node.flush_spatial_samples(now+.21)
        self.node.publish_control_state()
        status = json.loads(self.node.control_pub.publish.call_args.args[0].data)
        self.assertAlmostEqual(status['filtered_volts'], -.95)
        self.assertAlmostEqual(status['latest_filtered_volts'], -.95)
        self.assertAlmostEqual(status['signal_ratio'], .95)
        self.assertAlmostEqual(self.node.sensor.history[-1][2]*5/255, -.95)
        self.assertEqual(self.node.raw_pub.publish.call_args.args[0].data, 0)

    def test_adc_uses_A2_once_and_rejects_stale_or_replayed_samples(self):
        from peaceofmine_interfaces.msg import AdcSample, AdcSamples
        packet = AdcSamples(valid=True, session='sensor', config_revision=self.node.adc_revision)
        sample = AdcSample(channel=6, sequence=1, raw=16000, volts=1.5)
        now = time.time()
        sample.stamp.sec = int(now)
        sample.stamp.nanosec = int((now-int(now))*1e9)
        packet.samples.append(sample)
        self.node.adc_samples_cb(packet)
        self.assertEqual(self.node.seq, 1)
        self.assertAlmostEqual(self.node.last_raw_adc, 16000*self.node.adc_full_scale/32768/self.node.reference_voltage*255)
        self.assertAlmostEqual(self.node.last_filtered_adc, 1.5/self.node.reference_voltage*255)
        self.node.adc_samples_cb(packet)
        self.assertEqual(len(self.node.recent_samples), 1)
        sample.sequence = 2
        sample.stamp.sec -= 10
        packet.samples = [sample]
        self.node.adc_samples_cb(packet)
        self.assertEqual(self.node.seq, 1)
        packet.valid = False
        self.node.adc_samples_cb(packet)
        self.node.publish()
        self.assertFalse(json.loads(self.node.publisher.publish.call_args.args[0].data)['detector']['fresh'])

    def test_batched_adc_matches_bracketing_encoder_stamps_at_reversal(self):
        import math
        from builtin_interfaces.msg import Time
        from peaceofmine_interfaces.msg import AdcSample, AdcSamples
        from sensor_msgs.msg import JointState
        wall, start = int(time.time()), time.monotonic()

        def stamp(milliseconds):
            return Time(sec=wall, nanosec=milliseconds * 1000000)

        def encoder(milliseconds, degrees):
            message = JointState(name=['detector_fixture'], position=[math.radians(degrees)])
            message.header.stamp = stamp(milliseconds)
            self.node.fixture_cb(message)

        self.node.baseline, self.node.full_response = 0., 10.
        self.node.control_pub.publish = Mock()
        encoder(0, 12.)
        encoder(100, 20.)
        batch = AdcSamples(valid=True, session='bench', config_revision=self.node.adc_revision,
                           samples=[AdcSample(stamp=stamp(ms), channel=6, sequence=i, raw=16000, volts=v)
                                    for i, ms, v in [(1,75,1.2), (2,125,1.4)]])
        with patch('time.time', return_value=wall+.15), patch('time.monotonic', return_value=start+.15):
            self.node.adc_samples_cb(batch)
            self.assertEqual(len(self.node.sensor.history), 1)
            self.assertAlmostEqual(self.node.latest_peak['angle_deg'], 18.)
            self.node.command_cb(String(data='{"action":"clear_peak"}'))
        with patch('time.monotonic', return_value=start+.21):
            encoder(200, 12.)
            self.node.publish_control_state()
        points = list(self.node.sensor.history)
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[1][0]-points[0][0], .05, places=5)
        self.assertTrue(all(abs(p[3]-18.) < .001 for p in points))
        self.assertEqual(points[1][4], wall+.125)
        self.assertAlmostEqual(points[1][2], 1.4/self.node.reference_voltage*255)
        self.assertIsNone(self.node.latest_peak, 'queued samples before clear cannot restore a peak')
        control = json.loads(self.node.control_pub.publish.call_args.args[0].data)
        self.assertTrue(control['angle_fresh'])
        self.assertAlmostEqual(control['angle_deg'], 18., places=3)
        self.assertAlmostEqual(control['filtered_volts'], 1.4)
        self.assertAlmostEqual(control['sample_time'], start+.125, places=5)

    def test_missing_encoder_preserves_voltage_with_unknown_angle(self):
        now = time.monotonic()
        self.node.process_adc_sample(2.3, 2.1, 7, now, time.time())
        self.node.flush_spatial_samples(now+.21)
        sample = self.node.sensor.snapshot()['history'][0]
        self.assertIsNone(sample['angle_deg'])
        self.assertAlmostEqual(sample['filtered_adc'], 2.1/self.node.reference_voltage*255)
        self.assertIsNone(self.node.latest_peak)

    def test_detection_peak_expires_and_invalidates_after_movement(self):
        self.node.angle_cb(Float32(data=0.))
        self.node.odometry_cb(Odometry())
        self.node.signal_cb(Float32(data=1.))
        self.node.publish()
        self.assertTrue(json.loads(self.node.publisher.publish.call_args.args[0].data)['detector']['latest_peak']['fresh'])
        with patch('time.monotonic', return_value=time.monotonic()+31):
            self.node.publish()
            self.assertFalse(json.loads(self.node.publisher.publish.call_args.args[0].data)['detector']['latest_peak']['fresh'])
        odom = Odometry()
        odom.pose.pose.position.x = 1.
        self.node.odometry_cb(odom)
        self.assertIsNone(self.node.latest_peak)

    def test_peak_keeps_tracking_above_saturated_meter(self):
        self.node.angle_cb(Float32(data=1.))
        self.node.signal_cb(Float32(data=1.2))
        self.node.angle_cb(Float32(data=2.))
        self.node.signal_cb(Float32(data=1.5))
        self.assertEqual(self.node.signal,1.)
        self.assertEqual(self.node.latest_peak['angle_deg'],2.)
        self.assertEqual(self.node.latest_peak['signal'],1.5)
        self.node.angle_cb(Float32(data=3.))
        self.node.signal_cb(Float32(data=1.4))
        self.assertEqual(self.node.latest_peak['angle_deg'],2.)

    def test_threshold_hit_requires_fresh_position(self):
        self.node.signal_cb(Float32(data=1.))
        self.assertEqual(self.node.hits, [])
        self.node.signal_cb(Float32(data=.5))
        self.node.odometry_cb(Odometry())
        self.node.signal_cb(Float32(data=1.))
        self.assertEqual(len(self.node.hits), 1)
        self.node.publish()
        self.assertTrue(json.loads(self.node.publisher.publish.call_args.args[0].data)['detector']['detected'])

    def test_calibrated_plot_bounds_and_reset(self):
        self.node.arm_cb(String(data=json.dumps(dict(arm_calibration=dict(minimum=100, center=1500, maximum=4000)))))
        for angle, index in ((self.node.lower, 0), (self.node.upper, -1)):
            self.node.angle_cb(Float32(data=angle))
            self.node.signal_cb(Float32(data=.5))
            self.assertEqual(self.node.beam[index], .5)
        self.node.arm_cb(String(data=json.dumps(dict(arm_calibration=dict(minimum=100, center=1600, maximum=4000)))))
        self.assertTrue(all(value == 0 for value in self.node.beam))

    def test_successful_calibration_resets_threshold(self):
        self.node.threshold = .65
        self.node.command_cb(String(data='{"action":"apply","request_id":"x"}'))
        self.node.sensor_cb(String(data='{"command_result":{"request_id":"x","ok":true}}'))
        self.assertEqual(self.node.threshold, 1.)
