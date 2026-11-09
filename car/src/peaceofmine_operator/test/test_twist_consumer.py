"""Downstream pump watchdog, independent of the operator control process."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from rclpy.time import Time
from std_msgs.msg import Float32


class PumpWatchdogTest(unittest.TestCase):
    def test_pump_turns_off_when_control_heartbeat_stops(self):
        path = Path(__file__).parents[2] / 'svea_examples/scripts/twist_consumer.py'
        spec = importlib.util.spec_from_file_location('twist_watchdog', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        now = [Time(nanoseconds=1_000_000_000)]
        consumer = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: now[0]),
            actuation=Mock(), require_px4_armed=False, _pump_until=None, _last_msg=now[0], cmd_timeout=.6,
            _velocity=0., _steering=0., steering_sign=1., velocity_sign=1.)
        with patch.object(module.time, 'monotonic', return_value=1.):
            module.twist_consumer.pump_cb.callback(consumer, Float32(data=35.))
        consumer.actuation.send_xtr.assert_called_with(xtr1=35.)
        now[0] = Time(nanoseconds=1_300_000_000)
        with patch.object(module.time, 'monotonic', return_value=1.3):
            module.twist_consumer.loop.callback(consumer)
        consumer.actuation.send_xtr.assert_called_with(xtr1=0.)
        self.assertIsNone(consumer._pump_until)

    def test_pump_uses_aux4_with_aux5_idle(self):
        from svea_core.interfaces.actuation import ActuationInterface
        interface = SimpleNamespace(control_pub=Mock(), steering_percent=0., velocity_percent=0.,
                                    difflock=False, highgear=False, xtr2_percent=0.)
        for percent, channel in ((0., -1000.), (50., 0.), (100., 1000.), (float('nan'), -1000.)):
            ActuationInterface.send_xtr(interface, xtr1=percent)
            ActuationInterface.loop(interface)
            message = interface.control_pub.publish.call_args.args[0]
            self.assertEqual(message.aux4, channel)
            self.assertEqual(message.aux5, -1000.)
