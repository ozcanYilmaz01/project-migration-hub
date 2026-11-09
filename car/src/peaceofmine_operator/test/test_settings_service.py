import json
import tempfile
from pathlib import Path
import unittest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from peaceofmine_interfaces.srv import SaveSettings
from peaceofmine_operator import configuration
from test_arm_node import module


class SettingsServiceTest(unittest.TestCase):
    def test_dashboard_drive_settings_survive_node_restart(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / 'operator_config.json')
            options = [Parameter('config_file', value=path)]
            node_type = module('operator_settings_node').OperatorSettings
            node = node_type(parameter_overrides=options)
            try:
                valid = dict(input='rc', forward_percent=45, reverse_percent=0)
                request = dict(request_id='drive', section='driving', value=valid)
                node.command(String(data=json.dumps(request)))
                self.assertEqual(configuration.load(path)['driving'], valid)
                for invalid in [dict(valid, input='unknown'), dict(valid, reverse_percent=-1),
                                dict(valid, forward_percent=True), dict(valid, forward_percent=101)]:
                    node.command(String(data=json.dumps(dict(request, value=invalid))))
                    self.assertEqual(configuration.load(path)['driving'], valid)
                node.destroy_node()
                node = node_type(parameter_overrides=options)
                self.assertEqual(node.shared['driving'], valid)
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_probe_stroke_speeds_and_home_load_survive_restart(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / 'operator_config.json')
            options = [Parameter('config_file', value=path)]
            node_type = module('operator_settings_node').OperatorSettings
            node = node_type(parameter_overrides=options)
            try:
                valid = dict(down_mm_s=1.75, retract_mm_s=13., home_load_percent=15.)
                request = dict(request_id='stroke', section='probe_stroke', value=valid)
                node.command(String(data=json.dumps(request)))
                self.assertEqual(configuration.load(path)['probe_stroke'], valid)
                for invalid in (dict(valid, down_mm_s=0.), dict(valid, retract_mm_s=-1.), dict(valid, home_load_percent=61.)):
                    node.command(String(data=json.dumps(dict(request, value=invalid))))
                    self.assertEqual(configuration.load(path)['probe_stroke'], valid)
                node.destroy_node()
                node = node_type(parameter_overrides=options)
                self.assertEqual(node.shared['probe_stroke'], valid)
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_detector_accepts_fractional_calibration_span(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'operator_config.json')
            node=module('operator_settings_node').OperatorSettings(parameter_overrides=[Parameter('config_file',value=path)])
            try:
                request=SaveSettings.Request(section='metal_detector',value_json=json.dumps(dict(
                    baseline_adc=125.7,full_response_adc=126.2,reference_voltage=5.)),expected_revision=0)
                result=node.save(request,SaveSettings.Response())
                self.assertTrue(result.success,result.message)
                self.assertEqual(configuration.load(path)['metal_detector']['full_response_adc'],126.2)
                request.value_json=json.dumps(dict(baseline_adc=125.7,full_response_adc=125.7,reference_voltage=5.))
                request.expected_revision=result.revision
                self.assertFalse(node.save(request,SaveSettings.Response()).success)
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_revision_conflicts_and_invalid_documents_preserve_saved_settings(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'operator_config.json')
            node=module('operator_settings_node').OperatorSettings(parameter_overrides=[Parameter('config_file',value=path)])
            try:
                request=SaveSettings.Request(section='arm_motion',value_json=json.dumps(dict(max_speed_deg_s=10.,acceleration_deg_s2=34.332)),expected_revision=0)
                result=node.save(request,SaveSettings.Response())
                self.assertTrue(result.success,result.message)
                self.assertEqual(result.revision,1)
                original=Path(path).read_text()
                self.assertFalse(node.save(request,SaveSettings.Response()).success)
                for value in ('[]','null','{"max_speed_deg_s":0}','{broken'):
                    request.value_json=value
                    request.expected_revision=1
                    self.assertFalse(node.save(request,SaveSettings.Response()).success)
                    self.assertEqual(Path(path).read_text(),original)
                self.assertEqual(configuration.load(path)['section_revisions']['arm_motion'],1)
            finally:
                node.destroy_node()
                rclpy.shutdown()
