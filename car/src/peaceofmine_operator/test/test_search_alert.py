"""Runs the dashboard search-alert rule checks (test_search_alert.cjs) under Node."""
import pathlib
import shutil
import subprocess
import unittest


@unittest.skipUnless(shutil.which('node'), 'Node.js required')
class SearchAlertRules(unittest.TestCase):
    def test_search_alert_rules(self):
        script = pathlib.Path(__file__).with_name('test_search_alert.cjs')
        result = subprocess.run(['node', str(script)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr or result.stdout)


if __name__ == '__main__':
    unittest.main()
