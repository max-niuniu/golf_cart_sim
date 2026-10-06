"""Input and handover contract; no Gazebo or display required."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'control_ui', Path(__file__).resolve().parents[1] / 'scripts/control_ui.py')
ui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ui)


class ManualControlTest(unittest.TestCase):
    def setUp(self):
        self.control = ui.ManualCommand()

    def test_disabled_source_does_not_override_navigation(self):
        self.assertIsNone(self.control.command(1))
        self.control.stop(1)
        self.assertIsNone(self.control.command(1.1))

    def test_release_stops_speed_and_opposing_inputs_cancel(self):
        c = self.control
        c.enable()
        c.held['w'] = 'forward'
        c.held['a'] = 'left'
        self.assertEqual(c.command(1), (1.0, 0.5))
        c.held['s'] = 'reverse'
        self.assertEqual(c.command(1)[0], 0)
        del c.held['w']
        self.assertEqual(c.command(1)[0], -1)
        del c.held['s']
        self.assertEqual(c.command(1)[0], 0)

    def test_stop_clears_latched_inputs_and_relinquishes_topic(self):
        c = self.control
        c.enable()
        c.held['mouse'] = 'forward'
        c.steer = 0.3
        c.stop(10)
        self.assertFalse(c.enabled)
        self.assertEqual(c.command(10.1), (0, 0))
        self.assertIsNone(c.command(10.31))
        c.enable()
        self.assertEqual(c.command(11), (0, 0))

    def test_speed_and_steering_remain_bounded(self):
        c = self.control
        c.enable()
        c.speed, c.steer = 8, -2
        c.held['s'] = 'reverse'
        self.assertEqual(c.command(1), (-3, -0.5))


if __name__ == '__main__':
    unittest.main()
