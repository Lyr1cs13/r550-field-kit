"""Output-owner regression checks. Publishers below only append to Python lists."""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'Requires ROS message types')
class StopFeedbackTests(unittest.TestCase):
    def test_cancel_keeps_braking_and_latches_wheels_that_keep_turning(self):
        from motion_gate import MotionGate
        from motion_health import OemLease
        with tempfile.TemporaryDirectory() as directory:
            hold = Path(directory) / 'motion-hold'
            gate = SimpleNamespace(alive=Mock(), output=Mock(), ir_disable=Mock(), retreat_status=Mock(),
                lease=Mock(), oem=OemLease(), disable_burst=0, retreat_identity='',
                retreat_lease=-math.inf, last_scan=0., last_tick=0., last_motion=0.,
                retreat=SimpleNamespace(status='idle'), last_linear=0., hold_file=hold,
                uncommanded_since=None, get_logger=Mock())
            gate.lease.output.return_value=(0.,0.,0.)
            for i in range(25):
                now=1.+i*.05
                gate.last_retreat_odom=now
                gate.retreat_odom=(0.,0.,0.,.10,0.)
                with patch('motion_gate.time.monotonic',return_value=now):
                    MotionGate.tick(gate)
                self.assertEqual(hold.exists(),now-1.>=.65)
            self.assertEqual(gate.output.publish.call_count,25)
            self.assertTrue(all(c.args[0].linear.x==0 for c in gate.output.publish.call_args_list))
            gate.oem.receive(2.3,True)
            gate.lease.output.return_value=(.1,0.,0.)
            with patch('motion_gate.time.monotonic',return_value=2.3):
                MotionGate.tick(gate)
            self.assertEqual(gate.output.publish.call_args.args[0].linear.x,0.)
            self.assertEqual(gate.ir_disable.publish.call_args.args[0].data,0)
