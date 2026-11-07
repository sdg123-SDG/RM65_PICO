"""Native guardian portability only; never start a real arm or guardian."""
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

from launch_vdo_pico import bootstrap


class NativeGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            bootstrap()
        except FileNotFoundError:
            raise unittest.SkipTest('Native source unavailable')

    @unittest.skipUnless(os.name=='nt','Windows regression')
    def test_pid_query_does_not_signal_or_kill_child(self):
        from real.heartbeat import pid_alive
        child=subprocess.Popen([sys.executable,'-c','import sys; print("ready",flush=True); sys.stdin.read()'],
                               stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,
                               creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            self.assertEqual(child.stdout.readline().strip(),'ready')
            with patch('os.kill',side_effect=AssertionError('must not signal')):
                self.assertTrue(pid_alive(child.pid))
                self.assertTrue(pid_alive(os.getpid()))
            self.assertIsNone(child.poll())
        finally:
            child.communicate(input='done',timeout=5.)
        self.assertFalse(pid_alive(child.pid))

    @unittest.skipUnless(os.name=='nt','Windows regression')
    def test_heartbeat_default_is_absolute_temp_path(self):
        from real.heartbeat import DEFAULT_PATH
        self.assertTrue(DEFAULT_PATH.is_absolute())
        self.assertNotEqual(str(DEFAULT_PATH.parent),'\\tmp')

    @unittest.skipUnless(os.name=='nt','Windows regression')
    def test_guard_is_separate_process_group_without_launching(self):
        from real.protect import spawn_arm_guard
        with patch('real.protect.subprocess.Popen') as popen:
            popen.return_value.pid=123
            spawn_arm_guard(addr='not-a-network-connection',heartbeat=Path('test.hb'),pid=456)
            options=popen.call_args.kwargs
            self.assertTrue(options['creationflags'] & subprocess.CREATE_NEW_PROCESS_GROUP)
            self.assertTrue(options['creationflags'] & subprocess.CREATE_NO_WINDOW)


if __name__=='__main__':
    unittest.main()
