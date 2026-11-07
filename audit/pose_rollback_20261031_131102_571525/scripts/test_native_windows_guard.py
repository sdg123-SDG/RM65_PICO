"""Native guardian portability only; never start a real arm or guardian."""
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

from launch_vdo_pico import bootstrap


class NativeGuardTests(unittest.TestCase):
    def test_wrapper_emits_receipt_only_after_confirmed_hold(self):
        import tempfile
        from types import SimpleNamespace
        from launch_vdo_pico_real import install_guard_wrapper
        from real_pico_guard import clean_exit_valid
        import real.protect as protect
        import simu.session as sessions
        original_guard,original_session=protect.RealRuntimeGuard,sessions.ArmSession
        try:
            install_guard_wrapper(Path(tempfile.gettempdir()))
            with tempfile.TemporaryDirectory() as tmp:
                link=SimpleNamespace(exit_confirmed=False)
                guard=protect.RealRuntimeGuard(SimpleNamespace(link=link),addr='fake:8080')
                guard.clean_exit_path=Path(tmp)/'receipt.json'
                guard.close()
                self.assertFalse(guard.clean_exit_path.exists())
                link.exit_confirmed=True
                guard.close()
                self.assertTrue(clean_exit_valid(guard.clean_exit_path,os.getpid(),'fake:8080',guard.session_token))
        finally:
            protect.RealRuntimeGuard=original_guard
            sessions.ArmSession=original_session

    @classmethod
    def setUpClass(cls):
        try:
            bootstrap()
        except FileNotFoundError:
            raise unittest.SkipTest('Native source unavailable')

    def test_pico_guard_wrapper_uses_private_heartbeat_without_starting_process(self):
        import tempfile
        from launch_vdo_pico_real import install_guard_wrapper
        import real.protect as protect
        import simu.session as sessions
        original_guard,original_session=protect.RealRuntimeGuard,sessions.ArmSession
        try:
            install_guard_wrapper(Path(tempfile.gettempdir()))
            guard=protect.RealRuntimeGuard(object(),addr='not-a-network-connection')
            self.assertNotEqual(guard.heartbeat_path.name,'vdo-rm65b-real.hb')
            self.assertIn(str(os.getpid()),guard.heartbeat_path.name)
            with patch.object(original_guard,'install') as install, patch('subprocess.Popen') as popen, patch('time.sleep'):
                popen.return_value.poll.return_value=None
                guard.install()
            install.assert_called_once_with(spawn_guard=False)
            command=popen.call_args.args[0]
            self.assertIn(str(guard.heartbeat_path),command)
            self.assertIn('--pid',command)
        finally:
            protect.RealRuntimeGuard=original_guard
            sessions.ArmSession=original_session

    def test_clean_receipt_requires_matching_session_and_recent_confirmation(self):
        import json,time,tempfile
        from real_pico_guard import clean_exit_valid
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'clean.json'
            receipt={'pid':123,'addr':'fake:8080','time':time.time(),'hold_confirmed':True,'session_token':'session-A'}
            path.write_text(json.dumps(receipt))
            self.assertTrue(clean_exit_valid(path,123,'fake:8080','session-A'))
            self.assertFalse(clean_exit_valid(path,124,'fake:8080','session-A'))
            self.assertFalse(clean_exit_valid(path,123,'other:8080','session-A'))
            for change in ({'time':time.time()-60},{'hold_confirmed':False}):
                path.write_text(json.dumps({**receipt,**change}))
                self.assertFalse(clean_exit_valid(path,123,'fake:8080','session-A'))

    def test_safe_mode_does_not_accept_rejection_or_unrelated_ack(self):
        from real.safe_mode import apply_safe_mode
        for ack in ({'command':'movej','state':False},{'command':'movej_canfd','state':True}):
            class Link:
                def rpc(self,payload,**kwargs):
                    if payload['command']=='get_current_arm_state':
                        return {'arm_state':{'joint':[0]*6,'pose':[0]*6,'arm_err':0,'sys_err':0}}
                    return ack
            self.assertFalse(apply_safe_mode(Link())['ok'])

    def test_guard_recovery_does_not_repeat_motion_when_watch_repeats(self):
        from unittest.mock import Mock
        from real_pico_guard import make_recovery_handler
        native=Mock(return_value=1)
        recover=make_recovery_handler(native,None,123,'session-A')
        for _ in range(10):self.assertEqual(recover('fake:8080'),1)
        native.assert_called_once_with('fake:8080')

    def test_receipt_for_other_session_cannot_suppress_recovery(self):
        import tempfile,json,time
        from real_pico_guard import clean_exit_valid
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'clean.json'
            path.write_text(json.dumps({'pid':123,'addr':'fake','time':time.time(),'hold_confirmed':True,'session_token':'old'}))
            self.assertFalse(clean_exit_valid(path,123,'fake','new'))
            self.assertFalse(clean_exit_valid(path,123,'fake'))

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
