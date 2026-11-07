"""Loopback-only controller tests. Never opens a robot address."""
import json
import socket
import threading
import time
import unittest
from types import SimpleNamespace
import numpy as np
from launch_vdo_pico import bootstrap
bootstrap()
from latest_bridge import LatestTransport
from simu.mailbox import LatestBox

class LatestTests(unittest.TestCase):

    def make(self,ack=True,feedback=True,drop_state_after_hold=False):
        listener=socket.socket()
        listener.bind(('127.0.0.1',0)); listener.listen()
        client=socket.create_connection(listener.getsockname())
        server,_=listener.accept(); listener.close(); server.settimeout(.05)
        box=LatestBox(); box.put(SimpleNamespace(joint=np.zeros(6),pose=np.zeros(6),arm_err=0,sys_err=0))
        real=SimpleNamespace(link=SimpleNamespace(_sock=client,last_arm_state_received=time.monotonic()),plant=SimpleNamespace(),arm_state=box,_busy=False)
        import tempfile
        log_dir=tempfile.TemporaryDirectory(prefix='rm65-offline-')
        self.addCleanup(log_dir.cleanup)
        cfg={'trace_dir':log_dir.name,'state_timeout_s':.5,'max_stream_gap_s':.25,'max_following_error_deg':2.}
        transport=LatestTransport(real,cfg)
        done=threading.Event(); calls=[]
        def controller():
            buf=b'';joint=[0]*6;held=False;dropped=False
            try:
                while not done.is_set():
                    try:data=server.recv(4096)
                    except socket.timeout:continue
                    if not data:break
                    buf+=data
                    while b'\n' in buf:
                        line,buf=buf.split(b'\n',1);msg=json.loads(line);calls.append(msg)
                        if msg['command']=='movej_canfd':
                            joint=msg['joint']
                            if feedback:
                                server.sendall((json.dumps({'state':'joint_state','joint':joint,'arm_err':0})+'\r\n').encode())
                        if msg['command']=='get_current_arm_state' and feedback:
                            if held and drop_state_after_hold and not dropped:
                                dropped=True
                                continue
                            reply={'state':'current_arm_state','arm_state':{'joint':joint,'pose':[0]*6,'arm_err':0,'sys_err':0}}
                            server.sendall((json.dumps(reply)+'\r\n').encode())
                        if msg['command']=='movej' and ack:
                            held=True
                            joint=msg['joint']
                            server.sendall(b'{"command":"movej","receive_state":true}\r\n')
            except OSError:pass
        thread=threading.Thread(target=controller,daemon=True);thread.start()
        def clean():
            try:transport.stop()
            except RuntimeError:pass
            done.set();client.close();server.close();thread.join(.2)
        self.addCleanup(clean)
        return transport,calls

    def wait_for(self,fn,timeout=1.):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            if fn():return
            time.sleep(.005)
        self.fail('condition timed out')

    def test_latest_overwrites_without_backlog_and_input_never_calls_network(self):
        t,calls=self.make()
        start=time.monotonic()
        for k in range(10000):t.submit(np.ones(6)*(k/10000),True)
        self.assertLess(time.monotonic()-start,.5)
        np.testing.assert_allclose(t.target[0],.9999)
        self.assertEqual(calls,[])
        self.assertFalse(hasattr(t,'queue'))

    def test_stream_without_canfd_ack_and_release_confirmed(self):
        t,calls=self.make();t.start()
        for _ in range(12):
            t.submit(np.ones(6)*.4,True);time.sleep(.02)
        self.assertGreater(t.frames,3)
        self.assertIsNone(t.fault)
        self.assertLessEqual(np.max(np.abs(t.last_goal)),.401)
        t.cancel_input();self.wait_for(lambda:t.stop_confirmed)
        count=t.frames;time.sleep(.06);self.assertEqual(t.frames,count)
        self.assertEqual(sum(x['command']=='movej' for x in calls),1)
        self.assertFalse(any(x['command']=='set_arm_stop' for x in calls))

    def test_missing_stop_ack_faults_and_regrip_cannot_resume(self):
        t,calls=self.make(ack=False);t.start();t.submit(np.ones(6),True)
        self.wait_for(lambda:t.frames>0)
        t.cancel_input();self.wait_for(lambda:t.stop_deadline is not None)
        count=t.frames
        for _ in range(10):t.submit(np.ones(6)*2,True)
        self.wait_for(lambda:t.fault is not None,timeout=3.5)
        self.assertEqual(t.frames,count)
        self.assertIn('Hold unconfirmed',t.fault)

    def test_expired_target_stops_without_replay(self):
        t,calls=self.make();t.start();t.submit(np.ones(6),True)
        self.wait_for(lambda:t.frames>0)
        self.wait_for(lambda:any(x['command']=='movej' for x in calls))
        self.wait_for(lambda:t.stop_confirmed)
        count=t.frames;time.sleep(.06);self.assertEqual(count,t.frames)

    def test_missing_feedback_is_not_refreshed_by_cached_queries(self):
        t,calls=self.make(feedback=False)
        stamp=t.last_arm_state_received
        t.start()
        for _ in range(35):
            t.rpc({'command':'get_current_arm_state'})
            t.submit(np.ones(6),True);time.sleep(.02)
            if t.fault:break
        self.assertEqual(t.last_arm_state_received,stamp)
        self.assertIn('feedback stale',t.fault)

    def test_send_deadline_never_reports_success(self):
        from unittest.mock import patch
        t,calls=self.make()
        with patch('latest_bridge.select.select',return_value=([],[],[])):
            with self.assertRaisesRegex(TimeoutError,'10ms'):
                t._write({'command':'get_current_arm_state'})
        self.assertEqual(t.frames,0)

    def test_canfd_rejection_is_a_fault(self):
        t,calls=self.make()
        t.buf.extend(b'{"command":"movej_canfd","state":false}\r\n')
        with self.assertRaisesRegex(RuntimeError,'rejected CANFD'):
            t._read()

    def test_motion_ack_cannot_confirm_stop(self):
        t,calls=self.make()
        t.stop_confirmed=False
        t.stop_deadline=time.monotonic()+.5
        t.buf.extend(b'{"command":"movej_canfd","state":true}\r\n')
        t._read()
        self.assertFalse(t.stop_confirmed)
        self.assertIsNotNone(t.stop_deadline)

    def test_every_outbound_request_has_attempt_and_completion_records(self):
        from unittest.mock import patch
        t,calls=self.make()
        with patch.object(t,'record') as log:
            t._write({'command':'get_current_arm_state'})
        self.assertEqual([x.args[0] for x in log.call_args_list],['tx_attempt','tx_complete'])

    def test_following_gap_does_not_stop_or_hold_the_command(self):
        from unittest.mock import patch
        t,_=self.make()
        t.last_goal=np.array([2.001,0,0,0,0,0])
        t.velocity=np.zeros(6)
        t.submit(np.array([10.,0,0,0,0,0]),True)
        with patch.object(t,'_write') as send:
            t._tick(time.monotonic())
        self.assertIsNone(t.fault)
        self.assertEqual(send.call_count,1)
        self.assertGreater(t.last_goal[0], 2.001)

    def test_feedback_limit_keeps_sending_last_target_at_each_tick(self):
        from unittest.mock import patch
        t,_=self.make()
        t.last_goal=np.ones(6)*1.99
        t.velocity=np.ones(6)*30
        t.submit(np.ones(6)*10,True)
        with patch.object(t,'_write') as send:
            for _ in range(8):
                t._tick(time.monotonic())
        self.assertEqual(send.call_count,8)
        self.assertTrue(all(x.args[0]['command']=='movej_canfd' for x in send.call_args_list))
        self.assertGreater(np.max(np.abs(t.last_goal)), 2.)
        self.assertIsNone(t.fault)

    def test_ack_alone_does_not_confirm_hold(self):
        t,_=self.make()
        t.stop_deadline=time.monotonic()+1
        t.stop_confirmed=False;t.sent=True;t.hold_target=np.zeros(6)
        t.buf.extend(b'{"command":"movej","receive_state":true}\r\n')
        t._read()
        self.assertTrue(t.hold_accepted)
        self.assertFalse(t.stop_confirmed)
        self.assertTrue(t.sent)

    def test_regrip_after_confirmed_hold_can_move_again(self):
        t,calls=self.make();t.start();t.submit(np.ones(6)*.3,True)
        self.wait_for(lambda:t.frames>2)
        t.cancel_input();self.wait_for(lambda:t.stop_confirmed)
        before=t.frames
        self.assertTrue(t.acknowledge_release(0.,True))
        for _ in range(8):
            t.submit(np.ones(6)*.5,True);time.sleep(.02)
        self.assertGreater(t.frames,before)
        self.assertTrue(t.sent)

    def test_stream_uses_measured_canfd_feedback_without_mixed_queries(self):
        t,calls=self.make();t.start()
        for _ in range(15):
            t.submit(np.ones(6)*.4,True);time.sleep(.02)
        self.assertIsNone(t.fault)
        first=next(i for i,x in enumerate(calls) if x['command']=='movej_canfd')
        self.assertTrue(all(x['command']=='movej_canfd' for x in calls[first:]))
        self.assertGreater(np.max(np.abs(t.real.arm_state.get().joint)),.01)

    def test_observed_completion_only_reply_is_accepted_but_not_stable(self):
        t,_=self.make()
        t.stop_deadline=time.monotonic()+3.
        t.stop_confirmed=False;t.sent=True;t.hold_target=np.zeros(6)
        t.buf.extend(b'{"state":"current_trajectory_state","trajectory_state":true}\r\n')
        t._read()
        self.assertTrue(t.hold_accepted)
        self.assertFalse(t.stop_confirmed)
        self.assertTrue(t.sent)

    def test_duplicate_completion_does_not_extend_settle_deadline(self):
        t,_=self.make()
        t.stop_deadline=time.monotonic()+3.
        t.stop_confirmed=False;t.sent=True;t.hold_target=np.zeros(6)
        packet=b'{"state":"current_trajectory_state","trajectory_state":true}\r\n'
        t.buf.extend(packet);t._read();deadline=t.stop_deadline
        t.buf.extend(packet);t._read()
        self.assertEqual(t.stop_deadline,deadline)

    def test_false_completion_is_a_failure(self):
        t,_=self.make()
        t.stop_deadline=time.monotonic()+3.
        t.buf.extend(b'{"state":"current_trajectory_state","trajectory_state":false}\r\n')
        with self.assertRaisesRegex(RuntimeError,'rejected hold'):t._read()

    def test_controller_parser_text_does_not_pause_or_fault(self):
        t,_=self.make()
        t.submit(np.ones(6),True);t.sent=True;t.stop_confirmed=False
        t.buf.extend(b"1, string or '}' expected near '{'\r\n")
        t.buf.extend(b"1, '}' expected near 'follow'\r\n")
        t.buf.extend(b"1, ']' expected near end of file\r\n")
        t.buf.extend(b"1, ':' expected near 'v'\r\n")
        t._read()
        self.assertIsNone(t.protocol_pause)
        self.assertIsNone(t.fault)
        self.assertIsNotNone(t.target)
        self.assertFalse(t.stop_pending)
        self.assertFalse(t.stop_confirmed)

    def test_protocol_error_during_hold_does_not_resend_hold(self):
        t,_=self.make()
        t.sent=True;t.stop_confirmed=False;t.stop_deadline=time.monotonic()+3.
        deadline=t.stop_deadline
        t.buf.extend(b"2, '}' expected near end of file\r\n")
        t._read()
        self.assertFalse(t.stop_pending)
        self.assertIsNone(t.protocol_pause)
        self.assertEqual(t.stop_deadline, deadline)

    def test_repeated_protocol_errors_still_fault(self):
        t,_=self.make()
        t.buf.extend(b'bad\n'*3)
        with self.assertRaisesRegex(RuntimeError,'Repeated controller protocol errors'):
            t._read()

    def test_invalid_query_reply_is_recorded_and_does_not_block_hold(self):
        t,_=self.make()
        t.pending_query=('get_current_arm_state',time.monotonic())
        t.sent=True
        t.buf.extend(b'bad query reply\n')
        t._read()
        self.assertIsNone(t.pending_query)
        self.assertTrue(t.stop_pending)

    def test_lost_state_reply_does_not_block_future_read_only_queries(self):
        t,_=self.make()
        now=time.monotonic()
        t.pending_query=('get_current_arm_state',now-.151)
        t.stop_deadline=now+3.;t.hold_accepted=True
        t._expire_query(now)
        self.assertIsNone(t.pending_query)
        self.assertTrue(t.hold_accepted)
        self.assertFalse(t.stop_pending)

    def test_query_is_not_retried_early_and_missing_optional_reply_disables_diagnostics(self):
        t,_=self.make()
        now=time.monotonic()
        t.pending_query=('get_joint_en_state',now)
        t._expire_query(now+.1)
        self.assertIsNotNone(t.pending_query)
        t._expire_query(now+.16)
        self.assertIsNone(t.pending_query)
        self.assertFalse(t.diagnostics_enabled)

    def test_dropped_query_after_hold_completes_without_exiting(self):
        t,calls=self.make(drop_state_after_hold=True)
        t.start();t.submit(np.ones(6)*.3,True)
        self.wait_for(lambda:t.frames>3)
        t.cancel_input()
        self.wait_for(lambda:t.stop_confirmed,timeout=1.5)
        self.assertIsNone(t.fault)
        self.assertTrue(t.thread.is_alive())
        self.assertEqual(sum(x['command']=='movej' for x in calls),1)

    def test_native_sampler_cannot_republish_old_cached_state(self):
        from copy import copy
        t,_=self.make()
        old=t.rpc({'command':'get_current_arm_state'})
        t.raw_state=old
        current=copy(t.real.arm_state.get());current.joint=np.ones(6)
        t._publish_feedback(current)
        seq=t.real.arm_state.seq
        sampled=t.real.fetch_arm_state()
        np.testing.assert_array_equal(sampled.joint,np.ones(6))
        self.assertEqual(t.real.arm_state.seq,seq)
        sampled.joint[0]=999
        self.assertEqual(t.state().joint[0],1.)

    def test_command_keeps_speed_cap_when_far_from_measured_joint(self):
        from unittest.mock import patch
        t,_=self.make();t.last_goal=np.ones(6)*1.99;t.velocity=np.ones(6)*30
        t.submit(np.ones(6)*10,True)
        with patch.object(t,'_write'):t._tick(time.monotonic())
        np.testing.assert_allclose(t.velocity,np.ones(6)*30)
        self.assertGreater(t.last_goal[0],2.)

    def test_disabled_or_faulted_axis_blocks_even_with_zero_arm_error(self):
        for state,key,values in [('joint_en_state','en_state',[1,1,0,1,1,1]),('joint_err_flag','err_flag',[0,0,2,0,0,0])]:
            t,_=self.make()
            t.buf.extend((json.dumps({'state':state,key:values})+'\n').encode())
            with self.assertRaises(RuntimeError):t._read()

    def test_two_objects_in_one_line_are_two_feedback_messages(self):
        t,_=self.make()
        a=json.dumps({'state':'joint_state','joint':[1000]*6,'arm_err':0})
        b=json.dumps({'state':'joint_state','joint':[2000]*6,'arm_err':0})
        t.buf.extend((a+b+'\n').encode());t._read()
        self.assertIsNone(t.protocol_pause)
        self.assertEqual(t.state().joint[0],2.)

    def test_rearm_requires_fresh_physical_release_after_confirmed_hold(self):
        t,_=self.make()
        t.sent=True;t.cancel_input()
        self.assertTrue(t.rearm_required)
        self.assertFalse(t.acknowledge_release(0.,True))
        t.sent=False;t.stop_pending=False;t.stop_confirmed=True
        self.assertFalse(t.acknowledge_release(1.,True))
        self.assertFalse(t.acknowledge_release(0.,False))
        self.assertTrue(t.acknowledge_release(0.,True))
        self.assertFalse(t.rearm_required)

    def test_transport_rejects_second_worker(self):
        t,_=self.make();t.start()
        with self.assertRaisesRegex(RuntimeError,'cannot restart'):t.start()

    def test_cli_stop_and_resume_never_send_native_motion_rpc(self):
        t,calls=self.make()
        t.sent=True;t.target=(np.ones(6),time.monotonic(),0)
        self.assertEqual(t.real.Move_Stop_Cmd(),0)
        self.assertTrue(t.operator_paused)
        self.assertTrue(t.stop_pending)
        self.assertIsNone(t.target)
        self.assertEqual(calls,[])
        t.real.Move_Continue_Cmd()
        self.assertFalse(t.operator_paused)
        self.assertTrue(t.rearm_required)
        t.submit(np.ones(6),True)
        self.assertIsNone(t.target)

    def test_partial_socket_writes_preserve_complete_frame(self):
        from unittest.mock import patch
        t,_=self.make();native=t._sock;parts=[]
        class Partial:
            def fileno(self):return native.fileno()
            def send(self,data):
                n=min(3,len(data));parts.append(bytes(data[:n]));return n
        t._sock=Partial()
        with patch('latest_bridge.time.monotonic',return_value=1.):
            t._write({'command':'get_current_arm_state'})
        self.assertEqual(json.loads(b''.join(parts)),{'command':'get_current_arm_state'})
        self.assertTrue(b''.join(parts).endswith(b'\r\n'))



    def test_trace_disk_wait_does_not_block_enqueue(self):
        from latest_bridge import TraceWriter
        entered=threading.Event();release=threading.Event();rows=[]
        class Output:
            def write(self,row):entered.set();release.wait(1.);rows.append(row)
            def flush(self):pass
            def close(self):pass
        class Path:
            def open(self,*args,**kwargs):return Output()
        writer=TraceWriter(Path())
        try:
            writer.enqueue({'seq':0});self.assertTrue(entered.wait(1.))
            before=time.perf_counter()
            for k in range(1,101):writer.enqueue({'seq':k})
            self.assertLess(time.perf_counter()-before,.1)
        finally:
            release.set();writer.close()
        self.assertEqual([json.loads(x)['seq'] for x in rows],list(range(101)))

    def test_recorded_outbound_commands_never_catch_up_in_bursts(self):
        t,_=self.make();t.start()
        for _ in range(15):
            t.submit(np.ones(6)*.4,True);time.sleep(.02)
        t.stop()
        rows=[json.loads(x) for x in t.trace_path.read_text().splitlines()]
        tx=[x['perf_counter_s'] for x in rows if x['type']=='tx_complete']
        self.assertGreater(len(tx),5)
        self.assertGreaterEqual(min(b-a for a,b in zip(tx,tx[1:])),.019)

class LatestModelTests(unittest.TestCase):





    def test_regrip_during_hold_freezes_model_until_actual_release(self):
        from test_sim_real_bridge import BridgeTests
        from latest_bridge import LatestBridgeServo
        from test_vdo_pico import sample
        _,real,_=BridgeTests().make()
        cfg=json.loads((__import__('pathlib').Path(__file__).resolve().parents[1]/'config/sim_real_bridge.example.json').read_text())
        cfg['gripper_enabled']=False
        b=LatestBridgeServo(real,cfg)
        b.follower.sent=True;b.follower.stop_confirmed=False;b.follower.rearm_required=True
        b.follower.stop_deadline=time.monotonic()+3
        before=b.display_arm.q_cmd_deg.copy()
        for k in range(20):
            incoming=sample([0.,1.,-.01*k],grip=1.)
            cooked=b.prepare_sample(incoming)
            self.assertEqual(incoming.grip,1.)
            self.assertEqual(cooked.grip,0.)
            b.control.update(cooked,0.);b.after_control_tick(cooked)
        np.testing.assert_allclose(b.display_arm.q_cmd_deg,before,atol=1e-8)
        self.assertIsNone(b.follower.target)

    def test_model_ticks_submit_latest_without_native_motion_io(self):
        from test_sim_real_bridge import BridgeTests
        from latest_bridge import LatestBridgeServo
        from test_vdo_pico import sample
        helper=BridgeTests()
        _, real, link=helper.make()
        cfg=json.loads((__import__('pathlib').Path(__file__).resolve().parents[1]/'config/sim_real_bridge.example.json').read_text())
        cfg['gripper_enabled']=False
        b=LatestBridgeServo(real,cfg)
        s=sample([0.,1.,0.],click=True)
        b.prepare_sample(s);b.control.update(s,0.);b.after_control_tick(s)
        s.axis_click=False;s.grip=1.
        b.prepare_sample(s);b.control.update(s,0.);b.after_control_tick(s)
        for k in range(15):
            s.pos=np.array([0.,1.,-.001*k]);s.stamp=time.time()
            b.prepare_sample(s);b.control.update(s,0.);b.after_control_tick(s)
        self.assertIsNotNone(b.follower.target)
        self.assertEqual(link.sent,[])
        self.assertEqual(b.follower.frames,0)
        self.assertNotIn('feedback confirms motion',b.status_suffix())
        s.grip=0.
        b.prepare_sample(s);b.control.update(s,0.);b.after_control_tick(s)
        self.assertIsNone(b.follower.target)

if __name__=='__main__':unittest.main()
