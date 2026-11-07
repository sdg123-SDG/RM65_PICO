"""Separate PICO real-arm entry; leaves launch_rm65_box_real.py unchanged."""
from __future__ import annotations
import argparse
import hashlib
import inspect
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import uuid
import time
import numpy as np

from launch_vdo_pico import ROOT, bootstrap, check_backend
from sim_real_bridge import SimRealBridgeServo, SimRealDisconnectedServo, validate_bridge
from latest_bridge import LatestBridgeServo


def install_guard_wrapper(vdo):
    """Reuse native guard/safe-mode; only adapt process launch and PID query."""
    from real.protect import RealRuntimeGuard as NativeGuard
    import real.protect as protect
    installed = []
    class RequiredGuard(NativeGuard):
        def __init__(self, arm, *, addr, heartbeat=None):
            self.session_token=uuid.uuid4().hex
            path = Path(tempfile.gettempdir()) / f"rm65-pico-real-{os.getpid()}-{self.session_token}.hb"
            super().__init__(arm, addr=addr, heartbeat=heartbeat or path)
            self.clean_exit_path = path.with_suffix("."+self.session_token+".clean.json")
            self.process = None
            installed.append(self)

        def install(self, *, spawn_guard=True):
            if not spawn_guard:
                raise RuntimeError("Real PICO requires the native independent guard")
            super().install(spawn_guard=False)
            command = [sys.executable, str(ROOT/"scripts/real_pico_guard.py"),
                       "--vdo-dir", str(vdo), "--watch", "--real", self.addr,
                       "--heartbeat", str(self.heartbeat_path), "--pid", str(os.getpid()),
                       "--clean-exit", str(self.clean_exit_path), "--session-token", self.session_token]
            options = {"creationflags": subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {"start_new_session":True}
            try:
                guard_log=ROOT/'logs/real'/f'guard_{os.getpid()}_{self.session_token}.log'
                guard_log.parent.mkdir(parents=True,exist_ok=True)
                with guard_log.open('a',encoding='utf-8') as output:
                    self.process = subprocess.Popen(command, cwd=str(vdo),stdout=output,stderr=subprocess.STDOUT, **options)
                print('[guard] persistent log:',guard_log,flush=True)
                time.sleep(.25)
                if self.process.poll() is not None:
                    raise RuntimeError("Guard process exited during startup")
            except Exception:
                self.close()
                raise
            print(f"[guard] native guard + portable PID query pid={self.process.pid}, hb={self.heartbeat_path}", flush=True)
        def close(self):
            try:
                link=getattr(self.arm,'link',None)
                if getattr(link,'exit_confirmed',False):
                    receipt={'pid':os.getpid(),'addr':self.addr,'time':time.time(),
                             'hold_confirmed':True,'session_token':self.session_token}
                    temporary=self.clean_exit_path.with_suffix('.tmp')
                    temporary.write_text(json.dumps(receipt),encoding='utf-8')
                    temporary.replace(self.clean_exit_path)
            finally:
                super().close()
    protect.RealRuntimeGuard = RequiredGuard
    import simu.session as sessions
    class MirrorOnlySession(sessions.ArmSession):
        def ui_cmd(self, tokens):
            if any(str(t).lower() == "drive" for t in tokens):
                raise ValueError("PICO real entry uses measured mirror only; ui drive is disabled here")
            return super().ui_cmd(tokens)
    sessions.ArmSession = MirrorOnlySession
    return installed


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, default=ROOT/"config/sim_real_bridge.example.json")
    p.add_argument("--vdo-dir")
    p.add_argument("--real", help="override controller HOST:PORT; default from configuration")
    p.add_argument('--forward',choices=('+X','-X'),help='operator forward expressed in robot base axes')
    p.add_argument('--inspect-real',action='store_true',help='read-only inventory + offline FK check; no SDK/guard/motion')
    p.add_argument("--check", action="store_true", help="offline only; never connects to robot/PICO")
    p.add_argument('--no-gripper',action='store_true',help='explicit arm-only diagnostic mode; no real gripper commands')
    p.add_argument('--no-real',action='store_true',help='keep the real-bridge simulation/controller but open no RM65 socket')
    args, remaining = p.parse_known_args()
    forbidden = ("--no-guard", "--json-sim", "--xbox", "--exec", "--no-cli")
    if any(s.split("=")[0] in forbidden for s in remaining):
        p.error("Guard cannot be disabled; no startup commands, Xbox, JSON sim, or noninteractive arming")
    cfg = validate_bridge(json.loads(args.config.read_text(encoding="utf-8-sig")))
    if args.forward:
        cfg['robot_forward']=args.forward
    if args.no_gripper:
        cfg['gripper_enabled']=False
    if not args.no_real and not args.check and not args.inspect_real and cfg['gripper_enabled']:
        p.error('Realtime bridge v1 requires --no-gripper (start_real.bat -NoGripper)')
    vdo = bootstrap(args.vdo_dir)
    import launch_rm65_box_cli as app
    if "controller_factory" not in inspect.signature(app.main).parameters:
        raise RuntimeError("Install the existing VDO PICO controller hook with setup_vdo.ps1 first")
    if args.check:
        from real.real_arm import RealArm
        from simu.session import ArmSession
        if ('stream_no_reply' not in inspect.signature(RealArm.Movej_CANFD).parameters
                or 'display_arm' not in inspect.getsource(ArmSession.__init__)):
            raise RuntimeError('Native joint bridge patch missing. Run scripts/prepare_vdo.py first.')
        check_backend(vdo, cfg["camera"])
        entry = vdo/"launch_rm65_box_real.py"
        print("Original entry SHA256:", hashlib.sha256(entry.read_bytes()).hexdigest())
        print(f"Forward={cfg['robot_forward']}; source physics=robot-subtree gravity compensated; gripper_enabled={cfg['gripper_enabled']}; real_connected={not args.no_real}")
        print("SIM->REAL BRIDGE PROFILE OK. No controller connection, guard, SDK or motion started.")
        return 0
    if args.inspect_real:
        from probe_real_robot import ReadOnlyClient,collect
        from real_robot_profile import hardware_profile,check_fk
        from real.json_link import parse_real_addr
        import socket
        host,port=parse_real_addr(args.real or cfg['real_addr'])
        with socket.create_connection((host,port),timeout=2.) as sock:
            report=collect(ReadOnlyClient(sock))
        profile=hardware_profile(report)
        _,native,_=check_backend(vdo,cfg['camera'])  # plain simulated model only
        st=report['responses']['get_current_arm_state']['arm_state']
        result=check_fk(native.kin,np.asarray(st['joint'])/1000,st['pose'],profile)
        out=ROOT/'logs/real/inspection.json'
        out.parent.mkdir(parents=True,exist_ok=True)
        report['fk_validation']=result
        out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print('READ-ONLY hardware/model inspection OK:',result)
        print('No SDK, guard, enable, home or motion started. Saved:',out)
        return 0
    # Use the original (Windows-patched) runtime guard, not the old adapter's
    # alternate plant/IK/session wrapper. This hook only reserves UI ownership.
    import simu.session as sessions
    class BridgeSession(sessions.ArmSession):
        def __init__(self,*session_args,**kw):
            # Both bridge modes step source physics in the PICO worker.
            # The window only renders, matching the connected real entry.
            if args.no_real and kw.get('world_mode')!='none':
                kw['world_mode']='mirror'
            super().__init__(*session_args,**kw)
        def ui_cmd(self,tokens):
            if getattr(self,'display_arm',None) is not None and tokens:
                raise ValueError('PICO owns the simulated source; do not mix ui drive/mirror with the bridge')
            return super().ui_cmd(tokens)
        def start_viewer(self,**kw):
            if getattr(self,'display_arm',None) is not None:
                kw['show_left_ui']=False
            return super().start_viewer(**kw)
    sessions.ArmSession=BridgeSession
    # HTTP is opt-in for this entry; native CLI/viewer remain available.
    if not any(x.split("=")[0] == "--http" for x in remaining):
        remaining.append("--no-http")
    sys.argv = [str(vdo/"launch_rm65_box_real.py")] + remaining
    if not args.no_real:
        install_guard_wrapper(vdo)
    controller = SimRealDisconnectedServo if args.no_real else LatestBridgeServo
    return app.main(real='' if args.no_real else (args.real or cfg["real_addr"]), require_real=not args.no_real,
                    camera=cfg["camera"], gravcomp=True, xbox_on=True,
                    controller_name="pico-sim-real-preview" if args.no_real else "pico-sim-real",
                    controller_factory=lambda arm: controller(arm,cfg),
                    description=("PICO -> simulated RM65 -> real bridge preview (RM65 disconnected)"
                                 if args.no_real else "PICO -> simulated RM65 -> native real joint following"))


if __name__ == "__main__":
    raise SystemExit(main())
