"""Native real-arm mirror with explicit hand-guiding; no PICO worker."""
from __future__ import annotations

import json
import sys
from datetime import datetime

from launch_vdo_pico import ROOT, bootstrap


def main():
    bootstrap()
    import launch_rm65_box_cli as app
    from control.xbox_ee_servo import XboxEeServo

    class HandGuide(XboxEeServo):
        def __init__(self,arm):
            super().__init__(arm)
            self.robot=arm
            self.drag_attempted=False

        def drag_rpc(self,command):
            payload={'command':command}
            if command=='start_drag_teach':
                payload['trajectory_record']=0
            reply=self.robot.link.rpc(payload,timeout_s=3.)
            print('[hand-guide]',json.dumps(reply,ensure_ascii=False),flush=True)
            if reply.get('command')!=command or reply.get('drag_teach') is not True:
                raise RuntimeError(f'{command} not confirmed: {reply}')

        def start_drag(self,tokens):
            if tokens:
                raise ValueError('dragstart')
            if self.robot._busy:
                raise RuntimeError('Native motion is active')
            self.drag_attempted=True
            self.drag_rpc('start_drag_teach')
            print('[hand-guide] ACTIVE: move the physical arm by hand. dragstop = finish and capture joints.',flush=True)
            return 0

        def finish_drag(self,tokens):
            if tokens:
                raise ValueError('dragstop')
            self.drag_rpc('stop_drag_teach')
            self.drag_attempted=False
            state=self.robot.fetch_arm_state()
            result={'created_local':datetime.now().isoformat(),'joint_mdh_deg':state.joint.tolist(),
                    'arm_err':state.arm_err,'sys_err':state.sys_err,'source':'physical hand guiding'}
            path=ROOT/'logs/real/taught_pose.json'
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            print('[hand-guide] FINISHED:',json.dumps(result,ensure_ascii=False),flush=True)
            return 0

        def bind_shell(self,shell):
            # No Xbox/PICO activation command in this hand-guiding session.
            shell.register('dragstart',self.start_drag,usage='dragstart',help='Enable physical hand guiding')
            shell.register('dragstop',self.finish_drag,usage='dragstop',help='Finish hand guiding and save measured MDH joints')

        def stop(self):
            if self.drag_attempted:
                self.drag_rpc('stop_drag_teach')
                self.drag_attempted=False
            return super().stop()

    sys.argv=[str(app.__file__),'--no-http','--exec','dragstart']
    return app.main(real='192.168.1.18:8080',require_real=True,camera='d435i',
                    xbox_on=False,gravcomp=False,controller_factory=HandGuide,
                    controller_name='hand-guide',description='RM65 physical hand guiding with native mirror')


if __name__=='__main__':
    raise SystemExit(main())
