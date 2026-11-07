"""Parse hardware inventory and check model FK without changing the controller."""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np


def vector(value, n, name):
    a=np.asarray(value,dtype=float)
    if a.shape!=(n,) or not np.all(np.isfinite(a)):
        raise ValueError(f'Invalid hardware {name}')
    return a


def transform(raw):
    from kinemetics.pose import Pose
    p=vector(raw,6,'pose')
    t=np.eye(4)
    t[:3,3]=p[:3]/1e6
    t[:3,:3]=Pose.quat_to_mat(Pose.rpy_to_quat(*(p[3:]/1000)))
    return t


def hardware_profile(report):
    r=report['responses']
    needed=('get_current_arm_state','get_current_tool_frame','get_current_work_frame',
            'get_joint_min_pos','get_joint_max_pos','get_joint_en_state','get_joint_err_flag','get_collision_stage')
    missing=[k for k in needed if k not in r]
    if missing:
        raise ValueError('Required hardware queries missing: '+', '.join(missing))
    # Only the zero World frame has been commissioned here; never guess pose semantics.
    if np.max(np.abs(vector(r['get_current_work_frame']['pose'],6,'work frame'))) > 0:
        raise ValueError('Nonzero work frame needs separate commissioning; no frame change will be sent')
    en=vector(r['get_joint_en_state']['en_state'],6,'enable')
    errors=vector(r['get_joint_err_flag']['err_flag'],6,'joint errors')
    if not np.all(en==1) or np.any(errors!=0):
        raise ValueError('Hardware joints disabled or in error; no automatic enable/clear')
    lo=vector(r['get_joint_min_pos']['min_pos'],6,'min limits')/1000
    hi=vector(r['get_joint_max_pos']['max_pos'],6,'max limits')/1000
    if np.any(lo>=hi):
        raise ValueError('Invalid hardware joint limits')
    version=r.get('get_arm_software_info') or r.get('get_arm_software_version')
    if not version:
        raise ValueError('No recognized controller version response')
    model=version.get('Product_version')
    code=version.get('Ctrl_version')
    if model and not str(model).startswith('RM65'):
        raise ValueError(f'Not an RM65 controller: {model}')
    if not model and (not isinstance(code,int) or code>>16 != 0x6B):
        raise ValueError('Unknown controller product code; expected RM65-B 0x6B')
    return {'tool':r['get_current_tool_frame'], 'work':r['get_current_work_frame'],
            'tool_transform':transform(r['get_current_tool_frame']['pose']),
            'joint_min_deg':lo, 'joint_max_deg':hi, 'version':version,
            'collision_stage':r['get_collision_stage']['collision_stage']}


def check_fk(kin, joint_deg, controller_pose, profile, *, max_position_m=.0005, max_rotation_rad=.003):
    from kinemetics.pose import Pose
    fk=kin.fk_at(np.deg2rad(vector(joint_deg,6,'joints')))
    model=np.eye(4)
    model[:3,:3]=Pose.quat_to_mat(fk.quat)
    model[:3,3]=fk.pos
    model=model@profile['tool_transform']
    actual=transform(controller_pose)
    pos=float(np.linalg.norm(model[:3,3]-actual[:3,3]))
    rot=float(np.arccos(np.clip((np.trace(model[:3,:3].T@actual[:3,:3])-1)/2,-1,1)))
    if pos>max_position_m or rot>max_rotation_rad:
        raise ValueError(f'FK mismatch: {pos*1000:.3f} mm, {np.rad2deg(rot):.3f} deg; movement blocked')
    return {'position_error_mm':pos*1000, 'rotation_error_deg':float(np.rad2deg(rot)),
            'model_tcp_base_m':model[:3,3].tolist(), 'controller_tcp_base_m':actual[:3,3].tolist()}


def collect_on_link(link):
    from probe_real_robot import ReadOnlyClient, collect
    # Called before the native sampler starts. Preserve the existing TCP parser buffer.
    with link._lock:
        previous=link._sock.gettimeout()
        client=ReadOnlyClient(link._sock)
        client.buffer=link._buf
        try:
            result=collect(client)
        finally:
            link._buf=client.buffer
            link._sock.settimeout(previous)
    out=Path(__file__).resolve().parents[1]/'logs/real/startup_inventory.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return result


def posture_metrics(kin,joint_deg):
    """Finite-difference FK in isolated buffers; never move live qpos/control."""
    from kinemetics.pose import Pose
    q=np.deg2rad(vector(joint_deg,6,'joints'))
    fk=kin.fk_at(q)
    jac=np.empty((6,6))
    eps=1e-5
    for j in range(6):
        other=q.copy();other[j]+=eps
        f=kin.fk_at(other)
        jac[:3,j]=(f.pos-fk.pos)/eps
        jac[3:,j]=Pose.quat_sub_rotvec(fk.quat,f.quat)/eps
    s=np.linalg.svd(jac,compute_uv=False)
    return {'minimum_singular_value':float(s[-1]),
            'condition_number':float(s[0]/max(s[-1],1e-12)),
            'near_singular':bool(s[-1]<.002)}
