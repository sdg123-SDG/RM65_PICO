"""Read-only controller inventory. No RealArm, SDK, guards, or motion imports."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import time

QUERIES = (
    'get_arm_software_info', 'get_arm_software_version', 'get_current_arm_state',
    'get_current_tool_frame', 'get_current_work_frame',
    'get_joint_min_pos', 'get_joint_max_pos', 'get_joint_max_speed',
    'get_joint_en_state', 'get_joint_err_flag', 'get_collision_stage',
    'get_install_pose', 'get_gripper_state',
)


class ReadOnlyClient:
    def __init__(self, sock, timeout=1.5):
        self.sock, self.timeout, self.buffer = sock, timeout, bytearray()
        self.unmatched = []

    def query(self, command):
        if command not in QUERIES:
            raise ValueError('Read-only inventory command not allowed')
        self.sock.settimeout(self.timeout)
        self.sock.sendall((json.dumps({'command':command})+'\r\n').encode())
        deadline = time.monotonic()+self.timeout
        expected = {command, command.removeprefix('get_')}
        while True:
            left = deadline-time.monotonic()
            if left <= 0:
                raise TimeoutError(command)
            self.sock.settimeout(left)
            if b'\n' not in self.buffer:
                chunk = self.sock.recv(8192)
                if not chunk:
                    raise ConnectionError('Controller closed inventory connection')
                self.buffer.extend(chunk)
                if len(self.buffer) > 1048576:
                    raise ValueError('Oversized inventory response')
                continue
            line, _, rest = self.buffer.partition(b'\n')
            self.buffer = bytearray(rest)
            if not line.strip():
                continue
            msg = json.loads(line)
            if isinstance(msg, dict) and (msg.get('command') in expected or msg.get('state') in expected):
                return msg
            self.unmatched.append(msg)


def collect(client):
    report = {'created_utc':datetime.now(timezone.utc).isoformat(), 'read_only':True,
              'responses':{}, 'errors':{}}
    for cmd in QUERIES:
        try:
            reply = client.query(cmd)
            report['responses'][cmd] = reply
            print(cmd, json.dumps(reply, ensure_ascii=False), flush=True)
        except (OSError, ValueError) as exc:
            report['errors'][cmd] = str(exc)
            print(cmd, 'ERROR', exc, flush=True)
    report['unmatched'] = getattr(client,'unmatched',[])
    if report['unmatched']:
        print('Unmatched responses:',json.dumps(report['unmatched'],ensure_ascii=False),flush=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--real', default='192.168.1.18:8080')
    p.add_argument('--out', type=Path, default=Path(__file__).resolve().parents[1]/'logs/real/inventory.json')
    args=p.parse_args()
    host, _, port=args.real.partition(':')
    with socket.create_connection((host, int(port or 8080)), timeout=2.) as sock:
        result=collect(ReadOnlyClient(sock))
    result['address']=args.real
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n',encoding='utf-8')
    print('Saved read-only report:', args.out)
    required=set(QUERIES)-{'get_arm_software_info','get_arm_software_version','get_gripper_state'}
    valid=required.issubset(result['responses']) and bool(
        result['responses'].get('get_arm_software_info') or result['responses'].get('get_arm_software_version'))
    return 0 if valid else 1


if __name__=='__main__':
    raise SystemExit(main())
