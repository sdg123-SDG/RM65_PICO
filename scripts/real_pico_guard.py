"""Run the existing real-arm guard with a non-destructive Windows PID query."""
import argparse
import json
import time
import ctypes
from ctypes import wintypes
import os
from pathlib import Path
import sys


def pid_alive(pid):
    if pid <= 0:
        return False
    if os.name != "nt":
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    kernel.OpenProcess.restype = wintypes.HANDLE
    kernel.GetExitCodeProcess.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
    kernel.GetExitCodeProcess.restype = wintypes.BOOL
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel.CloseHandle.restype = wintypes.BOOL
    handle = kernel.OpenProcess(0x1000, False, int(pid))  # QUERY_LIMITED_INFORMATION
    if not handle:
        error = ctypes.get_last_error()
        if error == 87:  # no such PID
            return False
        if error == 5:  # access denied is not proof of process death
            return True
        raise OSError(error, "Cannot query guard PID")
    try:
        code = wintypes.DWORD()
        if not kernel.GetExitCodeProcess(handle, ctypes.byref(code)):
            raise ctypes.WinError(ctypes.get_last_error())
        return code.value == 259  # STILL_ACTIVE
    finally:
        kernel.CloseHandle(handle)


def clean_exit_valid(path, pid, addr, session_token=None):
    if path is None or pid is None or not session_token:
        return False
    try:
        data=json.loads(path.read_text(encoding='utf-8'))
        return (data.get('session_token')==session_token and data.get('pid')==pid and data.get('addr')==addr and
                data.get('hold_confirmed') is True and
                0<=time.time()-float(data.get('time',0))<=30.)
    except (OSError,ValueError,TypeError):
        return False


def make_recovery_handler(native_run_safe, path, pid, session_token):
    attempted=False
    recovery_result=1
    def run_safe(addr):
        nonlocal attempted,recovery_result
        if attempted:
            return recovery_result
        attempted=True
        if clean_exit_valid(path,pid,addr,session_token):
            print('[guard] confirmed clean exit; no reconnect or extra motion',flush=True)
            recovery_result=0
        else:
            recovery_result=native_run_safe(addr)
        return recovery_result
    return run_safe


def main():
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--vdo-dir", required=True)
    p.add_argument('--clean-exit', type=Path)
    p.add_argument('--pid', type=int)
    p.add_argument('--session-token')
    args, rest = p.parse_known_args()
    if args.pid:
        rest += ['--pid',str(args.pid)]
    sys.path.insert(0, str(Path(args.vdo_dir).resolve()))
    import real_arm_guard as guard
    import real.heartbeat as heartbeat
    guard.pid_alive = heartbeat.pid_alive = pid_alive
    native_run_safe=guard.run_safe
    run_safe=make_recovery_handler(native_run_safe,args.clean_exit,args.pid,args.session_token)
    guard.run_safe=run_safe
    sys.argv = [str(Path(guard.__file__))] + rest
    return guard.main()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
