"""Run the existing real-arm guard with a non-destructive Windows PID query."""
import argparse
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


def main():
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--vdo-dir", required=True)
    args, rest = p.parse_known_args()
    sys.path.insert(0, str(Path(args.vdo_dir).resolve()))
    import real_arm_guard as guard
    import real.heartbeat as heartbeat
    guard.pid_alive = heartbeat.pid_alive = pid_alive
    sys.argv = [str(Path(guard.__file__))] + rest
    return guard.main()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
