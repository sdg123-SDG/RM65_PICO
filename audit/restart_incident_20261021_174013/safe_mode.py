"""真机安全模式：退出透传、抱住当前角，不发急停。

控制器同一时刻通常只接受一条 JSON TCP。本模块给：
- 主进程 ``halt_for_exit``（连接还在时）
- ``real_arm_guard.py --watch``（须先于或随主程序启动）
- ``real_arm_rescue.py``（已知失败后手动救援）
"""

from __future__ import annotations

import time
from typing import Any

from .json_codec import decode_arm_state, encode_joint_deg
from .json_link import JsonArmLink, parse_real_addr

__all__ = ["apply_safe_mode", "connect_and_apply_safe"]


def apply_safe_mode(link, *, timeout_s: float = 2.5) -> dict[str, Any]:
    """读当前角 → ``movej`` 低速抱住（切走 CANFD）→ 停示教。

    不用 ``set_arm_stop``（最快速度急停会抖）。``joint`` 用控制器回读，不再做 MDH 符号变换。
    """
    log: dict[str, Any] = {"ok": False, "steps": []}

    def _try(name: str, payload: dict) -> None:
        try:
            msg = link.rpc(payload, timeout_s=timeout_s)
            log["steps"].append({"command": name, "ok": True, "reply": msg})
        except Exception as exc:
            log["steps"].append({"command": name, "ok": False, "error": str(exc)})

    _try("set_stop_teach", {"command": "set_stop_teach"})
    _try("stop_drag_teach", {"command": "stop_drag_teach"})
    try:
        raw = link.rpc({"command": "get_current_arm_state"}, timeout_s=timeout_s)
        st = decode_arm_state(raw)
        joint = encode_joint_deg(st.joint)
        log["joint"] = [float(x) for x in st.joint]
    except Exception as exc:
        log["steps"].append({"command": "get_current_arm_state", "ok": False, "error": str(exc)})
        return log
    _try(
        "movej",
        {
            "command": "movej",
            "joint": joint,
            "v": 10,
            "r": 0,
            "trajectory_connect": 0,
        },
    )
    log["ok"] = any(s.get("command") == "movej" and s.get("ok") for s in log["steps"])
    return log


def connect_and_apply_safe(
    addr: str,
    *,
    tries: int = 20,
    wait_s: float = 0.4,
    timeout_s: float = 2.5,
) -> dict[str, Any]:
    """重连控制器再抱住。主程序刚退出时端口可能还要几秒才释放。"""
    host, port = parse_real_addr(addr)
    last: OSError | None = None
    link = None
    for i in range(max(1, tries)):
        try:
            link = JsonArmLink.connect(host, port, timeout_s=2.0)
            break
        except OSError as exc:
            last = exc
            time.sleep(wait_s * (1.0 + 0.1 * i))
    if link is None:
        err = str(last) if last is not None else "connect failed"
        return {"ok": False, "error": err, "steps": []}
    try:
        return apply_safe_mode(link, timeout_s=timeout_s)
    finally:
        link.close()
