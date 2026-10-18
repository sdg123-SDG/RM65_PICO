"""XRoboToolkit controller input for Windows (no PICO Connect / OpenXR).

Loads ``PXREARobotSDK.dll`` from the unzipped PC Service package and parses the
same JSON state stream used by ``xrobotoolkit_sdk`` pybind bindings.

Requires:
  1. root start.bat running (teleop/jog/diagnostics or service mode)
  2. PICO XRoboToolkit app connected (WORKING) with Controller Tracking + Send ON
"""

from __future__ import annotations

import ctypes
import json
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SERVICE_DIR = Path(os.environ.get("RM65_XR_SERVICE_DIR", str(ROOT / "software" / "XRoboToolkit-PC-Service")))
DEFAULT_SDK_DIR = Path(os.environ.get("RM65_XR_SDK_DIR", str(DEFAULT_SERVICE_DIR / "SDK" / "x64")))

PXREAServerConnect = 1 << 2
PXREAServerDisconnect = 1 << 3
PXREADeviceFind = 1 << 4
PXREADeviceMissing = 1 << 5
PXREADeviceConnect = 1 << 9
PXREADeviceStateJson = 1 << 25
PXREADeviceCustomMessage = 1 << 26
PXREAFullMask = 0xFFFFFFFF


class PXREADevStateJson(ctypes.Structure):
    _fields_ = [
        ("devID", ctypes.c_char * 32),
        ("stateJson", ctypes.c_char * 16352),
    ]


CallbackType = ctypes.CFUNCTYPE(
    None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p
)


@dataclass
class ControllerSample:
    pos: np.ndarray | None
    quat_wxyz: np.ndarray | None
    grip: float
    trigger: float
    menu: bool
    stamp: float
    valid: bool
    primary_button: bool = False  # left X / right A
    secondary_button: bool = False  # left Y / right B
    axis_xy: np.ndarray | None = None  # thumbstick [axisX, axisY], about -1..1
    axis_click: bool = False
    headset_pos: np.ndarray | None = None
    headset_quat_wxyz: np.ndarray | None = None
    left_pos: np.ndarray | None = None
    left_quat_wxyz: np.ndarray | None = None
    left_grip: float = 0.0
    left_trigger: float = 0.0
    left_menu: bool = False
    left_valid: bool = False
    raw_stamp_ns: int = 0
    headset_stamp: float = 0.0


def _pose_csv_to_pos_quat(pose_csv: str) -> tuple[np.ndarray, np.ndarray]:
    """XRoboToolkit pose string: x,y,z,qx,qy,qz,qw -> pos + quat_wxyz."""
    vals = [float(x) for x in pose_csv.split(",")]
    if len(vals) < 7:
        raise ValueError(f"bad pose csv: {pose_csv!r}")
    pos = np.array(vals[0:3], dtype=np.float64)
    quat_wxyz = np.array([vals[6], vals[3], vals[4], vals[5]], dtype=np.float64)
    norm = float(np.linalg.norm(quat_wxyz))
    if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(quat_wxyz)) or norm < 1e-6:
        raise ValueError("non-finite pose or zero quaternion")
    quat_wxyz /= norm
    return pos, quat_wxyz


class XRoboToolkitInput:
    """Controller sampler for teleop_rm65_mujoco.

    ``hand="left"`` (default): primary pose/grip/trigger/menu come from left controller.
    ``hand="right"``: primary fields come from right controller; menu still uses left menu
    if you pass use_left_menu=True.
    """

    def __init__(
        self,
        sdk_dir: Path | None = None,
        hand: str = "left",
        use_left_menu: bool = True,
    ) -> None:
        if hand not in ("left", "right"):
            raise ValueError("hand must be 'left' or 'right'")
        self.hand = hand
        self.use_left_menu = use_left_menu
        self.sdk_dir = Path(sdk_dir or DEFAULT_SDK_DIR)
        dll_path = self.sdk_dir / "PXREARobotSDK.dll"
        if not dll_path.is_file():
            raise FileNotFoundError(
                f"PXREARobotSDK.dll not found at {dll_path}. "
                "Unzip XRoboToolkit-PC-Service.win.zip first."
            )

        # Ensure dependent DLLs in service root are discoverable.
        service_root = self.sdk_dir.parents[1]
        self._dll_dirs = [os.add_dll_directory(str(self.sdk_dir)), os.add_dll_directory(str(service_root))]
        if hasattr(os, "environ"):
            os.environ["PATH"] = str(self.sdk_dir) + os.pathsep + str(service_root) + os.pathsep + os.environ.get("PATH", "")

        self._dll = ctypes.CDLL(str(dll_path))
        self._dll.PXREAInit.argtypes = [ctypes.c_void_p, CallbackType, ctypes.c_uint]
        self._dll.PXREAInit.restype = ctypes.c_int
        self._dll.PXREADeinit.argtypes = []
        self._dll.PXREADeinit.restype = ctypes.c_int

        self._lock = threading.Lock()
        self._callback_counts = {
            "server_connect": 0,
            "server_disconnect": 0,
            "device_find": 0,
            "device_missing": 0,
            "device_connect": 0,
            "state_json": 0,
            "tracking_json": 0,
            "head_json": 0,
        }
        self._last_device_event = ""
        self._last_device_id = ""
        self._last_function_name = ""
        self._last_outer_keys: list[str] = []
        self._last_tracking_keys: list[str] = []
        self._last_controller_keys: list[str] = []
        self._last_left_controller_keys: list[str] = []
        self._last_packet_error = ""
        self._right_pose_csv: str | None = None
        self._left_pose_csv: str | None = None
        self._right_grip = 0.0
        self._right_trigger = 0.0
        self._right_menu = False
        self._right_primary = False
        self._right_secondary = False
        self._left_grip = 0.0
        self._left_trigger = 0.0
        self._left_menu = False
        self._left_primary = False
        self._left_secondary = False
        self._right_axis = (0.0, 0.0)
        self._left_axis = (0.0, 0.0)
        self._right_axis_click = False
        self._left_axis_click = False
        self._headset_pose_csv: str | None = None
        self._stamp_ns = 0
        self._left_stamp_ns = 0
        self._right_stamp_ns = 0
        self._head_stamp_ns = 0
        self._last_update = 0.0
        self._left_update = self._right_update = self._head_update = 0.0
        self._connected = False
        self._last_error = ""

        self._cb = CallbackType(self._on_callback)
        rc = self._dll.PXREAInit(None, self._cb, PXREAFullMask)
        if rc != 0:
            raise RuntimeError(f"PXREAInit failed with code {rc}. Is runService.bat running?")

    def _on_callback(self, context, cb_type, status, user_data) -> None:  # noqa: ANN001
        try:
            if cb_type == PXREAServerConnect:
                self._connected = True
                self._record_callback("server_connect")
                return
            if cb_type == PXREAServerDisconnect:
                self._connected = False
                self._record_callback("server_disconnect")
                with self._lock:
                    self._left_update = self._right_update = self._head_update = 0.0
                    self._left_stamp_ns = self._right_stamp_ns = self._head_stamp_ns = self._stamp_ns = 0
                return
            if cb_type in (PXREADeviceFind, PXREADeviceMissing, PXREADeviceConnect):
                event = {
                    PXREADeviceFind: "device_find",
                    PXREADeviceMissing: "device_missing",
                    PXREADeviceConnect: "device_connect",
                }[cb_type]
                device_id = ""
                if user_data:
                    raw_id = ctypes.cast(user_data, ctypes.c_char_p).value
                    if raw_id:
                        device_id = raw_id.decode("utf-8", errors="replace")
                with self._lock:
                    self._callback_counts[event] += 1
                    self._last_device_event = event
                    self._last_device_id = device_id
                    if cb_type == PXREADeviceMissing:
                        self._left_update = self._right_update = self._head_update = 0.0
                return
            if cb_type != PXREADeviceStateJson or not user_data:
                return
            dsj = ctypes.cast(user_data, ctypes.POINTER(PXREADevStateJson)).contents
            raw = dsj.stateJson.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")
            if not raw:
                return
            outer = json.loads(raw)
            function_name = str(outer.get("functionName", ""))
            with self._lock:
                self._callback_counts["state_json"] += 1
                if function_name == "Tracking":
                    self._callback_counts["tracking_json"] += 1
                self._last_function_name = function_name
                self._last_outer_keys = sorted(str(k) for k in outer.keys())
                self._last_packet_error = ""
            if "value" not in outer:
                with self._lock:
                    self._last_packet_error = "state JSON has no 'value' field"
                return
            value = json.loads(outer["value"]) if isinstance(outer["value"], str) else outer["value"]
            self._ingest(value)
        except Exception as exc:  # noqa: BLE001
            self._last_error = str(exc)

    def _record_callback(self, event: str) -> None:
        with self._lock:
            self._callback_counts[event] += 1
            self._last_device_event = event

    @property
    def diagnostics(self) -> dict:
        """Callback-level telemetry to separate service connection from PICO data flow."""
        with self._lock:
            return {
                "callback_counts": dict(self._callback_counts),
                "last_device_event": self._last_device_event,
                "last_device_id": self._last_device_id,
                "last_function_name": self._last_function_name,
                "last_outer_keys": list(self._last_outer_keys),
                "last_tracking_keys": list(self._last_tracking_keys),
                "last_controller_keys": list(self._last_controller_keys),
                "last_left_controller_keys": list(self._last_left_controller_keys),
                "last_packet_error": self._last_packet_error,
            }

    def diagnostics_summary(self) -> str:
        d = self.diagnostics
        c = d["callback_counts"]
        return (
            "SDK_DIAG: "
            f"server=({c['server_connect']}/{c['server_disconnect']}) "
            f"device(find/connect/missing)={c['device_find']}/{c['device_connect']}/{c['device_missing']} "
            f"json={c['state_json']} tracking={c['tracking_json']} head={c['head_json']} "
            f"fn={d['last_function_name'] or '-'} "
            f"keys={d['last_tracking_keys']} controller={d['last_controller_keys']} "
            f"left={d['last_left_controller_keys']} "
            f"device={d['last_device_event']}:{d['last_device_id']} "
            f"packet_error={d['last_packet_error'] or '-'}"
        )

    def _ingest(self, value: dict) -> None:
        """Timestamp each source, not any unrelated Head/Hand/Body packet."""
        controller = value.get("Controller", {})
        now = time.time()
        with self._lock:
            self._last_tracking_keys = sorted(str(k) for k in value.keys())
            self._last_controller_keys = sorted(str(k) for k in controller.keys()) if isinstance(controller, dict) else []
            left_controller = controller.get("left", {}) if isinstance(controller, dict) else {}
            self._last_left_controller_keys = sorted(str(k) for k in left_controller.keys()) if isinstance(left_controller, dict) else []
            packet_stamp = int(value.get("timeStampNs", 0))
            if packet_stamp:
                self._stamp_ns = packet_stamp
            if "right" in controller and (not packet_stamp or packet_stamp != self._right_stamp_ns):
                r = controller["right"]
                self._right_pose_csv = r.get("pose")
                self._right_trigger = float(r.get("trigger", 0.0))
                self._right_grip = float(r.get("grip", 0.0))
                self._right_menu = bool(r.get("menuButton", False))
                self._right_primary = bool(r.get("primaryButton", False))
                self._right_secondary = bool(r.get("secondaryButton", False))
                self._right_axis = (float(r.get("axisX", 0.0)), float(r.get("axisY", 0.0)))
                self._right_axis_click = bool(r.get("axisClick", False))
                self._right_stamp_ns = packet_stamp
                self._right_update = now
            elif "Controller" in value and "right" not in controller:
                self._right_update = 0.0
            if "left" in controller and (not packet_stamp or packet_stamp != self._left_stamp_ns):
                l = controller["left"]
                self._left_pose_csv = l.get("pose")
                self._left_trigger = float(l.get("trigger", 0.0))
                self._left_grip = float(l.get("grip", 0.0))
                self._left_menu = bool(l.get("menuButton", False))
                self._left_primary = bool(l.get("primaryButton", False))
                self._left_secondary = bool(l.get("secondaryButton", False))
                self._left_axis = (float(l.get("axisX", 0.0)), float(l.get("axisY", 0.0)))
                self._left_axis_click = bool(l.get("axisClick", False))
                self._left_stamp_ns = packet_stamp
                self._left_update = now
            elif "Controller" in value and "left" not in controller:
                self._left_update = 0.0
            if "Head" in value and isinstance(value["Head"], dict) and (not packet_stamp or packet_stamp != self._head_stamp_ns):
                self._headset_pose_csv = value["Head"].get("pose")
                self._head_update = now
                self._head_stamp_ns = packet_stamp
                self._callback_counts["head_json"] += 1
            self._last_update = now

    def sample(self) -> ControllerSample:
        with self._lock:
            stamp = self._left_update if self.hand == "left" else self._right_update
            left_stamp, right_stamp, head_stamp = self._left_update, self._right_update, self._head_update
            stamp_ns = self._left_stamp_ns if self.hand == "left" else self._right_stamp_ns
            right_csv = self._right_pose_csv
            left_csv = self._left_pose_csv
            rg, rt, rm = self._right_grip, self._right_trigger, self._right_menu
            rp, rs = self._right_primary, self._right_secondary
            lg, lt, lm = self._left_grip, self._left_trigger, self._left_menu
            lp_btn, ls_btn = self._left_primary, self._left_secondary
            r_axis, l_axis = self._right_axis, self._left_axis
            r_click, l_click = self._right_axis_click, self._left_axis_click
            hs_csv = self._headset_pose_csv

        now = time.time()
        fresh = stamp > 0 and 0 <= (now - stamp) < 0.25
        left_fresh = left_stamp > 0 and 0 <= (now - left_stamp) < 0.25
        right_fresh = right_stamp > 0 and 0 <= (now - right_stamp) < 0.25
        hs_pos = hs_quat = None
        if hs_csv and head_stamp > 0 and 0 <= (now - head_stamp) < 0.25:
            try:
                hs_pos, hs_quat = _pose_csv_to_pos_quat(hs_csv)
            except Exception:
                hs_pos = hs_quat = None

        left_pos = left_quat = None
        left_valid = False
        if left_csv and left_fresh:
            try:
                left_pos, left_quat = _pose_csv_to_pos_quat(left_csv)
                # Reject obvious "no tracking" identity pose at origin.
                if float(np.linalg.norm(left_pos)) < 1e-6 and abs(float(left_quat[0]) - 1.0) < 1e-6:
                    left_valid = False
                else:
                    left_valid = True
            except Exception:
                left_valid = False

        right_pos = right_quat = None
        right_valid = False
        if right_csv and right_fresh:
            try:
                right_pos, right_quat = _pose_csv_to_pos_quat(right_csv)
                if float(np.linalg.norm(right_pos)) < 1e-6 and abs(float(right_quat[0]) - 1.0) < 1e-6:
                    right_valid = False
                else:
                    right_valid = True
            except Exception:
                right_valid = False

        if self.hand == "left":
            pos, quat, valid = left_pos, left_quat, left_valid
            grip, trigger = lg, lt
            primary, secondary = lp_btn, ls_btn
            axis, axis_click = l_axis, l_click
        else:
            pos, quat, valid = right_pos, right_quat, right_valid
            grip, trigger = rg, rt
            primary, secondary = rp, rs
            axis, axis_click = r_axis, r_click

        menu = lm if (self.use_left_menu or self.hand == "left") else rm
        if not valid or not fresh:
            grip = trigger = 0.0
            primary = secondary = menu = axis_click = False
            axis = (0.0, 0.0)

        return ControllerSample(
            pos=pos,
            quat_wxyz=quat,
            grip=grip,
            trigger=trigger,
            menu=menu,
            stamp=stamp,
            valid=valid,
            primary_button=primary,
            secondary_button=secondary,
            axis_xy=np.array(axis, dtype=np.float64),
            axis_click=axis_click,
            headset_pos=hs_pos,
            headset_quat_wxyz=hs_quat,
            left_pos=left_pos,
            left_quat_wxyz=left_quat,
            left_grip=lg,
            left_trigger=lt,
            left_menu=lm,
            left_valid=left_valid,
            raw_stamp_ns=stamp_ns,
            headset_stamp=head_stamp,
        )

    @property
    def last_error(self) -> str:
        return self._last_error

    @property
    def service_connected(self) -> bool:
        return self._connected

    def close(self) -> None:
        try:
            self._dll.PXREADeinit()
        except Exception:
            pass


def start_service_hint() -> str:
    bat = ROOT / "start.bat"
    return (
        f"All-in-one launcher: {bat}\n"
        "Choose T or J to start the service and exactly one simulation together; "
        "choose S only for input diagnostics. On PICO, open XRoboToolkit, select this PC, "
        "wait for WORKING, then enable Head Tracking, Controller Tracking and Send."
    )
