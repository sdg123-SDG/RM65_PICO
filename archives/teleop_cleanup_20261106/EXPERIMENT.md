# vr_teleop + RM65 simulation trial

This folder contains an isolated clone of qqsq12321/vr_teleop plus a local input adapter for the existing PICO 4 Ultra + XRoboToolkit setup.

## What this trial runs

- Upstream RM65 MuJoCo model, pose tracker, EMA smoothing, full-pose inverse kinematics, and simulated EG2 gripper.
- The RM65 starts from a read-only snapshot of the physical arm's six current joint angles. This is a saved pose; the simulation does not connect to or command the arm.
- Hold either controller `grip` or `trigger` (`>= 0.55`) to follow by default; select one source with `--xrtoolkit-clutch grip|trigger` if desired. Releasing holds the simulated pose; pressing again re-centers the hand mapping to avoid a jump.
- Each clutch press requires live Head pose and maps horizontal headset-forward to RM65 base `+X` (switch to `-X` with `--view-forward-axis -x`). This heading stays fixed until release. Head pitch/roll do not tilt the base frame.
- If Head pose is absent, the trial intentionally waits and reports the missing input. It does not claim view-relative control with a guessed PICO/world direction.
- The controller index trigger continues to control only the simulated gripper while following.
- Controller samples are rejected when stale by the existing XRoboToolkit reader. If tracking is lost, the MuJoCo arm remains at its last command.
- The upstream PICO hand-tracking relay is not used; this adapter intentionally uses the controller the operator already has configured.
- The entry point is vr_teleop/example/teleop_sim.py. It does not import teleop_real.py, RM_API2, the RM65 controller socket, or any physical-arm command path.

## Setup

The isolated environment is .venv (Python 3.12). If package installation needs to be repeated, use the official PyPI index:

    .\.venv\Scripts\python.exe -m pip install --index-url https://pypi.org/simple "numpy>=2.4.1" "mujoco>=3.4.0"

The required official RM65 model and EG2 mesh repository is cloned to vr_teleop/third_party/rm_models.

## Run with the current PICO controller

Start E:\SDG-VLA\RM65_PICO\software\XRoboToolkit-PC-Service\runService.bat, connect XRoboToolkit on the headset to this PC, and enable Head Tracking, Controller Tracking, and Send. Head Tracking is required to calibrate the view-relative base axes. Then run:

    Set-Location E:\SDG-VLA\RM65_PICO\experiments\vr_teleop_rm65_pico_sim\vr_teleop
    .\.venv\Scripts\python.exe .\example\teleop_sim.py --robot rm65 --input-source xrtoolkit --xrtoolkit-hand left --xrtoolkit-clutch auto --view-forward-axis +x --position-scale 0.5

Enable Head, Controller, and Send on the XRoboToolkit headset app. Hold the left controller's side grip or trigger, then move it through a small range and rotate it. Release to hold; re-press to re-zero and recalibrate view-forward. The upstream RM65 preset has position_scale 1.5; this trial starts at 0.5. Close the MuJoCo window to stop.

The saved startup snapshot is `vr_teleop/config/rm65_current_pose.json`, captured from `get_current_arm_state` using a read-only query on 2026-10-31. The most recent query returned joint angles `[0.736, -44.480, -44.401, -10.088, -98.103, -3.775]` degrees. Refresh this snapshot if the physical arm is moved before a later simulation run.

## Upstream snapshot

- Repository: https://github.com/qqsq12321/vr_teleop
- Checked-out commit: a98b0b95a57b94756251a90d7d253b509f165c48
- This experiment contains local changes only in example/teleop_sim.py and util/xrtoolkit_input.py.

## Current run status (2026-10-31)

- The snapshot joints are within all six RM65 MuJoCo joint limits.
- Synthetic coordinate checks confirmed: head-forward maps to base `+X`, head-right maps to base `-Y`, vertical maps to `+Z`, and the mapping follows a 90-degree head yaw.
- The XRoboToolkit controller stream is reaching the adapter, but the latest stream contains no Head pose (`head=0`). The adapter refuses to engage until the headset sends Head orientation; it accepts either side grip or trigger as clutch by default.
- At startup the terminal prints grip/trigger/head status; the `[clutch] engaged` message reports the view-forward vector used for that trial.
- The upstream PICO relay path remains inactive. This trial still reads the existing XRoboToolkit controller stream and uses no real-arm command path.
