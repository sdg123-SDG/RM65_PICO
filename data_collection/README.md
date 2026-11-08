# Data collection tools (copy)

Working copy lives in `E:\SDG-VLA\数据` (`开始采集.bat` -> `start_collect.py` -> `采集工具\collect.py`);
this folder is a snapshot for version control. To use it, copy `collect.py` and `replay_session.py`
back into `数据\采集工具\` (collect.py resolves its data folders relative to that layout).

- `collect.py`: D405 (session root) + D435i (`d435i/`), color only, 30 fps stream saved at 10 fps.
  Records only while the arm follows the hand (teleop writes `follow_state.flag`); each follow
  period is one `episode`. Writes camera.csv / aligned.csv / robot.csv / session.json.
- `replay_session.py`: replays a session's aligned.csv in MuJoCo (simulation only).
- Real-arm replay: `experiments/vr_teleop_rm65_pico_sim/replay_real.py` (use `--no-gripper`;
  gripper force commands have stalled the controller link during replay).
