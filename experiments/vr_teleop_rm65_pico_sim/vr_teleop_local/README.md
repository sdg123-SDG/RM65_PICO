# vr_teleop 本地修改快照

`../vr_teleop` 是上游仓库 https://github.com/qqsq12321/vr_teleop 的本地克隆（自带 .git，外层仓库无法直接跟踪其中文件），
这里保存它相对上游的全部本地修改，用于恢复。

- 上游提交：`a98b0b9`（master）
- 子模块 `third_party/rm_models`：RealManRobot/rm_models 提交 `776c1a7`
- `teleop_sim.patch`：`example/teleop_sim.py`、`util/ik.py`、`util/wrist_tracker.py` 相对上游的差异
- `files/teleop_sim.py.current`：修改后的完整文件
- `files/config/rm65_current_pose.json`、`files/util/xrtoolkit_input.py`、`files/util/base_yaw.py`：上游没有的新增文件
- `replay_real_runs.py` + `run_225618_s2.txt` / `run_225618_s3.txt`：22:56 实机两段跟随的回放（拖拽 vs 工作空间限制）
- `replay_real_run.py` + `run_223720_residuals.txt`：用 2026-11-06 22:37 实机手部轨迹回放 IK 的对比脚本
- `offline_compare_base_yaw.py`：底座方位映射（方案 B）与原 IK 的离线对比脚本

恢复：

```
git clone https://github.com/qqsq12321/vr_teleop.git vr_teleop
cd vr_teleop && git checkout a98b0b9 && git submodule update --init
git -C third_party/rm_models checkout 776c1a7
git apply ../vr_teleop_local/teleop_sim.patch
cp ../vr_teleop_local/files/config/rm65_current_pose.json config/
cp ../vr_teleop_local/files/util/xrtoolkit_input.py util/
cp ../vr_teleop_local/files/util/base_yaw.py util/
```
