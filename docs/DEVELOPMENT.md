# 开发、验证与交接

## 代码结构

| 文件 | 作用 |
|---|---|
| setup.ps1 / install.bat | Python 依赖、官方软件获取与校验、模型检查 |
| start.ps1 / start.bat | 配置、一个服务与一个仿真/诊断的进程管理 |
| scripts/xr_input.py | Windows ctypes SDK、Controller/Head/按键与新鲜度 |
| scripts/xr_heading.py | 保持竖直轴的水平朝向映射 |
| scripts/sim_control.py / ik_link6.py | Grip 相对跟随、TCP、IK、伺服 |
| scripts/teleop_rm65_xrt.py | 遥操、状态、录制、回放 |
| scripts/jog_rm65_mujoco.py | 摇杆 |
| scripts/xr_recording.py / analyze_xr_recording.py | JSONL 录制、输入回放和分析 |
| assets/ | 全部本地 XML、网格与纹理 |
| config/example.json | 配置模板 |
| software/manifest.json | 固定版本 URL 与 SHA256 |

xr_direction_calibration.py 仅为回归检查保留，当前入口不使用手推校准。

## 无硬件检查

~~~powershell
.\.venv\Scripts\python.exe scripts\doctor.py --offline
.\.venv\Scripts\python.exe -m unittest discover -s scripts -p "test_*.py" -v
.\.venv\Scripts\python.exe scripts\make_xr_test_fixture.py
.\.venv\Scripts\python.exe scripts\teleop_rm65_xrt.py --replay logs\xr\synthetic_replay_fixture.jsonl --headless
~~~

合成记录明确标记 source=synthetic-test。检查覆盖竖直轴、方向冻结、Grip、回放断流、TCP 支点和六方向小位移。不能据此证明真实手势坐标对应或全工作空间精度。

参考源码对照为可选项，不把 PiperXR 当作运行依赖。运行全部 11 项且不跳过该项：

~~~powershell
git clone https://github.com/Arborseek/PiperXR.git refs/PiperXR
git -C refs/PiperXR checkout 59e4ba7c004c71c09aa44a8b53a6b49c90a19190
~~~

遵循其原始非商业授权。没有 refs/PiperXR 时，对照项明确跳过，其他测试正常运行。

## 人工确认

依次确认左手柄位置/Grip/扳机/摇杆按钮进入电脑，设置前方后 manual forward、握持时 following，再分别慢移前、右、上。记录手势及时间段，对比目标与实际 TCP，区分输入、映射、IK、伺服和接触。

截至 2026-10-18，用户确认手动朝向后能跟随；三轴精度、抓放成功率未标为通过。真实现场记录保留在作者工作目录，不公开。

发布提交代码、模型、文档与清单，不提交 .venv、真实轨迹、日志、个人配置、完整第三方仓库或密钥。GitHub Actions 仅做无硬件检查。
