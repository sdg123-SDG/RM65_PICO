# RM65 / PICO 遥操交接（2026-10-21）

## 给接手者的第一句话

目标是 **PICO 左手柄 → 已经能用的原生 MuJoCo 遥操 → 仿真六轴关节 → 原生 RealArm → 实体 RM65**。目前仿真到真机的连续跟随尚未验收成功。不要把发送成功、测试通过或网络可达当作实机成功。

用户已要求停止本轮工作，交给其他人。本轮停止后没有重新启动遥操、仿真或 XR 服务。接手者应从代码和下面的原始记录开始，而不是让用户再次重复完整排查。

## 路径与启动入口

| 内容 | 路径 |
| --- | --- |
| 遥操工程 | `E:\SDG-VLA\RM65_PICO` |
| 原生机器人工程 | `E:\SDG-VLA\vdo-rm65b` |
| 场景与模型 | `E:\SDG-VLA\vdo-sim` |
| 用户希望保留的原始真机入口 | `E:\SDG-VLA\vdo-rm65b\launch_rm65_box_real.py` |
| 当前桥接入口 | `E:\SDG-VLA\RM65_PICO\start_real.bat` → `start_real.ps1` → `scripts\launch_vdo_pico_real.py` |
| 当前桥接实现 | `scripts\sim_real_bridge.py` |
| 原生仿真 PICO 插件 | `scripts\vdo_pico.py` |
| 手柄输入/相对位移控制 | `scripts\xr_input.py`、`scripts\sim_control.py` |
| PC Service | `software\XRoboToolkit-PC-Service\RoboticsServiceProcess.exe` |
| Python | `.venv\Scripts\python.exe` |
| 现场场景 | `E:\SDG-VLA\vdo-sim\assets\scene\custom_rm65_box\scene_d435i_gravcomp.xml` |

当前真机入口（会连接硬件，不要仅为读文档而运行）：

```powershell
Set-Location E:\SDG-VLA\RM65_PICO
.\start_real.bat -NoGripper
```

离线入口检查（本轮已经跑过，检查时不连接控制器、不启动保护/SDK、不发运动）：

```powershell
Set-Location E:\SDG-VLA\RM65_PICO
.\start_real.bat -Check
```

`-NoGripper` 很重要：本轮夹爪输出禁用。用户要求夹爪由 PICO 扳机控制，不读取/同步实体夹爪开口；夹爪实机功能仍未验收。

## 用户要求与操作习惯

- PICO 4 Ultra、Windows、6DoF，头显和电脑通过局域网工作。运行遥操不是 USB 链路。
- 左手 Grip 按住才相对跟随，松开停止；下次握持重新取相对起点。
- 不增加 `pico arm` 等额外终端启用步骤，不自动回零、不自动移动到保存基准、不启动自动夹爪动作。
- 当前以位置跟随为主，保持腕部姿态；不能把自由姿态跟随说成已经验收。
- 用户曾指出面向可能应对应机器人 **-X**。当前本机配置却是 **+X**，方向应结合底座坐标和现场重新核实，不能靠观察窗口角度猜。
- 用户要求保存真实起始位姿为基准，文件见下文；这是保存记录，不是允许启动时自动运动过去。
- 用户提供的上一轮日志是实际真机运行数据，不能称为“微动测试数据”。之前曾允许小幅实验，本轮最后已要求全部停止；后续实机实验由接手者和现场操作者协调。

## 停止状态（2026-10-21 17:10 左右，中国时间）

- 本轮 PTY 真机桥接会话 `99583` 已退出，返回码 0；此前它因保持请求未确认报错，不能因返回码 0 判成功。
- 进程扫描没有找到可识别的 RM65 桥接/仿真/跟踪/示教/保护 Python 命令行，也没有 `RoboticsServiceProcess.exe`。
- 网络扫描没有到 `192.168.1.18:8080` 的连接，也没有现场 HTTP `8087` 监听。
- 系统仍列出 PID `19092 / 6548 / 30936 / 39520` 的 `python.exe`：命令行、可执行路径、用户名均不可读，内存各约 36 KB，父进程已不在查询结果中，没有查询到网络连接。无法确认归属，未批量杀掉其他工程的 Python；不能宣称全系统所有 Python 均已结束。
- 临时目录残留 `C:\Users\sdg17\AppData\Local\Temp\vdo-rm65b-real.hb`，内容指向 PID `19624`。这是退出后的遗留证据，未删除。接手前应重新核对 PID，不要仅凭心跳文件存在判定在运行。
- 停止后未继续发机械臂运动、使能、夹爪或示教命令。

## 当前配置和基准

`config\sim_real_bridge.local.json` 是当前个人配置。`config\local.json` 不存在，所以通用软件/场景路径从 `config\example.json` 读取。

| 字段 | 当前值 |
| --- | --- |
| `real_addr` | `192.168.1.18:8080` |
| `connect_real` | `true` |
| `robot_forward` | `+X`（现场仍需核实） |
| `linear_speed_m_s` | `0.08` |
| `scale` | `0.8` |
| `max_joint_step_deg` | 配置 `10`；当前发送实现再取与 `8` 的较小值 |
| `replay_delay_s` | `0` |
| `max_replay_lag_s` | `0.5` 秒 |
| `max_following_error_deg` | `2` 度 |
| `state_timeout_s` | `0.5` 秒 |
| `max_stream_gap_s` | `0.25` 秒 |
| `gripper_enabled` | 配置为 `true`；本轮用 `-NoGripper` 覆盖禁用 |

用户确认过的基准保存在 `config\real_baseline.local.json`：

```text
MDH / 应用关节（度）: [-1.524, 11.988, 76.758, -1.506, 66.292, 1.641]
控制器关节（度）:    [-1.524,-11.988,-76.758, -1.506,-66.292, 1.641]
auto_move_to_baseline: false
```

J2/J3/J5 的符号转换位于原生 RealPlant 边界，接手时不要再叠加一次符号翻转。基准与最后一次连接时的姿态不同，启动实际用实时关节初始化模型。

## 最后一次实测：明确失败点

原始证据：

- `logs\xr\20261021_170240_543635.jsonl`：逐帧真实手柄输入、控制目标、模型和实体反馈。
- `logs\real\trajectory_20261021_170240_539643.jsonl`：桥接两帧关节输出。
- `logs\real\bridge_error.json`：最后错误摘要，注意后续运行会覆盖它。
- `logs\real\post_timeout_probe_20261021.json`：退出后只读控制器查询。

头显最初没有连接；随后电脑持续收到 `Tracking` 数据，Controller 左右字段齐全。`head=0`，但用户通过左摇杆本体按下设置了 `heading=manual forward`，因此不能再把“Head 缺失”直接认定为此后无法跟随的原因。用户也已确认现场监护、清场和急停条件。

最后真实录制进入过 `following`，共向原生关节 API 提交两帧：

```text
初始应用关节: [-6.719, 14.257, 74.127, 0.542, 67.583, -3.533]
最后发送关节: [-6.73472268, 14.23751335, 74.15177973,
                0.56163262, 67.61375326, -3.57706200]
frames_sent=2
SIM_move=0.27 mm
REAL_joint_move=0.003 deg
sim_real_gap=0.044 deg
queue=0 / replay_lag=0
```

这些数据不足以证明实体跟随。两帧 API 返回值为 0，仅表示调用返回。录制里的有效运动段很短；不要仅据此指责用户没有正确移动，也不要据此推断整个方向映射已经正确。

**确定的异常发生在释放/保持路径**：

```text
JointFollower.hold()
  -> RealArm.leave_passthrough(confirmed=True)
  -> JsonMotionPolicy.leave_passthrough()
  -> RealPlant.leave_passthrough()
  -> JSON movej（当前实测关节，v=10）
  -> 3 秒超时，无应答，桥接报错退出

request:
{"command":"movej","joint":[-6716,-14257,-74126,542,-67585,-3533],
 "v":10,"r":0,"trajectory_connect":0}
reply=null; skipped=[]; error="TimeoutError: timed out"
```

更正此前口头表述：这里明确超时的是 **退出透传/保持的 `movej`**，不是已记录两帧输出的 `Movej_CANFD`。真实超时原因尚未查明，不能通过吞掉异常或把超时改成成功解决。

退出后只读查询：`arm_err=0`、`sys_err=0`、六轴均使能、六轴错误为零，关节基本仍在本次起点附近。控制器返回关节为 `[-6.712,-14.247,-74.128,0.552,-67.615,-3.534]` 度。`collision_stage=0`；`get_arm_software_info` 和 `get_gripper_state` 查询超时，软件版本备用查询成功。详细原始响应在只读报告中。

## 前一轮的真实失败，不要丢失

`logs\real\trajectory_20261021_164020_285860.jsonl` 及对应 `logs\xr\20261021_164020_309939.jsonl` 属于用户提供的真实运行阶段：仿真移动约 90 mm，FIFO 积压增长到约 144 帧、延迟 3.81 秒，最终因超过上限停止；真机关节变化明显落后。手柄能进入 following，不能归因于未连接。

`logs\real\trajectory_20261021_164613_065652.jsonl` 等其他现场日志也要保留。不要把磁盘记录直接自动回放到实体机械臂。

## 本轮已经做过的改动

最新发送器在 PICO 控制线程中同步调用原生 `RealArm.Movej_CANFD`，没有独立后台发送线程。移除了额外 5°/秒限速、0.1 秒预延迟和无应答旁路发送；保留关节反馈差、追踪过期和队列时效检查。目标是让真实 API 调用节奏约束仿真源的推进，当前实现仍需现场修复与验证。

原生滑条路径已经查过：`E:\SDG-VLA\vdo-rm65b\simu\session.py::_drive_from_ui`，最短约 20 ms、最大 8°步长、最小约 0.05°变化，然后 `arm.set_q` → `JsonMotionPolicy.canfd_deg` → `RealPlant.canfd`。最新桥接使用同一南向 CANFD 请求/应答实现，但桥接额外加入了反馈检查与释放时保持，不能说整套行为等同于滑条。

特别检查 `sim_real_bridge.py::after_control_tick`：目前按滑条方式在模型目标超前实际模型关节 >0.5°时取 command，否则取模型 qpos。模型还有自己的动力学跟随。接手者应通过录制核对“手柄目标 → 模型命令 → 模型实测 → 实体命令”的延迟来源，避免把模型的微小动力学变化当作全部操作者目标。

`RealPlant.canfd()` 现有实现将 `TimeoutError` 返回为 `SYS_NORMAL`，因此 API 返回 0 不是硬件执行确认。状态后缀中的英文 `native API returned success; feedback confirms motion` 也容易误读；真实执行只能看后续反馈。

`PicoEeServo` 曾有状态锁与 MuJoCo 数据锁顺序反转导致死锁，已调整状态字符串计算位置并补过并发测试；保留修复，避免回滚时重新引入。

## 工作区有未提交改动

不要 `git reset --hard` 或整目录覆盖。当前工程除了本轮桥接还包含此前安装、适配、录制等未提交修改。

`RM65_PICO` 新增/关键文件包括 `scripts\sim_real_bridge.py`、`scripts\launch_vdo_pico_real.py`、`scripts\vdo_pico.py`、`scripts\real_pico.py`、`scripts\probe_real_robot.py`、`scripts\test_sim_real_bridge.py`、`start_real.*`、`config\sim_real_bridge.*`、`docs\SIM_REAL_BRIDGE.md`、`patches\`。完整范围请重新 `git status --short`。

`vdo-rm65b` 当前脏文件：

```text
launch_rm65_box_cli.py
real/heartbeat.py
real/json_link.py
real/json_plant.py
real/json_policy.py
real/protect.py
real/real_arm.py
simu/cli.py
simu/rm65_arm.py
simu/session.py
launch_rm65_box_real_pico.py（未跟踪）
```

用户最初要求尽量不改 `launch_rm65_box_real.py`，后来允许修改来解决问题。该入口文件本身未出现在当前 git 差异里，但其依赖已经变更，不能称“原版环境完全没动”。本轮 `vdo-sim` 的 git 状态没有输出修改。

## 已验证范围和接手顺序

此前离线执行过：`scripts\test_sim_real_bridge.py` 36 项通过；`python -m unittest discover -s scripts -p 'test_*.py'` 共 82 项通过、1 项跳过；`start_real.bat -Check` 通过。测试使用假控制器/真实模型等，不证明真实设备联动。完整测试后有很小的代码清理和文字调整；停止交接后没有再次启动测试或硬件。

建议接手顺序：

1. 先保存两个工程现有差异及上述现场日志，读取 `bridge_error.json` 和对应 XR 录制。
2. 查明为什么 CANFD 后，`leave_passthrough` 的当前关节 `movej` 无响应。核对真实协议、原版滑条和退出流程；保持失败不能仅隐藏错误。
3. 用上一轮真实 XR 数据在断开硬件条件下复现，统计目标 TCP、模型命令/实测关节与发送点的时间差，检查 0.5°选择阈值及模型动力学的影响。
4. 修复后先跑针对性回归，再启动同一个真实场景做现场联动；分别用模型位移和硬件实测关节验收，记录按住、移动、松开、再次握持和退出全过程。
5. 用户的方向疑问 `+X/-X` 和夹爪仍单独待验收；不能因主链路偶尔发送成功就写“全部完成”。

本文件是交接记录，不是成功验收报告。没有上传 GitHub，没有修改已保存基准，没有清除现场证据。
