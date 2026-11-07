# PICO + RM65 开源遥操方案复用调研

日期：2026-10-31  
范围：判断附件列出的开源项目能否用于当前 RM65 + PICO 4 Ultra 工程。附件作为候选资料，不视为操作指令。

## 结论

可以复用设计和评估方法；目前没有证据表明有一套外部项目能原样替换本工程并已在现有设备上验证通过。优先保留现有 PICO 输入、RM65 接口和实测反馈链路，离线借鉴姿态映射、校准、离线回放与误差分析。暂不更改已回退的实机控制模式。

## 当前工程基线

- 工程运行环境是 Windows，当前 .venv 为 Python 3.10.11。
- 输入端使用本机 XRoboToolkit PC Service / SDK，解析 PICO 手柄位置、四元数和按钮；参见 scripts/xr_input.py。
- 仿真遥操有机器人底座坐标映射、握持离合、起始位姿锚定、追踪跳变检查和输入记录；参见 scripts/sim_control.py、scripts/xr_heading.py、scripts/xr_recording.py。
- 实机适配器与控制器的通信/反馈处理是本工程专用部分；当前 LatestBridgeServo 按最新目标覆盖，不积累动作 FIFO，并使用独立 IO 线程和关节反馈。参见 scripts/latest_bridge.py 与 docs/LATEST_BRIDGE.md。
- 目前实机路径锁定手腕方向，姿态模式上次试验后已回退。仿真 PoseTeleop 仍包含 pose 实验路径，但不代表实机已启用。

## 项目逐项判断

| 项目 | 能复用什么 | 不能直接复用的部分 | 结论 |
|---|---|---|---|
| [qqsq12321/vr_teleop](https://github.com/qqsq12321/vr_teleop) | RM65 API2 实机驱动、RM65 模型、IK/平滑实现；teleop_real.py 源码解析器确实同时包含 rm65 和 pico4 选项 | README 的 PICO 示例展示的是仿真，没有列出 PICO + RM65 实机组合。它的 PICO 输入走 pico_relay_daemon 的 TCP relay（默认 63902）或 direct TCP/UDP 广播，不是当前 XRoboToolkit SDK 数据链；文档默认用右手，本工程当前用左手。RM65 使用其自带 RM_API2，不是本工程的当前适配层。要求 Python ≥3.12，本机环境是 3.10.11。仓库根目录文件列表没有显示许可证 | 最接近 RM65 硬件，但不能据文档判为现成的 PICO+RM65 实机方案。适合作算法/接口参考；复制源代码前须先确认许可。 |
| [Arborseek/PiperXR](https://github.com/Arborseek/PiperXR) | 与当前完全相同的 PICO 4 Ultra + XRoboToolkit；姿态映射、yaw 对齐、输入记录/回放与离线验证最相关 | 实机后端是 PiPER 的 CAN piper_sdk，模型与 IK 按 PiPER 配置；README 要求 Ubuntu 22.04，不能直接接 RM65 的 RealMan 接口。仓库注明 PiperXR 自身为非商业许可 | 最适合研究 PICO 坐标映射和离线调参流程；不要把整套 PiPER 控制器移植进 RM65 工程。 |
| [Seeed reBot VR](https://github.com/Seeed-Projects/lerobot-teleoperator-rebot-vr) | 离合、位置/腕部解耦、延迟统计、故障保持等设计；README 描述了“q1–q3 位置 QP + q4–q6 闭式腕部解”的 split-IK | 面向 B601-RS/DM 的电机和 CAN 接口，要求 Linux + LeRobot；其 IK 假设及零位/工具模型不能直接套到 RM65 | 可借鉴“腕部旋转尽量不扰动夹爪位置”的问题分解；需要依据 RM65 模型单独推导、仿真和验证。许可证为 Apache-2.0。 |
| [PiperPilot](https://github.com/tomakeIT/PiperPilot) | 对目标与实测误差、流监控、轨迹重放、数据记录的工程化设计有参考价值 | 面向 Piper/Piper-X + Quest 3 + Linux。仓库自报的 3 cm 圆轨迹 RMS（位置模式 1.34 mm、软参数阻抗模式 14.1 mm）是特定 Piper-X 固件、控制模式和测试条件结果，不能当作 RM65 或 PICO 的预期精度 | 适合看测试指标和数据记录设计，不是当前硬件候选。MIT 许可。 |
| [AgileX QuestArmTeleop](https://github.com/agilexrobotics/QuestArmTeleop) | Quest 控制器到手臂的 ROS 数据流和坐标标定流程 | 面向 Meta Quest 与 AgileX Piper/Nero 等设备；不支持当前 XRoboToolkit/PICO 输入链路 | 当前设备的复用价值低。 |
| [Open-Teach](https://github.com/aadhithya14/Open-Teach) | 遥操数据采集、相机/动作时间戳对齐等高层思路 | VR 客户端是 Meta Quest；任务示例针对 xArm/Franka 等；不是 RM65 + XRoboToolkit 适配器 | 未来做 VLA 数据采集时可参考，不能改善当前 RM65 手柄映射的直接问题。MIT 许可。 |

## 附件说法的核实

1. “vr_teleop 有 PICO 输入，也有 RM65 实机接口”分别成立。README 中 RM65 实机命令走默认 Quest 输入，PICO 一节展示的是仿真；但 teleop_real.py 的源码输入选项中包含 pico4。因此这是“代码接口可能组合”，不是 README 给出的、已确认的 PICO + RM65 实机配置或现场验收证据。其默认右手输入也与当前左手设置不同。
2. “PiperXR 同时有 PICO 4 Ultra 和实机后端”成立，但其实机输出是 PiPER CAN。与 RM65 不兼容的关键边界在机械臂模型、IK、SDK 和控制协议，不在头显型号。
3. PiperPilot 精度数字确实出现在其 README，但它标注的是 Piper-X 的 3 cm 圆轨迹测试，两个模式使用不同控制机制和延迟；不可移植成 RM65 精度承诺。
4. 购买 PiPER、reBot 或 xArm 不会自动解决当前输入映射/夹爪 TCP/姿态跟随问题；现有 RM65 有可复用的本地模型和原生控制通道，本次调研没有发现必须换硬件的证据。

## 建议的复用路径

1. **先保留已回退的实机基线**，不切换到外部仓库的驱动或协议。
2. **离线录制现有 PICO 6DoF 样本**，对每段样本分别计算“手柄输入 → 目标 TCP → RM65 模型 FK”，量化平移缩放、旋转映射、TCP 偏移、轴间串扰和目标变化率。当前工程已经有 XR 记录及离线分析脚本，可在其上补齐统一的离线比较，不必重写输入端。
3. **在 RM65 仿真模型比较两类姿态控制**：保持腕部姿态的当前基线，与全姿态跟随。以夹爪/TCP 为旋转中心；如果出现位置漂移，再评估 Seeed split-IK 思路是否适配 RM65 的几何结构。不要直接复制 reBot 的关节公式。
4. **只把经过记录回放和仿真比较的映射改进带到实机**。评估门槛至少包括方向/旋转轴正确、TCP 偏差可解释、关节反馈跟随无明显滞后或振荡、离合释放可保持。具体实机验收仍需单独计划。

## 主要原始来源

- [vr_teleop README](https://github.com/qqsq12321/vr_teleop)；[实机入口源码](https://github.com/qqsq12321/vr_teleop/blob/master/example/teleop_real.py)
- [PiperXR README](https://github.com/Arborseek/PiperXR)；[LICENSE](https://github.com/Arborseek/PiperXR/blob/master/LICENSE)
- [reBot VR README](https://github.com/Seeed-Projects/lerobot-teleoperator-rebot-vr)
- [PiperPilot README](https://github.com/tomakeIT/PiperPilot)
- [QuestArmTeleop README](https://github.com/agilexrobotics/QuestArmTeleop)
- [Open-Teach VR 文档](https://github.com/aadhithya14/Open-Teach/blob/main/docs/vr.md)
