# RM65 + PICO 真机遥操（实验性，未现场验收）

> 本文为首版历史说明，已被 [真机采集适配版操作说明](REAL_PICO_V2.md) 替代。不要按下文旧的 TCP、关节符号、示例工作空间或 pico arm 流程操作。

这是独立真机入口。原有 `vdo-rm65b/launch_rm65_box_real.py` 不改；继续复用新版原生 RealArm、实测关节镜像、状态采样、运行保护与退出流程。普通 `start.bat -Mode teleop` 仍然是仿真。

**目前只经过假控制器与离线模型验证，没有连接、驱动真实机械臂。软件限幅与看门狗不是安全认证功能，不保证断网时立即停车，不替代硬件急停、隔离空间或现场监护。**

## 1. 先准备环境与参数

沿用本仓库 install.bat / setup_vdo.ps1 安装的依赖、XRoboToolkit PC Service、PICO 应用，以及同级最新 vdo-rm65b、vdo-sim。不要与原真机程序、其他控制软件或仿真 PICO 客户端同时运行。原真机入口需要先由现场人员确认可正常读取、镜像、停止；本入口不会自动辨认不同硬件或完成标定。

在 PowerShell 进入仓库：

```powershell
Set-Location E:\SDG-VLA\RM65_PICO
Copy-Item config\real_pico.example.json config\real_pico.local.json
notepad config\real_pico.local.json
```

修改本地配置，不要重复复制覆盖自己已有的配置。local 文件已加入 Git 忽略。

- `real_addr`：实际控制器地址，示例为 `192.168.1.18:8080`，不代表已检测在线。
- `camera`：与真实装配一致的模型，默认 d435i。模型 TCP 为 EG2 的 `eg2_tcp`；更换夹爪、加长连接件、装配偏置必须先更新并验证模型，不能直接沿用。
- `workspace_base_m`：**机器人底座坐标系、米**。示例 min `[0.05,-0.30,0.10]`、max `[0.55,0.30,0.65]` 只是示例，不是测量结果。必须按现场桌面、障碍物和初始末端位置设置；不能靠扩大范围绕过报错。
- `linear_speed_m_s=0.02`、`scale=0.5`：末端速度上限与手柄位移倍率。
- `joint_speed_deg_s=5`、`max_joint_step_deg=0.15`：关节速度与单帧变化上限。
- `session_radius_m=0.05`：每次启用时以实测 TCP 为中心的 5 cm 活动半径；重新启用前必须先 disarm。
- `max_following_error_deg=2`、`max_tcp_following_m=0.01`、`state_timeout_s=0.25`：跟随与反馈约束，不建议未经验证直接放宽。
- `gripper_enabled=false`：默认不发送夹爪命令；需要确认真实夹爪型号、控制接口、夹持空间后才能改 true。启用后仅有效跟随且按住 Grip 时接受扳机控制。

TCP 位置边界不是机械臂整体碰撞检测：肘部、腕部、工具或线缆仍可能碰撞。首次验收必须移除障碍物，保持人员远离运动范围，专人持硬件急停。

## 2. 离线检查（不会连接真机）

```powershell
.\start_real.bat -Check
```

也可以指定配置：

```powershell
.\start_real.bat -Profile config\real_pico.local.json -Check
```

检查模型、配置和原入口文件摘要；不启动控制器连接、保护子进程、PICO SDK 或运动。检查通过不代表现场可安全运动。

## 3. 联机启动与操作

完成现场安全检查后，由操作人员执行：

```powershell
.\start_real.bat
# 或指定地址/配置：
.\start_real.bat -Profile config\real_pico.local.json -Real 192.168.1.18:8080
```

启动器启动 PC Service 和独立真机程序。头显与电脑走局域网，不需要 USB；打开 XRoboToolkit，连接电脑 IP，等待 WORKING，开启 Head Tracking、Controller Tracking、Send。

1. 启动后是 **DISARMED**：显示实测关节镜像，PICO 不下发遥操运动。连接阶段仍会执行原程序的初始化与保护逻辑，不等同于“完全不与控制器通信”。
2. 松开左手 Grip 和扳机。手柄水平指向希望作为“前方”的方向，用拇指把左摇杆**垂直按进手柄一下并松开**，不是推动摇杆。确认 manual forward。它只建立水平朝向，竖直仍保持竖直。
3. 在电脑原生 CLI 输入 `pico status`，确认输入有效；然后输入 `pico arm`。需要新鲜反馈、有效朝向、Grip/扳机已释放、无报错且末端在工作空间内。
4. 按住左手 Grip，先只做几毫米的前后、左右、上下单轴移动；松开 Grip 请求保持。默认固定启用时的腕部方向，不跟随手柄旋转。不要一开始做大幅运动。
5. 暂停输入 `pico disarm`。断流、跟踪跳变等会撤销启用，重新设置朝向、确认状态后再 arm；严重通信/保护故障应退出、排查后重新启动，不能反复强行启用。
6. 正常结束使用原生 CLI `quit`。启动器等待原生停止/退出后关闭自己启动的服务；若真机进程未能退出，不会强制杀掉它，按提示检查并视情况使用硬件急停。不要把关闭终端当作急停操作。

**`pico arm` 是允许之后按 Grip 遥操，不会主动把机器臂送到仿真初始姿态。** 镜像以实测状态为准。默认关闭 HTTP；即便手动开启，也不要在遥操启用期间并行发运动命令。启用时另一个写入来源会被拦截并触发故障。不要使用 `ui drive`，此入口只允许实测镜像。

## 4. 坐标、控制与保护的实现边界

新代码位于 scripts/real_pico.py、launch_vdo_pico_real.py、real_pico_guard.py；原仓库同级新增 launch_rm65_box_real_pico.py 薄入口。原 launch_rm65_box_real.py 不改，也不改变原 RealArm / JSON / 保护模块的源码。已有仿真接入补丁保留。

PICO 相对位移 → 已标定水平朝向 → 模型世界/底座位置目标 → 独立无网络 IK → 限幅关节目标 → 原 RealArm 的关节 CANFD 通路。J2/J3/J5 符号转换沿用原协议边界，只转换一次。不使用可能受控制器当前工具/工作坐标系影响的 Cartesian CANFD；但本地 TCP 模型与真实装配仍必须一致，不能因此省略标定。

限幅、反馈时间戳、控制器错误、关节/TCP 跟随误差、发送间隔和单次活动范围检查均在新适配层。发送超时不算成功；流式发送完成只代表交给网络，不代表控制器已执行，通过实测状态监控跟随。Grip 释放使用实测关节请求退出透传/保持；停止请求失败时会报告未确认并交由原退出/保护流程处理，不宣称已经停稳。

保留原运行保护与独立守护进程，仅在新入口中替换 Windows PID 存活检查（使用只读进程查询，避免 os.kill(pid,0) 在 Windows 上终止目标）及心跳文件路径。控制循环停滞会中止连接、请求原退出恢复。原保护在网络中断、控制器占用、系统卡死或通信失败下仍可能无法发送停止，因此必须保留物理急停。

协议依据：[官方关节透传说明](https://develop.realman-robotics.com/robot/json/motionConfig/)、[新代控制器运动协议](https://develop.realman-robotics.com/robot4th/json/motionConfig/)、[Python Windows 进程信号说明](https://docs.python.org/3/library/os.html#os.kill)。控制器型号、固件与透传周期需现场确认；此实现不承诺硬实时控制。

## 5. 验收记录

2026-10-19 完成：假控制器测试、离线模型检查、原真机入口摘要检查。覆盖未启用不运动、重复启用不能重置边界、发送超时、过期/异常反馈、越界、跟随误差、保持失败、关节符号、其他写入者冲突和 Windows PID 检查。未做真实硬件急停、断网、控制器兼容性、停止距离或方向验收。

现场必须逐项验证：实测镜像角度正确；模型与真实 TCP 一致；低速单轴方向正确；Grip 释放与 disarm 可停止；错误反馈可撤销控制；最后再考虑夹爪和更大空间。测试网络故障前应先由具备机械臂安全经验的人员制定安全测试方案。
