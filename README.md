# RM65_PICO

用 **PICO 4 Ultra 左手柄**控制桌面 RM65-B / RMG24 的 **MuJoCo 仿真**。包含机械臂、桌子、立式木块、接收盒和 Gemini335 相机模型。

**遥操通过局域网传输：PICO 连接 Wi-Fi，电脑与它在同一可互访网络，通过电脑 IP 连接 XRoboToolkit PC Service。运行时无需 USB 连接。**

提供遥操、摇杆移动、场景预览、输入诊断、录制和离线回放。安装目录可放在任意盘符；启动器按自身位置寻找代码、模型、虚拟环境和软件，自动显示电脑当前网络 IPv4，不绑定作者电脑的路径或 IP。

**当前范围：Windows x64 + MuJoCo。相机是场景模型，不包含真实相机视频采集；本仓库不包含真机运动入口。**

## 第一次使用

1. 安装 [Python 3.10–3.12 x64](https://www.python.org/downloads/windows/)（建议 3.11），准备 PICO 4 Ultra，并让电脑与头显连接同一可互访局域网。
2. GitHub **Code → Download ZIP** 并解压，或执行：

   ~~~powershell
   git clone https://github.com/sdg123-SDG/RM65_PICO.git
   Set-Location RM65_PICO
   ~~~

3. 双击 install.bat。它建立 .venv、安装固定版本 Python 依赖、下载并校验 PC Service、PICO APK 和 ADB。
4. 若头显已安装 XRoboToolkit，直接跳过。否则用**头显浏览器**打开[官方 APK 发布页](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/tag/v1.1.1)，下载 XRoboToolkit-PICO-1.1.1.apk 并安装；按头显提示允许应用安装。另有可选的 USB/ADB 安装方式，见[安装说明](docs/INSTALL.md)，仅用于装应用。

5. 双击 start.bat，选 **T**（遥操）或 **J**（摇杆）。一次只开一个模式。
6. 电脑与头显连接同一局域网。头显打开 XRoboToolkit，选择启动窗口显示的电脑物理网络 IPv4，等待 **WORKING**，打开 **Head Tracking、Controller Tracking、Send**。
7. 初次建议明确设置前方：**松开左手 Grip → 手柄水平指向想作为“前方”的方向 → 用拇指把左摇杆垂直按进手柄里一下，再松开。** 这是按下摇杆本体，不是推摇杆；状态应显示 manual forward。
8. 按住左手 **Grip** 小幅移动手柄，松开即停止；食指扳机控制夹爪。先分别做 2–3 cm 的前、右、上移动，确认对应关系。

网络 FPS 在跳只能说明头显端在运行；电脑端应出现有效位置，遥操应显示 following。**只有夹爪能动时，优先完成第 7 步**。

## 启动入口

~~~powershell
.\start.bat
# 或直接选择模式：
.\start.bat -Mode teleop
.\start.bat -Mode jog
.\start.bat -Mode diagnostics
.\start.bat -Mode scene
~~~

遥操、摇杆和诊断入口均自动启动 PC Service。关闭仿真窗口或在启动终端按 Ctrl+C，结束该启动器管理的仿真和服务。场景预览不需要头显或服务。service 模式只启动服务，按 Enter 退出。

## 文档导航

| 想做什么 | 文档 |
|---|---|
| 从零安装、头显安装、离线迁移 | [安装说明](docs/INSTALL.md) |
| 按键、朝向设置、录制和回放 | [操作手册](docs/OPERATIONS.md) |
| 换目录、换电脑、速度/倍率/场景 | [配置说明](docs/CONFIGURATION.md) |
| WORKING 但不动、只有夹爪动、方向不对 | [故障排查](docs/TROUBLESHOOTING.md) |
| 代码结构、离线检查和参考对照 | [开发与验证](docs/DEVELOPMENT.md) |
| 本次整理的实际检查结果 | [验证记录](docs/VALIDATION.md) |
| 软件版本、官方来源与 SHA256 | [软件说明](software/README.md)、[下载清单](software/manifest.json) |
| 模型授权及参考项目来源 | [来源与授权](THIRD_PARTY_NOTICES.md)、[参考实现](docs/REFERENCES.md) |

## 当前验证状态

2026-10-18，在 Windows + PICO 4 Ultra 上确认了左手柄输入、夹爪，以及按下摇杆设置前方后的机械臂跟随。默认固定腕部姿态、速度上限 0.06 m/s。

头显姿态未进入电脑时，自动朝向会等待 Head Tracking；手动设置前方已在实际使用中成功。三轴方向精度、整个工作空间的直线轨迹和抓放完成率仍需实际分轴验证。仿真 IK、关节限位和速度限制可能使实际末端运动偏离手柄目标；离线合成检查不能替代实际验证。
