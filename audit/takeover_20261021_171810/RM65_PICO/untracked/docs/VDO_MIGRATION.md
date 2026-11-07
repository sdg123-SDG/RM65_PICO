# 2026-10-19：新版仿真适配

当前使用以下同级目录：

| 路径 | 用途 |
|---|---|
| RM65_PICO | PICO 软件、输入、方向映射、启动器与新版适配器 |
| vdo-rm65b | 最新控制程序、原生 CLI/HTTP/视觉、物理会话与运动接口 |
| vdo-sim | 最新场景、EG2 夹爪、相机与全部模型资源 |

本机都放在 E:\SDG-VLA 下。默认启动命令保持：

~~~powershell
Set-Location E:\SDG-VLA\RM65_PICO
.\start.bat -Mode teleop
# 或新版原生后端的摇杆控制：
.\start.bat -Mode jog
~~~

PICO 仍通过局域网选择电脑 IP，运行不需要 USB。原 PC Service、SDK、APK 继续使用。握住左 Grip 跟随、松开停止；扳机控制夹爪；缺 Head 时松开 Grip，手柄水平指前，垂直按一下左摇杆本体设置前方。

窗口的原生控制台保留，可输入 help、status、camera 等；新增 pico on / pico off / pico status。执行原生规划命令前建议 pico off；规划 busy 时适配器会停止发送并要求松开 Grip，避免两种运动命令抢占。

## 适配内容

- PICO 位移映射先在世界系生成，再明确转换到原生 base_link 系，处理基座高度与旋转。
- 通过新版 Rm65ExtArm.Movep_CANFD 下发法兰位姿，使用原生 IK/关节约束；不实例化旧 SimArm，不修改新版模型的伺服或动力学参数。
- 参考位姿取 FK(commanded joints)，避免将重力下垂当作新的手柄位移。日志区分目标、命令 FK 和实际 TCP。
- 从模型 eg2_tcp site 读到局部 TCP 为约 [0,0,0.158164] 米，不使用旧 RMG24 的 0.137 米硬编码。
- 夹爪调用原生 Set_Gripper_Position，0–1000 自动对应当前夹爪全行程。原生夹爪存在执行时间，新的扳机目标在其空闲后更新。
- 复用原生 ArmSession/world/viewer、CLI、相机与 HTTP。TCP 处红/绿/蓝箭头表示基座 +X/-Y/+Z，区别于原生法兰局部轴。
- 限制每拍位置和姿态增量；原生 IK 拒绝时显示错误，连续拒绝会要求松开重新启用。

相机在 config/local.json 中设置 vdo_camera 为 d435i / gemini335 / none。默认与新版原生入口一致为 D435i。新版后端使用 site 自动决定 TCP；旧 tcp_offset_m 不覆盖它。

## 源码与安装

config/vdo_sources.json 固定本次拉取的提交：

- vdo-rm65b：486497242172423ea9bfaf95b49516a218255210
- vdo-sim：cd4c8af57a92ba2852f1ffc03265cfd7d872274b

setup_vdo.ps1 使用已配置的 GitLab SSH 权限获取代码（服务 SSH 端口 9522），不分发凭据。既有目录不是对应版本时会停止，不重置用户文件。主安装器会调用它；已有代码时可执行：

~~~powershell
.\setup_vdo.ps1 -Offline
.\start.bat -Mode teleop -Check
~~~

原生 launcher 加入控制器 factory/overlay 接口、无 CLI 时的退出条件和启动失败处理；另为原生 CLI 补充 Windows msvcrt 键盘输入，解决 termios 缺失退出问题。两个补丁位于 patches/，可审阅、幂等应用。没有改动原生 real 后端，PICO 新入口拒绝真机及 JSON 后端参数。

## 验证范围

新版场景加载：nq=15、nv=14、nu=19；读取 EG2 TCP、原生夹爪行程、busy 拒绝写入。六方向 20 mm 合成手势通过原生 Movep_CANFD，命令 FK 约 19.7–19.9 mm。原生物理模型预先静置后，实际 TCP 增量约 19.6–20.0 mm，终点横向偏移不超过约 0.29 mm，过程中最大横向偏移约 1.45 mm；仅对应初始姿态附近的小位移，不能当成真实 PICO 或全工作空间的精度。

自动检查共 15 项，14 项通过、1 项可选 PiperXR 源码对照跳过。新入口已实际启动窗口、相机小窗、HTTP 和 PICO SDK，完成交互式 pico status 与 quit，并检查统一退出。干净源码上两个补丁的首次应用及重复应用均通过。随后现场日志确认收到手柄数据，但没有头部姿态，且设置前方时摇杆点击被轻微 Grip 压力消耗。现已加入回归检查：未运动时点击可记住 2 秒，在 Grip 松开后完成校准；正在运动时不会重设朝向。该修复需重启进程生效；真实方向、跟随感与抓放仍需现场确认。

旧记录默认回放入口仍对应旧物理环境，不用于宣称新版轨迹复现；新版可使用原生轨迹记录功能及 PICO 原始输入日志做分析。

仓库中的历史软件安装、网络操作文档继续适用；涉及旧 assets、RMG24、旧回放结果的段落以本说明为准。公开 CI 只能执行不依赖私有源码的回归检查，原生适配检查需要这两个已授权的同级仓库。
