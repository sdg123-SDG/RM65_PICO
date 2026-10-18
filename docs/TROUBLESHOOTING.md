# 故障排查

## 只有夹爪动

MISSING Head 表示收到手柄但缺少头部朝向。扳机能控制夹爪，机械臂则等待方向基准。

已实测解决步骤：**松开 Grip → 手柄水平指向前方 → 用拇指把左摇杆本体垂直按进手柄里一次，再松开 → manual forward → 按 Grip 移动**。

也可开启 Head Tracking + Send，等电脑收到头部姿态后使用自动模式。FPS 在跳不等于电脑 Head=OK。

## WORKING / FPS 在跳，但不跟随

- no fresh tracking：没有新鲜有效位置，检查 Controller Tracking、Send、头显清醒、手柄可追踪。
- released：未按住侧面 Grip。食指扳机和侧面 Grip 是不同按键。
- MISSING Head：按上一节设置前方。
- jump / 要求释放：可能 XR 重新定位或位移过大，释放 Grip 后重新启用。

关闭遥操/摇杆，运行：

~~~powershell
.\start.bat -Mode diagnostics
~~~

应有非空位置、Grip 随侧面按键从 0 到 1、age 小于约 0.25 s。SDK_DIAG 的 server 连接只证明本地 SDK 连到服务；tracking、head 计数与位置刷新才反映相应数据进入电脑。

## 完全连不上

确认同局域网，头显重新选择电脑；服务重启后旧 WORKING 状态可能未对应新实例。选物理网卡地址，不选 VMware/VPN/127.0.0.1。检查防火墙允许服务。校园/访客 Wi-Fi 可能隔离设备，可换到允许互访的网络。

一次只开一个服务、一个 Python SDK 客户端。从原启动窗口正常退出，再启动新模式。

## 方向偏、向上走斜

1. 用 frame=robot，重新手动设置前方。别把旋转后的屏幕左右当成底座左右。
2. 向上使用 XR 重力轴；不要恢复旧四点手推校准。
3. 每次只测一个方向，慢移 2–3 cm；快速手势会遇到速度限制。
4. 对比手柄、目标 TCP、实际 TCP。目标正确而实际偏时，继续查 IK、初始姿态、限位、碰撞和伺服，不能只改坐标矩阵。

## 安装和启动

| 情况 | 处理 |
|---|---|
| 找不到 Python | 安装 3.10–3.12 x64，或给 install.bat 加 -Python |
| SHA256 不一致 | 不运行该包，核对官方源和代理，替换对应缓存 |
| 下载慢/失败 | 按清单链接手动下载到 software/cache，再重跑 |
| DLL 失败 | 确认 Windows/Python 均 x64、SDK/x64 完整，安装附带 VC++ x64 运行库 |
| XML/mesh 缺失 | 保持整个 assets 相对结构，不只复制 scene.xml |
| 没窗口 | doctor --offline 检查模型，再查显卡/OpenGL 和桌面会话 |
| 已有服务/客户端 | 正常关闭原窗口；同路径服务可用 -ReuseService 接管 |
| 可选 APK 安装时 ADB unauthorized | 在头显接受 USB 调试授权；此项与运行时局域网遥操无关 |

服务残留时，在任务管理器确认完整路径和 PID，只结束对应实例。
