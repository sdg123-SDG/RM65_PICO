# 换电脑与配置

IP 不写进代码：启动器显示当前地址，头显选择正确电脑。换盘符建议在新位置重新解压/克隆、运行安装器建立 .venv。

~~~powershell
Copy-Item config\example.json config\local.json
~~~

local.json 不上传，启动器优先读取它。保留完整结构，不自动合并缺失字段。也可指定配置：

~~~powershell
.\start.bat -Mode teleop -Config "D:\my settings\rm65.json"
.\start.bat -Mode teleop -Check
~~~

-Check 检查配置路径及模型，不启动服务。相对路径均相对于仓库根目录，不是配置文件目录；也接受绝对路径。

| 配置 | 含义 |
|---|---|
| service_dir | 解压后的服务根目录，包含 RoboticsServiceProcess.exe 和 SDK/x64 |
| scene | 场景 XML |
| tcp_offset_m | link_6 局部夹持点，默认 [0,0,0.137] 米 |
| teleop.frame | robot：底座固定；view：旧的画面方向模式 |
| teleop.mode | position：固定腕部；pose：姿态也跟随 |
| teleop.scale | 手柄位移倍率，0.05–3 |
| teleop.speed | 速度，0.01–0.30 m/s，默认 0.06 |
| teleop.yaw_deg | 附加水平旋转角；手动设置前方后通常保持 0 |
| jog.frame / jog.speed | 摇杆方向与速度 |

使用已有服务时，把 service_dir 指向原解压目录。该路径的服务已运行时可显式接管：

~~~powershell
.\start.bat -Mode teleop -ReuseService
~~~

仅接管同路径的单个服务，退出也会停止它。其他遥操/诊断进程会阻止启动新模式。

## 直接运行 Python

默认路径按仓库位置推导。启动器设置 RM65_XR_SERVICE_DIR、RM65_XR_SDK_DIR、RM65_SCENE、RM65_TCP_OFFSET（逗号分隔的三个米数）。直接运行 Python 不会自动启动或管理服务，推荐使用根目录启动器。

## 换场景的边界

控制器专用于 RM65/RMG24，需要 base_link、link_6、joint_1..joint_6、joint_1_ctrl..joint_6_ctrl、rmg24_left/right、rmg24_ctrl、home keyframe 等命名。改变桌高、机器人初始位置或工具时，还需审核 scripts/sim_control.py 工作空间、scripts/home_pose.py 初始关节和 TCP。

目录/IP 自适应不表示任意机械臂可直接复用。新模型仍需对应几何、碰撞、关节限位与伺服参数。
