# 参考实现

Windows 输入用 [XRoboToolkit PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) SDK，以 ctypes 读 JSON，不需编译 Linux pybind。

| 项目 | 参考做法 | 对照提交 |
|---|---|---|
| [DROID](https://github.com/droid-dataset/droid) | 桌面单臂、相对起点、握持启停、重新定义前方 | 33ae6a67274f36d2e29525b86f23a56616ef43a7 |
| [PiperXR](https://github.com/Arborseek/PiperXR) | PICO/MuJoCo、水平朝向、冻结映射、录制回放 | 59e4ba7c004c71c09aa44a8b53a6b49c90a19190 |
| [XRoboToolkit 官方 Python 示例](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python) | 基座 IK、相对位姿、释放参考 | 79e5cb8a56e3455515ce1b476e993c764ec58739 |
| [vr_teleop](https://github.com/qqsq12321/vr_teleop) | RM65/PICO、TCP 与基座变换；输入方式不同 | 资料链接，不作依赖 |

各项目方向定义并不完全相同，不能把一个矩阵当成所有场景的校准。当前用保持重力轴的水平变换，缺头部数据时显式按下摇杆设置前方。

不打包整份参考工程，不安装其无关真机/总线依赖。数学思路与源码对照不等于获得额外再发布授权；使用参考源码应遵循原许可。
