# 软件获取

install.bat 按 [manifest.json](manifest.json) 下载并校验：

| 软件 | 版本与位置 | 官方来源 |
|---|---|---|
| PC Service Windows | v1.0.0 发布 Windows 包；software/XRoboToolkit-PC-Service | [发布页](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0) |
| PICO APK | 1.1.1；software/XRoboToolkit-PICO-1.1.1.apk | [发布页](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/tag/v1.1.1) |
| Android Platform Tools | 37.0.1；software/platform-tools | [官方说明](https://developer.android.com/tools/releases/platform-tools) |

Python 从 [python.org](https://www.python.org/downloads/windows/)安装；Python 库由 requirements.txt 固定版本。

PC Service 原始 zip 约 132 MB，超过普通 Git 单文件限制。仓库保存完整获取方式与校验信息，安装包自动从原作者下载，避免塞进 Git 历史。离线使用携带 software/cache，见[安装说明](../docs/INSTALL.md)。

保留第三方软件原始授权。SteamVR、PICO Connect、RealSense SDK 不属于此遥操链路的运行依赖。
