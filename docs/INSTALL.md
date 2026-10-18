# 安装与迁移

## 环境

- Windows 10/11 x64；PICO 4 Ultra、左手柄。
- CPython 3.10–3.12 x64，建议 3.11。安装器寻找 py、python 和用户目录中的标准 Python 安装；不依赖特定盘符。缺少 Python 时需先从[官方页面](https://www.python.org/downloads/windows/)安装。
- 电脑与头显在可互访的局域网；首次下载需访问 GitHub、PyPI、Google Android 下载站。
- USB 数据线；头显开启开发者模式并接受 USB 调试授权。

本 XRoboToolkit 链路不要求 SteamVR、OpenXR、PICO Connect、PICO Streaming Assistant。

## 电脑安装

双击 install.bat，或在仓库目录：

~~~powershell
.\install.bat
# 指定 Python：
.\install.bat -Python "C:\path\to\python.exe"
~~~

安装器建立 .venv、安装 requirements.txt 固定版本，从 software/manifest.json 指定的官方地址下载 PC Service/APK/ADB，校验 SHA256 后解压，最后离线加载模型。不会启动遥操服务。

安装失败可以重跑；正确缓存不重复下载。原始解压目录保留在 software/extract-* 便于恢复，不提交到 Git。

若缺少 VCRUNTIME140 等 DLL，运行包附带的 software/XRoboToolkit-PC-Service/VC_redist.x64.exe，按微软界面完成安装。不要从不明网站下载单个 DLL。

## 头显安装

USB 连接后戴上头显，允许 USB 调试：

~~~powershell
.\software\platform-tools\adb.exe devices
~~~

应显示 device。unauthorized 表示尚未在头显授权；空列表检查数据线、开发者模式和 USB 驱动。

~~~powershell
.\software\platform-tools\adb.exe install -r -g .\software\XRoboToolkit-PICO-1.1.1.apk
~~~

也可用头显浏览器从[官方发布页](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/tag/v1.1.1)下载安装。应用通常在“未知来源”。

## 第一次连接

1. 双击 start.bat 选 T。窗口显示当前带默认网关的网卡 IPv4；选择与头显同网段的物理 Wi-Fi/以太网，忽略 VMware、VPN、127.0.0.1。
2. Windows 防火墙出现联网提示时，对当前可信局域网允许 PC Service。
3. 在头显 XRoboToolkit 选择电脑，等待 WORKING。服务重启后应重新选择电脑。
4. 开启 Head Tracking、Controller Tracking、Send；按[操作手册](OPERATIONS.md)设置前方，小幅试动。

本机观察到头显 TCP 63901、SDK 回环 60061；更换服务版本/配置时应以实际监听为准。SDK 的 127.0.0.1:60061 不是头显连接地址。

## 无头显检查

~~~powershell
.\start.bat -Mode scene
.\.venv\Scripts\python.exe scripts\doctor.py --offline
.\.venv\Scripts\python.exe scripts\doctor.py
~~~

--offline 检查 Python/模型；不加参数还检查服务/DLL 文件是否存在。文件存在不代表实时输入已通。

## 离线转移

联网电脑安装后，软件安装包在 software/cache。额外准备 Python wheels：

~~~powershell
.\.venv\Scripts\python.exe -m pip download -r requirements.txt -d software\cache\wheels
~~~

复制仓库和 software/cache 到目标电脑。双方需相同 Windows x64 与 Python 次版本，例如都为 3.10。目标电脑先安装 Python，然后：

~~~powershell
.\install.bat -Offline
~~~

不要直接迁移 .venv，其中可能含旧路径；在新位置重建。Python 安装程序本身需提前从官方获取。只复制 Git 仓库、不复制缓存时仍需联网。
