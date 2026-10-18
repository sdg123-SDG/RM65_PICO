# RealMan RMG24 两指平行夹爪

睿尔曼官方 **RMG24** 仿真资源。由开源 STEP 装配转成 STL + MJCF，可单独加载。RM65 本体 `rm65b.xml` / `rm65b_gravcomp.xml` 已挂上该夹爪。

## 来源

- 开源仓库 [RealManRobot/Two-Finger-Gripper](https://github.com/RealManRobot/Two-Finger-Gripper)
- 本机 CAD：`res/rm65b/github/Two-Finger-Gripper/Hardware/CAD/RMG24 Claw 3D Model.stp`
- 产品参数：[RMG24 两指平行夹爪](https://www.realman-robotics.cn/cn/products/rmg24-gripper.html)

官方包没有 URDF，只有装配 STEP。`gen_rmg24.py` 按实体包围盒把零件分成底座 / 左指 / 右指。

## 目录

```text
rmg24/
├── rmg24.xml          # 可直接加载的 MuJoCo 模型
├── gen_rmg24.py       # STEP → STL + 重写 rmg24.xml（惯量）
└── meshes/
    ├── base.stl
    ├── finger_left.stl
    └── finger_right.stl
```

## 坐标系与运动

| 项 | 值 |
|----|-----|
| 原点 | 安装法兰中心（CAD 中开口平面 x = −55 mm 移到原点） |
| +Z | 沿夹指，离开法兰 |
| +Y | 开口方向（右指为正） |
| 行程 | 65 mm（每指滑动 32.5 mm） |
| q = 0 | CAD 姿态，接近完全闭合 |
| 质量 | 0.674 kg（按网格体积比例分配到三体） |

关节 `rmg24_left` / `rmg24_right` 为沿 Y 的滑移，equality 保证对称开合。官方是 **单自由度**：一个 WHJ03 关节把回转变成两指同步直线，指令是开口度 0–1000（0 mm–65 mm），外加速度、力矩，**不能左右独立**。仿真执行器 `rmg24_ctrl` 对应开口宽度 `[0, 0.065]` m。

## 加载

```bash
# 在 vdo-sim 根目录
python -c "import mujoco; mujoco.MjModel.from_xml_path('assets/objects/rmg24/rmg24.xml'); print('OK')"
```

重新从 STEP 导出（conda 环境 `vdo-cad`，由当时的 uv 环境复制）：

```bash
conda activate vdo-cad
python assets/objects/rmg24/gen_rmg24.py
```

依赖见 `requirements-cad.txt`（CadQuery 2.8 / Trimesh / NumPy / MuJoCo）。若环境不在了：

```bash
conda create -n vdo-cad python=3.12 -y
conda activate vdo-cad
# 需要代理时：export https_proxy=http://127.0.0.1:7897
uv pip install --python "$(which python)" -r assets/objects/rmg24/requirements-cad.txt
```
