# RealMan RM65-B

睿尔曼 RM65-B 标准版，6 自由度协作臂。工作半径 610 mm，\(d_6 = 144\) mm。

官方资料：[本体参数：RM65 系列参数及 D-H 模型](https://develop.realman-robotics.com/robot/robotParameter/RM65OntologyParameters/)；模型仓库：[RealManRobot/rm_models/RM65](https://github.com/RealManRobot/rm_models/tree/main/RM65)。

## 本仓库文件

```text
rm65b/
├── rm65b.xml              # MuJoCo + RMG24（URDF 默认姿态：原点、+Z 朝上）
├── rm65b_gravcomp.xml     # 同上，连杆/夹爪 body gravcomp=1
├── gripper_rmg24.xml      # RMG24 片段（被 rm65b.xml 引用）
├── gripper_rmg24_gravcomp.xml
├── rm65b_legacy.xml       # 历史备份：四连杆夹爪
├── rm65b_legacy_gravcomp.xml
├── gripper_rm65_legacy.xml
├── gripper_rm65_legacy_gravcomp.xml
├── urdf/RM65-B.urdf       # 官方 SolidWorks URDF
├── compare_mdh_mjcf.py    # 官方 MDH 与 MJCF 正向运动学对照
├── gen_arm_visual.py      # STL → 视觉 OBJ（旋转平面黑环 + 天线侧蓝面板）
├── imgs/                  # 官网本体参数页截图
├── textures/              # 手臂 albedo（白壳 / 黑环 / 蓝面板 / 旧夹爪贴图）
└── meshes/
    ├── *.STL              # 碰撞
    ├── gripper/           # 旧四连杆夹爪 STL（legacy）
    └── visual/            # 带 texcoord 的视觉 OBJ
```

## 基础参数

| 项目 | 值 |
|------|----|
| 自由度 | 6（仿人构型） |
| 工作半径 | 610 mm |
| 有效负载 | 5 kg |
| 自重 | 7.2 kg |
| 重复定位精度 | ±0.05 mm |
| TCP 线速度 | ≤ 1.8 m/s |
| 典型 / 峰值功率 | ≤ 150 W / ≤ 900 W |
| 安装角度 | 任意 |
| 底座尺寸 | φ107 mm |
| 材质 | 铝合金 / ABS |
| 工作温度 | 0–45 ℃ |
| 工作湿度 | 25–85%，无结露 |
| 关节制动 | J1–J3 硬抱闸，J4–J6 软抱闸 |

## 关节限位与最大角速度

角度、角速度两种单位并列。MJCF `range` / `ctrlrange` 使用弧度列。MuJoCo 关节无速度上限，角速度列为手册约束。

$$
\theta_{\mathrm{rad}} = \theta_{\mathrm{deg}}\cdot\frac{\pi}{180},\qquad
\omega_{\mathrm{rad/s}} = \omega_{\mathrm{deg/s}}\cdot\frac{\pi}{180}
$$

$$
180~^\circ/\mathrm{s} = \pi~\mathrm{rad/s},\qquad
225~^\circ/\mathrm{s} = \tfrac{5\pi}{4}~\mathrm{rad/s}
$$

| 关节 | 部位 | 角度范围 (°) | 角度范围 (rad) | 最大角速度 (°/s) | 最大角速度 (rad/s) |
|------|------|--------------|----------------|------------------|--------------------|
| J1 | 肩 | ±178 | ±3.106686 | 180 | 3.141593 |
| J2 | 肩 | ±130 | ±2.268928 | 180 | 3.141593 |
| J3 | 肘 | ±135 | ±2.356194 | 225 | 3.926991 |
| J4 | 腕 | ±178 | ±3.106686 | 225 | 3.926991 |
| J5 | 腕 | ±128 | ±2.234021 | 225 | 3.926991 |
| J6 | 腕 | ±360 | ±6.283185 | 225 | 3.926991 |

执行器 `forcerange` 对齐 URDF `effort`：60 / 60 / 30 / 10 / 10 / 10 N·m（手册未给额定力矩）。

## MDH 参数（改进 D-H）

坐标系约定见官网示意图。`offset` 为机械零位相对建模零位的偏差：

$$
\theta_{\mathrm{model}} = q_{\mathrm{joint}} + \theta_{\mathrm{offset}}
$$

Craig 改进 D-H 相邻齐次变换：

$$
{}^{i-1}T_i = \mathrm{Rot}_x(\alpha_{i-1})\,\mathrm{Trans}_x(a_{i-1})\,\mathrm{Rot}_z(\theta_i)\,\mathrm{Trans}_z(d_i)
$$

| \(i\) | \(a_{i-1}\) (mm) | \(\alpha_{i-1}\) (°) | \(d_i\) (mm) | \(\theta_{\mathrm{offset},i}\) (°) |
|-------|------------------|----------------------|--------------|------------------------------------|
| 1 | 0 | 0 | 240.5 | 0 |
| 2 | 0 | 90 | 0 | 90 |
| 3 | 256 | 0 | 0 | 90 |
| 4 | 0 | 90 | 210 | 0 |
| 5 | 0 | −90 | 0 | 0 |
| 6 | 0 | 90 | 144 | 0 |

连杆对应：\(d_1\) → `link_1`，\(a_2=256\) → `link_3`，\(d_4=210\) → `link_4`，\(d_6=144\) → `link_6`。MJCF 的 \(q\) 是关节角；J2、J3 的 90° offset 已写入连杆固定四元数，故 `q = 0` 为机械零位。

工作半径与零位法兰位置：

$$
R = a_2 + d_4 + d_6 = 256 + 210 + 144 = 610~\mathrm{mm}
$$

$$
\|p_{\mathrm{flange}}\|_{q=0} = d_1 + a_2 + d_4 + d_6 = 850.5~\mathrm{mm}
$$

（沿基座 \(+Z\) 伸直；610 mm 从 J2 轴线交点起算。）

## 连杆动力学（CAD 设计值）

单位：质量 kg，质心 mm，惯性 kg·mm²。\(L_i\) 写在连杆坐标系 \(\{i\}\) 下。质心在 \(\{i\}\) 中为 \(P_c=[x,y,z]^\top\)（先化为 m），质心坐标系惯量：

$$
I_c = L_i - m\left(P_c^\top P_c\, I_{3\times 3} - P_c P_c^\top\right)
$$

其中

$$
L_i=\begin{bmatrix} L_{xx} & L_{xy} & L_{xz} \\ L_{xy} & L_{yy} & L_{yz} \\ L_{xz} & L_{yz} & L_{zz} \end{bmatrix}
$$

MJCF `fullinertia` 为 \(I_c\) 的 \(I_{xx},I_{yy},I_{zz},I_{xy},I_{xz},I_{yz}\)（kg·m²），质心 `pos` 为 m。

| \(i\) | \(m\) | \(x\) | \(y\) | \(z\) | \(L_{xx}\) | \(L_{xy}\) | \(L_{xz}\) | \(L_{yy}\) | \(L_{yz}\) | \(L_{zz}\) |
|-------|-------|-------|-------|-------|------------|------------|------------|------------|------------|------------|
| 1 | 1.51 | 0.491 | 7.803 | −10.744 | 2928.466 | −32.63 | −5.816 | 2506.35 | 47.925 | 1756.017 |
| 2 | 1.653 | 183.722 | 0.103 | −1.665 | 1711.553 | −38.271 | 2314.91 | 70514.722 | 6.507 | 70036.186 |
| 3 | 0.726 | 0.029 | −90.105 | 4.039 | 7259.884 | 2.994 | −0.314 | 371.872 | 44.451 | 7228.758 |
| 4 | 0.671 | 0.007 | −9.486 | −8.041 | 794.014 | −0.821 | −0.655 | 596.235 | −34.785 | 486.228 |
| 5 | 0.647 | 0.032 | −83.769 | 2.326 | 5375.604 | 2.665 | −0.304 | 285.265 | 14.235 | 5359.769 |
| 6 | 0.107 | −0.506 | 0.255 | −10.801 | 50.918 | −3.136 | −0.699 | 47.42 | 0.388 | 60.35 |

手册无底座惯量。`base_link` 用 URDF：1.862 kg。CAD 六连杆 5.314 kg + 底座 ≈ 7.176 kg，手册自重 7.2 kg。

## 核对：MJCF / URDF / 官方文档

复现脚本：`compare_mdh_mjcf.py`。

### 运动学

用 \(\theta = q + \theta_{\mathrm{offset}}\) 的 MDH 与 MJCF 关节链对比（含零位、\(q_2=30^\circ\)、\(q_3=45^\circ\) 及手册奇异示例）。

| 项 | 结果 |
|----|------|
| \(d_1,a_2,d_4,d_6,\alpha_i\) | 与手册一致；`d6=144 mm` |
| `q=0` 相对 MDH 建模零位（不加 offset） | TCP 差约 997 mm、姿态差 180°，**不是**同一零位 |
| `q=0` 相对 MDH 机械零位（加 offset） | 法兰原点误差约 0.003 mm / 0.0005° |
| 误差来源 | URDF/MJCF 将 \(\pi/2\) 写成 `1.5708`；改为精确 \(\pi/2\) 后误差为 0 |
| `flange` + 夹爪 | MDH/URDF 只到 `link_6`；MJCF 法兰与 `link_6` 同原点同姿态，RMG24 安装面贴合腕部末端（旧四连杆 `pos="0 -0.01 0.03"` + `quat` 见 `rm65b_legacy.xml`） |
| 安装位姿 | MJCF 为 URDF 原点朝上；吊装 / 桌面由场景 `<attach>` 指定，手册 DH 不含 |

### 限位与速度

| 项 | 官方 | MJCF | URDF |
|----|------|------|------|
| 角度限位 | 上表度数 | 上表弧度（由度数精确换算） | 取整弧度（如 J6 `±6.28`，少约 0.18°） |
| 最大角速度 | J1–J2：180 °/s；J3–J6：225 °/s | 无关节速度上限 | 全部 `velocity=3.14`（≈180 °/s） |

### 动力学

`link_1`–`link_6` 的质量、质心、\(I_c\) 与上表 CAD 一致（含交叉惯量；`link_2` 的 \(I_{xz}\approx 0.00181\)）。以下不是手册连杆表的内容：

| 项 | 说明 |
|----|------|
| URDF `link_1` 质心 \(z\) | 误写 `-0.10744` m，应为 `-0.010744` m；MJCF 已按 CAD 纠正 |
| URDF `link_6` 质量 | 0.104 kg；手册 / MJCF 为 0.107 kg |
| `base_link` | 手册无；MJCF 用 URDF 对角+交叉项 |
| `damping` / `frictionloss` | 仿真粘滞/摩擦，手册无 |
| `armature` | WHJ 电机惯量折到输出：J1–2 0.551，J3 0.191，J4–6 0.072 |
| 夹爪质量 | RMG24 0.674 kg（`gripper_rmg24.xml`）；手册只到法兰。旧四连杆见 `gripper_rm65_legacy.xml` |
| `balanceinertia="true"` | 当前 \(I_c\) 已正定，编译时一般不改表值 |

## 工作空间

除去基座正上方与正下方的圆柱空间，工作范围为半径 **610 mm** 的球体。选安装位时应避开该圆柱区。截面上可操作度较好的区域呈环状。

## 奇异点

关节角格式 \([q_1,q_2,q_3,q_4,q_5,q_6]\)，单位 °，为机械关节角（与 MJCF `q` 相同）。

| 类型 | 条件 | 示例点位 |
|------|------|----------|
| 肩部奇异 | J5、J6 轴线交点落在过 J1 轴线且平行 J2 轴线的平面上 | `[90, 43.4, -105.7, 0, -30, 0]`、`[90, 43.4, -105.7, 0, 62.3, 0]` |
| 肘部奇异 | J5、J6 轴线交点在 J2–J3 轴线平面上，\(q_3=0\) → `[x, x, 0, x, x, x]` | `[-90, 60, 0, 0, 90, 0]`、`[0, 0, 0, 90, -60, 0]` |
| 腕部奇异 | J4、J6 轴线平行，\(q_5=0\) → `[x, x, x, x, 0, x]` | `[0, 60, 30, 0, 0, 0]` |
| 边界奇异 | 末端到达最远端，\(q_3=0\) 且 \(q_5=0\) → `[x, x, 0, x, 0, x]` | `[0, 0, 0, 0, 0, 0]`、`[-90, 90, 0, 0, 0, 0]`、`[-90, 45, 0, 0, 0, 0]` |

## 负载曲线

手册给出 RM65-B 末端负载曲线。\(L\) 为负载质心相对法兰平面的径向距离，\(Z\) 为法向距离。图见[官方页面](https://develop.realman-robotics.com/robot/robotParameter/RM65OntologyParameters/)。

## 外观与贴图

官方 URDF/STL 无 UV。视觉 geom 使用 `meshes/visual/*.obj`（`gen_arm_visual.py` 生成），碰撞仍用 STL。

贴图为图集：白壳 / 细黑环 / 底座蓝面板。黑环从关节原点沿旋转轴平移一小段，只画该薄片上的圆管外壁。网格点来自官方 STL（`gen_arm_visual.py` 的 `load_stl`）。J1 只在底座顶管（`link_1` 上同一条带会盖住 J2 远端端盖）。J2 只在 `link_1` 关节处。J4 画在 `link_3` / `link_4` 接口圆柱端面（`link_3` y≈−128 mm，`link_4` z≈−81 mm）。J5 画在 `link_4` 靠近关节 5 的 +Y 端圆柱（y≈+12 mm，r≈28 mm）；`link_5`、`link_6` 不涂黑。底座蓝 I/O 面板与天线凸起在同一侧（+X，略偏 −Y）。

| 文件 | 材质 | 用于 |
|------|------|------|
| `rm65_shell.png` | 白壳 + 细黑环 | `link_1`–`link_6` |
| `rm65_base.png` | 白壳 + 细黑环 + 深蓝 I/O 面板 | `base_link` |
| `rm65_joint.png` | 亚光黑 | 备用 |
| `rm65_gripper.png` | 深灰夹爪 | 夹爪 |

官网原图在 `imgs/`（[本体参数页](https://develop.realman-robotics.com/robot/robotParameter/RM65OntologyParameters/)）：

| 文件 | 内容 |
|------|------|
| `RM65.png` | MDH 坐标系与本体外观 |
| `RM_65_image1.png` | 关节尺寸 |
| `RM_65_image2.png` | 工作空间 |
| `RM_65_image3.png` | 可操作度较好区域 |
| `RM65-00.png` | 肩部奇异示意 |
| `RM_65_image4.png` / `5` | 肩部奇异点位 |
| `RM_65_image6.png` / `7` | 肘部奇异点位 |
| `RM_65_image8.png` | 同路径三维外观（页内未引用） |
| `RM_65_image9.png` | 腕部奇异点位 |
| `RM_65_image10.png`–`12` | 边界奇异点位 |
| `RM_65_image13.png` | RM65-B 负载曲线 |
| `RM_65_image14.png` | RM65-6F 负载曲线 |
| `RM_65_imageFB.png` | RM65-6FB 负载曲线 |
