# 3D 斜坡补偿定位 — 数学推导

## 1. 问题描述

在 FTC 比赛中，场地可能存在斜坡（如上坡道、不平整的泡沫垫等）。当机器人行驶在斜坡上时：

- **Pinpoint 里程计** 测量的是沿斜坡表面行进的距离（编码器记录的轮子滚动量）
- **地面投影** 需要的是水平面上的位移（用于场地图坐标）

如果不做斜坡补偿，上坡 15° 时 Pinpoint 报告的"前进 100 cm"实际水平位移只有 `100 × cos(15°) ≈ 96.6 cm`，累积误差不可忽视。

## 2. 坐标系定义

### 2.1 机器人体坐标系 (Body Frame, B)

- **X_B**: 机器人前进方向 (沿底盘向前)
- **Y_B**: 机器人左侧方向 (沿底盘向左)
- **Z_B**: 垂直于底盘向上

当机器人位于斜坡上时，体坐标系的 Z_B 轴不再与重力方向对齐。

### 2.2 水平世界坐标系 (World Frame, W)

- **X_W**: 场地水平面 X 方向
- **Y_W**: 场地水平面 Y 方向
- **Z_W**: 垂直向上 (与重力反向)

### 2.3 场地坐标系 (Field Frame, F)

- 与 W 相同，但 X_F 轴与机器人的初始朝向对齐
- 即 W 绕 Z 轴旋转 yaw 角 (\(\psi\)) 得到 F

## 3. IMU 角度定义

`getRobotYawPitchRollAngles()` 返回 ZYX 内旋欧拉角，定义在官方 **Robot Coordinate System**（X = 右, Y = 前, Z = 上）中。与 RR 体坐标系 (§2.1) 的对应关系如下：

\[
R = R_z(\psi) \cdot R_y(\phi) \cdot R_x(\theta)
\]

| 符号 | Robot CS 轴 | RR 体轴 | 映射关系 | 正方向 (右手定则) | 物理含义 |
|------|------------|--------|----------|-------------------|----------|
| \(\psi\) | Z (上) | z (上) | \(Z \mapsto z\) | 逆时针 | 航向角 (Yaw) |
| \(\theta\) | X (右) | \(-y\) (右) | \(X \mapsto -y\) | 抬头 | 俯仰角 (Pitch) |
| \(\phi\) | Y (前) | \(x\) (前) | \(Y \mapsto x\) | 左侧抬起 | 横滚角 (Roll) |

> **坐标系映射**: Robot CS 到 RR 体坐标系 (§2.1) 的映射为：
> \[
> X_{\text{Robot}} \mapsto -y_{\text{body}}, \quad
> Y_{\text{Robot}} \mapsto x_{\text{body}}, \quad
> Z_{\text{Robot}} \mapsto z_{\text{body}}
> \]
> FTC 官方文档定义（[Universal IMU Interface](https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html)）：
> - **Pitch** = 绕 Robot **X** 轴（右）旋转，\(R_x(\theta)\)
> - **Roll** = 绕 Robot **Y** 轴（前）旋转，\(R_y(\phi)\)
>
> 因此 ZYX 内旋欧拉角为 \(R = R_z(\psi) \cdot R_y(\phi) \cdot R_x(\theta)\)，即 Roll 先于 Pitch（注意与常规命名约定的差异）。映射到体坐标系：
> - Pitch (θ) = \(R_x(\theta)\)：绕 Robot X（右）→ 体坐标系绕 \(-y\)（右）轴，物理含义为前后倾斜
> - Roll (φ) = \(R_y(\phi)\)：绕 Robot Y（前）→ 体坐标系绕 \(x\)（前）轴，物理含义为左右倾斜

> **注意**: 本定位器使用 Pinpoint 内置 IMU 的航向角 \(\psi\)（已与编码器融合），仅使用 Hub IMU 的 pitch \(\theta\) 和 roll \(\phi\)。

## 4. 旋转矩阵推导

下面的推导在 **RR 体坐标系** (§2.1) 中进行，其中 X=前, Y=左, Z=上。在此坐标系中，pitch 和 roll 具有直观的物理含义：
- **Pitch (θ)**：绕 Y 轴（左右方向）旋转 = 前后倾斜
- **Roll (φ)**：绕 X 轴（前后方向）旋转 = 左右倾斜

> **与 Robot CS 的关系**: FTC SDK 的 `getPitch()` 返回绕 Robot X（右）的角度，`getRoll()` 返回绕 Robot Y（前）的角度。通过 Hub 方向配置 (`RevHubOrientationOnRobot`)，这些值被映射到体坐标系的 pitch/roll。代码中直接使用 `getPitch()` 作为体坐标系俯仰角 θ、`getRoll()` 作为体坐标系横滚角 φ。

在体坐标系中，从体坐标系 B 到世界坐标系 W 的旋转矩阵为：

\[
R = R_z(\psi) \cdot R_y(-\theta) \cdot R_x(\phi)
\]

> **关键符号约定（与旧版 D3 推导的差异）**：俯仰角 \(\theta\) 绕体坐标系 **\(-y\)（右）** 轴旋转（对应 Robot CS 绕 X 轴），因此在体坐标系中应以 \(R_y(-\theta)\) 表示。若误写为 \(R_y(+\theta)\)，则交叉项 \(v_y^{body}\sin\theta\sin\phi\) 的符号会被翻转。实测约定 pitch 前仰为正、roll 左侧抬起为正，正确的交叉项符号为 **负号**（详见 §5.2）。

其中各基础旋转矩阵为：

\[
R_x(\phi) = \begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi & \cos\phi
\end{bmatrix}, \quad
R_y(-\theta) = \begin{bmatrix}
\cos\theta & 0 & -\sin\theta \\
0 & 1 & 0 \\
\sin\theta & 0 & \cos\theta
\end{bmatrix}, \quad
R_z(\psi) = \begin{bmatrix}
\cos\psi & -\sin\psi & 0 \\
\sin\psi & \cos\psi & 0 \\
0 & 0 & 1
\end{bmatrix}
\]

### 4.1 倾斜旋转矩阵 (不含 Yaw)

先计算倾斜部分 \(R_{tilt} = R_y(-\theta) \cdot R_x(\phi)\)：

\[
\begin{aligned}
R_{tilt} &= \begin{bmatrix}
\cos\theta & 0 & -\sin\theta \\
0 & 1 & 0 \\
\sin\theta & 0 & \cos\theta
\end{bmatrix} \cdot
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi & \cos\phi
\end{bmatrix} \\[10pt]
&= \begin{bmatrix}
\cos\theta & -\sin\theta\sin\phi & -\sin\theta\cos\phi \\
0 & \cos\phi & -\sin\phi \\
\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi
\end{bmatrix}
\end{aligned}
\]

## 5. 速度投影

### 5.0 场地坐标系 → 体坐标系 (前置步骤)

**重要**: Pinpoint 的 `getVelX()` / `getVelY()` 返回的是**场地坐标系 (Field Frame)** 速度，而非体坐标系速度。Pinpoint 内部通过编码器+IMU 融合直接输出场地坐标系下的速度分量。因此在进行斜坡补偿之前，必须先将场地坐标系速度转换到体坐标系。

使用逆旋转矩阵 \(R_z(-\psi)\)：

\[
\begin{bmatrix} v_x^{body} \\ v_y^{body} \end{bmatrix} =
\begin{bmatrix}
\cos\psi & \sin\psi \\
-\sin\psi & \cos\psi
\end{bmatrix} \cdot
\begin{bmatrix} v_x^{field} \\ v_y^{field} \end{bmatrix}
\]

\[
\boxed{
\begin{aligned}
v_x^{body} &=  v_x^{field} \cos\psi + v_y^{field} \sin\psi \\
v_y^{body} &= -v_x^{field} \sin\psi + v_y^{field} \cos\psi
\end{aligned}}
\]

其中 \(\psi\) 为 Pinpoint 输出的航向角（已与编码器融合）。

### 5.1 体坐标系速度

转换后的体坐标系速度：

\[
\mathbf{V}_B = \begin{bmatrix} v_x^{body} \\ v_y^{body} \\ 0 \end{bmatrix}
\]

其中 \(v_x^{body}\) 为机器人前进方向速度，\(v_y^{body}\) 为机器人侧向速度。Z 分量恒为 0，因为机器人贴地运动。

### 5.2 投影到水平面

将体坐标系速度通过倾斜矩阵旋转（暂不旋转 yaw）：

\[
\mathbf{V}_{tilt} = R_{tilt} \cdot \mathbf{V}_B
\]

展开计算：

\[
\begin{aligned}
\mathbf{V}_{tilt} &= \begin{bmatrix}
\cos\theta & -\sin\theta\sin\phi & -\sin\theta\cos\phi \\
0 & \cos\phi & -\sin\phi \\
\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi
\end{bmatrix} \cdot
\begin{bmatrix} v_x^{body} \\ v_y^{body} \\ 0 \end{bmatrix} \\[10pt]
&= \begin{bmatrix}
v_x^{body} \cos\theta - v_y^{body} \sin\theta \sin\phi \\
v_y^{body} \cos\phi \\
v_x^{body} \sin\theta + v_y^{body} \cos\theta \sin\phi
\end{bmatrix}
\end{aligned}
\]

取前两个分量，得到水平面上的速度（在航向对齐坐标系中）：

\[
\boxed{
\begin{aligned}
v_x^{horiz} &= v_x^{body} \cos\theta - v_y^{body} \sin\theta \sin\phi \\[4pt]
v_y^{horiz} &= v_y^{body} \cos\phi
\end{aligned}}
\]

其中 θ = pitch（俯仰角，前后倾斜），φ = roll（横滚角，左右倾斜）。

### 5.3 旋转到场地坐标系

将水平面速度绕 Z 轴旋转航向角 \(\psi\)：

\[
\begin{bmatrix} v_x^{field} \\ v_y^{field} \end{bmatrix} =
\begin{bmatrix}
\cos\psi & -\sin\psi \\
\sin\psi & \cos\psi
\end{bmatrix} \cdot
\begin{bmatrix} v_x^{horiz} \\ v_y^{horiz} \end{bmatrix}
\]

\[
\boxed{
\begin{aligned}
v_x^{field} &= v_x^{horiz} \cos\psi - v_y^{horiz} \sin\psi \\
v_y^{field} &= v_x^{horiz} \sin\psi + v_y^{horiz} \cos\psi
\end{aligned}}
\]

## 6. 特例验证

### 6.1 平地 (\(\theta = 0, \phi = 0\))

\[
v_x^{horiz} = v_x^{body} \cdot 1 + v_y^{body} \cdot 0 = v_x^{body}, \quad v_y^{horiz} = v_y^{body} \cdot 1 = v_y^{body}
\]

结果与无补偿一致，符合预期。

### 6.2 纯俯仰 — 上下坡 (\(\theta > 0, \phi = 0\))

\[
v_x^{horiz} = v_x^{body} \cos\theta, \quad v_y^{horiz} = v_y^{body}
\]

- 前进速度被 \(\cos\theta\) 缩放：上坡时 Pinpoint 测量的是斜坡上的距离，水平位移 = 斜距 × cos(θ)
- 侧向速度不受俯仰影响

### 6.3 纯横滚 — 侧向倾斜 (\(\theta = 0, \phi > 0\))

\[
v_x^{horiz} = v_x^{body}, \quad v_y^{horiz} = v_y^{body} \cos\phi
\]

- 前进速度不受横滚影响
- 侧向速度被 \(\cos\phi\) 缩放

### 6.4 同时存在俯仰和横滚

交叉耦合项 \(-v_y^{body} \sin\theta \sin\phi\) 出现在 \(v_x^{horiz}\) 中，物理含义是：

> 当机器人既有俯仰又有横滚时，侧向运动在水平面上会产生一个前向分量。

例如机器人在斜坡上斜向行驶时，需要通过此交叉项精确补偿。

## 7. 位姿积分

采用与 `PinpointLocalizer` 同构的 \(SE(2)\) 变换架构，通过偏移变换 \(T_{WP}\) 管理世界坐标系与 Pinpoint 原点坐标系的关系，但不直接使用 Pinpoint 内部位置，而是对斜坡补偿后的速度独立积分。

### 7.1 补偿后航向角

设偏移变换 \(T_{WP}\) 的航向分量为 \(\psi_{WP}\)，Pinpoint 原始航向为 \(\psi_{PR}\)，则补偿后航向为：

\[
\boxed{\psi = \psi_{WP} + \psi_{PR}}
\]

由 `setPose` 中 \(T_{WP} = \text{pose}_{\text{desired}} \circ T_{PR}^{-1}\) 展开航向分量：

\[
\psi_{WP} = \psi_{\text{desired}} - \psi_{PR}^{\text{(at setPose)}}
\]

因此在后续帧中：

\[
\psi = \psi_{\text{desired}} + \left(\psi_{PR}^{\text{(current)}} - \psi_{PR}^{\text{(at setPose)}}\right)
\]

即补偿后航向 = 设置时的期望航向 + Pinpoint 航向的增量变化，等价于跟随 Pinpoint 的航向漂移。

### 7.2 欧拉积分

使用欧拉法对校正后的速度进行积分，航向使用补偿后 \(\psi\)：

\[
\boxed{
\begin{aligned}
x(t + \Delta t) &= x(t) + v_x^{field,comp} \cdot \Delta t \\
y(t + \Delta t) &= y(t) + v_y^{field,comp} \cdot \Delta t \\
\psi(t + \Delta t) &= \psi
\end{aligned}}
\]

其中 \(v_x^{field,comp}, v_y^{field,comp}\) 为 §5.3 中经补偿后航向 \(\psi\) 旋转得到的场地坐标系速度，\(\Delta t\) 为帧间隔。

## 8. 实现注意事项

### 8.1 场地坐标系 → 体坐标系转换 (§5.0)

- Pinpoint 的 `getVelX()` / `getVelY()` 返回场地坐标系速度，其内部通过编码器+IMU 融合直接输出
- 斜坡补偿公式 (§5.2) 的输入必须是体坐标系速度，因此必须先用逆旋转 \(R_z(-\psi)\) 转换
- 该转换使用的航向角 \(\psi\) 来自 Pinpoint 的 `getHeading()`，与速度数据同步

### 8.2 为何使用 Pinpoint 的航向而非 Hub IMU 的 yaw

- Pinpoint 的航向角已经与编码器数据进行了传感器融合，与位移数据同步
- 直接使用 Hub IMU 的 yaw 可能与 Pinpoint 的位置数据存在时间对齐问题

### 8.3 为何不直接使用 Pinpoint 的 position

- Pinpoint 内部维护的位置是斜坡面上的位置，不是水平面投影
- 在不同坡度的斜面上运动时，修正因子随 pitch/roll 动态变化，不能简单地用固定因子缩放累积位置
- 通过对速度逐帧校正再积分，可以适应坡度变化

### 8.4 角度单位

- 代码中所有三角函数计算使用**弧度**
- `YawPitchRollAngles.getPitch(AngleUnit.RADIANS)` 和 `getRoll(AngleUnit.RADIANS)` 返回弧度值

### 8.5 Hub IMU 安装方向

必须正确配置 `RevHubOrientationOnRobot`，否则 pitch/roll 读数与实际机器人姿态不匹配。常见配置：

```java
// Hub 水平安装, Logo 朝上, USB 口朝前
new RevHubOrientationOnRobot(
    RevHubOrientationOnRobot.LogoFacingDirection.UP,
    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
)
```

## 9. 误差分析

对于 FTC 常见坡度（≤ 15°），不补偿时的误差：

| 坡度 | \(\cos\theta\) | 行驶 2m 后的水平误差 |
|------|---------------|---------------------|
| 5°   | 0.9962        | 0.76 cm             |
| 10°  | 0.9848        | 3.04 cm             |
| 15°  | 0.9659        | 6.82 cm             |
| 20°  | 0.9397        | 12.06 cm            |

使用本定位器补偿后，理论上可将误差降至编码器精度级别（~1 cm），前提是 IMU 方向配置正确且 pitch/roll 读数稳定。

## 10. 定位更新方法

本节以数学语言完整描述 `PinpointD3Localizer.update()` 的定位更新流程，阐明其与 `PinpointLocalizer` 同构的 \(SE(2)\) 变换架构及斜坡补偿特有的独立积分策略。

### 10.1 坐标变换定义

定义两个 \(SE(2)\) 变换：

\[
\begin{aligned}
T_{WP} &\in SE(2): \quad \text{世界坐标系 } W \rightarrow \text{Pinpoint 原点坐标系 } P \quad (\text{偏移变换}) \\[4pt]
T_{PR} &\in SE(2): \quad \text{Pinpoint 原点坐标系 } P \rightarrow \text{机器人坐标系 } R \quad (\text{斜坡面位姿})
\end{aligned}
\]

其中 \(T_{PR}\) 由 Pinpoint 传感器每帧直接输出：

\[
T_{PR} = (x_p, y_p, \psi_p) = \bigl(\text{getPosX}, \text{getPosY}, \text{getHeading}\bigr)
\]

该位姿位于**斜坡面**上，即 Pinpoint 内部融合编码器与内置 IMU 后积分的原始结果。\(T_{WP}\) 为手工维护的偏移量，用于将 Pinpoint 原点映射到场地世界坐标系。

此外，系统独立维护一个斜坡补偿后的水平面位姿：

\[
\text{pose} = (x, y, \psi) \in SE(2)
\]

### 10.2 setPose — 设置位姿

当外部调用 `setPose(pose_desired)` 时，计算偏移变换：

\[
\boxed{T_{WP} = \text{pose}_{\text{desired}} \circ T_{PR}^{-1}}
\]

同时将内部维护的斜坡补偿位姿同步：

\[
\text{pose} = \text{pose}_{\text{desired}}
\]

其中 \(T_{PR}^{-1}\) 为当前 Pinpoint 位姿的逆变换：

\[
T^{-1} = \bigl(-x \cos\psi - y \sin\psi,\; x \sin\psi - y \cos\psi,\; -\psi\bigr)
\]

**重要**: 此公式与 `PinpointLocalizer.setPose()` 完全一致，保证了两个定位器在接口层面的互换性。

### 10.3 getPose — 获取位姿

\[
\boxed{\text{getPose}() = \text{pose}}
\]

其中 `pose` 为斜坡补偿后独立积分的水平面位姿。

**与 PinpointLocalizer 的区别**: `PinpointLocalizer.getPose()` 返回 \(T_{WP} \circ T_{PR}\)（斜坡面位姿），而本定位器返回独立积分的水平面位姿。这是因为斜坡面上的 Pinpoint 位置与水平面位置存在由坡度决定的几何差异，不能简单地用固定变换描述。

### 10.4 update — 定位更新

每帧执行以下步骤，完整流程如下：

\[
\boxed{
\begin{aligned}
& \textbf{输入: } \text{Pinpoint 传感器数据}, \text{Hub IMU pitch/roll}, \text{上一帧位姿 } \text{pose}_k \\[6pt]
& \textbf{输出: } \text{更新后位姿 } \text{pose}_{k+1}, \text{体坐标系水平速度 } (v_x^{horiz}, v_y^{horiz}, \omega)
\end{aligned}}
\]

#### 步骤 1: 读取 Pinpoint 原始位姿

\[
T_{PR} \leftarrow \bigl(\text{getPosX}, \text{getPosY}, \text{getHeading}\bigr)
\]

#### 步骤 2: 计算补偿后航向角

\[
\boxed{\psi = \psi_{WP} + \psi_{PR}}
\]

其中 \(\psi_{WP} = T_{WP}.\text{heading}\) 为偏移变换的航向分量，\(\psi_{PR} = T_{PR}.\text{heading}\) 为 Pinpoint 原始航向。推导见 §7.1。

#### 步骤 3: 场地坐标系 → 体坐标系速度转换

\[
\begin{bmatrix} v_x^{body} \\ v_y^{body} \end{bmatrix} =
R_z(-\psi_{PR}) \cdot \begin{bmatrix} v_x^{field} \\ v_y^{field} \end{bmatrix}
\]

**注意**: 此步骤使用 Pinpoint 原始航向 \(\psi_{PR}\)，因为速度旋转是纯几何变换，与 `setPose` 设置的偏移无关。详见 §5.0。

#### 步骤 4: 斜坡补偿

\[
\begin{aligned}
v_x^{horiz} &= v_x^{body} \cos\theta - v_y^{body} \sin\theta \sin\phi \\
v_y^{horiz} &= v_y^{body} \cos\phi
\end{aligned}
\]

其中 \(\theta\) 为 Hub IMU 俯仰角，\(\phi\) 为横滚角。推导见 §5.2。

#### 步骤 5: 水平面体坐标系 → 场地坐标系

\[
\begin{bmatrix} v_x^{field,comp} \\ v_y^{field,comp} \end{bmatrix} =
R_z(\psi) \cdot \begin{bmatrix} v_x^{horiz} \\ v_y^{horiz} \end{bmatrix}
\]

**注意**: 此步骤使用补偿后航向 \(\psi\)，确保积分结果与 `setPose` 设置的航向一致。推导见 §5.3。

#### 步骤 6: 欧拉积分

\[
\boxed{
\begin{aligned}
x_{k+1} &= x_k + v_x^{field,comp} \cdot \Delta t \\
y_{k+1} &= y_k + v_y^{field,comp} \cdot \Delta t \\
\psi_{k+1} &= \psi
\end{aligned}}
\]

其中 \(\Delta t = t_{\text{now}} - t_{\text{last}}\) 为帧间隔。

#### 步骤 7: 返回体坐标系水平速度

\[
\text{return } \bigl(v_x^{horiz}, v_y^{horiz}, \omega_{\text{pinpoint}}\bigr)
\]

其中 \(\omega_{\text{pinpoint}} = \text{getHeadingVelocity()}\) 为 Pinpoint 输出的角速度。供给上层 EKF 的 predict 步骤使用。

### 10.5 与 PinpointLocalizer 的架构对比

| 方面 | `PinpointLocalizer` | `PinpointD3Localizer` |
|------|---------------------|------------------------|
| `getPose()` | \(T_{WP} \circ T_{PR}\) | `pose`（独立积分的水平面位姿） |
| 位置来源 | Pinpoint 内部位置（`getPosX/Y`） | 斜坡补偿速度积分 |
| 航向来源 | \(T_{WP}.\text{heading} + T_{PR}.\text{heading}\) | 同左 |
| `setPose()` | \(T_{WP} = \text{pose} \circ T_{PR}^{-1}\) | 同左 |
| 斜坡补偿 | 无 | 速度投影到水平面（§5.2） |
| 变换矩阵 | 用于 `getPose()` 定位 | 仅用于 `setPose()` 航向偏移管理 |

**核心设计决策**: 不使用 Pinpoint 内部位置而选择独立积分，原因见 §8.3。

## 11. 参考文献

- FTC SDK IMU 文档: `SensorIMUOrthogonal.java` sample
- GoBilda Pinpoint 文档: https://www.gobilda.com/pinpoint-odometry-computer-imu-sensor-fusion-for-2-wheel-odometry/
- 旋转矩阵参考: ZYX Euler Angles convention