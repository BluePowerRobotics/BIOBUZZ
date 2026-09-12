# Chassis 控制器目录使用指南

本目录集中管理机器人的**底盘驱动与定位**相关代码，共 5 个 Java 文件。

## 依赖关系概览

```
ChassisTesterRed / ChassisTuner / ChassisExample     （可运行的 OpMode，用于测试）
        │
        └── Chassis            （底盘控制器，封装驱动+定位，被生产 TeleOp 复用）
              │
              └── RobotPosition → MecanumDrive（Road Runner）→ DriveLocalizer

ChassisLocalization_Encoder    （独立的"编码器航位推算"定位组件，不依赖 Road Runner）
```

> 生产链路（TeleOp 实际使用）：`OpModes/TeleOp.java → Controllers.Chassis.Chassis`
> 其余 OpMode 类用于接线/运动/定位调试。

---

## 1. Chassis.java —— 底盘控制器（核心）

**功能**
- 封装 Road Runner `MecanumDrive` 与单例定位器 `RobotPosition`。
- 提供**有头模式（机器人坐标系）**与**无头模式（场地坐标系）**两种手柄驾驶方式。
- 构造时按 `isTeleOp` 选择初始位姿：遥控用 `StopPose*`，自动用 `startPose*`（取自 `HypParams`）。
- 提供停车与遥测接口。

**使用方法（程序内调用）**

| 方法 | 说明 |
|---|---|
| `new Chassis(hardwareMap, teamColor, actionRunner, telemetry, isTeleOp)` | 遥控/自动通用构造 |
| `new Chassis(hardwareMap, teamColor, actionRunner, telemetry, startPose)` | 自定义初始位姿构造 |
| `update(Kx, Ky, Komega)` | 每帧调用，参数为左摇杆 X/Y、右摇杆 X（手柄原始值） |
| `exchangeUseNoHeadMode()` / `setUseNoHeadMode(boolean)` | 切换/设定无头模式 |
| `getUseNoHeadMode()` | 查询当前模式 |
| `stop()` | 停止底盘 |
| `telemetry()` | 输出速度遥测（Vx/Vy/Omega） |

**手柄约定**
- 左摇杆上/下 = 前后（`-Ky`），左摇杆左/右 = 横移（`-Kx`）。
- 右摇杆 X = 自转（`-Komega`）。
- 有头模式：速度在机器人自身坐标系。
- 无头模式：速度在场地坐标系，"前推 = 远离本方联盟"；基准朝向见 `driverHeading`（RED `+π/2`、BLUE `-π/2`）。

**需要标定/核实**
- `HypParams.maxV / maxOmega`（最大线/角速度）。
- `HypParams.StopPose* / startPose*`（初始位姿）。
- 无头模式下 `driverHeading` 与联盟朝向是否一致（推杆方向不得镜像）。

---

## 2. ChassisTuner.java —— 底盘电机调参工具

**功能**
- 独立的四轮同步测试 OpMode：验证 4 路电机接线、编码器方向与同步性。
- 按下 A 键后 4 个电机以 `RUN_TO_POSITION` 模式同向移动固定增量（默认 2000 ticks，功率 0.6）。

**使用方法**
1. Driver Station 上选择 `ChassisTuner`（group: Tests），Init 后 Start。
2. 按手柄 A 键，电机开始同向移动；期间屏幕显示各轮当前位置/目标与 busy 状态。
3. 全部到位后自动回 `RUN_USING_ENCODER` 并停机，可再次按 A 重复。

**注意**
- 电机端口名硬编码为 `fL / fR / bL / bR`，需与 RC 配置一致。
- 用于验证"所有轮是否同向、是否同时到位"；若方向错，请改硬件方向或在测试前调整。

---

## 3. ChassisTesterRed.java —— 红队底盘整机测试

**功能**
- 完整串接 `RobotPosition` + `Chassis` 的测试 OpMode：驱动底盘并在 FTC Dashboard 上实时绘制机器人位姿。
- 仅用于红队（内部固定 `TeamColor.RED`）。

**使用方法**
1. Driver Station 选择 `ChassisTester`（group: Tests；注：注册名与类名不一致，属历史遗留）。
2. 手柄操作：
   - 左摇杆：前后/横移；右摇杆 X：自转。
   - X 键：切换无头模式。
   - A 键：将定位重置为 `HypParams.ResetPoseRed`。
3. 运行中可在 FTC Dashboard 看到机器人位置图（field overlay）。

**注意**
- 该文件在构造 `Chassis` 之后才包 `MultipleTelemetry`，导致 Dashboard 收不到 Chassis 输出的数据行（建议参考生产版 `OpModes/TeleOp.java` 先包装再构造）。
- A 键重置位姿前需确保实车朝向与 `ResetPoseRed` 一致，否则无头模式方向会整体偏移。

---

## 4. ChassisLocalization_Encoder.java —— 编码器航位推算定位组件

**功能**
- 纯四轮编码器增量定位：通过麦轮运动学正解求 (vx, vy, ω)，再积分位姿 (x, y, θ)。
- **不依赖 Road Runner / IMU**，供自建运动学方案或教学使用。

**使用方法（作为组件被调用）**

```java
ChassisLocalization_Encoder loc = new ChassisLocalization_Encoder(fl, fr, bl, br, telemetry);
loc.init();                 // 初始化：清零位姿并记录编码器初值
// 循环中反复调用（自动节流 dt>=0.01s）：
loc.Localization();
loc.getX(); loc.getY(); loc.getTheta();   // 读取位姿
loc.ChassisLocationTelemetry();           // 输出 X/Y/θ 遥测
loc.ChassisVelocityTelemetry();           // 输出四轮线速度(cm/s)遥测
loc.resetPosition();                      // 仅清零位姿（注意：不完全复位，见下）
```

**需要标定**
- `L / W`：轮距/轴距（cm）。
- `EncodertoCm`：编码器脉冲→cm 的换算系数（由轮径+减速比+编码器线数标定）。
- `D`：旋转几何系数（本文件用 `√(L²+W²)`）。

**注意**
- 需保证电机处于 `RUN_USING_ENCODER` 模式。
- `resetPosition()` 只清零 (x,y,θ)，未重置内部时间/编码器历史值；运动途中调用会产生一次累计位移，请改用 `init()` 或在静止时重置。
- 此文件的 `D` 定义与 `ChassisExample`（`D=L+W`）不一致，跨文件移植公式时航向积分会偏差约 40%，移植需重新标定。

---

## 5. ChassisExample.java —— 自建麦轮控制 + 编码器定位教学样例

**功能**
- 独立的完整 OpMode 示例（不经过 Road Runner）：手柄 → 麦轮功率逆解 → 功率归一化 → 设功率，并在同一循环内做编码器定位，实时显示位姿与轮速。
- 自带死区、编码器重置、刹/滑模式、航向归一化等工具方法，适合理解麦轮运动学原理或作为自建方案起点。

**使用方法**
1. Driver Station 选择 `MecanumDrive_EncoderLocalization`（group: FTC）。
2. Init：重置编码器、设 `RUN_USING_ENCODER`；Start 后左摇杆移动、右摇杆 X 自转。
3. 观察"Global Pose / Encoder Counts / Motor Powers"遥测验证运动与定位。
4. Stop 后自动停机并显示最终位姿。

**需要标定**
- 电机端口名：`fL / fR / bL / bR`。
- 机械参数：`WHEEL_RADIUS_CM`、`L`、`W`、`GEAR_RATIO`、`ENCODER_PPR`。
- 方向：`frontRight/backRight` 是否 `REVERSE` 需按实际接线确认。

**注意**
- 旋转功率项 `turn * D / WHEEL_RADIUS_CM` 中 `D = L + W` 是教学用经验系数，非严格几何推导，实机需调 `TURN_SPEED`。
- 该示例为教学/试验性质，与生产 `Chassis`（Road Runner）为两套独立实现，勿混用参数。

---

## 操作速查表

| OpMode 名称 | 注册类 | 用途 | 关键按键 |
|---|---|---|---|
| `ChassisTuner` | ChassisTuner | 四轮电机同步测试 | A：移动 2000 ticks |
| `ChassisTester` | ChassisTesterRed | 红队整机驾驶测试 + 位姿绘制 | X：无头切换；A：重置位姿 |
| `MecanumDrive_EncoderLocalization` | ChassisExample | 自建麦轮控制教学样例 | 摇杆驾驶 |
| （无，控制器） | Chassis | 生产底盘控制器 | — |
| （无，组件） | ChassisLocalization_Encoder | 编码器定位组件 | — |

---

## 通用建议
1. 实机前先跑 `ChassisTuner` 确认电机方向/同步。
2. 无头模式务必实机验证四个推杆方向是否镜像（涉及 `driverHeading` 与场地坐标约定）。
3. 定位类先做"直行一段 + 原地转 360°"测试以标定轮径/编码器比例与旋转系数 `D`。
