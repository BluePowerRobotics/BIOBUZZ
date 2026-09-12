# 自动阶段避障移动 Action 设计方案

> 目标：让自动阶段在底盘执行 RR（Road Runner v1.0）路径移动时具备**遇障暂停、清障续行**能力，
> 平移过程中车头（Heading）始终沿运动方向切线，保证车头正前方传感器返回的 boolean 恒等于"路径前方是否有障碍"；
> 姿态调整所需的原地转向不产生位移、不响应障碍。
> 本文只做设计，不涉及实际代码改动。

---

## 1. 需求分解与已确认决策

| 需求点 | 结论 |
|---|---|
| 障碍输入 | 每帧一个 `boolean`（true = 前方有障碍）。由上层状态机/传感器封装提供，Action 内部只消费，不处理传感器物理量 |
| 车头朝向 | **平移过程**中 Heading 恒等于运动方向切线角（RR 1.0 直线/样条默认 Tangent 插值器天然满足；LINE 用"转向-直线-转向"三阶段显式保证）。原地转向不产生位移、仅作姿态准备，不要求朝向=切线 |
| 遇障行为 | 平移过程中若 boolean=true：**原地停止（四轮 0 功率）**，且**冻结轨迹时间参数**；boolean 恢复 false 后从断点继续，直到原 Action 完成 |
| 旋转阶段 | **仅平移期判定**（已确认）。原地转向阶段（含 LINE 起止朝向调整）不响应障碍（转动不产生位移，车头扫过方向时"前方"无意义） |
| 停/走切换 | **立即响应**，无去抖（已确认）。boolean 上升沿立即刹停，下降沿立即续行 |
| 末端停车 | 保持现状（已确认）。`GoToStopPose` 停车不走避让，本功能只作用于新封装的动作 |

设计动作划分：

- **LINE 模式**（三阶段位姿移动，目标位姿可位于场地任意位置）：
  1. 原地转到直线方向 θ（起点→目标点的连线方向）；
  2. 沿 θ 直线前进至目标点位置（此阶段响应避障，车头=θ=运动切线）；
  3. 原地转到目标位姿的 heading。
  若起点与目标点位置几乎重合、或某段朝向本就对准，则相应阶段自动跳过。
- **SPLINE 模式**：与 RR 原生 `splineTo` 几何完全一致（默认 tangent heading），仅叠加避障。

---

## 2. 现状分析（关键技术约束）

阅读结论来自 `RoadRunner/MecanumDrive.java` 与 `Auto/AutoActions/GoToStopPose.java`：

1. **RR 轨迹时间用墙上时钟推进**：
   `FollowTrajectoryAction.run()` 用 `t = Actions.now() - beginTs` 推进轨迹参数（[MecanumDrive.java L288-296](file:///f:/github/StarterKit/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RoadRunner/MecanumDrive.java#L288-L296)），`TurnAction` 同理。
   **推论**：若避障实现成"障碍时跳过 run()"，恢复时 dt 会以整段等待时间计入 → 轨迹参数瞬间跳变、目标点远超实际位置，HolonomicController 会以越限的误差暴力追赶甚至提前判结束。**所以必须在轨迹推进处内置"冻结时间"能力**，不能靠跳过调用实现。
2. `FollowTrajectoryAction` / `TurnAction` 是 `MecanumDrive` 的**非静态内部类**，天然可访问外层 drive 实例状态 → 可在 drive 上加一个"障碍供应商"位，被 FTA 每次 run 读取，改动收敛且对现有调用零影响。
3. `TurnAction` 与 `FollowTrajectoryAction` 由 `drive.actionBuilder(...)` 工厂创建（[L481-495](file:///f:/github/StarterKit/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RoadRunner/MecanumDrive.java#L481-L495)）；自动阶段动作统一挂在 `ActionRunner` 队列里逐帧 `run()`（[ActionRunner.java](file:///f:/github/StarterKit/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utility/ActionRunner.java)）。
4. 主循环每帧调用 `RobotPosition.getInstance().update()` 刷新定位，避障冻结期间即使不在 FTA 内部更新，定位也能保持正确。

---

## 3. 总体架构

```
Auto 状态机 (AutoRed/AutoBlue)
        │  每帧传 BooleanSupplier(障碍)
        ▼
ObstacleDriveAction  (新, Auto/AutoActions, 实现 RR Action)
        │  内部相位状态机: LINE=【转直线向】→【带避障直线】→【转目标朝向】; SPLINE=【带避障样条】
        │  仅平移相把障碍供应商注册到 drive
        ▼
MecanumDrive  (小改)
   ├─ drive.obstacleSupplier (可为 null)
   └─ FollowTrajectoryAction.run(): 每帧读 supplier
         ├─ true → 时间不推进 + 四轮 0 功率 + return true(未完成)
         └─ false → t += 实测 dt(增量累加)，正常走 RR 控制器
   (TurnAction 不改 = 原地转向天然不响应障碍)
```

关键点：**时间推进由墙上时钟改为"增量累加 dt"**，且**冻结帧仍刷新 lastTs**，因此障碍解除后下一帧只累加正常控制周期，轨迹从断点平滑续行。该改动在 supplier 为 null（现有 TeleOp/停车/调参流程）时行为与原来完全等价。

---

## 4. 分文件改动

### 4.1 `RoadRunner/MecanumDrive.java`（机制层，改动最小）

1. 新增字段与访问器：
   - `private java.util.function.BooleanSupplier obstacleSupplier = null;`
   - `public void setObstacleSupplier(BooleanSupplier s)` / `public void clearObstacleSupplier()`（置 null）。
2. 只改 `FollowTrajectoryAction` 的时间推进段，`TurnAction` **不动**（保证原地转向不响应障碍）：

   ```
   run(packet):
     now = Actions.now()
     if lastTs < 0: lastTs = now            // 首帧只记录基准，t=0
     else if supplier 为空 或 supplier()==false:
         t += now - lastTs                  // 增量累加，障碍冻结期不计时
     lastTs = now                           // 冻结帧也刷新，防止解冻后 dt 跳变
     if supplier 非空 且 supplier()==true:
         // —— 避障等待分支（t 不推进）——
         if !HypParams.OBSTACLE_WAIT_HOLD:
             updatePoseEstimate()                          // 保持定位与 poseHistory 新鲜
             四轮 setPower(0)                              // 默认：0功率 + BRAKE 抱闸刹停
         else:
             // 可选闭环保持：以冻结点 P(t) 为零速目标跑反馈，主动抵抗被推偏移
             // 用 timeTrajectory.get(t).value() 构造“零速度/角速度”的常值 Pose2dDual
             //   （若库无 Pose2dDual.constant(pose,order)，则退化为按位姿误差×增益的手写反馈）
             robotVel = updatePoseEstimate()
             command = HolonomicController(PARAMS.axial/lateral/headingGain + VelGain...)
                       .compute(零速常值目标, localizer.getPose(), robotVel)
             kinematics.inverse + MotorFeedforward/voltage → 四轮 setPower（同下方正常分支写法）
         return true                                    // 未完成，继续等待
     if t >= duration: 四轮 0 功率; return false        // 原有结束逻辑
     ... 原有控制器/功率输出/遥测逻辑不变（使用 t）...
   ```
   - 语义说明：障碍为 true 的每一帧都暂停（默认刹停/可选闭环保持）并保持轨迹参数 t 不变；障碍解除后从同一参数继续，速度曲线仍由 RR profile 保证，不跳变。`lastTs` 在冻结帧同样刷新，解冻后 dt 仍为正常控制周期。
   - **反馈一致性**：本改动只替换"时间的获取与是否推进"，`run()` 后半段原有的 HolonomicController（轴向/横向/航向的位置+速度反馈）、MotorFeedforward、`updatePoseEstimate()`、Dashboard 遥测与画图**逐行保留**——即非避障移动过程的闭环反馈与 RR 自带 action 完全相同。
   - 结论：supplier 为 null 时 = 原逻辑（等价于每次 run 累加实际经过时间），TeleOp/停车/调参不受影响。

   **抗碰撞/抗扰动能力说明**：
   - 跟随移动中被撞 → 与 RR 原版一致：HolonomicController 基于位姿/速度误差闭环纠偏，能把机器人拉回轨迹。
   - 避障等待中被撞 → 默认 `OBSTACLE_WAIT_HOLD=false`：0 功率 + BRAKE 抱闸（机械锁死可承受一般推挤）；开启后则为冻结点零速闭环保持，可主动顶住推挤（代价：被障碍顶死时持续堵转，需监视电流）。
   - 定位层限制：当前 localizer 为编码器+IMU（`DriveLocalizer`），**无法感知"轮子不转、整车被推"的外部位移**，被撞后位姿估计会失真（这是定位层共性问题，与避障机制无关）。本期维持现状，如需真正抗撞建议后续换 Pinpoint/OTOS 或加 AprilTag 绝对位姿校正。

### 4.2 新增 `Auto/AutoActions/ObstacleDriveAction.java`（策略/编排层）

实现 `com.acmerobotics.roadrunner.Action`，是自动状态机直接排队使用的对象。

**构造 API：**
```java
public enum Mode { LINE, SPLINE }

// 便捷模式：LINE=「转直线向→带避障直线→转目标朝向」的任意位姿移动；SPLINE=toPose 的 heading 作为样条末端切线方向角。toPose 均可位于场地任意位置
public ObstacleDriveAction(MecanumDrive drive, Mode mode, Pose2d toPose, BooleanSupplier obstacleAhead)
// 通用包装：直接包住调用方已用 actionBuilder 造好的任意 RR 路径 Action（支持多段/自定义样条）
public ObstacleDriveAction(MecanumDrive drive, Action rrPath, BooleanSupplier obstacleAhead)
```

**行为（run(packet) 每帧被 ActionRunner 调用）：**

1. 首个 run 帧才**惰性建轨迹**：`beginPose = RobotPosition.getInstance().getPose2d()`（取当时真实位姿，避免排队期间位姿过时）。建成后缓存。
2. 内部为"相位序列"状态机，逐相位执行一个 RR 单段 Action，前一相位返回 false 才进入下一相位：

   | Mode | 相位 0 | 相位 1 | 相位 2 |
   |---|---|---|---|
   | LINE | 转直线向 `turnTo(θ)`，θ=beginPose.position→toPose.position 连线角；`|wrap(θ−begin.heading)|>TURN_TOLERANCE` 才建 | 带避障直线段到目标点位置（见下） | 转目标朝向 `turnTo(toPose.heading)`；与进入本相位时的实际 heading 相差小于 TURN_TOLERANCE 则跳过 |
   | SPLINE | 单个 RR 样条 Action（同原生 `splineTo`） | — | — |

   > LINE 仅相位 1 是平移（注册供应商→避障）；相位 0/2 是纯原地旋转（TurnAction，不注册 → 不响应障碍）。
   > 若 `distance(beginPose.position, toPose.position) < LINE_MIN_DISTANCE`，跳过相位 1，LINE 退化为直接原地转到目标 heading。

3. **仅"平移相位"调用 `drive.setObstacleSupplier(obstacleAhead)`**；调 `child.run(packet)` 之后立即 `drive.clearObstacleSupplier()`（try/finally 保证被 ActionRunner.clear() 抢占时也不残留，避免影响后续 GoToStopPose 停车）。原地转向相位不注册供应商 → 转向不响应障碍。
4. 全部相位返回 false → 本 Action 结束返回 false。

**LINE 相位 1 直线段几何生成（车头=运动切线 θ）：**
```java
Pose2d lineStart = 位姿(位置=beginPose.position, heading=θ)   // 相位 0 原地转不改变位置
drive.actionBuilder(lineStart)
     .setTangent(θ)                // 强制直线段起点切向 = θ
     .splineTo(toPose.position, θ) // 终点切向也 = θ → 五次样条退化为 A→B 的直线
     .build();
```
- 说明：两端切向都取 θ 时，五次样条退化为 A→B 的直线；RR 默认 tangent heading ⇒ 平移全程车头=θ（运动切线），正前方传感器对准路径前方。
- `setTangent(θ)` 使直线段不依赖 builder 的切向连续性推断，保证严格直线（Dashboard 预览复核）。
- 相位 2（到达后转目标朝向）：平移结束时车头为 θ，用 `turnTo(toPose.heading)` 原地转到目标 heading；该段不注册供应商，不响应障碍。
- 目标位姿（位置+heading）可任意：即使 θ 需转 ≥90° 甚至接近 180° 也支持——那属于相位 0 的原地旋转，不做避障判定。

**SPLINE 几何**：直接等价 RR 原生样条
```java
drive.actionBuilder(beginPose).splineTo(toPose.position, toPose.heading.toDouble()).build();
```
（`toPose.heading` 作为样条末端切线角，默认 tangent heading ⇒ 全程车头=切线。）

**preview(packet)**：转发给当前缓存的内层 Action 的 preview，方便 Dashboard 显示路径。

**可选遥测**：在 `packet.put("obstacleBlocked", ...)` 输出当前是否处于避障等待，便于现场观测。

### 4.3 `Parameter/HypParams.java`（新增 @Config 可调项）

- `public static double LINE_TURN_TOLERANCE_RAD = Math.toRadians(2);` —— 原地转阶段跳过冗余旋转的阈值。
- `public static double LINE_MIN_DISTANCE_IN = 0.5;` —— 直线目标过近直接完成。
- `public static boolean OBSTACLE_WAIT_HOLD = false;` —— 避障等待期间保持方式：false=0 功率+抱闸刹停；true=以冻结点为零速目标的闭环保持（主动抗被推，注意堵转电流）。
- （已确认无需去抖参数，不做障碍迟滞。）

### 4.4 可选：新增验证用 OpMode `Auto/AutoOpModes/ObstacleDriveTester.java`

`@TeleOp(name="ObstacleDriveTester", group="Tests")`：手柄 A = 向前 24in LINE、B = 一次小曲率 SPLINE、X 按住 = 模拟障碍(true)，松开即清障；配合 Dashboard 场地图观察"停/续行"与路径是否为直线。仅用于验收，可赛后删除。

---

## 5. 自动状态机使用示例（示意，写入 AutoRed/AutoBlue 的 START 分支时用）

```java
// 障碍源：真实实现是传感器 → boolean（阈值、视角处理在传感器侧完成）
BooleanSupplier blocked = () -> frontObstacleSensor.isBlocked();

// 例：LINE 移动到任意位姿(0,30,-π/2)：先转直线向→直线前进(中途遇障即停)→再原地转到最终朝向 -π/2
actionRunner.add(new ObstacleDriveAction(drive,
        ObstacleDriveAction.Mode.LINE,
        new Pose2d(0, 30, -Math.PI / 2),
        blocked));

// 例：走 RR 样条（目标位姿任意）
actionRunner.add(new ObstacleDriveAction(drive,
        ObstacleDriveAction.Mode.SPLINE,
        new Pose2d(30, 30, Math.PI / 2),
        blocked));
```

状态机既有的"最后 3s 强制 clear + GoToStopPose 停车"逻辑保持不变；被 clear 时新 Action 会通过 try/finally 清掉障碍供应商，停车不会受避让影响（已确认）。

---

## 6. 边界与注意点

1. **原地转向不避障的语义**：LINE 相位 0/2 均为 `TurnAction`（纯原地旋转），完全不读供应商。样条/直线内部若 RR 自行并入小角旋转（跟随切线），属于平移相，会一并冻结——符合"前方沿切线"的语义。
2. **停在半路 vs 靠近终点**：障碍解除即从冻结参数 t 续行，profile 保证无速度跳变；若等待期间对手车已离开但占用了原轨迹附近，HolonomicController 会自动纠偏（可能伴随少量横移，仍属平移相避障范围）。
3. **供应商生命周期**：只在本 Action 的平移相位 `run()` 调用期间注册（try/finally 清空），`ActionRunner.clear()` 抢占后不残留；GoToStopPose 等后续动作完全不受影响。
4. **`Actions.now()` 语义**：以秒为单位的单调时钟；新时间推进不改变其单调性，仅在障碍为 true 时停止累加。
5. **多个 drive Action 同存**：沿用 RR 约定——同一时刻只执行一个 drive 路径 Action（ActionRunner 串行保证）。
6. **传感器视角假设**：传感器装在车头，采样范围须大致覆盖路径正前方；其阈值/距离/视角由传感器侧处理，不在本 Action 内。
7. **LINE 支持任意目标位姿**：目标位置与 heading 均任意；起止朝向由两段原地旋转负责，直线段方向自动取起止点连线。仅当起止位置几乎重合时退化为"直接原地转到目标 heading"。
8. **抗撞依赖定位层**：编码器+IMU 定位测不到"整车被推（轮不转）"的外部位移；移动中/续行后的纠偏以位姿估计为前提。真正抗撞需绝对位姿反馈（Pinpoint/OTOS/AprilTag），本期不做，仅文档注明。

---

## 7. 假设与决策清单

- [x] 障碍检测只在平移期生效；原地转向阶段忽略障碍（用户确认）。
- [x] 障碍 boolean 立即响应，不做去抖/迟滞（用户确认）。
- [x] 末端停车 GoToStopPose 保持现状，不纳入避让（用户确认）。
- [x] 避障 = "原地停 + 冻结轨迹时间参数 + 清障后断点续行"，不改轨迹几何。
- [x] 平移中车头=运动切线：LINE 用「转直线向→直线→转目标朝向」三阶段、SPLINE 用 RR 默认 tangent heading；原地转向仅为姿态准备、不响应障碍。
- [x] 避障等待保持方式做成可配：默认 0 功率+抱闸刹停，可选"冻结点零速闭环保持"（用户确认，`OBSTACLE_WAIT_HOLD` 开关）。
- [x] 反馈一致性：改动仅限时间来源/推进，HolonomicController + Feedforward + updatePoseEstimate 全部保留 → 移动过程与 RR 自带 action 反馈等价。
- [x] 定位层本期不动（编码器+IMU），碰撞导致的"整车位移不可观测"限制仅在文档注明（用户确认）。
- [x] 暂停机制放在 drive 层（`obstacleSupplier` + FTA 增量计时），不改 ActionRunner / AutoRed / AutoBlue 主循环结构。
- [x] RR 原生 `FollowTrajectoryAction` 的时间改为增量累加后，supplier 为空时与原行为等价，风险可控。

---

## 8. 验证步骤

1. **编译**：Android Studio 同步 / `gradlew :TeamCode:assembleDebug`，确认新增类无编译错误。
2. **直线度验证**：跑 ObstacleDriveTester 的 LINE，用 FTC Dashboard 场地图 overlay 确认路径预览为直线、机器人位姿轨迹贴合。
3. **停/续验证**：LINE 中途按住 X（模拟障碍）→ 机器人在数帧内刹停且不抖动；持续按住 >2s 后松开 → 从断点平滑续行至终点，期间 Dashboard 观察 xError 不发散、无明显超调。
4. **冻结正确性验证**：按住 X 达 5s 再松开，机器人应仍停在障碍点附近且**不会跳向轨迹终点**（验证时间冻结而非跳过）。
5. **SPLINE 对照**：同一段样条分别用普通 RR 动作与本 Action（无障碍）跑，确认轨迹完全一致（"同 RR spline"）。
6. **转向不避障验证**：LINE 的相位 0（转直线向）与相位 2（转目标朝向）期间按住 X，机器人应照常完成原地旋转；进入相位 1 直线段后若 X 仍按住应立即刹停、松开续行。
7. **任意位姿验证**：给 LINE 一个非沿直线方向的最终 heading（如反向），确认机器人到达目标位置后原地转到该 heading；末端旋转阶段按住 X 不影响转向完成。
8. **反馈回归**：无障碍跑本 Action 与跑 RR 自带同路径 action，Dashboard 上 xError/yError/headingError 曲线应一致（验证反馈功能未被削弱）。
9. **抗扰/等待保持验证**：默认刹停——等待期间轻推机器人能稳住、大力推偏后在续行时被闭环拉回轨迹；将 `OBSTACLE_WAIT_HOLD` 置 true 后，等待期间主动顶住推挤（注意推挤过大时的电流/堵转）。
10. **回归**：现有 AutoBlue/AutoRed（停车逻辑）、TeleOp 底盘手控不受影响。
