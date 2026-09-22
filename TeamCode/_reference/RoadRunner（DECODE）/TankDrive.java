package org.firstinspires.ftc.teamcode.RoadRunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.messages.TankCommandMessage;
import org.firstinspires.ftc.teamcode.messages.TankLocalizerInputsMessage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

/**
 * Tank 驱动实现类，用于控制采用坦克式驱动的机器人
 * 提供了完整的轨迹跟随、姿态控制和定位功能
 * 
 * 主要功能：
 * 1. 基于编码器的轮式定位
 * 2. 轨迹规划和跟随（使用 Ramsete 控制器）
 * 3. 机器人姿态控制
 * 4. 电机驱动和速度控制
 * 5. 实时数据记录和可视化
 */
@Config
public final class TankDrive {
    /**
     * 坦克驱动参数类，包含所有驱动相关的配置参数
     * 这些参数需要根据实际机器人硬件进行调整
     */
    public static class Params {
        /**
         * IMU 方向配置
         * 参考文档：https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
         */
        // IMU 标志朝向
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        // IMU USB 接口朝向
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        /**
         * 驱动模型参数
         */
        // 每编码器 tick 对应的英寸数
        public double inPerTick = 0;
        // 轮距（以编码器 tick 为单位）
        public double trackWidthTicks = 0;

        /**
         * 前馈控制参数（以 tick 为单位）
         */
        // 静态摩擦力补偿
        public double kS = 0;
        // 速度增益
        public double kV = 0;
        // 加速度增益
        public double kA = 0;

        /**
         * 路径规划参数（以英寸为单位）
         */
        // 最大轮速
        public double maxWheelVel = 50;
        // 最小加速度
        public double minProfileAccel = -30;
        // 最大加速度
        public double maxProfileAccel = 50;

        /**
         * 转向规划参数（以弧度为单位）
         */
        // 最大角速度（与路径规划共享）
        public double maxAngVel = Math.PI;
        // 最大角加速度
        public double maxAngAccel = Math.PI;

        /**
         * 路径控制器参数
         */
        // Ramsete 控制器参数，范围 (0, 1)
        public double ramseteZeta = 0.7;
        // Ramsete 控制器参数，正数
        public double ramseteBBar = 2.0;

        /**
         * 转向控制器参数
         */
        // 转向位置增益
        public double turnGain = 0.0;
        // 转向速度增益
        public double turnVelGain = 0.0;
    }

    /**
     * 全局参数实例，可通过 FTC Dashboard 进行调整
     */
    public static Params PARAMS = new Params();

    /**
     * 坦克驱动运动学模型，用于计算轮速和机器人运动
     */
    public final TankKinematics kinematics = new TankKinematics(PARAMS.inPerTick * PARAMS.trackWidthTicks);

    /**
     * 默认转向约束，包含最大角速度和角加速度
     */
    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    
    /**
     * 默认速度约束，包含轮速和角速度限制
     */
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    
    /**
     * 默认加速度约束，包含最小和最大加速度
     */
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    /**
     * 左侧电机列表
     */
    public final List<DcMotorEx> leftMotors;
    
    /**
     * 右侧电机列表
     */
    public final List<DcMotorEx> rightMotors;

    /**
     * 延迟初始化的 IMU 实例，用于获取机器人姿态
     */
    public final LazyImu lazyImu;

    /**
     * 电压传感器，用于获取电池电压以进行电压补偿
     */
    public final VoltageSensor voltageSensor;

    /**
     * 定位器实例，用于估计机器人的位姿
     */
    public final Localizer localizer;
    
    /**
     * 位姿历史记录，用于绘制机器人运动轨迹
     */
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    /**
     * 估计位姿记录器，用于 FTC Dashboard 可视化
     */
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    
    /**
     * 目标位姿记录器，用于 FTC Dashboard 可视化
     */
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    
    /**
     * 驱动命令记录器，用于 FTC Dashboard 可视化
     */
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);

    /**
     * 坦克驱动命令记录器，用于 FTC Dashboard 可视化
     */
    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);

    /**
     * 基于轮式编码器的定位器实现
     * 使用左右轮的编码器读数来估计机器人的位姿
     */
    public class DriveLocalizer implements Localizer {
        /**
         * 左侧编码器列表
         */
        public final List<Encoder> leftEncs;
        
        /**
         * 右侧编码器列表
         */
        public final List<Encoder> rightEncs;
        
        /**
         * 当前机器人位姿
         */
        private Pose2d pose;

        /**
         * 上一次左侧编码器位置
         */
        private double lastLeftPos;
        
        /**
         * 上一次右侧编码器位置
         */
        private double lastRightPos;
        
        /**
         * 初始化标志
         */
        private boolean initialized;

        /**
         * 构造函数
         * 
         * @param pose 初始位姿
         */
        public DriveLocalizer(Pose2d pose) {
            // 初始化左侧编码器
            {
                List<Encoder> leftEncs = new ArrayList<>();
                for (DcMotorEx m : leftMotors) {
                    Encoder e = new OverflowEncoder(new RawEncoder(m));
                    leftEncs.add(e);
                }
                this.leftEncs = Collections.unmodifiableList(leftEncs);
            }

            // 初始化右侧编码器
            {
                List<Encoder> rightEncs = new ArrayList<>();
                for (DcMotorEx m : rightMotors) {
                    Encoder e = new OverflowEncoder(new RawEncoder(m));
                    rightEncs.add(e);
                }
                this.rightEncs = Collections.unmodifiableList(rightEncs);
            }

            // TODO: 如果需要，反转编码器方向
            //   leftEncs.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

            this.pose = pose;
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public PoseVelocity2d update() {
            // 收集左侧编码器读数
            List<PositionVelocityPair> leftReadings = new ArrayList<>(), rightReadings = new ArrayList<>();
            double meanLeftPos = 0.0, meanLeftVel = 0.0;
            for (Encoder e : leftEncs) {
                PositionVelocityPair p = e.getPositionAndVelocity();
                meanLeftPos += p.position;
                meanLeftVel += p.velocity;
                leftReadings.add(p);
            }
            meanLeftPos /= leftEncs.size();
            meanLeftVel /= leftEncs.size();

            // 收集右侧编码器读数
            double meanRightPos = 0.0, meanRightVel = 0.0;
            for (Encoder e : rightEncs) {
                PositionVelocityPair p = e.getPositionAndVelocity();
                meanRightPos += p.position;
                meanRightVel += p.velocity;
                rightReadings.add(p);
            }
            meanRightPos /= rightEncs.size();
            meanRightVel /= rightEncs.size();

            // 记录编码器输入数据
            FlightRecorder.write("TANK_LOCALIZER_INPUTS",
                     new TankLocalizerInputsMessage(leftReadings, rightReadings));

            // 初始化处理
            if (!initialized) {
                initialized = true;

                lastLeftPos = meanLeftPos;
                lastRightPos = meanRightPos;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            // 计算轮式增量并转换为机器人运动
            Twist2dDual<Time> twist = kinematics.forward(new TankKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            meanLeftPos - lastLeftPos,
                            meanLeftVel
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            meanRightPos - lastRightPos,
                            meanRightVel,
                    }).times(PARAMS.inPerTick)
            ));

            // 更新上一次编码器位置
            lastLeftPos = meanLeftPos;
            lastRightPos = meanRightPos;

            // 更新位姿
            pose = pose.plus(twist.value());

            // 返回速度估计
            return twist.velocity().value();
        }
    }

    /**
     * 构造函数，初始化坦克驱动系统
     * 
     * @param hardwareMap 硬件映射对象，用于获取电机和传感器
     * @param pose 初始位姿
     */
    public TankDrive(HardwareMap hardwareMap, Pose2d pose) {
        // 检查 Lynx 模块固件是否过时
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        // 设置所有 Lynx 模块的批量缓存模式为自动
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // 初始化左侧和右侧电机
        // TODO: 确保配置文件中有这些名称的电机（或修改名称）
        //   如果有多个电机，可以添加到列表中
        //   参考文档：https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "left"));
        rightMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "right"));

        // 设置电机的零功率行为为制动
        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: 如果需要，反转电机方向
        //   leftMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        // 初始化 IMU
        // TODO: 确保配置文件中有这个名称的 IMU（可以是 BNO 或 BHI）
        //   参考文档：https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        // 获取电压传感器
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 初始化定位器
        localizer = new DriveLocalizer(pose);

        // 记录参数
        FlightRecorder.write("TANK_PARAMS", PARAMS);
    }

    /**
     * 设置驱动功率
     * 将机器人速度指令转换为左右轮的功率
     * 
     * @param powers 机器人速度指令，包含线速度和角速度
     */
    public void setDrivePowers(PoseVelocity2d powers) {
        // 将机器人速度指令转换为轮速
        TankKinematics.WheelVelocities<Time> wheelVels = new TankKinematics(2).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        // 计算最大功率幅值，用于归一化
        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        // 设置左侧电机功率
        for (DcMotorEx m : leftMotors) {
            m.setPower(wheelVels.left.get(0) / maxPowerMag);
        }
        // 设置右侧电机功率
        for (DcMotorEx m : rightMotors) {
            m.setPower(wheelVels.right.get(0) / maxPowerMag);
        }
    }

    /**
     * 轨迹跟随动作类，用于控制机器人跟随预设轨迹
     * 使用 Ramsete 控制器来计算控制命令
     */
    public final class FollowTrajectoryAction implements Action {
        /**
         * 时间轨迹对象
         */
        public final TimeTrajectory timeTrajectory;
        
        /**
         * 动作开始时间戳
         */
        private double beginTs = -1;

        /**
         * 轨迹的 x 坐标点，用于可视化
         */
        private final double[] xPoints;
        
        /**
         * 轨迹的 y 坐标点，用于可视化
         */
        private final double[] yPoints;

        /**
         * 构造函数
         * 
         * @param t 要跟随的时间轨迹
         */
        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            // 生成轨迹的点，用于可视化
            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            // 计算当前时间
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            // 检查轨迹是否完成
            if (t >= timeTrajectory.duration) {
                // 停止所有电机
                for (DcMotorEx m : leftMotors) {
                    m.setPower(0);
                }
                for (DcMotorEx m : rightMotors) {
                    m.setPower(0);
                }

                return false;
            }

            // 获取当前路径参数
            DualNum<Time> x = timeTrajectory.profile.get(t);

            // 计算目标位姿
            Pose2dDual<Arclength> txWorldTarget = timeTrajectory.path.get(x.value(), 3);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            // 更新位姿估计
            updatePoseEstimate();

            // 使用 Ramsete 控制器计算控制命令
            PoseVelocity2dDual<Time> command = new RamseteController(kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar)
                    .compute(x, txWorldTarget, localizer.getPose());
            driveCommandWriter.write(new DriveCommandMessage(command));

            // 计算轮速并应用前馈控制
            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            // 设置电机功率
            for (DcMotorEx m : leftMotors) {
                m.setPower(leftPower);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(rightPower);
            }

            // 添加遥测数据
            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            // 计算并添加误差数据
            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // 可视化轨迹和机器人
            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            // 绘制目标机器人
            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            // 绘制实际机器人
            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            // 绘制轨迹
            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            // 预览轨迹
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    /**
     * 转向动作类，用于控制机器人执行指定的转向动作
     */
    public final class TurnAction implements Action {
        /**
         * 时间转向对象，包含转向的轨迹信息
         */
        private final TimeTurn turn;

        /**
         * 动作开始时间戳
         */
        private double beginTs = -1;

        /**
         * 构造函数
         * 
         * @param turn 时间转向对象
         */
        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            // 计算当前时间
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            // 检查转向是否完成
            if (t >= turn.duration) {
                // 停止所有电机
                for (DcMotorEx m : leftMotors) {
                    m.setPower(0);
                }
                for (DcMotorEx m : rightMotors) {
                    m.setPower(0);
                }

                return false;
            }

            // 计算目标位姿
            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            // 更新位姿估计
            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            // 计算控制命令，包含位置和速度反馈
            PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<>(
                    Vector2dDual.constant(new Vector2d(0, 0), 3),
                    txWorldTarget.heading.velocity().plus(
                            PARAMS.turnGain * localizer.getPose().heading.minus(txWorldTarget.heading.value()) +
                            PARAMS.turnVelGain * (robotVelRobot.angVel - txWorldTarget.heading.velocity().value())
                    )
            );
            driveCommandWriter.write(new DriveCommandMessage(command));

            // 计算轮速并应用前馈控制
            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            // 设置电机功率
            for (DcMotorEx m : leftMotors) {
                m.setPower(leftPower);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(rightPower);
            }

            // 可视化转向过程
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            // 绘制目标机器人
            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            // 绘制实际机器人
            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            // 绘制转向中心点
            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            // 预览转向中心点
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    /**
     * 更新位姿估计
     * 调用定位器的 update() 方法，并记录位姿历史
     * 
     * @return 当前速度估计
     */
    public PoseVelocity2d updatePoseEstimate() {
        // 调用定位器更新位姿
        PoseVelocity2d vel = localizer.update();
        // 添加到位姿历史
        poseHistory.add(localizer.getPose());

        // 限制位姿历史长度
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        // 记录估计位姿
        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));

        return vel;
    }

    /**
     * 绘制位姿历史
     * 在 Canvas 上绘制机器人的运动轨迹
     * 
     * @param c Canvas 对象，用于绘制图形
     */
    private void drawPoseHistory(Canvas c) {
        // 准备轨迹点数组
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        // 填充轨迹点
        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        // 绘制轨迹
        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    /**
     * 创建轨迹动作构建器
     * 用于构建复杂的轨迹动作序列
     * 
     * @param beginPose 起始位姿
     * @return 轨迹动作构建器
     */
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,  // 转向动作工厂
                FollowTrajectoryAction::new,  // 轨迹跟随动作工厂
                new TrajectoryBuilderParams(
                        1e-6,  // 路径点精度
                        new ProfileParams(
                                0.25, 0.1, 1e-2  // 路径规划参数
                        )
                ),
                beginPose, 0.0,  // 起始位姿和时间偏移
                defaultTurnConstraints,  // 默认转向约束
                defaultVelConstraint, defaultAccelConstraint  // 默认速度和加速度约束
        );
    }
}
