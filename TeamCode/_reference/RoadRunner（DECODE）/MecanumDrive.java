package org.firstinspires.ftc.teamcode.RoadRunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * Mecanum 驱动实现类，用于控制采用 Mecanum 轮的机器人
 * 提供了完整的轨迹跟随、姿态控制和定位功能
 * 
 * 主要功能：
 * 1. 基于编码器和 IMU 的轮式定位
 * 2. 轨迹规划和跟随
 * 3. 机器人姿态控制
 * 4. 电机驱动和速度控制
 * 5. 实时数据记录和可视化
 */
@Config
public final class MecanumDrive {
    /**
     * Mecanum 驱动的参数配置类
     * 包含了机器人硬件和控制相关的所有参数
     */
    public static class Params {
        // IMU 安装方向参数
        // TODO: 根据实际硬件安装情况填写这些值
        //   参考 https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        // 驱动模型参数
        public double inPerTick = 0.001999;
        public double lateralInPerTick = inPerTick;
        public double trackWidthTicks = 5765.723859478465;

        // 电机前馈参数（tick 单位）
        public double kS = 1.2019792244145228;
        public double kV = 0.00035261972751626935666666666666667;
        public double kA = 0.00007;

        // 路径规划参数（英寸单位）
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // 转向规划参数（弧度单位）
        public double maxAngVel = Math.PI;
        public double maxAngAccel = Math.PI;

        // 路径控制器增益
        public double axialGain = 2;
        public double lateralGain = 3;
        public double headingGain = 2;

        public double axialVelGain = 0.5;
        public double lateralVelGain = 1;
        public double headingVelGain = 1;
    }

    /**
     * 全局参数实例，可通过 FTC Dashboard 实时调整
     */
    public static Params PARAMS = new Params();
    /**
     * Mecanum 轮运动学模型
     */
    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    /**
     * 默认转向约束
     */
    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    
    /**
     * 默认速度约束
     */
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    
    /**
     * 默认加速度约束
     */
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    /**
     * 四个驱动电机
     */
    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    /**
     * 电压传感器，用于电机电压补偿
     */
    public final VoltageSensor voltageSensor;

    /**
     * 延迟初始化的 IMU 传感器
     */
    public final LazyImu lazyImu;

    /**
     * 定位器实例
     */
    public final Localizer localizer;
    
    /**
     * 位姿历史记录，用于绘制轨迹
     */
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    /**
     * 数据记录器，用于记录估计位姿
     */
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    
    /**
     * 数据记录器，用于记录目标位姿
     */
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    
    /**
     * 数据记录器，用于记录驱动命令
     */
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    
    /**
     * 数据记录器，用于记录 Mecanum 命令
     */
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    /**
     * 基于驱动电机编码器和 IMU 的定位器实现
     */
    public class DriveLocalizer implements Localizer {
        /**
         * 四个电机的编码器
         */
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        
        /**
         * IMU 传感器
         */
        public final IMU imu;

        /**
         * 上次编码器位置
         */
        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        
        /**
         * 上次航向角
         */
        private Rotation2d lastHeading;
        
        /**
         * 初始化标志
         */
        private boolean initialized;
        
        /**
         * 当前位姿
         */
        private Pose2d pose;

        /**
         * 构造函数
         * @param pose 初始位姿
         */
        public DriveLocalizer(Pose2d pose) {
            // 初始化编码器
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));

            // 获取 IMU 实例
            imu = lazyImu.get();

            // TODO: 如果需要，反转编码器方向
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

            this.pose = pose;
        }

        /**
         * 设置当前位姿
         * @param pose 要设置的位姿
         */
        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        /**
         * 获取当前位姿
         * @return 当前位姿
         */
        @Override
        public Pose2d getPose() {
            return pose;
        }

        /**
         * 更新位姿估计
         * @return 当前速度估计
         */
        @Override
        public PoseVelocity2d update() {
            // 获取编码器位置和速度
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            // 获取 IMU 角度
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            // 记录本地化输入数据
            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            // 计算航向角
            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            // 初始化处理
            if (!initialized) {
                initialized = true;

                // 记录初始编码器位置
                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                // 记录初始航向角
                lastHeading = heading;

                // 返回零速度
                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            // 计算航向角变化
            double headingDelta = heading.minus(lastHeading);
            
            // 使用运动学模型计算机器人运动
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            // 更新上次编码器位置
            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            // 更新上次航向角
            lastHeading = heading;

            // 更新位姿
            pose = pose.plus(new Twist2d(
                    twist.line.value(),
                    headingDelta
            ));

            // 返回速度估计
            return twist.velocity().value();
        }
    }

    /**
     * 构造函数 — 使用外部 Localizer（如 EKFLocalizer / AdaptiveEKFLocalizer）。
     * @param hardwareMap 硬件映射
     * @param localizer   外部定位器实例
     */
    public MecanumDrive(HardwareMap hardwareMap, Localizer localizer) {
        // 检查 Lynx 模块固件是否最新
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        // 设置所有 Lynx 模块的缓存模式
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: 确保你的配置中有这些名称的电机（或修改它们）
        //   参考 https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "fL");
        leftBack = hardwareMap.get(DcMotorEx.class, "bL");
        rightBack = hardwareMap.get(DcMotorEx.class, "bR");
        rightFront = hardwareMap.get(DcMotorEx.class, "fR");
        // 显式设置所有电机方向，确保运动学模型与物理方向一致
        // 标准配置：左侧 FORWARD，右侧 REVERSE
        // 如果 X 前进时偏向某一侧，优先用 MecanumMotorDirectionDebugger 排查
        // 哪个电机方向反了，对应改为 REVERSE/FORWARD
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // 设置电机零功率行为为制动
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: 确保你的配置中有这个名称的 IMU（可以是 BNO 或 BHI）
        //   参考 https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        // 获取电压传感器
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 使用外部传入的 Localizer
        this.localizer = localizer;

        // 记录参数
        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    /**
     * 构造函数 — 使用内置轮式里程计定位器（向后兼容）。
     * @param hardwareMap 硬件映射
     * @param pose 初始位姿
     */
    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        // 检查 Lynx 模块固件是否最新
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        // 设置所有 Lynx 模块的缓存模式
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: 确保你的配置中有这些名称的电机（或修改它们）
        //   参考 https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "fL");
        leftBack = hardwareMap.get(DcMotorEx.class, "bL");
        rightBack = hardwareMap.get(DcMotorEx.class, "bR");
        rightFront = hardwareMap.get(DcMotorEx.class, "fR");
        // 显式设置所有电机方向，确保运动学模型与物理方向一致
        // 标准配置：左侧 FORWARD，右侧 REVERSE
        // 如果 X 前进时偏向某一侧，优先用 MecanumMotorDirectionDebugger 排查
        // 哪个电机方向反了，对应改为 REVERSE/FORWARD
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);


        // 设置电机零功率行为为制动
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: 如果需要，反转电机方向
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: 确保你的配置中有这个名称的 IMU（可以是 BNO 或 BHI）
        //   参考 https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        // 获取电压传感器
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 初始化定位器（Pinpoint 里程计；构造时对共享设备执行一次 resetPosAndIMU）
        localizer =  new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, pose);

        // 记录参数
        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    /**
     * 设置驱动功率
     * @param powers 机器人速度向量
     */
    public void setDrivePowers(PoseVelocity2d powers) {
        // 计算轮速
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        // 计算最大功率幅度，用于归一化
        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        // 设置电机功率
        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    /**
     * 轨迹跟随动作
     */
    public final class FollowTrajectoryAction implements Action {
        /**
         * 时间轨迹
         */
        public final TimeTrajectory timeTrajectory;
        
        /**
         * 开始时间戳
         */
        private double beginTs = -1;

        /**
         * 轨迹的 x 和 y 坐标点，用于绘制
         */
        private final double[] xPoints, yPoints;

        /**
         * 构造函数
         * @param t 时间轨迹
         */
        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            // 生成轨迹点，用于绘制
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

        /**
         * 执行动作
         * @param p 遥测数据包
         * @return 是否继续执行
         */
        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            // 初始化开始时间
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            // 检查是否完成轨迹
            if (t >= timeTrajectory.duration) {
                // 停止所有电机
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            // 获取目标位姿
            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            // 更新位姿估计
            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            // 计算控制命令
            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            // 计算轮速
            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            // 计算前馈控制
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            // 设置电机功率
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            // 添加遥测数据
            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            // 计算误差
            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // 绘制轨迹和机器人
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

        /**
         * 预览动作
         * @param c 画布
         */
        @Override
        public void preview(Canvas c) {
            // 绘制轨迹预览
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    /**
     * 转向动作
     */
    public final class TurnAction implements Action {
        /**
         *
         *
         * 时间转向
         */
        private final TimeTurn turn;

        /**
         * 开始时间戳
         */
        private double beginTs = -1;

        /**
         * 构造函数
         * @param turn 时间转向
         */
        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        /**
         * 执行动作
         * @param p 遥测数据包
         * @return 是否继续执行
         */
        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            // 初始化开始时间
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            // 检查是否完成转向
            if (t >= turn.duration) {
                // 停止所有电机
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            // 获取目标位姿
            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            // 更新位姿估计
            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            // 计算控制命令
            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            // 计算轮速
            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            // 设置电机功率
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            // 绘制
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

        /**
         * 预览动作
         * @param c 画布
         */
        @Override
        public void preview(Canvas c) {
            // 绘制转向中心点预览
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    /**
     * 更新位姿估计
     * @return 当前速度估计
     */
    public PoseVelocity2d updatePoseEstimate() {
        // 更新定位器
        PoseVelocity2d vel = localizer.update();
        
        // 添加到位姿历史
        poseHistory.add(localizer.getPose());
        
        // 保持历史记录大小
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        // 记录估计位姿
        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));
        
        return vel;
    }

    /**
     * 绘制位姿历史
     * @param c 画布
     */
    private void drawPoseHistory(Canvas c) {
        // 提取位姿历史点
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

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
     * @param beginPose 起始位姿
     * @return 轨迹动作构建器
     */
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}
