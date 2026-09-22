package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.TwoDeadWheelInputsMessage;

/**
 * 两轮全向定位器实现
 * 使用两个独立的编码轮（一个平行，一个垂直）和 IMU 来估计机器人的位姿
 * 适用于需要精确定位的机器人系统
 */
@Config
public final class TwoDeadWheelLocalizer implements Localizer {
    /**
     * 两轮定位器参数类
     * 包含编码器安装位置的配置参数
     */
    public static class Params {
        /**
         * 平行编码器的 y 坐标（以 tick 为单位）
         */
        public double parYTicks = 0.0;
        
        /**
         * 垂直编码器的 x 坐标（以 tick 为单位）
         */
        public double perpXTicks = 0.0;
    }

    /**
     * 全局参数实例，可通过 FTC Dashboard 进行调整
     */
    public static Params PARAMS = new Params();

    /**
     * 平行编码器
     */
    public final Encoder par;
    
    /**
     * 垂直编码器
     */
    public final Encoder perp;
    
    /**
     * IMU 传感器，用于获取机器人朝向
     */
    public final IMU imu;

    /**
     * 上一次平行编码器的位置
     */
    private int lastParPos;
    
    /**
     * 上一次垂直编码器的位置
     */
    private int lastPerpPos;
    
    /**
     * 上一次机器人朝向
     */
    private Rotation2d lastHeading;

    /**
     * 每编码器 tick 对应的英寸数
     */
    private final double inPerTick;

    /**
     * 上一次原始朝向速度
     */
    private double lastRawHeadingVel;
    
    /**
     * 朝向速度偏移量，用于处理角度环绕问题
     */
    private double headingVelOffset;
    
    /**
     * 初始化标志
     */
    private boolean initialized;
    
    /**
     * 当前机器人位姿
     */
    private Pose2d pose;

    /**
     * 构造函数
     * 
     * @param hardwareMap 硬件映射对象，用于获取电机
     * @param imu IMU 传感器，用于获取机器人朝向
     * @param inPerTick 每编码器 tick 对应的英寸数
     * @param initialPose 初始位姿
     */
    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick, Pose2d initialPose) {
        // 初始化编码器
        // TODO: 确保配置文件中有这些名称的**电机**（或修改名称）
        //   编码器应该插入与命名电机匹配的插槽
        //   参考文档：https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

        // TODO: 如果需要，反转编码器方向
        //   par.setDirection(DcMotorSimple.Direction.REVERSE);

        this.imu = imu;

        this.inPerTick = inPerTick;

        // 记录参数
        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);

        pose = initialPose;
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
        // 获取编码器读数
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        // 获取 IMU 读数
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        // 使用角度单位来解决 https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070 问题
        AngularVelocity angularVelocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        AngularVelocity angularVelocity = new AngularVelocity(
                UnnormalizedAngleUnit.RADIANS,
                (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
                angularVelocityDegrees.acquisitionTime
        );

        // 记录传感器输入数据
        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        // 计算当前朝向
        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        // 处理角度环绕问题，参考 https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        // 首帧 (未初始化) 仅记录上一帧角速度, 避免 |rawHeadingVel - 0| > π 时误加 ±2π 偏移
        double rawHeadingVel = angularVelocity.zRotationRate;
        if (initialized && Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        // 初始化处理
        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        // 计算编码器位置变化和朝向变化
        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        // 计算机器人运动增量
        // 补偿编码器安装位置引起的测量误差
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        // x 方向位移和速度（补偿旋转影响）
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        // y 方向位移和速度（补偿旋转影响）
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                // 旋转角度和角速度
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        // 更新上一次的编码器位置和朝向
        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        // 更新位姿
        pose = pose.plus(twist.value());
        // 返回速度估计
        return twist.velocity().value();
    }
}
