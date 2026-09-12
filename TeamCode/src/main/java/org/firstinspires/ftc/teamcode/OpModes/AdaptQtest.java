package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

/**
 * AdaptQ 功能测试 —— 仅启动 IMU，实时显示三方向 Q 倍增因子。
 *
 * <p>模拟 {@code AdaptiveEKFLocalizer.adaptQ()} 的核心逻辑，同时展示 D2 (角加速度)
 * 和 D3 (角速度) 两种模式的 Q 输出，用于调优阈值参数。
 *
 * <p>Dashboard 可调参数:
 * <ul>
 *   <li>{@code thetaDeg} — 模拟机器人的场坐标航向 (度)，用于测试旋转投影</li>
 *   <li>{@code ANGULAR_ACCEL_THRESHOLD} — D2 角加速度阈值 (rad/s²)</li>
 *   <li>{@code JERK_THRESHOLD} — yaw 角加速度阈值 (rad/s²)</li>
 *   <li>{@code ANGULAR_VEL_THRESHOLD} — D3 角速度阈值 (rad/s)</li>
 * </ul>
 */
@Config
@TeleOp(name = "AdaptQ Test", group = "Fusion")
public class AdaptQtest extends LinearOpMode {

    // ---- Dashboard 可调参数 (与 AdaptiveEKFLocalizer 默认值一致) ----
    public static double thetaDeg = 0.0;                    // 模拟航向 (度)
    public static double ANGULAR_ACCEL_THRESHOLD = 5.0;     // rad/s² (D2 pitch/roll 角加速度)
    public static double JERK_THRESHOLD = 4.0;              // rad/s² (yaw 角加速度)
    public static double ANGULAR_VEL_THRESHOLD = 1.0;       // rad/s (D3 pitch/roll 角速度)
    public static double VEL_BOOST_MAX = 10.0;              // D3 角速度最大 boost
    public static double ACCEL_BOOST_MAX = 4.0;             // D2 角加速度最大 boost
    public static double Q_DECAY = 0.85;

    // ---- 内部状态 ----
    private double lastPitchRate = 0;
    private double lastRollRate = 0;
    private double lastYawRate = 0;
    private boolean ratesInitialized = false;

    // D2: 角加速度 → Q boost
    private double qAccelX = 1.0;
    private double qAccelY = 1.0;
    private double qAccelTheta = 1.0;

    // D3: 角速度 → Q boost
    private double qVelX = 1.0;
    private double qVelY = 1.0;
    private double qVelTheta = 1.0;

    // 原始数据缓存 (用于 telemetry 展示)
    private double pitchRate = 0, rollRate = 0, yawRate = 0;
    private double pitchAccel = 0, rollAccel = 0, yawAccel = 0;
    private double fieldAccelX = 0, fieldAccelY = 0;
    private double fieldVelX = 0, fieldVelY = 0;

    private long lastLoopTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- IMU 初始化 (与 IMUtest 相同) ----
        IMU hubImu = hardwareMap.get(IMU.class, "imu");
        hubImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));

        telemetry.addLine("AdaptQ Test Ready");
        telemetry.addLine("Dashboard 可调: thetaDeg, ANGULAR_ACCEL_THRESHOLD, JERK_THRESHOLD, ANGULAR_VEL_THRESHOLD");
        telemetry.update();

        waitForStart();
        lastLoopTime = System.nanoTime();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = (now - lastLoopTime) / 1e9;
            double safeDt = Math.max(dt, 1e-6);
            lastLoopTime = now;

            // ---- 航向 (模拟场坐标系旋转) ----
            double theta = Math.toRadians(thetaDeg);
            double cosT = Math.cos(theta);
            double sinT = Math.sin(theta);

            AngularVelocity angVel = hubImu.getRobotAngularVelocity(AngleUnit.RADIANS);
            if (angVel != null) {
                // IMU 体坐标系 (X=右, Y=前) → RR 体坐标系 (X=前, Y=左):
                //   xRotationRate (绕 Robot X/右) → pitch (绕 RR Y/左, 前后倾斜)
                //   yRotationRate (绕 Robot Y/前) → roll  (绕 RR X/前, 左右倾斜)
                pitchRate = angVel.xRotationRate;
                rollRate  = angVel.yRotationRate;
                yawRate   = angVel.zRotationRate;

                if (!ratesInitialized) {
                    // 首帧仅采样, 避免 (rate - 0) / safeDt 误触发
                    lastPitchRate = pitchRate;
                    lastRollRate  = rollRate;
                    lastYawRate   = yawRate;
                    ratesInitialized = true;
                } else {
                // ---- 角加速度 ----
                pitchAccel = (pitchRate - lastPitchRate) / safeDt;
                rollAccel  = (rollRate  - lastRollRate)  / safeDt;
                yawAccel   = Math.abs((yawRate - lastYawRate) / safeDt);

                // ---- 体坐标系 → 场坐标系旋转 (Rz(θ)) ----
                // angular velocity vector in body: [rollRate, pitchRate]^T
                fieldAccelX = rollAccel * cosT - pitchAccel * sinT;
                fieldAccelY = rollAccel * sinT + pitchAccel * cosT;

                fieldVelX = rollRate * cosT - pitchRate * sinT;
                fieldVelY = rollRate * sinT + pitchRate * cosT;

                // ========== D2: 角加速度 (冲击检测) ==========
                // 交叉映射: pitch 绕 body Y → X 方向轮子腾空 → X 不确定度 (取 fieldY)
                //           roll  绕 body X → Y 方向轮子腾空 → Y 不确定度 (取 fieldX)
                qAccelX = updateBoost(qAccelX, Math.abs(fieldAccelY), ANGULAR_ACCEL_THRESHOLD, ACCEL_BOOST_MAX);
                qAccelY = updateBoost(qAccelY, Math.abs(fieldAccelX), ANGULAR_ACCEL_THRESHOLD, ACCEL_BOOST_MAX);
                qAccelTheta = updateBoost(qAccelTheta, yawAccel, JERK_THRESHOLD, ACCEL_BOOST_MAX);

                // ========== D3: 角速度 (坡度变化检测) ==========
                // 交叉映射同 D2
                qVelX = updateBoost(qVelX, Math.abs(fieldVelY), ANGULAR_VEL_THRESHOLD, VEL_BOOST_MAX);
                qVelY = updateBoost(qVelY, Math.abs(fieldVelX), ANGULAR_VEL_THRESHOLD, VEL_BOOST_MAX);
                qVelTheta = updateBoost(qVelTheta, yawAccel, JERK_THRESHOLD, ACCEL_BOOST_MAX);

                // 更新上一帧状态
                lastPitchRate = pitchRate;
                lastRollRate  = rollRate;
                lastYawRate   = yawRate;
                }
            }

            // ---- Telemetry 输出 ----
            telemetry.addData("=== 原始数据 (RR Body) ===", "");
            telemetry.addData("pitchRate (rad/s)", "%.4f", pitchRate);
            telemetry.addData("rollRate  (rad/s)", "%.4f", rollRate);
            telemetry.addData("yawRate   (rad/s)", "%.4f", yawRate);

            telemetry.addLine();
            telemetry.addData("pitchAccel (rad/s²)", "%.4f", pitchAccel);
            telemetry.addData("rollAccel  (rad/s²)", "%.4f", rollAccel);
            telemetry.addData("yawAccel   (rad/s²)", "%.4f", yawAccel);

            telemetry.addLine();
            telemetry.addData("=== 场坐标投影 (theta=%.1f°) ===", thetaDeg);
            telemetry.addData("fieldAccelX", "%.4f", fieldAccelX);
            telemetry.addData("fieldAccelY", "%.4f", fieldAccelY);
            telemetry.addData("fieldVelX",   "%.4f", fieldVelX);
            telemetry.addData("fieldVelY",   "%.4f", fieldVelY);

            telemetry.addLine();
            telemetry.addData("=== D2: 角加速度 Q (冲击) ===", "");
            telemetry.addData("qAccelX    ", "%.4f", qAccelX);
            telemetry.addData("qAccelY    ", "%.4f", qAccelY);
            telemetry.addData("qAccelTheta", "%.4f", qAccelTheta);

            telemetry.addLine();
            telemetry.addData("=== D3: 角速度 Q (坡度) ===", "");
            telemetry.addData("qVelX      ", "%.4f", qVelX);
            telemetry.addData("qVelY      ", "%.4f", qVelY);
            telemetry.addData("qVelTheta  ", "%.4f", qVelTheta);

            telemetry.addLine();
            telemetry.addData("dt (ms)", "%.2f", dt * 1000);

            telemetry.update();
        }
    }

    /**
     * 单个方向的 Q 倍增因子更新。
     *
     * <p>当 magnitude 超过 threshold 时提升 boost (上限 maxBoost)，
     * 否则衰减 (下限 1.0)。
     *
     * @param current   当前 boost 值
     * @param magnitude 信号幅值
     * @param threshold 触发阈值
     * @param maxBoost  最大 boost 上限
     * @return 更新后的 boost 值
     */
    private double updateBoost(double current, double magnitude, double threshold, double maxBoost) {
        if (magnitude > threshold) {
            return Math.min(maxBoost, current * (1.0 + magnitude / threshold));
        } else {
            return Math.max(1.0, current * Q_DECAY);
        }
    }
}