package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.AdaptiveEKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.AdaptiveUKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.EKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.UKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * 多定位器对比测试 OpMode。
 *
 * <p>使用手柄操控机器人，在 FTC Dashboard 上同时绘制
 * {@link PinpointLocalizer}、{@link MT1Localizer}、{@link EKFLocalizer}、
 * {@link AdaptiveEKFLocalizer} 的轨迹和位姿。
 *
 * <h3>误差测量</h3>
 * <ol>
 *   <li>在 FTC Dashboard 中设置 {@code testX / testY / testHeading} 为目标位姿</li>
 *   <li>驾驶机器人到达 TestPose</li>
 *   <li>按下手柄 A 键 → 记录所有定位器的位姿并计算误差</li>
 *   <li>误差以 (Δx, Δy, Δθ°) 形式显示在 telemetry 中</li>
 * </ol>
 *
 * <p>颜色对应:
 * <ul>
 *   <li>绿色 — PinpointLocalizer</li>
 *   <li>橙色 — MT1Localizer (视觉)</li>
 *   <li>粉色 — EKFLocalizer (固定 Q/R)</li>
 *   <li>紫色 — AdaptiveEKFLocalizer</li>
 *   <li>青色 — UKFLocalizer (固定 Q/R)</li>
 *   <li>蓝色 — AdaptiveUKFLocalizer</li>
 *   <li>红色虚线 — TestPose (目标点)</li>
 * </ul>
 *
 * <p>注意: MT1 的轨迹、当前位姿与误差只在 HIVE 倾角解算成功
 * ({@code isValid() && isHiveEstimated()}) 时才记录/绘制 —— 解算失败时
 * {@code MT1Localizer#getPose()} 会回退为未经修正的受污染位姿。
 */
@Config
@TeleOp(name = "Fusion Test", group = "Fusion")
public class FusionTestOpMode extends LinearOpMode {

    private static final double IN_PER_TICK = 0.001999;

    // ---- TestPose (可在 Dashboard 动态调整) ----
    public static double testX = 61.57;
    public static double testY = 59.27;
    public static double testHeading = Math.PI/2;  // 弧度

    // ---- 误差记录 ----
    private boolean prevAPressed = false;
    private boolean errorRecorded = false;

    private double errPinpointX, errPinpointY, errPinpointTheta;
    private double errMt1X,       errMt1Y,       errMt1Theta;
    private double errEkfX,       errEkfY,       errEkfTheta;
    private double errAdaptiveX,  errAdaptiveY,  errAdaptiveTheta;
    private double errUkfX,       errUkfY,       errUkfTheta;
    private double errAdaptiveUkfX, errAdaptiveUkfY, errAdaptiveUkfTheta;

    // ---- 轨迹历史 (用于绘制) ----
    private final List<Pose2d> pinpointHistory    = new LinkedList<>();
    private final List<Pose2d> mt1History         = new LinkedList<>();
    private final List<Pose2d> ekfHistory         = new LinkedList<>();
    private final List<Pose2d> adaptiveHistory    = new LinkedList<>();
    private final List<Pose2d> ukfHistory         = new LinkedList<>();
    private final List<Pose2d> adaptiveUkfHistory = new LinkedList<>();
    private static final int MAX_HISTORY = 80;

    /** 开始后校准阶段的记录时长 (毫秒) */
    public static int CALIB_DURATION_MS = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- 初始化 Limelight ----
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        Pose2d initialPose = new Pose2d(0, 0, 0);

        // ---- 创建所有定位器 ----
        PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, IN_PER_TICK, initialPose);
        MT1Localizer mt1 = new MT1Localizer(limelight);
        EKFLocalizer ekf = new EKFLocalizer(hardwareMap, limelight, initialPose);
        AdaptiveEKFLocalizer adaptiveEkf = new AdaptiveEKFLocalizer(hardwareMap, limelight, "imu", initialPose, false);
        UKFLocalizer ukf = new UKFLocalizer(hardwareMap, limelight, initialPose);
        AdaptiveUKFLocalizer adaptiveUkf = new AdaptiveUKFLocalizer(hardwareMap, limelight, "imu", initialPose, false);

        // ---- 创建驱动 ----
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        // ---- 开始后校准：按 A 记录 MT1 位姿并取均值，作为起始位姿 ----
        Pose2d calibratedPose = calibrateViaVision(mt1);
        pinpoint.setPose(calibratedPose);
        ekf.setPose(calibratedPose);
        adaptiveEkf.setPose(calibratedPose);
        ukf.setPose(calibratedPose);
        adaptiveUkf.setPose(calibratedPose);
        drive.localizer.setPose(calibratedPose);

        telemetry.addLine("Calibration done, driving enabled");
        telemetry.update();

        // ---- 主循环 ----
        while (opModeIsActive()) {
            // === 手柄控制 ===
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            // === 更新所有定位器 ===
            drive.updatePoseEstimate();
            pinpoint.update();
            mt1.update();
            ekf.update();
            adaptiveEkf.update();
            ukf.update();
            adaptiveUkf.update();

            // === 获取位姿 ===
            Pose2d pinpointPose    = pinpoint.getPose();
            Pose2d mt1Pose         = mt1.getPose();
            Pose2d ekfPose         = ekf.getPose();
            Pose2d adaptiveEkfPose = adaptiveEkf.getPose();
            Pose2d ukfPose         = ukf.getPose();
            Pose2d adaptiveUkfPose = adaptiveUkf.getPose();

            // MT1 位姿仅在该帧 HIVE 倾角解算成功时才未被污染: 失败帧 getPose() 会回退为
            // 未修正的原始位姿 (见 MT1Localizer 类注释), 因此不能混入轨迹与误差统计
            boolean mt1Usable = mt1.isValid() && mt1.isHiveEstimated();

            // === 记录轨迹历史 ===
            addToHistory(pinpointHistory, pinpointPose);
            if (mt1Usable) {
                addToHistory(mt1History, mt1Pose);
            }
            addToHistory(ekfHistory, ekfPose);
            addToHistory(adaptiveHistory, adaptiveEkfPose);
            addToHistory(ukfHistory, ukfPose);
            addToHistory(adaptiveUkfHistory, adaptiveUkfPose);

            // === A 键触发误差记录 ===
            boolean aPressed = gamepad1.a;
            if (aPressed && !prevAPressed) {
                recordErrors(pinpointPose, mt1Pose, mt1Usable, ekfPose, adaptiveEkfPose, ukfPose, adaptiveUkfPose);
            }
            prevAPressed = aPressed;

            // ============ Telemetry: 位姿 ============
            telemetry.addData("Pinpoint",    formatPose(pinpointPose));
            telemetry.addData("MT1",         formatPose(mt1Pose));
            telemetry.addData("EKF",         formatPose(ekfPose));
            telemetry.addData("AdaptiveEKF", formatPose(adaptiveEkfPose));
            telemetry.addData("UKF",         formatPose(ukfPose));
            telemetry.addData("AdaptiveUKF", formatPose(adaptiveUkfPose));

            // ============ Telemetry: TestPose ============
            Pose2d testPose = new Pose2d(testX, testY, testHeading);
            telemetry.addData("TestPose", formatPose(testPose));
            telemetry.addData("Press A to record error", "");

            // ============ Telemetry: 已记录误差 ============
            if (errorRecorded) {
                telemetry.addLine("=== Errors (Δx, Δy, Δθ°) ===");
                telemetry.addData("Pinpoint err",
                        formatErr(errPinpointX, errPinpointY, errPinpointTheta));
                telemetry.addData("MT1 err",
                        formatErr(errMt1X, errMt1Y, errMt1Theta));
                telemetry.addData("EKF err",
                        formatErr(errEkfX, errEkfY, errEkfTheta));
                telemetry.addData("AdaptiveEKF err",
                        formatErr(errAdaptiveX, errAdaptiveY, errAdaptiveTheta));
                telemetry.addData("UKF err",
                        formatErr(errUkfX, errUkfY, errUkfTheta));
                telemetry.addData("AdaptiveUKF err",
                        formatErr(errAdaptiveUkfX, errAdaptiveUkfY, errAdaptiveUkfTheta));
            }

            // ============ Telemetry: MT1Localizer 实时质量指标 ============
            telemetry.addLine("--- MT1Localizer ---");
            telemetry.addData("MT1.valid",               mt1.isValid());
            telemetry.addData("MT1.tagIds",              mt1.getTagIds());
            telemetry.addData("MT1.tagCount",            mt1.getTagCount());
            telemetry.addData("MT1.avgDist (m)",         mt1.getAvgDist());
            telemetry.addData("MT1.avgArea",             mt1.getAvgArea());
            telemetry.addData("MT1.span (m)",            mt1.getSpan());
            telemetry.addData("MT1.maxFiducialSkew",     mt1.getMaxFiducialSkew());
            telemetry.addData("MT1.ambiguity (m)",       mt1.getAmbiguity());
            telemetry.addData("MT1.angularAmb (rad)",    mt1.getAngularAmbiguity());
            telemetry.addData("MT1.captureLatency (ms)", mt1.getCaptureLatency());
            telemetry.addData("MT1.timestamp (s)",       mt1.getTimestamp());

            // ============ Telemetry: MT1Localizer HIVE 观测 ============
            telemetry.addLine("--- MT1Localizer HIVE ---");
            telemetry.addData("MT1.hiveEstimated",       mt1.isHiveEstimated());
            telemetry.addData("MT1.hiveState",           mt1.getHiveState());
            telemetry.addData("MT1.hiveAngle (deg)",     Math.toDegrees(mt1.getHiveAngle()));
            telemetry.addData("MT1.pitchCheckErr (deg)", mt1.getHivePitchCheckErrDeg());
            telemetry.addData("MT1.rawZ (in)",           mt1.getRawZIn());
            telemetry.addData("MT1.rawPitch (deg)",      Math.toDegrees(mt1.getRawPitch()));

            double[] std = mt1.getStdDevs();
            if (std != null && std.length >= 6) {
                telemetry.addData("MT1.stdX (m)",         std[0]);
                telemetry.addData("MT1.stdY (m)",         std[1]);
                telemetry.addData("MT1.stdZ (m)",         std[2]);
                telemetry.addData("MT1.stdRoll (deg)",    std[3]);
                telemetry.addData("MT1.stdPitch (deg)",   std[4]);
                telemetry.addData("MT1.stdYaw (deg)",     std[5]);
            }

            // ============ Telemetry: 各滤波器实际应用的 Q/R 矩阵 (对角值) ============
            // EKF/UKF (对照组): Q 与 R 均为固定值。
            // AdaptiveEKF/UKF: Q 每帧按 IMU 工况自适应 (Q = qBase × boost), R 视觉自适应。
            double[] q, r;

            // --- EKF (对照) ---
            telemetry.addLine("--- EKF (对照, 固定 Q/R) ---");
            q = ekf.getEKF().getQDiag();
            r = ekf.getEKF().getRDiag();
            telemetry.addData("EKF.Q diag (x,y,θ)", formatDiag(q));
            telemetry.addData("EKF.R diag (x,y,θ)", formatDiag(r));

            // --- AdaptiveEKF ---
            telemetry.addLine("--- AdaptiveEKF (自适应 Q/R) ---");
            q = adaptiveEkf.getEKF().getQDiag();
            r = adaptiveEkf.getEKF().getRDiag();
            telemetry.addData("AEKF.Q diag (x,y,θ)", formatDiag(q));
            telemetry.addData("AEKF.R diag (x,y,θ)", formatDiag(r));

            // --- UKF (对照) ---
            telemetry.addLine("--- UKF (对照, 固定 Q/R) ---");
            q = ukf.getUKF().getQDiag();
            r = ukf.getUKF().getRDiag();
            telemetry.addData("UKF.Q diag (x,y,θ)", formatDiag(q));
            telemetry.addData("UKF.R diag (x,y,θ)", formatDiag(r));

            // --- AdaptiveUKF ---
            telemetry.addLine("--- AdaptiveUKF (自适应 Q/R) ---");
            q = adaptiveUkf.getUKF().getQDiag();
            r = adaptiveUkf.getUKF().getRDiag();
            telemetry.addData("AUKF.Q diag (x,y,θ)", formatDiag(q));
            telemetry.addData("AUKF.R diag (x,y,θ)", formatDiag(r));


            telemetry.update();

            // === Dashboard 图形绘制 ===
            TelemetryPacket packet = new TelemetryPacket();

            // TestPose — 红色
            packet.fieldOverlay().setStroke("#F44336");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), testPose);

            // 轨迹绘制
            drawTrail(packet.fieldOverlay(), pinpointHistory,    "#4CAF50", 2);
            drawTrail(packet.fieldOverlay(), mt1History,         "#FF9800", 2);
            drawTrail(packet.fieldOverlay(), ekfHistory,         "#E91E63", 2);
            drawTrail(packet.fieldOverlay(), adaptiveHistory,    "#9C27B0", 2);
            drawTrail(packet.fieldOverlay(), ukfHistory,         "#00BCD4", 2);
            drawTrail(packet.fieldOverlay(), adaptiveUkfHistory, "#2196F3", 2);

            // 当前位姿绘制
            packet.fieldOverlay().setStroke("#4CAF50");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), pinpointPose);

            if (mt1Usable) {
                packet.fieldOverlay().setStroke("#FF9800");
                packet.fieldOverlay().setStrokeWidth(2);
                Drawing.drawRobot(packet.fieldOverlay(), mt1Pose);
            }

            packet.fieldOverlay().setStroke("#E91E63");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), ekfPose);

            packet.fieldOverlay().setStroke("#9C27B0");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), adaptiveEkfPose);

            packet.fieldOverlay().setStroke("#00BCD4");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), ukfPose);

            packet.fieldOverlay().setStroke("#2196F3");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), adaptiveUkfPose);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    // ==================== 开始后校准 ====================

    /**
     * 通过 MT1 视觉定位记录起始位姿：按下 A 后采集一段时间内的有效位姿，
     * 返回均值作为校准结果（位置算术平均，朝向圆周平均）。
     */
    private Pose2d calibrateViaVision(MT1Localizer mt1) {
        telemetry.addLine("Calibration: press A to record start pose (MT1)");
        telemetry.update();

        boolean prevA = false;
        boolean isRecording = false;
        long recordingStartMs = 0;
        List<Pose2d> samples = new ArrayList<>();

        while (opModeIsActive()) {
            mt1.update();
            Pose2d pose = mt1.getPose();

            // 仅 HIVE 倾角解算成功的帧可用于校准: 失败帧 getPose() 会回退为受污染的原始位姿
            boolean usable = mt1.isValid() && mt1.isHiveEstimated();

            telemetry.addLine("--- Calibration (MT1) ---");
            if (usable) {
                telemetry.addData("X (in)", "%.2f", pose.position.x);
                telemetry.addData("Y (in)", "%.2f", pose.position.y);
                telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading.toDouble()));
            } else {
                telemetry.addLine("No valid pose (需要 valid + hiveEstimated)");
            }
            telemetry.addData("Tag Count", mt1.getTagCount());
            telemetry.addData("Ambiguity (m)", "%.4f", mt1.getAmbiguity());
            telemetry.addData("Hive State", mt1.getHiveState());
            telemetry.addData("Hive Angle (deg)", "%.2f", Math.toDegrees(mt1.getHiveAngle()));
            telemetry.addData("Pitch Check Err (deg)", "%.2f", mt1.getHivePitchCheckErrDeg());

            boolean a = gamepad1.a;
            if (a && !prevA && !isRecording) {
                isRecording = true;
                recordingStartMs = System.currentTimeMillis();
                samples.clear();
            }
            prevA = a;

            if (isRecording) {
                if (usable) {
                    samples.add(pose);
                }
                if (System.currentTimeMillis() - recordingStartMs >= CALIB_DURATION_MS) {
                    isRecording = false;
                    if (!samples.isEmpty()) {
                        return computeMeanPose(samples);
                    }
                    telemetry.addLine("No valid samples, press A to retry");
                }
            }

            telemetry.addLine();
            telemetry.addData("Recording", isRecording ? "Active" : "Inactive");
            if (isRecording) {
                telemetry.addData("Time remaining (ms)",
                        CALIB_DURATION_MS - (System.currentTimeMillis() - recordingStartMs));
                telemetry.addData("Samples", samples.size());
            }
            telemetry.update();
        }
        // OpMode 停止时才执行到这里
        return new Pose2d(0, 0, 0);
    }

    /** 位置算术平均，朝向圆周平均。 */
    private static Pose2d computeMeanPose(List<Pose2d> samples) {
        double sumX = 0, sumY = 0;
        double sumSin = 0, sumCos = 0;
        for (Pose2d p : samples) {
            sumX += p.position.x;
            sumY += p.position.y;
            sumSin += Math.sin(p.heading.toDouble());
            sumCos += Math.cos(p.heading.toDouble());
        }
        return new Pose2d(sumX / samples.size(), sumY / samples.size(), Math.atan2(sumSin, sumCos));
    }

    // ==================== 误差计算 ====================

    private void recordErrors(Pose2d pp, Pose2d mt, boolean mtUsable, Pose2d ek, Pose2d ae, Pose2d uk, Pose2d au) {
        Pose2d ref = new Pose2d(testX, testY, testHeading);

        errPinpointX     = pp.position.x - ref.position.x;
        errPinpointY     = pp.position.y - ref.position.y;
        errPinpointTheta = normalizeAngle(pp.heading.toDouble() - ref.heading.toDouble());

        if (mtUsable) {
            errMt1X      = mt.position.x - ref.position.x;
            errMt1Y      = mt.position.y - ref.position.y;
            errMt1Theta  = normalizeAngle(mt.heading.toDouble() - ref.heading.toDouble());
        } else {
            // 该帧 HIVE 倾角无解或被拒, MT1 位姿为未修正的受污染位姿, 不参与误差统计
            errMt1X     = Double.NaN;
            errMt1Y     = Double.NaN;
            errMt1Theta = Double.NaN;
        }

        errEkfX          = ek.position.x - ref.position.x;
        errEkfY          = ek.position.y - ref.position.y;
        errEkfTheta      = normalizeAngle(ek.heading.toDouble() - ref.heading.toDouble());

        errAdaptiveX     = ae.position.x - ref.position.x;
        errAdaptiveY     = ae.position.y - ref.position.y;
        errAdaptiveTheta = normalizeAngle(ae.heading.toDouble() - ref.heading.toDouble());

        errUkfX          = uk.position.x - ref.position.x;
        errUkfY          = uk.position.y - ref.position.y;
        errUkfTheta      = normalizeAngle(uk.heading.toDouble() - ref.heading.toDouble());

        errAdaptiveUkfX     = au.position.x - ref.position.x;
        errAdaptiveUkfY     = au.position.y - ref.position.y;
        errAdaptiveUkfTheta = normalizeAngle(au.heading.toDouble() - ref.heading.toDouble());

        errorRecorded = true;
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ==================== 工具 ====================

    private static void addToHistory(List<Pose2d> history, Pose2d pose) {
        history.add(pose);
        if (history.size() > MAX_HISTORY) {
            history.remove(0);
        }
    }

    private static void drawTrail(com.acmerobotics.dashboard.canvas.Canvas c,
                                  List<Pose2d> history, String color, int width) {
        if (history.size() < 2) return;
        c.setStroke(color);
        c.setStrokeWidth(width);
        c.setFill(color);
        double[] xs = new double[history.size()];
        double[] ys = new double[history.size()];
        for (int i = 0; i < history.size(); i++) {
            xs[i] = history.get(i).position.x;
            ys[i] = history.get(i).position.y;
        }
        c.strokePolyline(xs, ys);
    }

    private static String formatPose(Pose2d pose) {
        return String.format("(%.2f, %.2f, %.1f°)",
                pose.position.x, pose.position.y,
                Math.toDegrees(pose.heading.toDouble()));
    }

    /** 将 Q/R 矩阵对角线 {a, b, c} 格式化为字符串 (%.5g，自动适配大/小数值)。 */
    private static String formatDiag(double[] d) {
        return String.format("[%.5g, %.5g, %.5g]", d[0], d[1], d[2]);
    }

    private static String formatErr(double dx, double dy, double dtheta) {
        // MT1 误差在记录时刻 HIVE 倾角无解时被置为 NaN (见 recordErrors)
        if (Double.isNaN(dx) || Double.isNaN(dy) || Double.isNaN(dtheta)) {
            return "n/a";
        }
        return String.format("(%+.2f, %+.2f, %+.1f°)",
                dx, dy, Math.toDegrees(dtheta));
    }
}