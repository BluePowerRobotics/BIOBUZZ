package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;

import java.util.ArrayList;
import java.util.List;

/**
 * MT1Localizer 视觉定位测试 OpMode。
 *
 * <p>在 telemetry 实时显示 Limelight MegaTag1 的全局位姿与质量指标。
 * 按下手柄 <b>A</b> 键后，开始记录一段时长内的有效位姿，结束后计算
 * 均值（位置取算术平均，朝向取圆周平均）并显示在 telemetry。
 */
@Config
@TeleOp(name = "MT1 Test", group = "Vision")
public class MT1Test extends LinearOpMode {

    /** 记录时长 (毫秒) */
    public static int RECORDING_DURATION_MS = 2000;
    /** 测试团队颜色 */
    public static TeamColor teamColor = TeamColor.RED;

    private MT1Localizer mt1;

    // ---- 记录状态 ----
    private boolean isRecording = false;
    private long recordingStartMs = 0;
    private final List<Pose2d> samples = new ArrayList<>();

    // ---- 均值结果缓存 ----
    private boolean hasMean = false;
    private double meanX = 0, meanY = 0, meanHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("MT1 Test Readying……");
        telemetry.update();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        mt1 = new MT1Localizer(limelight, teamColor);

        telemetry.addLine("MT1 Test Ready");
        
        telemetry.addLine("A: record pose for " + RECORDING_DURATION_MS + " ms, then average");
        telemetry.update();

        waitForStart();

        boolean prevA = false;

        while (opModeIsActive()) {
            if (mt1 == null) {
                telemetry.addLine("Limelight unavailable — fix config and restart.");
                telemetry.update();
                sleep(250);
                continue;
            }

            mt1.update();
            Pose2d pose = mt1.getPose();

            // ---- 实时位姿显示 ----
            telemetry.addLine("--- Real-time Pose ---");
            if (mt1.isValid()) {
                telemetry.addData("X (in)", "%.2f", pose.position.x);
                telemetry.addData("Y (in)", "%.2f", pose.position.y);
                telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading.toDouble()));
            } else {
                telemetry.addLine("No valid pose");
            }
            telemetry.addData("Tag Count", mt1.getTagCount());
            telemetry.addData("Fiducial IDs", mt1.getTagIds());
            telemetry.addData("Ambiguity (m)", "%.4f", mt1.getAmbiguity());

            // ---- HIVE 观测 (倾角 / 瞬时状态 / 贴地残差) ----
            Pose2d rawPose = mt1.getRawPose();
            telemetry.addLine();
            telemetry.addLine("--- HIVE Observation ---");
            telemetry.addData("Estimated", mt1.isHiveEstimated());
            telemetry.addData("Hive State", mt1.getHiveState());
            telemetry.addData("Hive Angle (deg)", "%.2f", Math.toDegrees(mt1.getHiveAngle()));
            telemetry.addData("Pitch Check Err (deg)", "%.2f", mt1.getHivePitchCheckErrDeg());
            telemetry.addData("Pivot Height (in)", "%.2f", HypParams.hivePivotHeightIn);

            // ---- 原始位姿 vs 修正位姿 ----
            telemetry.addLine();
            telemetry.addLine("--- Raw vs Corrected ---");
            telemetry.addData("Raw X / Y (in)", "%.2f / %.2f", rawPose.position.x, rawPose.position.y);
            telemetry.addData("Raw Heading (deg)", "%.2f", Math.toDegrees(rawPose.heading.toDouble()));
            telemetry.addData("Raw Z (in)", "%.2f", mt1.getRawZIn());
            telemetry.addData("Raw Pitch (deg)", "%.2f", Math.toDegrees(mt1.getRawPitch()));
            telemetry.addData("ΔX / ΔY (in)",
                    "%.2f / %.2f", pose.position.x - rawPose.position.x, pose.position.y - rawPose.position.y);
            telemetry.addData("ΔHeading (deg)", "%.2f",
                    Math.toDegrees(pose.heading.toDouble() - rawPose.heading.toDouble()));

            // ---- A 键边沿触发：开始记录 ----
            boolean a = gamepad1.a;
            if (a && !prevA && !isRecording) {
                isRecording = true;
                recordingStartMs = System.currentTimeMillis();
                samples.clear();
            }
            prevA = a;

            // ---- 记录期间收集有效位姿 ----
            if (isRecording) {
                if (mt1.isValid()) {
                    samples.add(pose);
                }
                // 到达时长自动停止并计算均值
                if (System.currentTimeMillis() - recordingStartMs >= RECORDING_DURATION_MS) {
                    isRecording = false;
                    computeMean();
                }
            }

            // ---- 记录状态 ----
            telemetry.addLine();
            telemetry.addData("Recording", isRecording ? "Active" : "Inactive");
            if (isRecording) {
                telemetry.addData("Time remaining (ms)",
                        RECORDING_DURATION_MS - (System.currentTimeMillis() - recordingStartMs));
                telemetry.addData("Samples", samples.size());
            }

            // ---- 均值结果 ----
            if (hasMean) {
                telemetry.addLine();
                telemetry.addLine("--- Mean Pose ---");
                telemetry.addData("Mean X (in)", "%.2f", meanX);
                telemetry.addData("Mean Y (in)", "%.2f", meanY);
                telemetry.addData("Mean Heading (deg)", "%.2f", Math.toDegrees(meanHeading));
                telemetry.addData("Samples", samples.size());
            }

            telemetry.update();

            // ---- Dashboard 场地视图绘制 ----
            TelemetryPacket packet = new TelemetryPacket();
            // 原始位姿（未做 HIVE 倾角修正）— 橙色
            if (mt1.isValid()) {
                packet.fieldOverlay().setStroke("#FF9800");
                packet.fieldOverlay().setStrokeWidth(2);
                Drawing.drawRobot(packet.fieldOverlay(), rawPose);
            }
            // 修正位姿（HIVE 贴地修正后）— 绿色
            if (mt1.isHiveEstimated()) {
                packet.fieldOverlay().setStroke("#4CAF50");
                packet.fieldOverlay().setStrokeWidth(2);
                Drawing.drawRobot(packet.fieldOverlay(), pose);
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    /** 计算并缓存均值：位置算术平均，朝向圆周平均。 */
    private void computeMean() {
        if (samples.isEmpty()) {
            hasMean = false;
            return;
        }

        double sumX = 0, sumY = 0;
        double sumSin = 0, sumCos = 0;
        for (Pose2d p : samples) {
            sumX += p.position.x;
            sumY += p.position.y;
            double h = p.heading.toDouble();
            sumSin += Math.sin(h);
            sumCos += Math.cos(h);
        }

        meanX = sumX / samples.size();
        meanY = sumY / samples.size();
        meanHeading = Math.atan2(sumSin, sumCos);
        hasMean = true;
    }
}