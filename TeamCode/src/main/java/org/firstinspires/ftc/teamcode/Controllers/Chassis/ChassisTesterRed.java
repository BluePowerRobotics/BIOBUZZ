package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * 底盘 + 定位联调测试 (红方)。
 *
 * <p>流程：
 * <ol>
 *   <li><b>视觉均值定初值</b>：开始后按 A，采集 {@link #CALIB_DURATION_MS} 毫秒内所有
 *       HIVE 解算成功 ({@code isValid() && isHiveEstimated()}) 的 MT1 位姿并取均值
 *       (位置算术平均 / 朝向圆周平均)，作为初始位姿 (参照 FusionTestOpMode)，随后进入驾驶；</li>
 *   <li><b>手柄操作</b>：左摇杆平移 + 右摇杆旋转；X 切换有头/无头；
 *       左扳机瞄准 Red_Audience_Down、右扳机瞄准 Red_Audience_Up (此时右摇杆失效)；
 *       A 复位到 {@link HypParams#ResetPoseRed}；</li>
 *   <li><b>实时可视化</b>：Dashboard 场地图上绘制当前位姿、行驶轨迹与两个球门位置；</li>
 *   <li><b>遥测</b>：打印 {@link RobotPosition#getHiveState()} 跟踪状态及 MT1 逐帧观测细节。</li>
 * </ol>
 *
 * <p>注意：MT1 的 {@code getPose()} 只在 HIVE 解算成功时才未被污染，
 * 因此校准与可视化均以 {@code isValid() && isHiveEstimated()} 作为可用判据。
 */
@Config
@TeleOp(name = "ChassisTesterRed", group = "Test")
public class ChassisTesterRed extends LinearOpMode {

    /** 开始后视觉校准的采样时长 (毫秒)，Dashboard 可调 */
    public static int CALIB_DURATION_MS = 2000;
    /** Dashboard 轨迹最多保留的点数，Dashboard 可调 */
    public static int MAX_TRAIL_POINTS = 500;

    private Chassis chassis;
    private ActionRunner actionRunner;

    /** Dashboard 轨迹历史 (场地坐标系) */
    private final List<Pose2d> trail = new LinkedList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TeamColor teamColor = TeamColor.RED;

        // Chassis 构造时即完成 RobotPosition 初始化 (Limelight + 自适应 EKF)，
        // 故先以 ResetPoseRed 作为 EKF 初值，待视觉校准取得均值后再 ResetPoseTo 覆盖。
        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, HypParams.ResetPoseRed);

        telemetry.addLine("=== ChassisTesterRed ===");
        telemetry.addData("Team", "RED (Limelight pipeline 0)");
        telemetry.addData("Fallback ResetPose", formatPose(HypParams.ResetPoseRed));
        telemetry.addLine("A (after start) = run vision calibration");
        telemetry.addLine("Left Stick = drive, Right Stick X = rotate");
        telemetry.addLine("Left Trigger = aim Red Audience Down");
        telemetry.addLine("Right Trigger = aim Red Audience Up");
        telemetry.addLine("X = toggle head / no-head mode");
        telemetry.update();

        waitForStart();

        // ==== 1. 视觉均值确定初始位姿 ====
        Pose2d calibratedPose = calibrateViaVision();
        RobotPosition.getInstance().ResetPoseTo(calibratedPose);
        trail.clear();

        // 校准结束时若 A 仍被按住，需等其松开，避免主循环的 aWasReleased() 误触发复位
        while (opModeIsActive() && gamepad1.a) {
            idle();
        }

        // ==== 2. 手柄操作 + 实时可视化 ====
        while (opModeIsActive()) {
            // 定位更新：Pinpoint 预测 + MT1 视觉更新 (自适应 Q/R + 马氏门控) + HIVE 状态跟踪
            RobotPosition.getInstance().update();

            // ---- 底盘：左摇杆平移；扳机进瞄准 (右摇杆失效，航向自动指向球门) ----
            if (gamepad1.left_trigger > 0.5) {
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, HypParams.RedAudienceDown);
            } else if (gamepad1.right_trigger > 0.5) {
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, HypParams.RedAudienceUp);
            } else {
                chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            // 切换有头 / 无头模式
            if (gamepad1.xWasReleased()) {
                chassis.exchangeUseNoHeadMode();
            }


            // ---- 轨迹记录 ----
            Pose2d pose = RobotPosition.getInstance().getPose2d();
            addToTrail(pose);

            MT1Localizer mt1 = RobotPosition.getInstance().getFusionLocalizer().getMT1();
            Pose2d mt1Pose = mt1.getPose();
            boolean mt1Usable = mt1.isValid() && mt1.isHiveEstimated();

            // ---- 遥测：状态 / 位姿 ----
            telemetry.addData("Team", "RED");
            telemetry.addData("useNoHeadMode", chassis.getUseNoHeadMode());
            telemetry.addData("Pose", formatPose(pose));
            telemetry.addData("Vx (in/s)", "%.2f", RobotPosition.getInstance().getVx());
            telemetry.addData("Vy (in/s)", "%.2f", RobotPosition.getInstance().getVy());
            telemetry.addData("Omega (deg/s)", "%.2f", Math.toDegrees(RobotPosition.getInstance().getOmega()));
            telemetry.addData("MT1 usable", mt1Usable);

            // ---- 遥测：RobotPosition 的 HIVE 状态跟踪 ----
            telemetry.addLine("--- HIVE (RobotPosition) ---");
            telemetry.addData("HiveState (tracked)", RobotPosition.getInstance().getHiveState());

            // 执行动作队列（每帧一次）。Chassis 在动作运行期间屏蔽手柄输入，
            // 因此这里必须在循环内调用，否则队列非空时底盘会一直被屏蔽。
            actionRunner.update();

            // ---- 遥测：MT1 逐帧观测细节 (与跟踪值的来源对比) ----
            telemetry.addLine("--- MT1 observation ---");
            telemetry.addData("MT1.hiveEstimated", mt1.isHiveEstimated());
            telemetry.addData("MT1.hiveState", mt1.getHiveState());
            telemetry.addData("MT1.hiveAngle (deg)", "%.2f", Math.toDegrees(mt1.getHiveAngle()));
            telemetry.addData("MT1.pitchCheckErr (deg)", "%.2f", mt1.getHivePitchCheckErrDeg());
            telemetry.addData("MT1.rawZ (in)", "%.2f", mt1.getRawZIn());
            telemetry.addData("MT1.rawPitch (deg)", "%.2f", Math.toDegrees(mt1.getRawPitch()));
            telemetry.addData("MT1.tagCount", mt1.getTagCount());
            telemetry.addData("MT1.tagIds", mt1.getTagIds());
            telemetry.addData("MT1.ambiguity (m)", "%.4f", mt1.getAmbiguity());
            telemetry.addData("MT1.pose", mt1Usable ? formatPose(mt1Pose) : "n/a (hive rejected)");
            telemetry.update();

            // ---- Dashboard 图形 ----
            TelemetryPacket packet = new TelemetryPacket();

            // 轨迹 (青色折线)
            drawTrail(packet.fieldOverlay());

            // 球门目标点 (橙色圆圈 + 圆点)
            packet.fieldOverlay().setStroke("#FF9800");
            packet.fieldOverlay().setFill("#FF9800");
            packet.fieldOverlay().setStrokeWidth(2);
            strokeGoal(packet, HypParams.RedAudienceUp);
            strokeGoal(packet, HypParams.RedAudienceDown);

            // 当前位姿 (红色)
            packet.fieldOverlay().setStroke("#F44336");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), pose);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        chassis.stop();
    }

    // ==================== 视觉校准 ====================

    /**
     * 视觉均值校准：按 A 后采集 {@link #CALIB_DURATION_MS} 毫秒内所有 HIVE 解算成功的
     * MT1 位姿，返回均值作为初始位姿 (位置算术平均，朝向圆周平均)。
     *
     * <p>校准期间底盘保持静止，但仍每帧调用 {@link RobotPosition#update()}，
     * 由融合定位器内部推进 Pinpoint 并拉取最新 MT1 结果。
     */
    private Pose2d calibrateViaVision() {
        MT1Localizer mt1 = RobotPosition.getInstance().getFusionLocalizer().getMT1();

        telemetry.addLine("=== Vision Calibration (MT1) ===");
        telemetry.addLine("Press A to record start pose");
        telemetry.update();

        boolean prevA = false;
        boolean isRecording = false;
        long recordingStartMs = 0;
        List<Pose2d> samples = new ArrayList<>();

        while (opModeIsActive()) {
            chassis.stop();
            RobotPosition.getInstance().update();

            Pose2d pose = mt1.getPose();
            // 仅 HIVE 解算成功的帧可参与均值：失败帧 getPose() 会回退为受污染的原始位姿
            boolean usable = mt1.isValid() && mt1.isHiveEstimated();

            telemetry.addLine("=== Vision Calibration (MT1) ===");
            if (usable) {
                telemetry.addData("Live pose", formatPose(pose));
            } else {
                telemetry.addLine("No valid pose (need valid + hiveEstimated)");
            }
            telemetry.addData("Tag Count", mt1.getTagCount());
            telemetry.addData("Tag Ids", mt1.getTagIds());
            telemetry.addData("Ambiguity (m)", "%.4f", mt1.getAmbiguity());
            telemetry.addData("Hive State", mt1.getHiveState());
            telemetry.addData("Hive Angle (deg)", "%.2f", Math.toDegrees(mt1.getHiveAngle()));
            telemetry.addData("Pitch Check Err (deg)", "%.2f", mt1.getHivePitchCheckErrDeg());
            telemetry.addData("Recording", isRecording ? "Active" : "Inactive");
            if (isRecording) {
                telemetry.addData("Samples", samples.size());
                telemetry.addData("Time left (ms)",
                        CALIB_DURATION_MS - (System.currentTimeMillis() - recordingStartMs));
            }

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
                        Pose2d mean = computeMeanPose(samples);
                        telemetry.addData("Calibrated start pose", formatPose(mean));
                        telemetry.addData("Samples used", samples.size());
                        telemetry.update();
                        return mean;
                    }
                    telemetry.addLine("No valid samples, press A to retry");
                }
            }
            telemetry.update();
        }
        // OpMode 被停止时才会执行到这里
        return HypParams.ResetPoseRed;
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

    // ==================== Dashboard 绘制 ====================

    private void addToTrail(Pose2d pose) {
        trail.add(pose);
        while (trail.size() > MAX_TRAIL_POINTS) {
            trail.remove(0);
        }
    }

    /** 绘制行驶轨迹 (青色折线)。 */
    private void drawTrail(com.acmerobotics.dashboard.canvas.Canvas c) {
        if (trail.size() < 2) {
            return;
        }
        double[] xs = new double[trail.size()];
        double[] ys = new double[trail.size()];
        for (int i = 0; i < trail.size(); i++) {
            xs[i] = trail.get(i).position.x;
            ys[i] = trail.get(i).position.y;
        }
        c.setStroke("#00BCD4");
        c.setStrokeWidth(2);
        c.strokePolyline(xs, ys);
    }

    /** 绘制球门目标点标记。 */
    private void strokeGoal(TelemetryPacket packet, Pose2d goal) {
        packet.fieldOverlay().strokeCircle(goal.position.x, goal.position.y, 3);
        packet.fieldOverlay().fillCircle(goal.position.x, goal.position.y, 1);
    }

    // ==================== 工具 ====================

    private static String formatPose(Pose2d pose) {
        return String.format("(%.2f, %.2f, %.1f°)",
                pose.position.x, pose.position.y,
                Math.toDegrees(pose.heading.toDouble()));
    }
}