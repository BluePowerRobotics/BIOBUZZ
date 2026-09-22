package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.AutoActions.GoToStopPose;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;

/**
 * 红队自动阶段主程序框架
 * 使用状态机：START -> PARKING -> STOP
 */
@Autonomous(name = "AutoRed", group = "Auto")
public class AutoRed extends LinearOpMode {

    /** 自动阶段状态机 */
    private enum AutoState {
        START, PARKING, STOP
    }

    private ActionRunner actionRunner;
    private MecanumDrive drive;
    private Chassis chassis;

    /** 队伍颜色（红方，Limelight pipeline 0） */
    private final TeamColor teamColor = TeamColor.RED;

    /** 红队起始与停车位姿 */
    private final Pose2d startPose = HypParams.startPoseRed;
    private final Pose2d stopPose = HypParams.StopPoseRed;

    /** 当前状态，初始为 START */
    private AutoState currentState = AutoState.START;

    /** 主动请求停车标志 */
    private boolean shouldPark = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        actionRunner = new ActionRunner();

        // 初始化底盘与定位：Chassis 内部完成 RobotPosition 初始化（含 Limelight + 自适应 EKF）
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, startPose);
        drive = RobotPosition.getInstance().getDrive();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // 比赛计时从 START 按下后开始
        ElapsedTime matchTime = new ElapsedTime();

        while (opModeIsActive() && currentState != AutoState.STOP) {
            // 更新位姿
            RobotPosition.getInstance().update();

            // 剩余时间（毫秒）
            long remainingMs = HypParams.AUTONOMOUS_DURATION_MS - (long) matchTime.milliseconds();

            // ===== 全局停车判定：任意仍可运行的状态下，超时或主动停车时立即中断所有 Action 并转入 PARKING =====
            if (currentState != AutoState.PARKING && currentState != AutoState.STOP) {
                if (remainingMs < HypParams.PARK_TIME_THRESHOLD_MS || shouldPark) {
                    actionRunner.clear();
                    actionRunner.add(new GoToStopPose(drive, stopPose));
                    currentState = AutoState.PARKING;
                }
            }

            // ===== 状态机主循环 =====
            switch (currentState) {
                case START:
                    // todo: 在此添加自动阶段起始动作（得分、放置等）
                    shouldPark = true;
                    break;

                case PARKING:
                    if (!actionRunner.isBusy()) {
                        currentState = AutoState.STOP;
                    }
                    break;

                case STOP:
                    break;
            }

            actionRunner.update();

            telemetry.addData("State", currentState);
            telemetry.addData("Remaining(ms)", remainingMs);
            telemetry.update();
        }

        // 确保底盘停稳
        chassis.stop();
    }
}