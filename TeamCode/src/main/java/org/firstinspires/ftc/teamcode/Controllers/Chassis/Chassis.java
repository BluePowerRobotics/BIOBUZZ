 package org.firstinspires.ftc.teamcode.Controllers.Chassis;

 import com.acmerobotics.dashboard.config.Config;
 import com.acmerobotics.roadrunner.Pose2d;
 import com.acmerobotics.roadrunner.PoseVelocity2d;
 import com.acmerobotics.roadrunner.Vector2d;
 import com.qualcomm.robotcore.hardware.HardwareMap;

 import org.firstinspires.ftc.robotcore.external.Telemetry;
 import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
 import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
 import org.firstinspires.ftc.teamcode.utility.ActionRunner;
 import org.firstinspires.ftc.teamcode.Parameter.HypParams;
 import org.firstinspires.ftc.teamcode.Parameter.TeamColor;

@Config
public class Chassis {

    private final MecanumDrive drive;
    private final ActionRunner actionRunner;
    private boolean useNoHeadMode = HypParams.InitialUseNoHeadMode;
    private final Telemetry telemetry;
    private double lastKx = 0, lastKy = 0, lastKomega = 0;

    private final TeamColor teamColor;

    public Chassis(HardwareMap hardwareMap, TeamColor teamColor, ActionRunner actionRunner, Telemetry telemetry, boolean isTeleOp) {
        this.teamColor = teamColor;
        Pose2d initPose;
        if (isTeleOp) {
            initPose = (teamColor == TeamColor.RED) ?
                    HypParams.StopPoseRed : HypParams.StopPoseBlue;
        } else {
            initPose = (teamColor == TeamColor.RED) ?
                    HypParams.startPoseRed : HypParams.startPoseBlue;
        }
        RobotPosition.RobotPositioninit(hardwareMap, initPose);
        this.drive = RobotPosition.getInstance().getDrive();
        this.actionRunner = actionRunner;
        this.telemetry = telemetry;

    }

    public Chassis(HardwareMap hardwareMap, TeamColor teamColor, ActionRunner actionRunner, Telemetry telemetry, Pose2d startPose) {
        this.teamColor = teamColor;
        RobotPosition.RobotPositioninit(hardwareMap, startPose);
        this.drive = RobotPosition.getInstance().getDrive();
        this.actionRunner = actionRunner;
        this.telemetry = telemetry;
    }

    public void setUseNoHeadMode(boolean useNoHeadMode){
        this.useNoHeadMode = useNoHeadMode;
    }
    public void exchangeUseNoHeadMode(){
        useNoHeadMode = !useNoHeadMode;
    }
    public boolean getUseNoHeadMode(){
        return useNoHeadMode;
    }
    public void stop(){
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0,0),
                0));
    }

    public void update(double Kx, double Ky, double Komega){
        lastKx = Kx;
        lastKy = Ky;
        lastKomega = Komega;
        if(!actionRunner.isBusy()){
            // 摇杆 → 底盘速度映射：Ky/Kx 取反以匹配 FTC SDK 手柄惯例（上推为负、右推为正）
            // 官方 SDK: forward = -gamepad1.left_stick_y, strafe = gamepad1.left_stick_x
            // Road Runner: PoseVelocity2d.y 正值 = 向左横移，故 strafe 也需取反
            double forwardVel = -Ky;
            double strafeVel = -Kx;
            double omega = -Komega;
            if(useNoHeadMode){
                // 操作手基础朝向：BLUE 面向 -pi/2 (y-为前), RED 面向 pi/2 (y+为前)
                double driverHeading = (teamColor == TeamColor.RED) ? Math.PI / 2 : -Math.PI / 2;
                // 摇杆输入在操作手主观坐标系中，旋转到场地坐标系后再旋转到机器人坐标系
                // 复合效果等价于用 (theta - driverHeading) 替代原 theta
                double theta = RobotPosition.getInstance().getTheta() - driverHeading;
                double cos = Math.cos(theta);
                double sin = Math.sin(theta);
                // 将操作手坐标系速度旋转到机器人坐标系
                double forwardRobot = forwardVel * cos + strafeVel * sin;
                double strafeRobot = -forwardVel * sin + strafeVel * cos;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardRobot, strafeRobot), omega));
            }
            else{
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardVel, strafeVel), omega));
            }
        }
    }
    public void telemetry(){
        /*
        telemetry.addData("X",RobotPosition.getInstance().getX());
        telemetry.addData("Y",RobotPosition.getInstance().getY());
        telemetry.addData("Heading",Math.toDegrees(RobotPosition.getInstance().getTheta()));
        telemetry.addData("useNoHeadMode", useNoHeadMode);
        telemetry.addData("HeadingPID_kP", headingPID.getKP());
        telemetry.addData("HeadingPID_kI", headingPID.getKI());
        telemetry.addData("HeadingPID_kD", headingPID.getKD());
        /*
        telemetry.addData("lfP",drive.leftFront.getPower());
        telemetry.addData("rfP",drive.rightFront.getPower());
        telemetry.addData("lbP",drive.leftBack.getPower());
        telemetry.addData("rbP",drive.rightBack.getPower());
        */
        telemetry.addData("Vx",RobotPosition.getInstance().getVx());
        telemetry.addData("Vy",RobotPosition.getInstance().getVy());
        telemetry.addData("Omega",Math.toDegrees(RobotPosition.getInstance().getOmega()));
        /*
        telemetry.addData("lfV",drive.leftFront.getVelocity());
        telemetry.addData("rfV",drive.rightFront.getVelocity());
        telemetry.addData("lbV",drive.leftBack.getVelocity());
        telemetry.addData("rbV",drive.rightBack.getVelocity());
        
        telemetry.addData("Kx", lastKx);
        telemetry.addData("Ky", lastKy);
        telemetry.addData("Komega", lastKomega);
        */
        
    }
}