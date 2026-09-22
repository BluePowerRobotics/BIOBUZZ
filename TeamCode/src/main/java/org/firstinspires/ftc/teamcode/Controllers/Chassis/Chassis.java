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
/*
todo：
1. 根据定位与目标位置，实现一键转向目标瞄准
2. 瞄准状态下，一操右摇杆失效，
   有头时，左摇杆前后控制车与球门连线上的前进后退，左摇杆左右控制底盘绕球门旋转（始终指向球门）
   无头时，左摇杆前后控制沿场地的前后平移，左摇杆左右控制沿场地的左右平移，但始终指向球门。
 */
@Config
public class Chassis {

    private final MecanumDrive drive;
    private final ActionRunner actionRunner;
    private boolean useNoHeadMode = HypParams.InitialUseNoHeadMode;
    private final Telemetry telemetry;
    private double lastKx = 0, lastKy = 0, lastKomega = 0;

    /** 瞄准模式航向环 P 增益 (rad/s per rad)，Dashboard 可调 */
    public static double aimKp = 3.0;
    /** 瞄准模式角速度指令上限 (rad/s)，防止过冲 */
    public static double aimMaxOmega = Math.PI;

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
        RobotPosition.RobotPositioninit(hardwareMap, initPose, teamColor);
        this.drive = RobotPosition.getInstance().getDrive();
        this.actionRunner = actionRunner;
        this.telemetry = telemetry;

    }

    public Chassis(HardwareMap hardwareMap, TeamColor teamColor, ActionRunner actionRunner, Telemetry telemetry, Pose2d startPose) {
        this.teamColor = teamColor;
        RobotPosition.RobotPositioninit(hardwareMap, startPose, teamColor);
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
        } else {
            // Action 运行期间手柄输入被屏蔽，此时主动刹停，避免电机维持上一帧功率；
            // 本帧稍后由 ActionRunner.update() 下发实际的驱动指令
            stop();
        }
    }

    /**
     * 瞄准模式重载：右摇杆失效，航向自动锁定目标，左摇杆只控制平移。
     *
     * <p>目标方位角由定位位姿解算 des = atan2(targetY - y, targetX - x)，
     * 经 P 环 ({@link #aimKp} / {@link #aimMaxOmega}) 输出角速度指令，且始终走最短转向方向。
     *
     * <ul>
     *   <li><b>有头</b>（useNoHeadMode=false）：车头已锁定目标，机器人前方即"车-球门连线"方向，
     *       左摇杆前后 = 沿连线前进后退，左摇杆左右 = 绕球门旋转（沿连线切向）；</li>
     *   <li><b>无头</b>（useNoHeadMode=true）：左摇杆前后/左右 = 沿场地前后/左右平移，
     *       摇杆方向由操作手坐标系旋转到机器人坐标系，同时车头始终指向目标。</li>
     * </ul>
     *
     * @param Kx         左摇杆左右（FTC 惯例：右推为正）
     * @param Ky         左摇杆前后（FTC 惯例：上推为负）
     * @param targetPose 目标位置（仅取 x/y，heading 忽略）
     */
    public void update(double Kx, double Ky, Pose2d targetPose){
        lastKx = Kx;
        lastKy = Ky;
        if(!actionRunner.isBusy()){
            double forwardVel = -Ky;
            double strafeVel = -Kx;
            // 右摇杆失效：角速度由"始终指向目标"的航向环给出
            double omega = aimOmega(targetPose);
            if(useNoHeadMode){
                // 无头：摇杆输入在操作手/场地坐标系中，旋转到机器人坐标系
                double driverHeading = (teamColor == TeamColor.RED) ? Math.PI / 2 : -Math.PI / 2;
                double theta = RobotPosition.getInstance().getTheta() - driverHeading;
                double cos = Math.cos(theta);
                double sin = Math.sin(theta);
                double forwardRobot = forwardVel * cos + strafeVel * sin;
                double strafeRobot = -forwardVel * sin + strafeVel * cos;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardRobot, strafeRobot), omega));
            }
            else{
                // 有头：车头锁定目标后，机器人前方即指向球门的方向
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardVel, strafeVel), omega));
            }
        } else {
            // Action 运行期间手柄输入被屏蔽，此时主动刹停（同普通重载）
            stop();
        }
    }

    /**
     * todo：P控制不够精确，改成PD或PID
     * 解算"始终指向目标"的角速度指令：目标在场地坐标系中的方位角
     * des = atan2(targetY - y, targetX - x)，与当前航向求最短转向误差后做 P 控制并限幅。
     *
     * @param targetPose 目标位置（仅取 x/y）
     * @return 角速度指令 (rad/s，逆时针为正)
     */
    private double aimOmega(Pose2d targetPose){
        RobotPosition robotPosition = RobotPosition.getInstance();
        double desiredHeading = Math.atan2(
                targetPose.position.y - robotPosition.getY(),
                targetPose.position.x - robotPosition.getX());
        // 误差归一化到 [-π, π)，保证始终走最短转向方向
        double error = Math.atan2(
                Math.sin(desiredHeading - robotPosition.getTheta()),
                Math.cos(desiredHeading - robotPosition.getTheta()));
        double omega = aimKp * error;
        return Math.max(-aimMaxOmega, Math.min(aimMaxOmega, omega));
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