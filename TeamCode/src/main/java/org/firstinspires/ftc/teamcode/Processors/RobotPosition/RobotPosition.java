package org.firstinspires.ftc.teamcode.Processors.RobotPosition;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.utility.filter.EMA;

//todo：改为Adaptive EKF，并通过MT1Localizer估计hive状态
@Config
public class RobotPosition {
    static MecanumDrive drive;
    HardwareMap hardwareMap;
    Localizer localizer;

    /** IMU，复用 Road Runner 已初始化的实例（设备名 "imu"） */
    private IMU imu;

    public Pose2d currentPose;
    public PoseVelocity2d currentVelocity2d;

    private static RobotPosition instance;

    public static RobotPosition getInstance(){
        if(instance==null){
            throw new IllegalStateException("RobotPosition not initialized, call setInstance first");
        }
        return instance;
    }
    private RobotPosition(){
    }

;
    public static RobotPosition RobotPositioninit(HardwareMap hardwareMap, Pose2d initpose) {

        instance=new RobotPosition();
        instance.hardwareMap = hardwareMap;

        instance.currentPose = initpose != null ? initpose : new Pose2d(0,0,0);
        instance.drive=new MecanumDrive(hardwareMap,instance.currentPose);
        instance.localizer=instance.drive.localizer;
        // IMU 复用 Road Runner 已初始化的实例
        instance.imu = instance.drive.lazyImu.get();
        return instance;
    }

    /**
     * 使用 localizer 自带的 setPose 方法重置机器人的位姿。
     * 各 localizer 实现（如 PinpointLocalizer）会正确处理内部状态，
     * 无需手动维护修正偏移量。
     *
     * @param pose 目标位姿（真实位姿）
     */
    public void ResetPoseTo(Pose2d pose) {
        localizer.setPose(pose);
        currentPose = pose;
    }

    // 每帧调用：更新定位器并返回当前位姿
    public Pose2d update() {
        currentVelocity2d = drive.updatePoseEstimate();

        if (instance.localizer != null) {
            try {
                // 位姿已在 drive.updatePoseEstimate() → localizer.update() 中更新完毕，
                // 此处只需读取最新位姿，不再重复调用 localizer.update()
                // instance.localizer.update();
                Pose2d p = instance.localizer.getPose();
                if (p != null) {
                    instance.currentPose = p;
                }
            } catch (Exception ignored) {
                // 如果 localizer 的方法抛异常，保持现有 pose
            }
        }
        org.firstinspires.ftc.teamcode.utility.Vector2D pose = new org.firstinspires.ftc.teamcode.utility.Vector2D(instance.getX(), instance.getY());
        return instance.currentPose;
    }


    public Pose2d getPose2d(){        return currentPose;    }

    public double getX(){     return currentPose.position.x;    }
    public double getY(){   return currentPose.position.y;    }
    public double getTheta(){ return currentPose.heading.toDouble();    }
    public double getVx(){
        double vxField = currentVelocity2d.linearVel.x;
        double vyField = currentVelocity2d.linearVel.y;
        double theta = getTheta();
        return vxField * Math.cos(theta) + vyField * Math.sin(theta);
    }
    public double getVy(){
        double vxField = currentVelocity2d.linearVel.x;
        double vyField = currentVelocity2d.linearVel.y;
        double theta = getTheta();
        return -vxField * Math.sin(theta) + vyField * Math.cos(theta);
    }
    public MecanumDrive getDrive(){return drive;}
    public double getOmega(){return currentVelocity2d.angVel;}

    // ---- IMU 功能（原 IMUSensor.java 合并于此） ----

    /**
     * 读取 IMU 的 yaw/pitch/roll 角。
     */
    public YawPitchRollAngles getYawPitchRollAngles() {
        return imu.getRobotYawPitchRollAngles();
    }

    /**
     * 获取指定单位的 yaw（航向角）。
     * @param angleUnit 角度单位
     */
    public double getYaw(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    /**
     * 重置 IMU yaw 为 0。
     * 注意：Road Runner 定位依赖 IMU yaw 计算航向增量，运行中调用会破坏位姿估计，仅应在初始化/标定时使用。
     */
    public void resetYaw() {
        imu.resetYaw();
    }

}