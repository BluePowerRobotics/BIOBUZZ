package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.OTOSKt;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * 使用 SparkFun OTOS 传感器的定位器实现
 * OTOS (Omnidirectional Tracking and Orientation System) 是一个集成了 IMU 和视觉传感器的定位系统
 */
@Config
public class OTOSLocalizer implements Localizer {
    /**
     * OTOS 定位器的参数配置类
     */
    public static class Params {
        public double angularScalar = 1.0; // 角度缩放因子
        public double linearScalar = 1.0; // 线性缩放因子

        // 注意：单位是英寸和弧度
        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0); // OTOS 传感器的偏移量
    }

    /**
     * 全局参数实例，可通过 FTC Dashboard 实时调整
     */
    public static Params PARAMS = new Params();

    /**
     * SparkFun OTOS 传感器实例
     */
    public final SparkFunOTOS otos;
    
    /**
     * 当前位姿
     */
    private Pose2d currentPose;

    /** 最近一次下发到 OTOS 的缩放参数 (用于检测 Dashboard 变更并增量同步) */
    private double appliedLinearScalar = Double.NaN;
    private double appliedAngularScalar = Double.NaN;

    /**
     * 构造函数
     * @param hardwareMap 硬件映射
     * @param initialPose 初始位姿
     */
    public OTOSLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        // TODO: 确保你的配置中有这个名称的 OTOS 设备
        //   参考 https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        currentPose = initialPose;
        otos.setPosition(OTOSKt.toOTOSPose(currentPose));
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.calibrateImu();
        otos.setLinearScalar(PARAMS.linearScalar);
        otos.setAngularScalar(PARAMS.angularScalar);
        otos.setOffset(PARAMS.offset);
        appliedLinearScalar = PARAMS.linearScalar;
        appliedAngularScalar = PARAMS.angularScalar;
    }

    /**
     * 获取当前位姿
     * @return 当前位姿
     */
    @Override
    public Pose2d getPose() {
        return currentPose;
    }

    /**
     * 设置当前位姿
     * @param pose 要设置的位姿
     */
    @Override
    public void setPose(Pose2d pose) {
        currentPose = pose;
        otos.setPosition(OTOSKt.toOTOSPose(currentPose));
    }

    /**
     * 更新位姿估计
     * @return 当前速度估计
     */
    @Override
    public PoseVelocity2d update() {
        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose, otosVel, otosAcc);

        // 运行时同步 Dashboard 可调的缩放参数, 使调参立即生效 (仅在变更时下发, 减少 I2C 写入)
        if (PARAMS.linearScalar != appliedLinearScalar) {
            otos.setLinearScalar(PARAMS.linearScalar);
            appliedLinearScalar = PARAMS.linearScalar;
        }
        if (PARAMS.angularScalar != appliedAngularScalar) {
            otos.setAngularScalar(PARAMS.angularScalar);
            appliedAngularScalar = PARAMS.angularScalar;
        }

        currentPose = OTOSKt.toRRPose(otosPose);
        Vector2d fieldVel = new Vector2d(otosVel.x, otosVel.y);
        Vector2d robotVel = Rotation2d.exp(otosPose.h).inverse().times(fieldVel);
        return new PoseVelocity2d(robotVel, otosVel.h);
    }
}
