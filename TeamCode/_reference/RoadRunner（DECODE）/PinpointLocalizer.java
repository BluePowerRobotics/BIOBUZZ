package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;

/**
 * 使用 GoBilda Pinpoint 驱动的定位器实现
 * Pinpoint 是一个集成了编码器和 IMU 的定位系统
 */
@Config
public final class PinpointLocalizer implements Localizer {
    /**
     * Pinpoint 定位器的参数配置类
     */
    public static class Params {
        public double parYTicks = 2460; // 平行编码器的 y 位置（tick 单位）
        public double perpXTicks = -1970; // 垂直编码器的 x 位置（tick 单位）
    }

    /**
     * 全局参数实例，可通过 FTC Dashboard 实时调整
     */
    public static Params PARAMS = new Params();

    /**
     * GoBilda Pinpoint 驱动实例
     */
    public final GoBildaPinpointDriver driver;
    
    /**
     * 初始平行编码器方向和初始垂直编码器方向
     */
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    /**
     * 世界坐标系到 Pinpoint 坐标系的变换
     */
    private Pose2d txWorldPinpoint;
    
    /**
     * Pinpoint 坐标系到机器人坐标系的变换
     */
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    /**
     * 构造函数
     * @param hardwareMap 硬件映射
     * @param inPerTick 每个编码器 tick 对应的英寸数
     * @param initialPose 初始位姿
     */
    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // TODO: 确保你的配置中有这个名称的 Pinpoint 设备
        //   参考 https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);

        // TODO: 如果需要，反转编码器方向
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();
        txWorldPinpoint = initialPose;
    }

    /**
     * 设置当前位姿
     * @param pose 要设置的位姿
     */
    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    /**
     * 获取当前位姿
     * @return 当前位姿
     */
    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    /**
     * 更新位姿估计
     * @return 当前速度估计
     */
    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            // 更新 Pinpoint 到机器人的变换
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            
            // 计算世界坐标系下的速度
            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
            
            // 将世界坐标系速度转换为机器人坐标系速度
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}