package org.firstinspires.ftc.teamcode.Processors.FusionLocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Processors.D3Localizer.PinpointD3Localizer;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.utility.filter.UKF.UKF;

/**
 * 对照 UKF 定位器 —— Pinpoint(里程计) + Limelight MegaTag1(视觉) + UKF 融合。
 *
 * <p>支持两种里程计模式：
 * <ul>
 *   <li><b>D2</b> (默认): {@link PinpointLocalizer}，标准 2D 里程计，无斜坡补偿</li>
 *   <li><b>D3</b>: {@link PinpointD3Localizer}，3D 斜坡补偿里程计，需要 Hub IMU</li>
 * </ul>
 *
 * <p>作为<b>固定权重对照组</b>，与 {@link AdaptiveUKFLocalizer} 的区别：
 * <ul>
 *   <li>Q 与 R 均为固定值（构造时默认值，每帧重写），不做 IMU/视觉自适应</li>
 *   <li>视觉更新不做马氏距离门控，仅做有效性过滤</li>
 *   <li>不需要 IMU 硬件 (D2 模式)</li>
 * </ul>
 *
 * <p>每帧调用 {@link #update()} 即可完成：
 * <ol>
 *   <li>里程计更新 → 获取速度</li>
 *   <li>UKF 预测 (固定 Q)</li>
 *   <li>Limelight 有效时 → UKF 更新 (固定 R)</li>
 * </ol>
 */
@Config
public class UKFLocalizer implements Localizer {

    private final UKF ukf;
    /** 里程计定位器 (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    private final Localizer odom;
    private final MT1Localizer mt1;
    private final boolean useD3;

    // ---- 固定 Q/R 参数 (位置与角度独立) ----
    /** 过程噪声 — 位置 (in²/s) */
    public static double QbasePos = 0.01;
    /** 过程噪声 — 角度 (rad²/s) */
    public static double QbaseAngle = 0.01;
    /** 观测噪声 — 位置 (in²) */
    public static double RbasePos = 0.01;
    /** 观测噪声 — 角度 (rad²) */
    public static double RbaseAngle = 0.05;

    /** 最近一次里程计速度缓存 */
    private PoseVelocity2d lastVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

    // ==================== 构造 ====================

    /**
     * D2 模式构造 (标准 2D 里程计)。
     *
     * @param hardwareMap  硬件映射
     * @param limelight    已启动的 Limelight3A 实例
     * @param initialPose  初始位姿 (x, y, heading)
     */
    public UKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight, Pose2d initialPose) {
        this.ukf = new UKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
        ukf.setQ(QbasePos, QbasePos, QbaseAngle);
        ukf.setR(RbasePos, RbasePos, RbaseAngle);
        this.odom = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);
        this.mt1 = new MT1Localizer(limelight);
        this.useD3 = false;
    }

    /**
     * D2 模式简化构造: 初始位姿 (0, 0, 0)。
     */
    public UKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight) {
        this(hardwareMap, limelight, new Pose2d(0, 0, 0));
    }

    /**
     * D3 模式构造 (3D 斜坡补偿里程计)。
     *
     * @param hardwareMap   硬件映射
     * @param limelight     已启动的 Limelight3A 实例
     * @param imuDeviceName Hub IMU 设备名 (如 "imu")
     * @param initialPose   初始位姿 (x, y, heading)
     */
    public UKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                        String imuDeviceName, Pose2d initialPose) {
        this.ukf = new UKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
        ukf.setQ(QbasePos, QbasePos, QbaseAngle);
        ukf.setR(RbasePos, RbasePos, RbaseAngle);
        this.odom = new PinpointD3Localizer(hardwareMap, 0.001999, imuDeviceName, initialPose);
        this.mt1 = new MT1Localizer(limelight);
        this.useD3 = true;
    }

    // ==================== 核心循环 ====================

    /**
     * 每帧调用一次，完成：
     * <ol>
     *   <li>里程计更新 → 获取速度</li>
     *   <li>UKF 预测 (固定 Q)</li>
     *   <li>Limelight 有效时 → UKF 更新 (固定 R)</li>
     * </ol>
     *
     * @return 当前速度估计
     */
    @Override
    public PoseVelocity2d update() {
        double now = getNow();

        // ---- 1. 里程计速度 ----
        lastVel = odom.update();

        // ---- 2. 固定 Q/R (对照组: 每帧写回常量, 不做任何自适应) ----
        ukf.setQ(QbasePos, QbasePos, QbaseAngle);
        ukf.setR(RbasePos, RbasePos, RbaseAngle);

        // ---- 3. UKF 预测 (使用固定 Q) ----
        ukf.predict(lastVel.linearVel.x, lastVel.linearVel.y, lastVel.angVel, now);

        // ---- 4. MT1 视觉 → UKF 更新 (固定 R, 无门控, 仅有效性过滤) ----
        mt1.update();
        if (mt1.isValid()) {
            Pose2d visionPose = mt1.getPose();              // (英寸, 英寸, 弧度)
            ukf.update(
                    visionPose.position.x,                  // 英寸
                    visionPose.position.y,                  // 英寸
                    visionPose.heading.toDouble(),          // 弧度
                    mt1.getTimestamp()
            );
        }

        return lastVel;
    }

    // ==================== Localizer 接口 ====================

    /** 设置定位器位姿。 */
    @Override
    public void setPose(Pose2d pose) {
        ukf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
    }

    // ==================== Q/R 设置接口 ====================

    /**
     * 允许外部手动调整 Q 矩阵。
     * 注: 每帧 update() 会以 {@link #QbasePos} 等常量覆盖。
     */
    public void setQ(double qx, double qy, double qtheta) {
        ukf.setQ(qx, qy, qtheta);
    }

    /**
     * 允许外部手动调整 R 矩阵。
     * 注: 每帧 update() 会以 {@link #RbasePos} 等常量覆盖。
     */
    public void setR(double rx, double ry, double rtheta) {
        ukf.setR(rx, ry, rtheta);
    }

    // ==================== 输出 ====================

    /** @return 融合后的位姿 {x, y, heading} (英寸, 英寸, 弧度) */
    @Override
    public Pose2d getPose() {
        double[] pose = ukf.getPose();
        return new Pose2d(pose[0], pose[1], pose[2]);
    }

    /** @return 原始 UKF 实例 */
    public UKF getUKF() { return ukf; }

    /** @return MT1 视觉定位器 */
    public MT1Localizer getMT1() { return mt1; }

    /** @return 里程计定位器 (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    public Localizer getOdom() { return odom; }

    /** @return 是否为 D3 模式 */
    public boolean isD3() { return useD3; }

    // ==================== 重置 ====================

    /** 重置定位到指定位姿。 */
    public void reset(Pose2d pose) {
        ukf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
    }

    // ==================== 内部工具 ====================

    private double getNow() {
        return System.nanoTime() / 1e9;
    }
}
