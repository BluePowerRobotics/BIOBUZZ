package org.firstinspires.ftc.teamcode.Processors.VisionLocalizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;

import java.util.List;

/**
 * 基于 Limelight MegaTag1 的视觉定位器。
 *
 * 每帧调用 {@link #update()} 从 Limelight 拉取最新结果，
 * 通过 {@link #getPose()} 获取全局位姿，{@link #getAmbiguity()} 获取不确定度。
 *
 * <p>典型用法 (与 EKF 融合):
 * <pre>{@code
 *   MT1Localizer mt1 = new MT1Localizer(limelight);
 *   // 每帧:
 *   mt1.update();
 *   if (mt1.isValid()) {
 *       double[] pose = mt1.getPose();       // {x, y, theta}
 *       double ambiguity = mt1.getAmbiguity(); // 不确定度
 *       // 根据 ambiguity 动态调整 R, 调用 ekf.update(...)
 *   }
 * }</pre>
 */
public class MT1Localizer implements Localizer {

    private final Limelight3A limelight;

    /** 单位转换: 1 m = 39.3701 in */
    private static final double M_TO_INCH = 39.37007874;

    // ---- 最新结果缓存 ----
    private LLResult latestResult;
    private Pose3D botpose;
    private boolean valid;

    // ---- MegaTag1 质量指标 ----
    /** MT1 标准偏差 [x, y, z, roll, pitch, yaw] (米/度) */
    private double[] stdDevs;
    /** 用于解算的标签数量 */
    private int tagCount;
    /** 标签平均距离 (米) */
    private double avgDist;
    /** 标签平均面积 */
    private double avgArea;
    /** 标签跨度 (米) */
    private double span;
    /** 位姿时间戳 (秒) */
    private double timestamp;
    /** 捕获延迟 (秒) */
    private double captureLatency;

    // ---- 单标签最大倾斜度 (越大越模糊) ----
    private double maxFiducialSkew;

    // ==================== 构造 ====================

    /**
     * @param limelight 已初始化并调用过 {@link Limelight3A#start()} 的 Limelight3A 实例
     */
    public MT1Localizer(Limelight3A limelight) {
        this.limelight = limelight;
        this.valid = false;
        this.stdDevs = new double[6];
    }

    // ==================== 核心更新 ====================

    /**
     * 从 Limelight 拉取最新 MegaTag1 结果。
     * 应在每帧循环中调用。
     *
     * @return 零速度（视觉定位器无法提供速度）
     */
    @Override
    public PoseVelocity2d update() {
        latestResult = limelight.getLatestResult();

        if (latestResult == null || !latestResult.isValid()) {
            valid = false;
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }

        botpose = latestResult.getBotpose();
        if (botpose == null) {
            valid = false;
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }

        // 提取 MegaTag1 质量指标
        stdDevs = latestResult.getStddevMt1();
        tagCount = latestResult.getBotposeTagCount();
        avgDist = latestResult.getBotposeAvgDist();
        avgArea = latestResult.getBotposeAvgArea();
        span = latestResult.getBotposeSpan();
        timestamp = latestResult.getTimestamp();
        captureLatency = latestResult.getCaptureLatency();

        // 计算单标签最大倾斜度 (skew 越大 → 姿态解算越模糊)
        maxFiducialSkew = 0.0;
        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult fr : fiducials) {
                double skew = fr.getSkew();
                if (skew > maxFiducialSkew) {
                    maxFiducialSkew = skew;
                }
            }
        }

        valid = true;
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    // ==================== 位姿输出 (Localizer 接口) ====================

    /**
     * @return 全局位姿 (英寸, 英寸, 弧度)
     *         坐标系: FTC 标准场地坐标系, 原点为场地中心
     */
    @Override
    public Pose2d getPose() {
        if (!valid || botpose == null) {
            return new Pose2d(0, 0, 0);
        }
        return new Pose2d(
                botpose.getPosition().x * M_TO_INCH,               // 米 → 英寸
                botpose.getPosition().y * M_TO_INCH,               // 米 → 英寸
                Math.toRadians(botpose.getOrientation().getYaw())  // 度 → 弧度
        );
    }

    /** 视觉定位器不支持设置位姿。 */
    @Override
    public void setPose(Pose2d pose) {
        // no-op
    }

    /** @return 原始位姿数组 {x, y, theta} (英寸, 英寸, 弧度) */
    public double[] getPoseArray() {
        if (!valid || botpose == null) {
            return new double[]{0, 0, 0};
        }
        return new double[]{
                botpose.getPosition().x * M_TO_INCH,
                botpose.getPosition().y * M_TO_INCH,
                Math.toRadians(botpose.getOrientation().getYaw())
        };
    }

    /**
     * @return 原始 Pose3D 对象，包含完整的 6DOF 位姿
     */
    public Pose3D getBotpose() {
        return botpose;
    }

    /**
     * @return 位姿的标准偏差 {@code double[6] = {x, y, z, roll, pitch, yaw}} (米/度)
     */
    public double[] getStdDevs() {
        // 返回副本, 避免调用方直接改写内部状态
        return stdDevs.clone();
    }

    // ==================== 不确定度 ====================

    /**
     * 综合不确定度指标。
     * 基于 MT1 标准偏差计算的平面位置不确定度 (米)。
     * 值越小 = 定位越可靠。
     *
     * <p>计算公式: sqrt(stdX^2 + stdY^2)
     * <p>典型阈值参考:
     * <ul>
     *   <li>&lt; 0.05  → 高置信度</li>
     *   <li>0.05~0.15 → 中等置信度</li>
     *   <li>&gt; 0.15  → 低置信度，建议丢弃</li>
     * </ul>
     *
     * @return 平面位置不确定度 (米)
     */
    public double getAmbiguity() {
        if (!valid || stdDevs == null || stdDevs.length < 2) {
            return Double.MAX_VALUE;
        }
        return Math.sqrt(stdDevs[0] * stdDevs[0] + stdDevs[1] * stdDevs[1]);
    }

    /**
     * 角度不确定度 (弧度)。
     * 基于 MT1 标准偏差的 yaw 分量。
     *
     * @return yaw 不确定度 (弧度)
     */
    public double getAngularAmbiguity() {
        if (!valid || stdDevs == null || stdDevs.length < 6) {
            return Double.MAX_VALUE;
        }
        return Math.toRadians(stdDevs[5]);
    }

    // ==================== 质量指标 ====================

    /** @return 解算使用的 AprilTag 数量 */
    public int getTagCount() {
        return tagCount;
    }

    /** @return 标签平均距离 (米) */
    public double getAvgDist() {
        return avgDist;
    }

    /** @return 标签平均面积 */
    public double getAvgArea() {
        return avgArea;
    }

    /** @return 标签跨度 (米) */
    public double getSpan() {
        return span;
    }

    /** @return 单标签最大倾斜度 (与姿态模糊相关) */
    public double getMaxFiducialSkew() {
        return maxFiducialSkew;
    }

    /** @return 位姿时间戳 (秒), 与 {@link System#nanoTime()} 不同基准 */
    public double getTimestamp() {
        return timestamp;
    }

    /** @return 捕获延迟 (秒) */
    public double getCaptureLatency() {
        return captureLatency;
    }

    /** @return 当前是否有有效定位结果 */
    public boolean isValid() {
        return valid;
    }

    // ==================== 便捷判断 ====================

    /**
     * 判断当前定位是否足够可靠 (用于 EKF 更新门控)。
     * 综合条件: 有效 + 标签数>=2 + 不确定度<阈值。
     *
     * @param ambiguityThreshold 不确定度阈值 (米)
     * @return true 如果定位可靠
     */
    public boolean isReliable(double ambiguityThreshold) {
        return valid && tagCount >= 2 && getAmbiguity() < ambiguityThreshold;
    }
}