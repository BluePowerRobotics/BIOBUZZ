package org.firstinspires.ftc.teamcode.Processors.VisionLocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;

import java.util.List;

/**
 * 基于 Limelight MegaTag1 的视觉定位器。
 *
 * <p>每帧调用 {@link #update()} 从 Limelight 拉取最新结果后:
 * <ul>
 *   <li>{@link #getPose()}    修正后的全局位姿 (英寸, 英寸, 弧度)</li>
 *   <li>{@link #getRawPose()} 未修正的原始位姿 (对比调试用)</li>
 *   <li>{@link #getHiveAngle()} / {@link #getHiveState()} / {@link #isHiveEstimated()} HIVE 倾角观测</li>
 *   <li>{@link #getAmbiguity()} / {@link #getStdDevs()} 质量指标 (供 EKF 自适应 R)</li>
 * </ul>
 *
 * <p><b>HIVE 解算</b> (实现见 MT1Localizer.md §三, 推导见 OneMap_MT1Localizer.md):
 * Limelight 中加载的是 HIVE <b>水平参考状态</b> 的地图, 而实际 HIVE 绕枢轴
 * (过 (0,0,h), 方向沿 y 轴) 旋转了 θ, 因此 botpose 与真实位姿相差一个绕 y 轴的旋转:
 * {@code T' = T_rot(φ)·T}, 其中 {@code φ = -θ}。利用机器人贴地约束 (z ≈ 0) 解出 φ,
 * 再复原真实位姿并得到倾角观测 θ = -φ。
 *
 * <p><b>队伍颜色</b>: 构造时按 {@link TeamColor} 切换 Limelight pipeline
 * (红方 0 / 蓝方 1), 以加载本方 HIVE 的场地地图; 不传颜色时默认红方。
 *
 * <p><b>本类只输出逐帧观测</b> (倾角与瞬时状态), 不做时间滤波、不做状态保持 ——
 * 连续估计 (滤波 / 滞回 / 保持上一次状态) 由 RobotPosition 等外部模块负责。
 */
@Config
public class MT1Localizer implements Localizer {

    /** HIVE 抬升状态: 逐帧瞬时分类, 不含滞回 (MT1Localizer.md §3.4) */
    public enum HiveState {
        /** 倾角足够小 (|θ| ≤ hiveCellDownAngleDeg), HIVE 视为水平/中间 */
        MIDDLE,
        /** AUDIENCE_UP 侧抬升 */
        AUDIENCE_UP,
        /** AUDIENCE_DOWN 侧抬升 */
        AUDIENCE_DOWN,
        /** 本帧无有效观测 (无标签 / 无解 / 姿态交叉校验失败 / 倾角被拒) */
        UNKNOWN
    }

    private final Limelight3A limelight;

    /** 单位转换: 1 m = 39.3701 in */
    private static final double M_TO_INCH = 39.37007874;

    /** 红方使用的 Limelight pipeline 索引 */
    private static final int PIPELINE_RED = 0;
    /** 蓝方使用的 Limelight pipeline 索引 */
    private static final int PIPELINE_BLUE = 1;

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
    /** 捕获延迟 (毫秒, 见 SDK LLResult.getCaptureLatency 文档) */
    private double captureLatency;

    // ---- 单标签最大倾斜度 (越大越模糊) ----
    private double maxFiducialSkew;

    // ---- 原始位姿 (未修正, 英寸/弧度) ----
    private double rawXIn;
    private double rawYIn;
    private double rawZIn;
    private double rawYaw;
    private double rawPitch;

    // ---- HIVE 解算观测 (逐帧, 无滤波/无状态保持) ----
    /** HIVE 倾角观测 θ = -φ (弧度), 未解出时为 NaN */
    private double hiveAngle = Double.NaN;
    /** 本帧 HIVE 倾角观测的瞬时状态 */
    private HiveState hiveState = HiveState.UNKNOWN;
    /** 本帧是否成功解出倾角观测 */
    private boolean hiveEstimated = false;
    /** 修正后的真实位姿 (英寸, 英寸, 弧度) */
    private double correctedXIn;
    private double correctedYIn;
    private double correctedYaw;
    /**
     * 姿态交叉校验偏差 (度): 由位置解出的 φ 与由 rawPitch 独立推算的 φ 之差 (取绝对值)。
     * 二者不一致说明本帧 z' 退化 (见 {@link #getRawZIn()} 说明), 该帧被拒绝。
     * 无观测时为 NaN。
     */
    private double pitchCheckErrDeg = Double.NaN;

    // ==================== 构造 ====================

    /**
     * 默认按红方初始化 (pipeline 0)。
     *
     * @param limelight 已初始化并调用过 {@link Limelight3A#start()} 的 Limelight3A 实例
     */
    public MT1Localizer(Limelight3A limelight) {
        this(limelight, TeamColor.RED);
    }

    /**
     * @param limelight 已初始化并调用过 {@link Limelight3A#start()} 的 Limelight3A 实例
     * @param teamColor 队伍颜色: {@link TeamColor#RED} 加载 pipeline 0, {@link TeamColor#BLUE} 加载 pipeline 1
     */
    public MT1Localizer(Limelight3A limelight, TeamColor teamColor) {
        this.limelight = limelight;
        this.valid = false;
        this.stdDevs = new double[6];
        // 按队伍颜色切换 pipeline (红 0 / 蓝 1), 决定加载哪一方 HIVE 的场地地图
        limelight.pipelineSwitch(teamColor == TeamColor.BLUE ? PIPELINE_BLUE : PIPELINE_RED);
    }

    // ==================== 核心更新 ====================

    /**
     * 从 Limelight 拉取最新 MegaTag1 结果, 并解算 HIVE 倾角观测与修正位姿。
     * 应在每帧循环中调用。
     *
     * @return 零速度（视觉定位器无法提供速度）
     */
    @Override
    public PoseVelocity2d update() {
        latestResult = limelight.getLatestResult();

        if (latestResult == null || !latestResult.isValid()) {
            valid = false;
            resetHiveObservation();
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }

        botpose = latestResult.getBotpose();
        if (botpose == null) {
            valid = false;
            resetHiveObservation();
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

        // 原始位姿: 米 → 英寸, 度 → 弧度
        rawXIn = botpose.getPosition().x * M_TO_INCH;
        rawYIn = botpose.getPosition().y * M_TO_INCH;
        rawZIn = botpose.getPosition().z * M_TO_INCH;
        rawYaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        rawPitch = botpose.getOrientation().getPitch(AngleUnit.RADIANS);

        valid = true;

        // 单地图 HIVE 解算: 输出倾角观测并得到修正后的真实位姿
        solveHiveObservation();

        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    // ==================== HIVE 解算 (MT1Localizer.md §三) ====================

    /**
     * 解算本帧的 HIVE 倾角观测与修正位姿 (MT1Localizer.md §3.2 ~ §3.4)。
     *
     * <p>解算失败 (无解 / 姿态交叉校验失败 / 解算参数非法) 时 {@link #isHiveEstimated()} 为 false,
     * 此时 {@link #getPose()} 回退为未修正的原始位姿。
     */
    private void solveHiveObservation() {
        resetHiveObservation();

        final double h = HypParams.hivePivotHeightIn;
        if (!(h > 0)) {
            // 枢轴高度未标定: 不解算, 避免输出无意义的倾角观测
            return;
        }

        // ---- §3.2 贴地约束: A·sinφ + B·cosφ = C ----
        final double a = rawXIn;
        final double b = rawZIn - h;
        final double c = -h;
        final double radius = Math.hypot(a, b);

        // 可解性检查: |C| > R 时该帧无解
        if (radius < 1e-9 || Math.abs(c) > radius) {
            return;
        }

        final double phi = solvePhi(a, b, c, Math.toRadians(HypParams.hiveCellUpAngleDeg));
        if (Double.isNaN(phi)) {
            return;
        }

        // ---- 退化帧判别: 用姿态 (pitch) 独立交叉校验 φ ----
        // φ 存在两条相互独立的信息通路:
        //   位置通路: z' 高度约束解出的 φ (上面这行)
        //   姿态通路: 由 R' = Ry(φ)·R_true 且机器人贴地 (真实 pitch ≈ 0), 有
        //             pitch' ≈ asin(sinφ · cos(ψ'))
        // |z'| → 0 的退化帧会让位置通路塌回 φ ≈ 0, 但姿态通路依然反映真实旋转,
        // 两者显著不一致即可判定该帧退化 (位置修正量同样不可信, 故整帧丢弃)。
        // 注意: 该判据以"机器人贴地平放"为前提, 机器人在斜坡上时会失效。
        final double sinPredicted = Math.sin(phi) * Math.cos(rawYaw);
        final double predictedPitch = Math.asin(Math.max(-1.0, Math.min(1.0, sinPredicted)));
        pitchCheckErrDeg = Math.toDegrees(Math.abs(normalize(rawPitch - predictedPitch)));
        if (pitchCheckErrDeg > HypParams.hivePitchCheckTolDeg) {
            return;
        }

        // ---- §3.3 位置修正 ----
        final double cosPhi = Math.cos(phi);
        final double sinPhi = Math.sin(phi);
        correctedXIn = a * cosPhi - rawZIn * sinPhi + h * sinPhi;
        correctedYIn = rawYIn;

        // ---- §3.3 姿态修正: R = Ry(-φ)·R', yaw = atan2(R10, R00) ----
        // R' 的第一列即机器人 x 轴在场坐标系下的方向 (SDK 约定: 内旋 yaw → pitch → roll)
        final double r00 = Math.cos(rawYaw) * Math.cos(rawPitch);
        final double r10 = Math.sin(rawYaw) * Math.cos(rawPitch);
        final double r20 = -Math.sin(rawPitch);
        correctedYaw = Math.atan2(r10, cosPhi * r00 - sinPhi * r20);

        // ---- §3.1 / §3.4 倾角观测与瞬时状态 ----
        hiveAngle = -phi;                       // θ = -φ
        hiveState = classifyHive(hiveAngle);
        hiveEstimated = true;
    }

    /**
     * 解三角方程 A·sinφ + B·cosφ = C (MT1Localizer.md §3.2-2)。
     *
     * <p>两族解为 α - atan2(B,A) 与 π - α - atan2(B,A) (α = asin(C/R)),
     * 二者恒相差 π - 2α ∈ [0, 2π], 因此除退化几何外只有一族落入 ±maxTilt 窗口;
     * 若两族都在窗口内, 取 |φ| 较小者 (本类不保存 φ_last, 见类注释)。
     *
     * @param a       A = x' (英寸)
     * @param b       B = z' - h (英寸)
     * @param c       C = -h (英寸)
     * @param maxTilt 取根筛选用的最大倾角 (弧度)
     * @return φ (弧度), 无解时返回 NaN
     */
    private static double solvePhi(double a, double b, double c, double maxTilt) {
        final double radius = Math.hypot(a, b);
        final double base = Math.atan2(b, a);
        final double alpha = Math.asin(c / radius);

        final double phi1 = normalize(alpha - base);
        final double phi2 = normalize(Math.PI - alpha - base);

        final boolean in1 = Math.abs(phi1) <= maxTilt;
        final boolean in2 = Math.abs(phi2) <= maxTilt;
        if (in1 && in2) {
            return Math.abs(phi1) <= Math.abs(phi2) ? phi1 : phi2;
        }
        if (in1) {
            return phi1;
        }
        if (in2) {
            return phi2;
        }
        return Double.NaN;
    }

    /**
     * 逐帧瞬时分类 (MT1Localizer.md §3.4), <b>不含滞回</b>:
     * <ul>
     *   <li>|θ| ≤ hiveCellDownAngleDeg → {@link HiveState#MIDDLE}</li>
     *   <li>hiveCellDownAngleDeg &lt; |θ| &lt; hiveCellUpAngleDeg → HIVE 稳定位于一侧,
     *       按 θ 正负判定 {@link HiveState#AUDIENCE_UP} / {@link HiveState#AUDIENCE_DOWN}</li>
     * </ul>
     * |θ| ≥ hiveCellUpAngleDeg 的帧已在 {@link #solvePhi} 取根时被拒绝, 不会进入本方法。
     */
    private static HiveState classifyHive(double theta) {
        if (Math.abs(theta) <= Math.toRadians(HypParams.hiveCellDownAngleDeg)) {
            return HiveState.MIDDLE;
        }
        final boolean up = HypParams.hivePositiveAngleIsAudienceUp == (theta > 0);
        return up ? HiveState.AUDIENCE_UP : HiveState.AUDIENCE_DOWN;
    }

    /** 角度归一化到 [-π, π)。 */
    private static double normalize(double angle) {
        double result = (angle + Math.PI) % (2 * Math.PI);
        if (result < 0) {
            result += 2 * Math.PI;
        }
        return result - Math.PI;
    }

    /** 清空本帧 HIVE 观测 (无效帧或解算失败时调用)。 */
    private void resetHiveObservation() {
        hiveEstimated = false;
        hiveAngle = Double.NaN;
        hiveState = HiveState.UNKNOWN;
        pitchCheckErrDeg = Double.NaN;
    }

    // ==================== 位姿输出 (Localizer 接口) ====================

    /**
     * @return HIVE 解算成功时为修正后的真实位姿, 否则回退为未修正的原始位姿
     *         (英寸, 英寸, 弧度); 无有效结果时为 (0,0,0)。
     *         坐标系: FTC 标准场地坐标系, 原点为场地中心
     */
    @Override
    public Pose2d getPose() {
        if (!valid || botpose == null) {
            return new Pose2d(0, 0, 0);
        }
        if (hiveEstimated) {
            return new Pose2d(correctedXIn, correctedYIn, correctedYaw);
        }
        return getRawPose();
    }

    /** 视觉定位器不支持设置位姿。 */
    @Override
    public void setPose(Pose2d pose) {
        // no-op
    }

    /**
     * @return 未经 HIVE 修正的原始位姿 (英寸, 英寸, 弧度), 供与修正位姿对比
     */
    public Pose2d getRawPose() {
        if (!valid || botpose == null) {
            return new Pose2d(0, 0, 0);
        }
        return new Pose2d(rawXIn, rawYIn, rawYaw);
    }

    /**
     * @return 原始位姿的高度 z (英寸)。
     *
     * <p><b>z 是倾角信息的位置通路载体</b>: 由解算模型 F(φ) = x'·sinφ + (z'-h)·cosφ + h,
     * 恒有 F(0) = z'。因此 |z'| ≈ 0 时位置通路解出的 φ 必然退化到 0 (θ = 0),
     * 与 HIVE 的真实倾角无关。此类帧由 {@link #getHivePitchCheckErrDeg()} 的姿态通路交叉校验识别并丢弃。
     * 若 HIVE 已明显倾斜而本值持续 ≈ 0, 说明当前加载的 fmap 未包含本方 HIVE 的
     * <b>水平参考态</b>标签 (或位姿来自场地上其他固定标签), 属于配置问题而非算法问题 (调试用)。
     */
    public double getRawZIn() {
        return rawZIn;
    }

    /**
     * @return 原始位姿的 pitch (弧度, 未经修正)。
     *         地图被 HIVE 带动旋转时 pitch 会随之明显变化, 可作为倾角的旁证
     *         (θ = 30° 且地图为水平参考态时, 本值约为 ±30°·cos(航向))。
     */
    public double getRawPitch() {
        return rawPitch;
    }

    /**
     * @return 本帧识别到的所有 fiducial ID, 逗号分隔。
     *         调试用: 确认只包含本方 HIVE 的标签 (混入固定场地标签会稀释倾角信息)
     */
    public String getTagIds() {
        if (latestResult == null) {
            return "";
        }
        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        if (fiducials == null) {
            return "";
        }
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (ids.length() > 0) {
                ids.append(',');
            }
            ids.append(fr.getFiducialId());
        }
        return ids.toString();
    }

    /** @return 当前输出位姿的数组形式 {x, y, theta} (英寸, 英寸, 弧度) */
    public double[] getPoseArray() {
        Pose2d pose = getPose();
        return new double[]{pose.position.x, pose.position.y, pose.heading.toDouble()};
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

    // ==================== HIVE 观测输出 ====================

    /**
     * @return HIVE 倾角观测 θ = -φ (弧度); 未解出时为 {@link Double#NaN}
     */
    public double getHiveAngle() {
        return hiveAngle;
    }

    /**
     * @return 本帧 HIVE 倾角的瞬时状态 (无滞回); 无有效观测时为 {@link HiveState#UNKNOWN}
     */
    public HiveState getHiveState() {
        return hiveState;
    }

    /**
     * @return 本帧是否成功解出 HIVE 倾角观测。
     *         外部据此区分"确实水平 (MIDDLE)"与"没看见 / 不可解 (UNKNOWN)"。
     */
    public boolean isHiveEstimated() {
        return hiveEstimated;
    }

    /**
     * @return 姿态交叉校验偏差 (度): 位置通路解出的 φ 与姿态通路推算的 φ 之差。
     *         该值超过 {@link HypParams#hivePitchCheckTolDeg} 的帧被视为 z' 退化帧并丢弃,
     *         因此本值只在 {@link #isHiveEstimated()} 为 true 时 ≤ 容差。
     *         无观测时为 {@link Double#NaN}
     */
    public double getHivePitchCheckErrDeg() {
        return pitchCheckErrDeg;
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

    /** @return 捕获延迟 (毫秒) */
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
