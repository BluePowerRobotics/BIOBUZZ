package org.firstinspires.ftc.teamcode.utility.filter.UKF;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.simple.SimpleMatrix;

/**
 * 3维无迹卡尔曼滤波器 (Unscented Kalman Filter)，用于 Pinpoint(里程计速度) 和
 * Limelight(视觉位姿) 的融合定位。
 *
 * <p>与 EKF 的区别在于 UKF 使用 sigma 点无迹变换替代雅可比线性化，
 * 通过 2n+1 个确定性采样点直接传播非线性分布，精确到三阶矩。
 *
 * <p>状态向量: [x, y, theta] (全局坐标系下的机器人位姿)
 * <br>控制输入(predict): Pinpoint 提供的机器人局部速度 [vx, vy, omega]
 * <br>观测(update): Limelight 提供的全局位姿 [x, y, theta]
 *
 * <p>UKF 参数: α=1, β=2, κ=0 → λ=0, 7 sigma 点 (2n+1)
 *
 * <p>参考:
 * <ul>
 *   <li>Wan & van der Merwe (2001), "The Unscented Kalman Filter"
 *   <li>Julier & Uhlmann (1997), "A New Extension of the Kalman Filter
 *       to Nonlinear Systems"
 * </ul>
 */
public class UKF {

    // ==================== UKF 参数 (n=3) ====================

    /** 状态维度 */
    private static final int N = 3;

    /** Sigma 点散布参数 α = 1, 控制 sigma 点到均值的距离 */
    private static final double ALPHA = 1.0;

    /** 最优参数 β = 2 (高斯分布) */
    private static final double BETA = 2.0;

    /** 次级缩放参数 κ = 0 */
    private static final double KAPPA = 0.0;

    /** λ = α²(n+κ) - n = 1·3 - 3 = 0 */
    private final double lambda;

    /** n + λ = 3 + 0 = 3 */
    private final double nPlusLambda;

    /** Sigma 点总数 = 2n+1 = 7 */
    private static final int NUM_SIGMA = 2 * N + 1;

    /** Sigma 点均值权重 (长度 7) */
    private final double[] wm;

    /** Sigma 点协方差权重 (长度 7) */
    private final double[] wc;

    // ==================== Cholesky 正则化 ====================

    /** 第一级 Cholesky 正则化 ε */
    private static final double CHOL_EPS_1 = 1e-6;

    /** 第二级 Cholesky 正则化 ε */
    private static final double CHOL_EPS_2 = 1e-3;

    // ==================== 滤波器状态 ====================

    /** 状态向量 [x, y, theta] (3x1) */
    private SimpleMatrix state;

    /** 状态协方差矩阵 (3x3) */
    private SimpleMatrix P;

    /** 过程噪声协方差矩阵 Q (3x3) */
    private SimpleMatrix Q;

    /** 观测噪声协方差矩阵 R (3x3) */
    private SimpleMatrix R;

    /** 观测矩阵 H = I (3x3)，因为观测直接就是状态 */
    private final SimpleMatrix H = SimpleMatrix.identity(N);

    /** 上一次 predict 的时间戳，用于计算 dt */
    private Double lastPredictTime = null;

    /** 上一次 update 的时间戳，用于拒绝过时观测 */
    private Double lastUpdateTime = null;

    // ==================== 构造 ====================

    /**
     * @param initialX      初始 x (英寸)
     * @param initialY      初始 y (英寸)
     * @param initialTheta  初始朝向 (弧度)
     */
    public UKF(double initialX, double initialY, double initialTheta) {
        // ---- 状态初始化 (与 EKF 一致) ----
        state = new SimpleMatrix(new double[][]{
                {initialX},
                {initialY},
                {initialTheta}
        });

        // 初始协方差：位置中等不确定，角度不确定较小
        P = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.01}
        });

        // 默认过程噪声 (dt 相关，实际 predict 中会乘以 dt)
        Q = new SimpleMatrix(new double[][]{
                {0.002, 0,     0     },
                {0,     0.002, 0     },
                {0,     0,     0.002 }
        });

        // 默认观测噪声（视觉测量通常角度噪声比位置噪声大）
        R = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.05}
        });

        // ---- UKF 权重计算 ----
        this.lambda = ALPHA * ALPHA * (N + KAPPA) - N;        // = 0
        this.nPlusLambda = N + lambda;                         // = 3

        this.wm = new double[NUM_SIGMA];
        this.wc = new double[NUM_SIGMA];

        wm[0] = lambda / nPlusLambda;                          // = 0
        wc[0] = lambda / nPlusLambda + (1 - ALPHA * ALPHA + BETA); // = 2

        double commonWeight = 1.0 / (2.0 * nPlusLambda);       // = 1/6
        for (int i = 1; i < NUM_SIGMA; i++) {
            wm[i] = commonWeight;
            wc[i] = commonWeight;
        }
    }

    // ==================== 核心滤波 ====================

    /**
     * 预测步骤 —— 每帧调用，接收 Pinpoint 提供的机器人局部速度。
     * 使用无迹变换 (Unscented Transform) 传播状态分布。
     *
     * @param vx        机器人局部坐标系下的 x 方向速度 (in/s)
     * @param vy        机器人局部坐标系下的 y 方向速度 (in/s)
     * @param omega     机器人角速度 (rad/s)
     * @param timestamp 当前时间戳 (秒)
     */
    public void predict(double vx, double vy, double omega, double timestamp) {
        // 首次调用：仅记录时间戳，不做预测
        if (lastPredictTime == null) {
            lastPredictTime = timestamp;
            return;
        }

        double dt = timestamp - lastPredictTime;
        lastPredictTime = timestamp;

        // 异常 dt 保护
        if (dt <= 0 || dt > 1.0) {
            return;
        }

        // ---- 1. 生成 sigma 点 ----
        double[][] sigmaPoints = generateSigmaPoints();

        // ---- 2. 传播每个 sigma 点通过非线性运动模型 ----
        // 运动模型: x' = x + dt * (vx * cosθ - vy * sinθ)
        //           y' = y + dt * (vx * sinθ + vy * cosθ)
        //           θ' = θ + dt * ω
        double[][] propagated = new double[NUM_SIGMA][N];
        for (int i = 0; i < NUM_SIGMA; i++) {
            double theta = sigmaPoints[i][2];
            propagated[i][0] = sigmaPoints[i][0] + dt * (vx * Math.cos(theta) - vy * Math.sin(theta));
            propagated[i][1] = sigmaPoints[i][1] + dt * (vx * Math.sin(theta) + vy * Math.cos(theta));
            propagated[i][2] = sigmaPoints[i][2] + dt * omega;
        }

        // ---- 3. 计算加权均值 (圆形均值处理角度) ----
        double xMean = 0.0, yMean = 0.0;
        double sumSinTheta = 0.0, sumCosTheta = 0.0;

        for (int i = 0; i < NUM_SIGMA; i++) {
            xMean += wm[i] * propagated[i][0];
            yMean += wm[i] * propagated[i][1];
            sumSinTheta += wm[i] * Math.sin(propagated[i][2]);
            sumCosTheta += wm[i] * Math.cos(propagated[i][2]);
        }
        double thetaMean = Math.atan2(sumSinTheta, sumCosTheta);

        // ---- 4. 计算加权协方差 P = Σ wc[i]·(Xᵢ - x̂)(Xᵢ - x̂)ᵀ + Q·dt ----
        SimpleMatrix P_new = new SimpleMatrix(N, N);
        for (int i = 0; i < NUM_SIGMA; i++) {
            double dx = propagated[i][0] - xMean;
            double dy = propagated[i][1] - yMean;
            double dtheta = normalizeAngle(propagated[i][2] - thetaMean);

            double w = wc[i];
            P_new.set(0, 0, P_new.get(0, 0) + w * dx * dx);
            P_new.set(0, 1, P_new.get(0, 1) + w * dx * dy);
            P_new.set(0, 2, P_new.get(0, 2) + w * dx * dtheta);
            P_new.set(1, 0, P_new.get(1, 0) + w * dy * dx);
            P_new.set(1, 1, P_new.get(1, 1) + w * dy * dy);
            P_new.set(1, 2, P_new.get(1, 2) + w * dy * dtheta);
            P_new.set(2, 0, P_new.get(2, 0) + w * dtheta * dx);
            P_new.set(2, 1, P_new.get(2, 1) + w * dtheta * dy);
            P_new.set(2, 2, P_new.get(2, 2) + w * dtheta * dtheta);
        }

        // 加上过程噪声 Q·dt (体现过程噪声随时间累积)
        P = P_new.plus(Q.scale(dt));

        // ---- 5. 更新状态 ----
        state = new SimpleMatrix(new double[][]{
                {xMean},
                {yMean},
                {thetaMean}
        });
    }

    /**
     * 更新步骤 —— 仅在 Limelight 检测有效时调用，接收视觉全局位姿。
     * 使用 Joseph 形式更新协方差以保证数值稳定性。
     *
     * @param xMeas      Limelight 测得的全局 x (英寸)
     * @param yMeas      Limelight 测得的全局 y (英寸)
     * @param thetaMeas  Limelight 测得的全局朝向 (弧度)
     * @param timestamp  观测时间戳 (秒)
     */
    public void update(double xMeas, double yMeas, double thetaMeas, double timestamp) {
        // 拒绝过时观测
        if (lastUpdateTime != null && timestamp <= lastUpdateTime) {
            return;
        }
        lastUpdateTime = timestamp;

        // ---- 1. 观测向量 z = [xMeas, yMeas, thetaMeas]ᵀ ----
        SimpleMatrix z = new SimpleMatrix(new double[][]{
                {xMeas},
                {yMeas},
                {thetaMeas}
        });

        // ---- 2. 生成 sigma 点 (基于当前状态和协方差) ----
        double[][] sigmaPoints = generateSigmaPoints();

        // ---- 3. 将 sigma 点通过观测模型 (H = I, 观测直接就是状态) ----
        // 因此 Z_i = H · X_i = X_i，无需额外变换
        double[][] zSigma = sigmaPoints;  // 共享引用，仅读不写

        // ---- 4. 计算预测观测均值 ẑ = Σ wm[i]·Z_i ----
        // 等同于当前状态均值 (因为 H=I)
        double zxMean = 0.0, zyMean = 0.0;
        double sumSinZ = 0.0, sumCosZ = 0.0;
        for (int i = 0; i < NUM_SIGMA; i++) {
            zxMean += wm[i] * zSigma[i][0];
            zyMean += wm[i] * zSigma[i][1];
            sumSinZ += wm[i] * Math.sin(zSigma[i][2]);
            sumCosZ += wm[i] * Math.cos(zSigma[i][2]);
        }
        double zThetaMean = Math.atan2(sumSinZ, sumCosZ);

        // ---- 5. 新息 y_innov = z - ẑ ----
        SimpleMatrix yInnov = new SimpleMatrix(new double[][]{
                {z.get(0, 0) - zxMean},
                {z.get(1, 0) - zyMean},
                {normalizeAngle(z.get(2, 0) - zThetaMean)}
        });

        // ---- 6. 新息协方差 S = Σ wc[i]·(Z_i - ẑ)(Z_i - ẑ)ᵀ + R ----
        SimpleMatrix S = computeCovarianceFromSigma(zSigma, zxMean, zyMean, zThetaMean)
                .plus(R);

        // ---- 7. 交叉协方差 Pxz = Σ wc[i]·(X_i - x̂)(Z_i - ẑ)ᵀ ----
        double xMean = state.get(0, 0);
        double yMean = state.get(1, 0);
        double thetaMean = state.get(2, 0);

        SimpleMatrix Pxz = computeCrossCovariance(
                sigmaPoints, xMean, yMean, thetaMean,
                zSigma, zxMean, zyMean, zThetaMean
        );

        // ---- 8. 卡尔曼增益 K = Pxz · S⁻¹ ----
        SimpleMatrix K = Pxz.mult(safeInvert(S));

        // ---- 9. 状态更新 x = x + K · y_innov ----
        state = state.plus(K.mult(yInnov));
        state.set(2, 0, normalizeAngle(state.get(2, 0)));

        // ---- 10. Joseph 形式协方差更新 P = P - K·S·Kᵀ ----
        // Joseph 形式保证对称性和正半定性
        P = P.minus(K.mult(S).mult(K.transpose()));

        // 对称性强制 (消除浮点累积误差)
        P = P.plus(P.transpose()).scale(0.5);
    }

    /**
     * 视觉观测门控 —— 计算马氏距离 (Mahalanobis distance) 判断该视觉观测是否为离群点。
     *
     * <p>观测模型 H = I，因此新息 {@code y = z - x̂}，新息协方差 {@code S = P + R}。
     * 远离 AprilTag 时视觉误差极大，直接用其更新会将滤波器带偏；通过马氏距离门控拒绝离群观测。
     *
     * @param xMeas         视觉全局 x (英寸)
     * @param yMeas         视觉全局 y (英寸)
     * @param thetaMeas     视觉全局朝向 (弧度)
     * @param gateThreshold 马氏距离门控阈值 (无量纲)
     * @return true 表示测量可接受 (马氏距离 ≤ gateThreshold)
     */
    public boolean gateVision(double xMeas, double yMeas, double thetaMeas, double gateThreshold) {
        SimpleMatrix z = new SimpleMatrix(new double[][]{
                {xMeas},
                {yMeas},
                {thetaMeas}
        });
        SimpleMatrix innov = z.minus(state);
        innov.set(2, 0, normalizeAngle(innov.get(2, 0)));

        // H = I → S = P + R
        SimpleMatrix S = P.plus(R);
        SimpleMatrix d2 = innov.transpose().mult(safeInvert(S)).mult(innov);
        return d2.get(0, 0) <= gateThreshold * gateThreshold;
    }

    // ==================== 输出 ====================

    /**
     * @return 当前融合位姿 double[3] = {x, y, theta} (英寸, 英寸, 弧度)
     */
    public double[] getPose() {
        return new double[]{
                state.get(0, 0),
                state.get(1, 0),
                state.get(2, 0)
        };
    }

    /**
     * @return 状态协方差矩阵的对角线元素 {var_x, var_y, var_theta}
     */
    public double[] getCovarianceDiag() {
        return new double[]{
                P.get(0, 0),
                P.get(1, 1),
                P.get(2, 2)
        };
    }

    /**
     * @return 当前过程噪声矩阵 Q 的对角线元素 {q_x, q_y, q_theta}
     */
    public double[] getQDiag() {
        return new double[]{
                Q.get(0, 0),
                Q.get(1, 1),
                Q.get(2, 2)
        };
    }

    /**
     * @return 当前观测噪声矩阵 R 的对角线元素 {r_x, r_y, r_theta}
     */
    public double[] getRDiag() {
        return new double[]{
                R.get(0, 0),
                R.get(1, 1),
                R.get(2, 2)
        };
    }

    // ==================== 噪声参数调整 ====================

    /**
     * 实时设置过程噪声协方差矩阵 Q 的对角线值。
     *
     * @param qx       x 方向过程噪声 (in²/s)
     * @param qy       y 方向过程噪声 (in²/s)
     * @param qtheta   theta 方向过程噪声 (rad²/s)
     */
    public void setQ(double qx, double qy, double qtheta) {
        Q = new SimpleMatrix(new double[][]{
                {qx, 0,  0     },
                {0,  qy, 0     },
                {0,  0,  qtheta}
        });
    }

    /**
     * 直接设置完整的 Q 矩阵 (3x3)。
     */
    public void setQ(SimpleMatrix Q) {
        this.Q = Q;
    }

    /**
     * 实时设置观测噪声协方差矩阵 R 的对角线值。
     *
     * @param rx       x 方向观测噪声 (in²)
     * @param ry       y 方向观测噪声 (in²)
     * @param rtheta   theta 方向观测噪声 (rad²)
     */
    public void setR(double rx, double ry, double rtheta) {
        R = new SimpleMatrix(new double[][]{
                {rx, 0,  0     },
                {0,  ry, 0     },
                {0,  0,  rtheta}
        });
    }

    /**
     * 直接设置完整的 R 矩阵 (3x3)。
     */
    public void setR(SimpleMatrix R) {
        this.R = R;
    }

    // ==================== 重置 ====================

    /**
     * 重置滤波器到新的初始位姿，同时清空时间戳。
     */
    public void reset(double x, double y, double theta) {
        state = new SimpleMatrix(new double[][]{
                {x},
                {y},
                {theta}
        });
        P = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.01}
        });
        lastPredictTime = null;
        lastUpdateTime = null;
    }

    // ==================== Sigma 点生成 ====================

    /**
     * 基于当前状态和协方差生成 2n+1 个 sigma 点。
     *
     * <p>算法:
     * <ol>
     *   <li>计算缩放协方差 S = (n+λ)·P</li>
     *   <li>Cholesky 分解 S = L·Lᵀ (含正则化降级)</li>
     *   <li>sigma₀ = state</li>
     *   <li>sigmaᵢ = state + L_colᵢ (i=1..n)</li>
     *   <li>sigmaᵢ₊ₙ = state - L_colᵢ (i=1..n)</li>
     * </ol>
     *
     * @return double[7][3] sigma 点数组
     */
    private double[][] generateSigmaPoints() {
        double[][] sigmaPoints = new double[NUM_SIGMA][N];

        // 缩放协方差: S = (n+λ) * P = 3 * P
        DMatrixRMaj L = choleskyDecompose(P, nPlusLambda);

        double x0 = state.get(0, 0);
        double y0 = state.get(1, 0);
        double t0 = state.get(2, 0);

        // Sigma 点 0: 均值本身
        sigmaPoints[0][0] = x0;
        sigmaPoints[0][1] = y0;
        sigmaPoints[0][2] = t0;

        // Sigma 点 1..N: state + L_col
        // Sigma 点 N+1..2N: state - L_col
        for (int j = 0; j < N; j++) {
            double lj0 = L.get(0, j);
            double lj1 = L.get(1, j);
            double lj2 = L.get(2, j);

            // +L_col
            sigmaPoints[j + 1][0] = x0 + lj0;
            sigmaPoints[j + 1][1] = y0 + lj1;
            sigmaPoints[j + 1][2] = t0 + lj2;

            // -L_col
            sigmaPoints[j + N + 1][0] = x0 - lj0;
            sigmaPoints[j + N + 1][1] = y0 - lj1;
            sigmaPoints[j + N + 1][2] = t0 - lj2;
        }

        return sigmaPoints;
    }

    // ==================== Cholesky 分解 (含正则化降级) ====================

    /**
     * 计算缩放协方差矩阵的 Cholesky 分解 L，使得 L·Lᵀ = scale·P。
     *
     * <p>三级降级策略:
     * <ol>
     *   <li>直接分解 scale·P</li>
     *   <li>失败 → scale·P + 1e-6·I，重试</li>
     *   <li>仍失败 → scale·P + 1e-3·I，重试</li>
     *   <li>仍失败 → 返回对角 fallback sqrt(trace(scale·P)/3)·I</li>
     * </ol>
     *
     * @param Pmat  协方差矩阵 P
     * @param scale 缩放因子 (n+λ)
     * @return Cholesky 下三角矩阵 L (3x3)
     */
    private DMatrixRMaj choleskyDecompose(SimpleMatrix Pmat, double scale) {
        CholeskyDecomposition_F64<DMatrixRMaj> chol =
                DecompositionFactory_DDRM.chol(N, true);  // true = lower triangular

        // 尝试 1: 直接分解
        DMatrixRMaj S = toDenseScaled(Pmat, scale);
        if (chol.decompose(S)) {
            return extractL(chol);
        }

        // 尝试 2: 加小正则化 ε₁
        S = toDenseScaledRegularized(Pmat, scale, CHOL_EPS_1);
        if (chol.decompose(S)) {
            return extractL(chol);
        }

        // 尝试 3: 加大正则化 ε₂
        S = toDenseScaledRegularized(Pmat, scale, CHOL_EPS_2);
        if (chol.decompose(S)) {
            return extractL(chol);
        }

        // 降级 fallback: 对角矩阵 sqrt(trace(scale·P)/3)·I
        double trace = scale * (Pmat.get(0, 0) + Pmat.get(1, 1) + Pmat.get(2, 2));
        double diag = Math.sqrt(Math.max(trace / N, 1e-12));

        DMatrixRMaj L = new DMatrixRMaj(N, N);
        L.set(0, 0, diag);
        L.set(1, 1, diag);
        L.set(2, 2, diag);
        return L;
    }

    /** 构造 scale·P，复制为 DMatrixRMaj */
    private DMatrixRMaj toDenseScaled(SimpleMatrix Pmat, double scale) {
        DMatrixRMaj result = new DMatrixRMaj(N, N);
        result.set(0, 0, scale * Pmat.get(0, 0));
        result.set(0, 1, scale * Pmat.get(0, 1));
        result.set(0, 2, scale * Pmat.get(0, 2));
        result.set(1, 0, scale * Pmat.get(1, 0));
        result.set(1, 1, scale * Pmat.get(1, 1));
        result.set(1, 2, scale * Pmat.get(1, 2));
        result.set(2, 0, scale * Pmat.get(2, 0));
        result.set(2, 1, scale * Pmat.get(2, 1));
        result.set(2, 2, scale * Pmat.get(2, 2));
        return result;
    }

    /** 构造 scale·P + ε·I，复制为 DMatrixRMaj */
    private DMatrixRMaj toDenseScaledRegularized(SimpleMatrix Pmat, double scale, double eps) {
        DMatrixRMaj result = new DMatrixRMaj(N, N);
        result.set(0, 0, scale * Pmat.get(0, 0) + eps);
        result.set(0, 1, scale * Pmat.get(0, 1));
        result.set(0, 2, scale * Pmat.get(0, 2));
        result.set(1, 0, scale * Pmat.get(1, 0));
        result.set(1, 1, scale * Pmat.get(1, 1) + eps);
        result.set(1, 2, scale * Pmat.get(1, 2));
        result.set(2, 0, scale * Pmat.get(2, 0));
        result.set(2, 1, scale * Pmat.get(2, 1));
        result.set(2, 2, scale * Pmat.get(2, 2) + eps);
        return result;
    }

    /** 从 Cholesky 分解中提取下三角矩阵 L */
    private DMatrixRMaj extractL(CholeskyDecomposition_F64<DMatrixRMaj> chol) {
        DMatrixRMaj L = new DMatrixRMaj(N, N);
        chol.getT(L);
        return L;
    }

    // ==================== 协方差计算辅助 ====================

    /**
     * 从 sigma 点计算加权协方差矩阵。
     * Σ wc[i]·(X_i - mean)(X_i - mean)ᵀ
     */
    private SimpleMatrix computeCovarianceFromSigma(
            double[][] sigma, double mx, double my, double mtheta) {
        SimpleMatrix result = new SimpleMatrix(N, N);
        for (int i = 0; i < NUM_SIGMA; i++) {
            double dx = sigma[i][0] - mx;
            double dy = sigma[i][1] - my;
            double dt = normalizeAngle(sigma[i][2] - mtheta);
            double w = wc[i];

            result.set(0, 0, result.get(0, 0) + w * dx * dx);
            result.set(0, 1, result.get(0, 1) + w * dx * dy);
            result.set(0, 2, result.get(0, 2) + w * dx * dt);
            result.set(1, 0, result.get(1, 0) + w * dy * dx);
            result.set(1, 1, result.get(1, 1) + w * dy * dy);
            result.set(1, 2, result.get(1, 2) + w * dy * dt);
            result.set(2, 0, result.get(2, 0) + w * dt * dx);
            result.set(2, 1, result.get(2, 1) + w * dt * dy);
            result.set(2, 2, result.get(2, 2) + w * dt * dt);
        }
        return result;
    }

    /**
     * 计算交叉协方差矩阵 Pxz。
     * Σ wc[i]·(X_i - xMean)(Z_i - zMean)ᵀ
     */
    private SimpleMatrix computeCrossCovariance(
            double[][] xSigma, double xm, double ym, double tm,
            double[][] zSigma, double zxm, double zym, double ztm) {
        SimpleMatrix result = new SimpleMatrix(N, N);
        for (int i = 0; i < NUM_SIGMA; i++) {
            double dx = xSigma[i][0] - xm;
            double dy = xSigma[i][1] - ym;
            double dt = normalizeAngle(xSigma[i][2] - tm);

            double dzx = zSigma[i][0] - zxm;
            double dzy = zSigma[i][1] - zym;
            double dzt = normalizeAngle(zSigma[i][2] - ztm);

            double w = wc[i];

            result.set(0, 0, result.get(0, 0) + w * dx * dzx);
            result.set(0, 1, result.get(0, 1) + w * dx * dzy);
            result.set(0, 2, result.get(0, 2) + w * dx * dzt);
            result.set(1, 0, result.get(1, 0) + w * dy * dzx);
            result.set(1, 1, result.get(1, 1) + w * dy * dzy);
            result.set(1, 2, result.get(1, 2) + w * dy * dzt);
            result.set(2, 0, result.get(2, 0) + w * dt * dzx);
            result.set(2, 1, result.get(2, 1) + w * dt * dzy);
            result.set(2, 2, result.get(2, 2) + w * dt * dzt);
        }
        return result;
    }

    // ==================== 内部工具 ====================

    /**
     * 将角度归一化到 [-π, π] 范围。
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    /**
     * 安全求逆 —— S = Pzz + R 在 R 配置过小或数值退化时可能奇异。
     * 求逆失败时对角加扰动 (jitter) 后重试, 避免抛出异常或产生爆炸增益。
     */
    private SimpleMatrix safeInvert(SimpleMatrix m) {
        try {
            return m.invert();
        } catch (RuntimeException e) {
            SimpleMatrix jittered = m.plus(SimpleMatrix.identity(m.numRows()).scale(1e-9));
            return jittered.invert();
        }
    }
}