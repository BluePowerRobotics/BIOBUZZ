package org.firstinspires.ftc.teamcode.Processors.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.filter.EMA;

/**
 * 压敏开关：把"模拟压力传感器"封装成一个实时布尔开关量。
 *
 * 电路说明：
 *   薄膜压敏电阻(FSR)与电位器串联分压后接到模拟输入口。
 *   压力越大，FSR 阻值越小，采样电压越高；电位器用于调整偏置/量程。
 *   本类用 EMA 对电压滤波，当滤波电压超过阈值时输出 pressed = true。
 *
 * 归一化约定：
 *   内部所有电压与阈值一律除以满量程电压 VOLTAGE_REFERENCE (3.3V)，
 *   归一化到 [0, 1] 区间。因此 getRawVoltage()/getVoltage()/getThreshold()/
 *   setThreshold() 等接口返回/接收的都是归一化数值。
 *
 * 用法（OpMode 主循环中，每帧调用 update()）：
 *   PressureSwitcher pressure = new PressureSwitcher(hardwareMap, "pressure");
 *   while (opModeIsActive()) {
 *       pressure.update();                    // 采样 + 滤波 + 判定
 *       if (pressure.isPressed()) { ... }     // 实时布尔开关量
 *   }
 */
public class PressureSwitcher {

    /** 模拟输入满量程电压（V），所有电压/阈值的归一化基准 */
    public static final double VOLTAGE_REFERENCE = 3.3;

    /** 默认 EMA 平滑系数（越小越平滑、响应越慢） */
    private static final double DEFAULT_ALPHA = 0.8;
    /** 默认归一化压力阈值 */
    private static final double DEFAULT_THRESHOLD = 0.5;

    private final AnalogInput analogInput;
    private final EMA ema;

    /** 归一化压力阈值 [0,1]，滤波电压超过它即判定为按压 */
    private double threshold;

    /** 最近一次 update() 采到的归一化原始电压（未滤波） */
    private double rawVoltage;
    /** 最近一次 update() 的按压判定结果 */
    private boolean pressed;

    /**
     * 使用默认阈值与平滑系数创建压敏开关
     *
     * @param hardwareMap 硬件映射表
     * @param deviceName  模拟输入在配置中的设备名
     */
    public PressureSwitcher(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap.get(AnalogInput.class, deviceName),
                DEFAULT_ALPHA, DEFAULT_THRESHOLD);
    }

    /**
     * 指定阈值与平滑系数创建压敏开关
     *
     * @param hardwareMap 硬件映射表
     * @param deviceName  模拟输入在配置中的设备名
     * @param alpha       EMA 平滑系数，范围 (0, 1]
     * @param threshold   归一化压力阈值，范围 [0, 1]
     */
    public PressureSwitcher(HardwareMap hardwareMap, String deviceName,
                            double alpha, double threshold) {
        this(hardwareMap.get(AnalogInput.class, deviceName), alpha, threshold);
    }

    /**
     * @param analogInput 已获取的模拟输入设备
     * @param alpha       EMA 平滑系数，范围 (0, 1]
     * @param threshold   归一化压力阈值，范围 [0, 1]
     */
    public PressureSwitcher(AnalogInput analogInput, double alpha, double threshold) {
        if (threshold < 0 || threshold > 1) {
            throw new IllegalArgumentException("threshold must be in range [0, 1]");
        }
        this.analogInput = analogInput;
        this.ema = new EMA(alpha);
        this.threshold = threshold;
        this.rawVoltage = 0.0;
        this.pressed = false;
    }

    /**
     * 采样一次模拟输入：读取归一化原始电压 → EMA 滤波 → 更新按压判定。
     * 应在 OpMode 主循环中每帧调用。
     *
     * @return 本次采样后是否按压（滤波电压超过阈值）
     */
    public boolean update() {
        double raw = normalize(analogInput.getVoltage());
        rawVoltage = raw;
        pressed = ema.update(raw) > threshold;
        return pressed;
    }

    /**
     * 当前是否按压（电压超过阈值），由最近一次 update() 计算得到
     */
    public boolean isPressed() {
        return pressed;
    }

    /**
     * 归一化原始电压（未滤波），来自最近一次 update()，范围 [0,1]
     */
    public double getRawVoltage() {
        return rawVoltage;
    }

    /**
     * 归一化滤波电压（EMA 输出），来自最近一次 update()，范围 [0,1]
     */
    public double getVoltage() {
        return ema.getFilteredValue();
    }

    /**
     * 调节归一化压力阈值
     *
     * @param threshold 归一化阈值，范围 [0, 1]
     */
    public void setThreshold(double threshold) {
        if (threshold < 0 || threshold > 1) {
            throw new IllegalArgumentException("threshold must be in range [0, 1]");
        }
        this.threshold = threshold;
    }

    /**
     * 获取当前归一化压力阈值
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * 调节 EMA 平滑系数，实时生效
     *
     * @param alpha 平滑系数，范围 (0, 1]
     */
    public void setAlpha(double alpha) {
        ema.setAlpha(alpha);
    }

    /**
     * 获取当前 EMA 平滑系数
     */
    public double getAlpha() {
        return ema.getAlpha();
    }

    /**
     * 重置滤波器状态与判定结果（阈值/alpha 保持不变）
     */
    public void reset() {
        ema.reset();
        rawVoltage = 0.0;
        pressed = false;
    }

    /**
     * 把模拟输入电压(V)归一化到 [0,1]，无效读数按 0 处理
     */
    private static double normalize(double voltageVolts) {
        double v = voltageVolts / VOLTAGE_REFERENCE;
        if (Double.isNaN(v) || v < 0) {
            return 0.0;
        }
        return Math.min(v, 1.0);
    }
}
