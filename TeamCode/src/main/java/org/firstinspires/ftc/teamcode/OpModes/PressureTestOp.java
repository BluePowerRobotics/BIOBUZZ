package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Processors.Sensors.PressureSwitcher;

/**
 * PressureSwitcher 压敏开关测试程序。
 *
 * 每帧调用 update() 采样，实时显示：
 *   Raw Voltage      归一化原始电压（未滤波）
 *   Filtered Voltage 归一化滤波电压（EMA 输出）
 *   Threshold        当前归一化阈值
 *   Pressed          按压判定结果
 *   Alpha            当前 EMA 平滑系数
 *
 * 调参方式：FTC Dashboard → Config → PressureTestOp
 *   threshold    归一化阈值 [0, 1]，超过该值的滤波电压判定为按压
 *   alpha        EMA 平滑系数 (0, 1]，越接近 1 响应越快、抗噪越差
 *   resetFilter  置 true 会重置一次滤波器状态（仅上升沿生效）
 *
 * 校准建议见 Processors/Sensors/PressurSwitcher.md：
 *   先读取典型压力下的 Filtered Voltage，再把 threshold 调到略低于该值。
 */
@Config
@TeleOp(name = "PressureTestOp", group = "Tests")
public class PressureTestOp extends LinearOpMode {

    /** 模拟输入在机器人配置中的设备名（硬件参数，改动需重新 init） */
    private static final String DEVICE_NAME = "pressure_sensor";


    // ================= FTC Dashboard 可调参数 =================

    /** 归一化压力阈值 [0, 1] */
    public static double threshold = 0.5;

    /** EMA 平滑系数 (0, 1] */
    public static double alpha = 0.8;

    /** 置 true 时重置一次滤波器状态 */
    public static boolean resetFilter = false;


    /** 上一次的 resetFilter 值，用于检测上升沿 */
    private boolean lastResetFilter = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PressureSwitcher pressure = new PressureSwitcher(hardwareMap, DEVICE_NAME);

        telemetry.addData("Device", DEVICE_NAME);
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Tune threshold/alpha/resetFilter in FTC Dashboard > Config");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 采样 + EMA 滤波 + 阈值判定（必须每帧调用）
            pressure.update();

            // 上升沿重置滤波器
            if (resetFilter && !lastResetFilter) {
                pressure.reset();
            }
            lastResetFilter = resetFilter;

            // 每帧应用 Dashboard 参数（带范围保护，避免非法值抛异常）
            pressure.setThreshold(clamp01(threshold));            // threshold ∈ [0, 1]
            pressure.setAlpha(Math.max(0.01, Math.min(1.0, alpha))); // alpha ∈ (0, 1]

            // ---- 遥测 ----
            telemetry.addData("Raw Voltage", "%.3f", pressure.getRawVoltage());
            telemetry.addData("Filtered Voltage", "%.3f", pressure.getVoltage());
            telemetry.addData("Threshold", "%.3f", pressure.getThreshold());
            telemetry.addData("Pressed", pressure.isPressed() ? 1 : 0);
            telemetry.addData("Alpha", "%.3f", pressure.getAlpha());
            telemetry.update();
        }
    }

    /** 把数值限制到 [0, 1] */
    private static double clamp01(double value) {
        if (value < 0) {
            return 0;
        }
        if (value > 1) {
            return 1;
        }
        return value;
    }
}
