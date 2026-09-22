package org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utility.PID.PIDSVAController;
import org.firstinspires.ftc.teamcode.utility.PID.SlotConfig;

/**
 * 原型发射机构（飞轮）控制器 —— 软件速度闭环。
 *
 * <p>与 {@code Sweeper} 采用 Hub 固件速度闭环不同，这里用软件闭环：
 * 电机置于 {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}，{@code setPower} 直接控制功率，
 * 每帧用 {@code utility.PID} 的 {@link PIDSVAController} 按
 * (目标转速 - 实际转速) 计算输出。原型阶段选软件闭环是为了能在 FTC Dashboard 上
 * 实时整定 kP/kI/kD/kS/kV/kA，并观察超调与稳态误差。
 *
 * <p><b>单位约定</b>：转速统一使用 {@link DcMotorEx#getVelocity()} 的默认单位 tick/s。
 * kV 的量纲为 (功率)/(tick/s)，需实机标定。
 */
@Config
public class PrototypeShooter {

    /** 电机名固定为 "shooter" */
    private static final String MOTOR_NAME = "shooter";

    public DcMotorEx motor;

    private Telemetry telemetry;

    // ==================== 可调参数 (FTC Dashboard 实时生效) ====================

    /** 速度环 P 增益 */
    public static double kP = 0.0004;
    /** 速度环 I 增益 */
    public static double kI = 0.0002;
    /** 速度环 D 增益 */
    public static double kD = 0.0;

    /** 积分上限（防止积分饱和） */
    public static double maxI = 1.0;
    /** I 分离阈值 (tick/s)：误差小于该值才累加积分，超出则清零积分 */
    public static double iZone = 3000.0;

    /** 静态摩擦前馈系数（克服死区） */
    public static double kS = 0.0;
    /** 速度前馈系数，量纲为 (功率)/(tick/s) */
    public static double kV = 0.0002;
    /** 加速度前馈系数（当前按 0 加速度使用） */
    public static double kA = 0.0;

    /** 输出下限（功率） */
    public static double outputMin = -1.0;
    /** 输出上限（功率） */
    public static double outputMax = 1.0;

    /** 电机方向：由飞轮装配方向决定 */
    public static boolean motorReversed = false;

    /** 目标转速 (tick/s)，0 表示停转 */
    public static int targetVelocity = 0;

    // ==================== 运行时状态 ====================

    private final PIDSVAController controller = new PIDSVAController();
    /** 复用同一个 slot：每帧用 withXxx 刷新参数后 resetSlot（见 SlotConfig 类注释） */
    private final SlotConfig slot = new SlotConfig();

    /** 上一次 update 的时间戳 (秒) */
    private double lastTime;

    /** 最近一次下发的功率（供遥测） */
    private double power = 0;

    public PrototypeShooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        motor.setDirection(motorReversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        // 软件闭环：关闭固件速度环，让 setPower 直接控制功率
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // 飞轮惯性大，停转后让其自然滑行
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controller.withSlot0(new SlotConfig()
                .withKP(kP).withKI(kI).withKD(kD)
                .withMaxI(maxI).withIZone(iZone)
                .withKS(kS).withKV(kV).withKA(kA)
                .withOutputLimits(outputMin, outputMax));

        lastTime = now();
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    /**
     * 设置目标转速。
     *
     * @param velocity 目标转速 (tick/s)，0 表示停转
     */
    public void setTargetVelocity(int velocity) {
        targetVelocity = velocity;
    }

    /** 停转（目标转速置 0，下一帧 update 会主动收油门） */
    public void stop() {
        targetVelocity = 0;
    }

    /**
     * 每帧调用一次，执行软件速度闭环。
     */
    public void update() {
        double currentTime = now();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        // 首帧或长时间停顿后 dt 异常，用标称循环周期兜底，避免微分项发散
        if (dt <= 0 || dt > 1.0) {
            dt = 0.02;
        }

        double currentVelocity = motor.getVelocity();

        // 每帧刷新参数，使 Dashboard 上的改动即时生效
        // （SlotConfig 要求复用同一实例、用 withXxx 修改后 resetSlot，
        //   控制器的积分/微分状态保存在 controller 内，不受影响）
        slot.withKP(kP).withKI(kI).withKD(kD)
                .withMaxI(maxI).withIZone(iZone)
                .withKS(kS).withKV(kV).withKA(kA)
                .withOutputLimits(outputMin, outputMax);
        controller.resetSlot(slot);

        // VelCycle = true：目标转速作为前馈速度项，由 kS/kV 换算成基础功率
        power = controller.calculate(targetVelocity, currentVelocity, dt, true);
        motor.setPower(power);
    }

    // ==================== 读取 ====================

    /** @return 当前转速 (tick/s) */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /** @return 最近一次下发的功率 */
    public double getPower() {
        return power;
    }

    /** @return 目标转速 (tick/s) */
    public int getTargetVelocity() {
        return targetVelocity;
    }

    /** @return 速度误差 (tick/s) */
    public double getError() {
        return targetVelocity - getVelocity();
    }

    /** @return 电机电流 (A) */
    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    /** 输出遥测（调用方需自行 telemetry.update()） */
    public void setTelemetry() {
        telemetry.addData("Shooter target (tick/s)", targetVelocity);
        telemetry.addData("Shooter velocity (tick/s)", "%.0f", getVelocity());
        telemetry.addData("Shooter error (tick/s)", "%.0f", getError());
        telemetry.addData("Shooter power", "%.3f", power);
        telemetry.addData("Shooter current (A)", "%.2f", getCurrent());
    }
}
