package org.firstinspires.ftc.teamcode.RoadRunner;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.messages.ThreeDeadWheelInputsMessage;

/**
 * 三轮全向定位器实现
 * 使用三个独立的编码轮（两个平行，一个垂直）来估计机器人的位姿
 * 适用于全向驱动机器人的精确定位
 */
@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    /**
     * 三轮定位器参数类
     * 包含编码器安装位置的配置参数
     */
    public static class Params {
        /**
         * 第一个平行编码器的 y 坐标（以 tick 为单位）
         */
        public double par0YTicks = 0.0;
        
        /**
         * 第二个平行编码器的 y 坐标（以 tick 为单位）
         */
        public double par1YTicks = 1.0;
        
        /**
         * 垂直编码器的 x 坐标（以 tick 为单位）
         */
        public double perpXTicks = 0.0;
    }

    /**
     * 全局参数实例，可通过 FTC Dashboard 进行调整
     */
    public static Params PARAMS = new Params();

    /**
     * 第一个平行编码器
     */
    public final Encoder par0;
    
    /**
     * 第二个平行编码器
     */
    public final Encoder par1;
    
    /**
     * 垂直编码器
     */
    public final Encoder perp;

    /**
     * 每编码器 tick 对应的英寸数
     */
    public final double inPerTick;

    /**
     * 上一次第一个平行编码器的位置
     */
    private int lastPar0Pos;
    
    /**
     * 上一次第二个平行编码器的位置
     */
    private int lastPar1Pos;
    
    /**
     * 上一次垂直编码器的位置
     */
    private int lastPerpPos;
    
    /**
     * 初始化标志
     */
    private boolean initialized;
    
    /**
     * 当前机器人位姿
     */
    private Pose2d pose;

    /**
     * 构造函数
     * 
     * @param hardwareMap 硬件映射对象，用于获取电机
     * @param inPerTick 每编码器 tick 对应的英寸数
     * @param initialPose 初始位姿
     */
    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // 初始化编码器
        // TODO: 确保配置文件中有这些名称的**电机**（或修改名称）
        //   编码器应该插入与命名电机匹配的插槽
        //   参考文档：https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par0")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par1")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

        // TODO: 如果需要，反转编码器方向
        //   par0.setDirection(DcMotorSimple.Direction.REVERSE);

        this.inPerTick = inPerTick;

        // 记录参数
        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);

        pose = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        // 获取编码器读数
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        // 记录编码器输入数据
        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        // 初始化处理
        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        // 计算编码器位置变化
        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        // 三角测量需要两平行轮间距非零；若间距≈0 (Dashboard 误配置) 则跳过本帧, 避免除零产生 Inf/NaN
        double parSep = PARAMS.par0YTicks - PARAMS.par1YTicks;
        if (Math.abs(parSep) < 1e-9) {
            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;
            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        // 计算机器人运动增量
        // 使用三角测量原理计算 x、y 位移和旋转角度
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        // x 方向位移和速度
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (parSep),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (parSep),
                        }).times(inPerTick),
                        // y 方向位移和速度
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (parSep) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (parSep) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                // 旋转角度和角速度
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (parSep),
                        (par0PosVel.velocity - par1PosVel.velocity) / (parSep),
                })
        );

        // 更新上一次编码器位置
        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        // 更新位姿
        pose = pose.plus(twist.value());
        // 返回速度估计
        return twist.velocity().value();
    }
}
