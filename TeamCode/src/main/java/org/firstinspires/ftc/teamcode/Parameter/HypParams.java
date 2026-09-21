package org.firstinspires.ftc.teamcode.Parameter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

@Config
/**
 * 全局超参数配置类
 * 用于集中管理机器人控制系统中的所有超参数，超参应为无需拟合的常量，如工程参数等
 */
//todo：添加球门位置、hive初始状态、hive高度阈值等超参
public class HypParams {
    /**
     * 底盘最大速度（英寸/秒）
     */
    public static double maxV = 2.0;

    /**
     * 底盘最大角速
     */
    public static double maxOmega = Math.PI;

    /**
     * todo:红队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseRed = new Pose2d(-41.3, 55,0);

    /**
     * todo:蓝队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseBlue = new Pose2d(-41.3, -55, 0);
    /**
     * todo:蓝队自动停车姿态（单位：英寸，弧度）
     */
    public static Pose2d StopPoseBlue = new Pose2d(0, -24, Math.PI);
    /**
     * todo:红队自动停车姿态（单位：英寸，弧度）
     */
    public static Pose2d StopPoseRed = new Pose2d(0, 24, Math.PI);

    /**
     * todo:红队重置姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d ResetPoseRed = new Pose2d(63, -60.7, -Math.PI/2);

    /**
     * todo:蓝队重置姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d ResetPoseBlue = new Pose2d(63, 60.7, Math.PI/2);

    /**
     * 初始操控模式标志
     * true=无头模式（场心地坐标系），false=有头模式（机器人坐标系）
     */
    public static boolean InitialUseNoHeadMode = false;

    /**
     * todo:停车时间阈值（单位：毫秒）
     * 自动阶段剩余时间小于此值时执行停车
     */
    public static long PARK_TIME_THRESHOLD_MS = 3000;

    /**
     * 自动阶段总时长（单位：毫秒）
     */
    public static long AUTONOMOUS_DURATION_MS = 30000;

    /**
     * 球门位置（瞄准目标点，仅 x/y 有意义，heading 忽略）：
     * Red_Audience_Up: (17.7, -12.75)
     * Red_Audience_Down: (-17.7, -12.75)
     * Blue_Audience_Up: (17.7, 12.75)
     * Blue_Audience_Down: (-17.7, 12.75)
     */
    public static Pose2d RedAudienceUp = new Pose2d(17.7, -12.75, 0);
    public static Pose2d RedAudienceDown = new Pose2d(-17.7, -12.75, 0);
    public static Pose2d BlueAudienceUp = new Pose2d(17.7, 12.75, 0);
    public static Pose2d BlueAudienceDown = new Pose2d(-17.7, 12.75, 0);

    // ==================== HIVE 单地图解算超参 ====================
    // 单位约定：长度一律英寸（与场地坐标系一致），角度一律度（变量名带 Deg 后缀），
    // 解算内部再转换为弧度（见 MT1Localizer.solveHiveObservation）。

    /**
     * todo:HIVE 枢轴中心相对场地地面的高度 h（英寸）
     * 用于 MT1Localizer.md §3.2 的贴地约束解算；误差会线性传入 x 修正量，必须实机标定
     */
    public static double hivePivotHeightIn = 43.95;

    /**
     * todo:CELL 放下时（HIVE 水平/中间）的倾角上界（度）
     * |θ| ≤ 该值 → MIDDLE；同时也是"稳定位于一侧"的倾角下界，需实机标定
     */
    public static double hiveCellDownAngleDeg = 23.0;

    /**
     * todo:CELL 抬起时（HIVE 稳定位于一侧）的倾角上界（度），必须 > hiveCellDownAngleDeg
     * |θ| 超过该值 → 该帧直接拒绝（该值同时作为三角方程取根筛选的范围），需实机标定
     */
    public static double hiveCellUpAngleDeg = 35.0;

    /**
     * todo:姿态交叉校验容差（度）
     * 由位置解出的 φ 与由 rawPitch 独立推算的 φ 之差超过该值 → 判为 z' 退化帧, 丢弃该帧 HIVE 观测。
     * 前提: 机器人贴地平放（真实 pitch ≈ 0）。需实机标定
     */
    public static double hivePitchCheckTolDeg = 10.0;

    /**
     * todo:HIVE 倾角符号约定
     * true  → θ > 0 表示 AUDIENCE_UP 侧抬升
     * false → θ > 0 表示 AUDIENCE_DOWN 侧抬升
     * 具体对应关系由 fmap 与 HIVE 安装方向决定，需实机标定
     */
    public static boolean hivePositiveAngleIsAudienceUp = Boolean.FALSE;
}