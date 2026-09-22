package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * 绘图工具类，用于在 FTC Dashboard 上绘制机器人和相关信息
 * 这是一个工具类，所有方法都是静态的，不允许实例化
 */
public final class Drawing {
    /**
     * 私有构造方法，防止实例化
     */
    private Drawing() {}

    /**
     * 在指定的 Canvas 上绘制机器人
     * 
     * @param c Canvas 对象，用于绘制图形
     * @param t 机器人的位姿信息，包含位置和朝向
     */
    public static void drawRobot(Canvas c, Pose2d t) {
        // 机器人半径，单位为英寸
        final double ROBOT_RADIUS = 9;

        // 设置线条宽度为 1
        c.setStrokeWidth(1);
        // 绘制机器人主体圆形
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        // 计算机器人朝向箭头的中间点
        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        // 计算箭头的起点（圆形边缘）
        Vector2d p1 = t.position.plus(halfv);
        // 计算箭头的终点（超出圆形边缘）
        Vector2d p2 = p1.plus(halfv);
        // 绘制表示机器人朝向的箭头线
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
}
