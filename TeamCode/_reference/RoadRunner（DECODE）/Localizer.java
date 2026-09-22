package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

/**
 * 定位器接口，定义了机器人定位系统的核心方法
 * 所有定位实现类（如轮式编码器定位、视觉定位等）都必须实现此接口
 * 
 * 定位器的主要功能：
 * 1. 跟踪机器人在场上的位置和朝向（位姿）
 * 2. 提供机器人的速度估计
 * 3. 允许手动设置初始位姿
 */
public interface Localizer {
    /**
     * 设置机器人的当前位姿
     * 通常在初始化时或需要重置定位时调用
     *
     *
     * @param pose 要设置的位姿，包含 x、y 坐标和朝向角度
     */
    void setPose(Pose2d pose);

    /**
     * 获取当前的位姿估计
     * 
     * @return 当前的位姿估计，包含 x、y 坐标和朝向角度
     * @note 此方法不会更新位姿估计，必须调用 update() 方法来更新
     */
    Pose2d getPose();

    /**
     * 更新定位器的位姿估计
     * 通常在每个循环周期中调用，根据传感器数据计算新的位姿
     * 
     * @return 当前的速度估计，包含线速度和角速度
     */
    PoseVelocity2d update();
}
