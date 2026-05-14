#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ur_goto_node", node_options);

    // 声明并读取目标位置参数 (如果命令行没传，就用默认值 0.0)
    if (!node->has_parameter("target_x")) node->declare_parameter<double>("target_x", 0.0);
    if (!node->has_parameter("target_y")) node->declare_parameter<double>("target_y", 0.0);
    if (!node->has_parameter("target_z")) node->declare_parameter<double>("target_z", 0.0);

    double tx, ty, tz;
    node->get_parameter("target_x", tx);
    node->get_parameter("target_y", ty);
    node->get_parameter("target_z", tz);

    RCLCPP_INFO(node->get_logger(), "接收到目标位置指令: X=%.3f, Y=%.3f, Z=%.3f", tx, ty, tz);

    // 启动线程
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    // 初始化 MoveGroup
    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    move_group.setMaxVelocityScalingFactor(0.2); // 设为 20% 速度以保证安全
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setPlannerId("PRMstar");
    move_group.setPlanningTime(10.0);
    // 1. 获取当前位姿，为了保留当前的姿态(四元数)不变
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
    geometry_msgs::msg::Pose target_pose = current_pose;

    // 2. 覆盖当前的 XYZ 位置为我们传入的参数
    target_pose.position.x = tx;
    target_pose.position.y = ty;
    target_pose.position.z = tz;

    // 3. 设置目标位姿并进行 PTP 规划
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(node->get_logger(), "正在规划前往目标点...");
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // 4. 执行运动
    if (success) {
        RCLCPP_INFO(node->get_logger(), "规划成功，开始移动！");
        move_group.execute(plan);
        RCLCPP_INFO(node->get_logger(), "成功到达指定坐标点！");
    } else {
        RCLCPP_ERROR(node->get_logger(), "规划失败！目标点不可达，可能超出机械臂工作空间或发生干涉。");
    }

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
