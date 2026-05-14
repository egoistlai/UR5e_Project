#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ur_ready_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    RCLCPP_INFO(node->get_logger(), "正在前往标准准备姿态 (Joint Space)...");

    // 直接给定 6 个关节的绝对弧度值，强制变成一个健康的“L型”姿态
    // 对应: [基座0, 肩部-90度, 肘部90度, 腕1-90度, 腕2-90度, 腕3 0度]
    std::vector<double> joint_group_positions = {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};
    
    // 使用 setJointValueTarget 替代 setPoseTarget
    // 这样彻底绕过了 IK 求解器，100% 避免奇异解！
    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "关节空间规划成功，开始执行...");
        move_group.execute(plan);
        RCLCPP_INFO(node->get_logger(), "已到达健康姿态！现在可以安全地执行笛卡尔插补了。");
    } else {
        RCLCPP_ERROR(node->get_logger(), "回位规划失败！");
    }

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
