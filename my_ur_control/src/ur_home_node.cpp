#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ur_home_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    
    // 归位动作不需要太快，稳妥为主，设置为 20% 速度
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    RCLCPP_INFO(node->get_logger(), "接收到复位指令，正在规划前往初始默认姿态 (Home/Upright)...");

    // 设定 UR 机械臂经典的 Upright 竖直/初始安全姿态
    // 对应关节: [基座 0°, 肩部 -90°, 肘部 0°, 腕1 -90°, 腕2 0°, 腕3 0°]
    // 这个姿态会让机械臂垂直指向上方，占用空间最小，且绝对不会与桌面/地面干涉
    std::vector<double> home_joint_positions = {0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0};
    
    move_group.setJointValueTarget(home_joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "复位路径规划成功，开始移动...");
        move_group.execute(plan);
        RCLCPP_INFO(node->get_logger(), "机械臂已成功复位至初始状态！");
    } else {
        RCLCPP_ERROR(node->get_logger(), "复位规划失败！请检查是否有障碍物阻挡。");
    }

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
