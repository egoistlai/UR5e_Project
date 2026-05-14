#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <cmath>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ur_circ_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    move_group.setMaxVelocityScalingFactor(0.1); 
    move_group.setMaxAccelerationScalingFactor(0.1);

    RCLCPP_INFO(node->get_logger(), "获取起点位姿准备画圆...");
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // 圆弧参数设置
    const double radius = 0.1; // 半径 10 厘米
    const double center_x = start_pose.position.x - radius;
    const double center_y = start_pose.position.y;
    const double center_z = start_pose.position.z;

    // 将 360 度 (2π) 切割为 60 个离散路点
    int num_points = 360;
    for (int i = 1; i <= num_points; ++i) {
        double theta = (2.0 * M_PI * i) / num_points;
        geometry_msgs::msg::Pose waypoint = start_pose;
        waypoint.position.x = center_x + radius * std::cos(theta);
        waypoint.position.y = center_y + radius * std::sin(theta);
        waypoint.position.z = center_z;
        waypoints.push_back(waypoint);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.002;      // 插补步长
    const double jump_threshold = 0.0;

    RCLCPP_INFO(node->get_logger(), "正在计算笛卡尔圆弧路径 (CIRC)...");
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.9) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        RCLCPP_INFO(node->get_logger(), "圆弧路径计算完成 (%.2f%%)，开始执行！", fraction * 100.0);
        move_group.execute(plan);
    } else {
        RCLCPP_ERROR(node->get_logger(), "圆弧规划失败！遇到奇异点或超出工作空间。");
    }

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
