#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <vector>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ur_lin_node", node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    static const std::string PLANNING_GROUP = "ur_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // 降低运行速度，保证直线平稳
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    RCLCPP_INFO(node->get_logger(), "获取当前末端位姿作为起点...");
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

    // 1. 创建一个路点集合 (Waypoints)
    std::vector<geometry_msgs::msg::Pose> waypoints;
    
    // 2. 计算目标位姿 (Z 向上 15cm，X 向前 10cm)
    geometry_msgs::msg::Pose target_pose = start_pose;
    target_pose.position.z += 0.15;
    target_pose.position.x += 0.10;
    
    // 将目标点加入路点集合中
    // 如果你想走多段直线，可以继续 push_back 更多点
    waypoints.push_back(target_pose);

    // 3. 设置笛卡尔路径插补参数
    const double eef_step = 0.01;      // 步长：每隔 0.01 米 (1厘米) 插入一个点
    const double jump_threshold = 0.0; // 跳跃阈值：0 代表禁用。防止机械臂在插补时发生关节突变(跳跃)

    moveit_msgs::msg::RobotTrajectory trajectory;
    
    RCLCPP_INFO(node->get_logger(), "正在计算笛卡尔直线路径 (Cartesian Path)...");
    
    // 4. 调用 computeCartesianPath 进行直线轨迹计算
    // 返回值 fraction 代表规划成功的比例 (0.0 到 1.0)
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(node->get_logger(), "直线路径计算完成，成功率: %.2f%%", fraction * 100.0);

    // 5. 如果成功率高于 90%，则认为可以安全执行
    if (fraction > 0.9) {
        RCLCPP_INFO(node->get_logger(), "路径有效！开始沿着直线移动...");
        
        // 将计算出的轨迹打包进 Plan 对象中
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = trajectory;
        
        // 执行运动
        move_group.execute(my_plan);
        RCLCPP_INFO(node->get_logger(), "直线运动执行完毕！");
    } else {
        RCLCPP_ERROR(node->get_logger(), "直线规划失败，只有 %.2f%% 的路径可达！可能存在碰撞或奇点。", fraction * 100.0);
    }

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
