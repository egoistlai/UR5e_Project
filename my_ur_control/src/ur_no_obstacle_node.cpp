#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ur_no_obstacle_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    // 1. 创建一个发布者，用于向 RViz 发送视觉虚拟模型 (不参与碰撞)
    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // 构造那个 10x10x60 厘米的虚拟柱子
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = node->now();
    marker.ns = "ghost_obstacle";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // 位置和真实避障代码里的一模一样
    marker.pose.position.x = 0.35;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.30;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.10;
    marker.scale.y = 0.10;
    marker.scale.z = 0.60;
    
    // 设置为半透明的红色，方便你截图时能看透里面的机械臂
    marker.color.r = 1.0f; // 红色
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.6f; // 60% 不透明度

    // 连续发布几次，确保 RViz 能够接收到并显示出来
    for(int i=0; i<5; i++){
        marker_pub->publish(marker);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    RCLCPP_INFO(node->get_logger(), "虚拟障碍物(仅视觉)已生成！MoveIt对其不可见。");

    // 2. 控制机械臂前往目标点
    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    
    // 故意把速度调得非常非常慢 (5%)，方便你完美抓拍“撞击穿模”的瞬间！
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0; 
    target_pose.position.x = 0.45;
    target_pose.position.y = 0.25;
    target_pose.position.z = 0.40;

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(node->get_logger(), "规划直接路径 (无避障检测)...");
    
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "机械臂开始移动，请准备好截图！");
        move_group.execute(plan);
    }

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
