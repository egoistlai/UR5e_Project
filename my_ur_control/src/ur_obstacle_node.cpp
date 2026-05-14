#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ur_obstacle_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    // ================= 1. 创建障碍物 =================
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame(); // 通常是 "base_link"
    collision_object.id = "my_obstacle_box";

    // 定义障碍物的形状: 一个长方体 (柱子)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.10; // 10 厘米厚
    primitive.dimensions[primitive.BOX_Y] = 0.10; // 10 厘米宽 (之前是40，太宽把路堵死了)
    primitive.dimensions[primitive.BOX_Z] = 0.60; // 60 厘米高

    // 柱子的位置
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.35;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.30; // 质心高度

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // 将障碍物发布到 MoveIt 场景中
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(node->get_logger(), "障碍物已成功添加到规划场景中！");

    // ================= 2. 规划避障路径 =================
    // 设置目标点：位于障碍物的正后方 (X=0.5)
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0; 
    target_pose.position.x = 0.45; // 往前伸
    target_pose.position.y = 0.25; // 往左侧面伸，强迫它从侧面绕开柱子
    target_pose.position.z = 0.40; // 抬高一点

    move_group.setPoseTarget(target_pose);
    move_group.setPlanningTime(10.0); // 给规划器足够的时间去寻找绕路方案

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(node->get_logger(), "开始进行避障轨迹规划...");
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "避障规划成功！机械臂准备绕行...");
        move_group.execute(plan);
    } else {
        RCLCPP_ERROR(node->get_logger(), "避障规划失败！可能目标点不可达或被完全封死。");
    }

    // 可选：运行完后移除障碍物
    // std::vector<std::string> object_ids = {"my_obstacle_box"};
    // planning_scene_interface.removeCollisionObjects(object_ids);

    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
