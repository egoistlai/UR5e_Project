#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

int main(int argc, char** argv)
{
    // 1. 初始化 ROS 2 节点
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    // 自动声明从参数服务器获取的参数（MoveIt 2 必需）
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ur_ptp_node", node_options);

    // 2. 启动一个独立的线程用于处理回调 (Executor)
    // ⚠️ 关键点：MoveIt 需要在后台不断接收 /joint_states 和 TF 坐标系变化，
    // 如果不用多线程 Executor，节点会卡死在等待状态信息上。
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    // 3. 实例化 MoveGroupInterface
    // UR 机械臂默认的规划组名称是 "ur_manipulator"
    static const std::string PLANNING_GROUP = "ur_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // 可选：设置运动速度和加速度的缩放比例 (0.0 ~ 1.0)
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    // 4. 定义硬编码的目标关节角度 (单位：弧度 Radian)
    // UR5e 的 6 个关节顺序通常是: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    // 下面这个姿态是一个常见的“准备姿势” (类似一个 L 型)
    std::vector<double> target_joint_values = {
        0.0,            // Base
        -1.5708,        // Shoulder (约 -90度)
        1.5708,         // Elbow    (约  90度)
        -1.5708,        // Wrist 1  (约 -90度)
        -1.5708,        // Wrist 2  (约 -90度)
        0.0             // Wrist 3
    };

    // 将目标位置传给 MoveGroup
    move_group.setJointValueTarget(target_joint_values);

    // 5. 进行运动规划
    RCLCPP_INFO(node->get_logger(), "开始规划到目标关节位置...");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // 6. 如果规划成功，则执行运动
    if (success) {
        RCLCPP_INFO(node->get_logger(), "规划成功！开始执行运动...");
        move_group.execute(my_plan);
        RCLCPP_INFO(node->get_logger(), "运动执行完毕！");
    } else {
        RCLCPP_ERROR(node->get_logger(), "规划失败，请检查目标点是否超出限位或发生碰撞。");
    }

    // 7. 关闭节点并清理线程
    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
