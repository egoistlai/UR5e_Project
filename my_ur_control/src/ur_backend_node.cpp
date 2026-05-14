#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <mutex>

class URBackendNode : public rclcpp::Node
{
public:
    URBackendNode(const rclcpp::NodeOptions& options)
    : Node("ur_backend_node", options), is_moving_(false)
    {
        // 1. 订阅 GUI 发来的关节 PTP 目标
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/gui_joint_target", 10,
            std::bind(&URBackendNode::jointTargetCallback, this, std::placeholders::_1));

        // 2. 订阅 GUI 发来的笛卡尔 LIN 目标
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/gui_cartesian_target", 10,
            std::bind(&URBackendNode::poseTargetCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "后端控制节点已启动，正在等待 MoveGroup 初始化...");
    }

    // MoveGroup 的初始化必须在节点开始 spin 之后进行
    void initMoveGroup()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
        // 为了安全起见，全局限速 30%
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.3);
        RCLCPP_INFO(this->get_logger(), "MoveGroup 初始化完成！随时可以接收 GUI 指令。");
    }

private:
    // ==========================================
    // 回调 1：处理 PTP 关节运动
    // ==========================================
    void jointTargetCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(move_mutex_);
        if (is_moving_) {
            RCLCPP_WARN(this->get_logger(), "机械臂正在运动中，忽略新的 PTP 指令！");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "收到 PTP 运动指令，开始规划...");
        is_moving_ = true;

        move_group_->setJointValueTarget(msg->position);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            move_group_->execute(plan);
            RCLCPP_INFO(this->get_logger(), "PTP 运动执行完毕！");
        } else {
            RCLCPP_ERROR(this->get_logger(), "PTP 规划失败！可能是目标位置干涉或超出限位。");
        }
        is_moving_ = false;
    }

    // ==========================================
    // 回调 2：处理 LIN 直线运动
    // ==========================================
    void poseTargetCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(move_mutex_);
        if (is_moving_) {
            RCLCPP_WARN(this->get_logger(), "机械臂正在运动中，忽略新的 LIN 指令！");
            return;
        }

        // ⚠️ 关键点：GUI 发送的是 X/Y/Z 的“相对偏移量”！
        // 比如 GUI 输入 X=0.1，意味着在“当前位置”的基础上向前移动 10 厘米
        RCLCPP_INFO(this->get_logger(), "收到 LIN 直线偏移指令: X=%.2f, Y=%.2f, Z=%.2f", 
                    msg->position.x, msg->position.y, msg->position.z);
        is_moving_ = true;

        // 获取当前末端位姿
        geometry_msgs::msg::Pose start_pose = move_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        
        // 叠加 GUI 传来的偏移量
        target_pose.position.x += msg->position.x;
        target_pose.position.y += msg->position.y;
        target_pose.position.z += msg->position.z;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        if (fraction > 0.9) {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group_->execute(plan);
            RCLCPP_INFO(this->get_logger(), "LIN 直线运动执行完毕！");
        } else {
            RCLCPP_ERROR(this->get_logger(), "LIN 直线规划失败！只有 %.2f%% 的路径可达。", fraction * 100.0);
        }
        is_moving_ = false;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    
    std::mutex move_mutex_;  // 防止并发执行的线程锁
    bool is_moving_;         // 标记当前是否正在运动
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<URBackendNode>(node_options);

    // 启动多线程执行器 (保证 ROS 回调和 MoveIt 规划互不阻塞)
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    // 在后台线程启动后，初始化 MoveGroup
    node->initMoveGroup();

    spinner_thread.join(); 
    rclcpp::shutdown();
    return 0;
}
