#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <fstream>
#include <chrono>
#include <map>
#include <cmath> // 引入 cmath 以使用 std::abs

using namespace std::chrono_literals;

class DataRecorderNode : public rclcpp::Node
{
public:
    DataRecorderNode() : Node("ur_data_recorder")
    {
        std::string file_name = "ur_motion_data.csv";
        csv_file_.open(file_name);
        if (csv_file_.is_open()) {
            csv_file_ << "timestamp,"
                      << "shoulder_pan,shoulder_lift,elbow,wrist_1,wrist_2,wrist_3,"
                      << "x,y,z,qx,qy,qz,qw\n";
            RCLCPP_INFO(this->get_logger(), "成功创建数据记录文件: %s", file_name.c_str());
            RCLCPP_INFO(this->get_logger(), "当前模式：【仅在运动时记录数据】");
        } else {
            RCLCPP_ERROR(this->get_logger(), "无法创建 CSV 文件！");
        }

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&DataRecorderNode::jointStateCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            20ms, std::bind(&DataRecorderNode::recordDataCallback, this));
    }

    ~DataRecorderNode()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "数据记录完毕，文件已关闭。");
        }
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            latest_joints_[msg->name[i]] = msg->position[i];
        }
    }

    void recordDataCallback()
    {
        if (latest_joints_.empty()) return;

        std::vector<std::string> ur_joints = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };
        
        for (const auto& j : ur_joints) {
            if (latest_joints_.find(j) == latest_joints_.end()) return; 
        }

        // ==========================================================
        // === 运动检测逻辑 ===
        // ==========================================================
        bool is_moving = false;
        
        // 阈值设为 0.0001 弧度 (约 0.005 度)，足以过滤掉仿真传感器的微小噪声
        const double MOTION_THRESHOLD = 0.0001; 

        if (previous_joints_.empty()) {
            is_moving = true; // 为了保证有起始数据，强制记录第一帧
        } else {
            for (const auto& j : ur_joints) {
                if (std::abs(latest_joints_[j] - previous_joints_[j]) > MOTION_THRESHOLD) {
                    is_moving = true;
                    break; // 只要有一个关节在动，就判定为运动状态
                }
            }
        }

        // 如果没有运动，直接退出回调函数，不写入 CSV，也不查询 TF
        if (!is_moving) {
            return; 
        }
        // ==========================================================

        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform("base_link", "tool0", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            return;
        }

        auto now = this->get_clock()->now();
        double timestamp = now.seconds();

        if (csv_file_.is_open()) {
            csv_file_ << std::fixed << timestamp << ","
                      << latest_joints_["shoulder_pan_joint"] << ","
                      << latest_joints_["shoulder_lift_joint"] << ","
                      << latest_joints_["elbow_joint"] << ","
                      << latest_joints_["wrist_1_joint"] << ","
                      << latest_joints_["wrist_2_joint"] << ","
                      << latest_joints_["wrist_3_joint"] << ","
                      << t.transform.translation.x << ","
                      << t.transform.translation.y << ","
                      << t.transform.translation.z << ","
                      << t.transform.rotation.x << ","
                      << t.transform.rotation.y << ","
                      << t.transform.rotation.z << ","
                      << t.transform.rotation.w << "\n";
        }

        // 更新历史记录，用于下一帧的比对
        previous_joints_ = latest_joints_;
    }

    std::ofstream csv_file_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::map<std::string, double> latest_joints_;
    std::map<std::string, double> previous_joints_; // 新增：保存上一帧的数据
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataRecorderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
