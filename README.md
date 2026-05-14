# 🤖 UR5e Robot ROS 2 Motion Control & Simulation

本项目是一个基于 **ROS 2 Humble** 与 **MoveIt 2** 框架的 UR5e 六自由度机械臂高保真运动控制与仿真系统。项目涵盖了从正逆运动学底层建模、高精度笛卡尔空间轨迹插补（直线/圆弧），到复杂环境动态避障的完整机器人开发流水线。

## 📂 代码框架 (Repository Structure)

本项目采用标准的 ROS 2 工作空间结构，包含以下 4 个核心功能包（Packages）：

```text
📦 UR5e_Project (src)
 ┣ 📂 Universal_Robots_ROS2_Description       # 机械臂数字孪生模型
 ┃ ┗ 📜 包含 UR5e 机械臂的 URDF/XACRO 模型、碰撞网格(Meshes)及运动学极限参数。
 ┣ 📂 Universal_Robots_ROS2_Gazebo_Simulation # 物理引擎环境
 ┃ ┗ 📜 包含 Gazebo 仿真环境配置文件、控制器(ros2_control)接口及传感器插件。
 ┣ 📂 my_ur_control                           # 核心后端控制算法包 (C++)
 ┃ ┣ 📜 ur_ptp_node.cpp         : 关节空间点到点(PTP)运动规划
 ┃ ┣ 📜 ur_lin_node.cpp         : 笛卡尔空间高精度直线(LIN)插补
 ┃ ┣ 📜 ur_circ_node.cpp        : 笛卡尔空间圆弧(CIRC)插补 (360密集采样优化)
 ┃ ┣ 📜 ur_goto_node.cpp        : 空间坐标系目标点定位
 ┃ ┣ 📜 ur_home_node.cpp        : 一键安全复位(Upright)至零位
 ┃ ┣ 📜 ur_ready_node.cpp       : 奇异点规避专用准备姿态节点
 ┃ ┣ 📜 ur_obstacle_node.cpp    : 基于 FCL 引擎的动态避障 (对照组 A)
 ┃ ┣ 📜 ur_no_obstacle_node.cpp : 干涉穿透测试 (对照组 B)
 ┃ ┗ 📜 ur_data_recorder.cpp    : 轨迹数据高频采集与 CSV 导出
 ┗ 📂 my_ur_gui                               # 前端人机交互界面 (Python)
   ┗ 📜 提供友好的图形化界面(GUI)，用于向后端下发控制指令与参数配置。


🛠️ 环境配置 (Prerequisites)
为了确保系统稳定运行，请在 Ubuntu 22.04 LTS 操作系统下配置以下环境：

ROS 2 版本: Humble Hawksbill

仿真环境: Gazebo Classic 11

运动规划框架: MoveIt 2

编译工具: colcon, CMake (C++17)

依赖安装
在终端中执行以下命令，安装 ROS 2 相关依赖：
sudo apt update
sudo apt install ros-humble-desktop ros-humble-moveit ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers

🚀 编译与安装 (Build Instructions)
创建工作空间并克隆本仓库：
mkdir -p ~/ur_ws/src
cd ~/ur_ws/src
git clone <你的GitHub仓库链接> .

安装缺失的依赖树 (rosdep)：
cd ~/ur_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

编译工作空间：
colcon build --packages-select Universal_Robots_ROS2_Description Universal_Robots_ROS2_Gazebo_Simulation my_ur_control my_ur_gui

刷新环境变量：
source install/setup.bash

🎮 使用说明 (Usage)
Step 1: 启动物理仿真与运动规划环境
首先启动 Gazebo 仿真环境与 MoveIt 2 规划流水线：
# 请根据你的实际 launch 文件名启动仿真环境
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
ros2 launch ur_moveit_config ur_moveit.launch.py

Step 2: 启动前端交互界面 (GUI)
打开新终端，运行人机交互界面：
source install/setup.bash
ros2 run my_ur_gui ur_gui_node

Step 3: 后端控制节点快速验证 (CLI 方式)
你也可以直接通过命令行调用核心算法节点。以下是几个典型场景的测试命令：

姿态复位 (回到绝对安全状态)
ros2 run my_ur_control ur_home_node --ros-args -p use_sim_time:=true

执行高精度圆弧插补 (消除多边形效应)
ros2 run my_ur_control ur_ready_node --ros-args -p use_sim_time:=true  # 先进入避免奇异点的最佳姿态
ros2 run my_ur_control ur_circ_node --ros-args -p use_sim_time:=true

运行动态避障对照实验
ros2 run my_ur_control ur_home_node --ros-args -p use_sim_time:=true
ros2 run my_ur_control ur_obstacle_node --ros-args -p use_sim_time:=true     # 实验组：完美侧滑绕行
ros2 run my_ur_control ur_no_obstacle_node --ros-args -p use_sim_time:=true  # 对照组：无视碰撞发生穿模干涉

📊 核心技术亮点 (Highlights)
算法调优: 弃用了默认的低密度 RRT 规划，结合 PRM* 算法与微米级 (eef_step = 0.002m) 的插补步长，实现了理论完美的空间圆弧轨迹。
奇异点深剖: 通过 ur_data_recorder 捕获底层 CSV 数据，逆向解构了腕部奇异位形 ($\theta_5 \to \pi$) 的触发机制，并用姿态强约束彻底解决了规划崩溃问题。
数字孪生避障: 在规划场景(Planning Scene)中动态注入刚性障碍物，结合 FCL 碰撞检测，实现了 100% 安全的三维空间自主绕行。

***

**💡 使用方法：**
1. 在你本地 `~/ur_yt_ws/src` 目录下，运行 `nano README.md`。
2. 将上面的文本粘贴进去（记得在 **编译与安装** 那一步里，把 `<你的GitHub仓库链接>` 替换成你真实的 URL）。
3. 保存退出，然后按照我们刚才的一键上传三连操作：
   ```bash
   git add README.md
   git commit -m "Add project README with usage instructions and architecture"
   git push
