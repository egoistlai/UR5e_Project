cmake_minimum_required(VERSION 3.8)
project(my_ur_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 找到我们写的源文件并生成可执行文件
add_executable(ur_ptp_node src/ur_ptp_node.cpp)

# 链接对应的库
ament_target_dependencies(ur_ptp_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

add_executable(ur_lin_node src/ur_lin_node.cpp)
ament_target_dependencies(ur_lin_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

add_executable(ur_circ_node src/ur_circ_node.cpp)
ament_target_dependencies(ur_circ_node
 rclcpp
 moveit_ros_planning_interface
 geometry_msgs
)

add_executable(ur_goto_node src/ur_goto_node.cpp)
ament_target_dependencies(ur_goto_node
 rclcpp
 moveit_ros_planning_interface
 geometry_msgs
)

add_executable(ur_data_recorder src/ur_data_recorder.cpp)
ament_target_dependencies(ur_data_recorder
  rclcpp
  sensor_msgs
  tf2_ros
  geometry_msgs
)

add_executable(ur_backend_node src/ur_backend_node.cpp)
ament_target_dependencies(ur_backend_node
  rclcpp
  moveit_ros_planning_interface
  sensor_msgs
  geometry_msgs
)

add_executable(ur_ready_node src/ur_ready_node.cpp)
ament_target_dependencies(ur_ready_node
 rclcpp
 moveit_ros_planning_interface
)

add_executable(ur_obstacle_node src/ur_obstacle_node.cpp)
ament_target_dependencies(ur_obstacle_node
 rclcpp
 moveit_ros_planning_interface
 moveit_msgs
 shape_msgs
 geometry_msgs
)

add_executable(ur_no_obstacle_node src/ur_no_obstacle_node.cpp)
ament_target_dependencies(ur_no_obstacle_node
 rclcpp
 moveit_ros_planning_interface
 visualization_msgs
 geometry_msgs
)

add_executable(ur_home_node src/ur_home_node.cpp)
ament_target_dependencies(ur_home_node
 rclcpp
 moveit_ros_planning_interface
)

# 定义安装路径（让 ros2 run 能够找到它）
install(TARGETS
  ur_ptp_node
  ur_lin_node
  ur_circ_node
  ur_goto_node
  ur_ready_node
  ur_obstacle_node
  ur_no_obstacle_node
  ur_home_node
  ur_data_recorder
  ur_backend_node
 DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
