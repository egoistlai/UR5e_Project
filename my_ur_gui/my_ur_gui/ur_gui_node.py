#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# ==========================================
# 1. ROS 2 后台工作线程
# ==========================================
class ROS2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        # 让 ROS 2 节点在子线程中 spin，防止卡死 GUI 主线程
        rclpy.spin(self.node)

# ==========================================
# 2. ROS 2 节点类 (负责通信)
# ==========================================
class URGUIControllerNode(Node):
    def __init__(self):
        super().__init__('ur_gui_node')
        # 创建两个发布者：一个发关节目标(PTP)，一个发笛卡尔目标(LIN)
        self.joint_pub = self.create_publisher(JointState, '/gui_joint_target', 10)
        self.pose_pub = self.create_publisher(Pose, '/gui_cartesian_target', 10)

    def send_joint_target(self, joint_values):
        msg = JointState()
        msg.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        msg.position = joint_values
        self.joint_pub.publish(msg)
        self.get_logger().info(f"已发布关节 PTP 目标: {[round(v, 2) for v in joint_values]}")

    def send_cartesian_target(self, x, y, z):
        msg = Pose()
        # 这里只做 XYZ 平移示例，姿态默认用之前代码里的逻辑
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        self.pose_pub.publish(msg)
        self.get_logger().info(f"已发布笛卡尔 LIN 目标: X={x}, Y={y}, Z={z}")

# ==========================================
# 3. PyQt5 GUI 主界面类
# ==========================================
class URControlPanel(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('UR5e 机械臂控制台 (ROS 2)')
        self.resize(500, 400)
        main_layout = QVBoxLayout()

        # --- 关节控制区 (PTP) ---
        group_joints = QGroupBox("关节空间控制 (PTP)")
        layout_joints = QGridLayout()
        
        self.sliders = []
        self.labels = []
        joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3']
        
        for i, name in enumerate(joint_names):
            layout_joints.addWidget(QLabel(name), i, 0)
            
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-314, 314) # 对应 -3.14 到 3.14 弧度 (*100)
            slider.setValue(0)
            # 给第二个关节和第三个关节设一个默认的 L 型姿态
            if i == 1 or i == 3 or i == 4: slider.setValue(-157) 
            if i == 2: slider.setValue(157)
                
            self.sliders.append(slider)
            layout_joints.addWidget(slider, i, 1)
            
            val_label = QLabel(str(slider.value() / 100.0))
            self.labels.append(val_label)
            layout_joints.addWidget(val_label, i, 2)
            
            # 绑定滑动事件
            slider.valueChanged.connect(lambda val, idx=i: self.labels[idx].setText(str(val/100.0)))

        btn_ptp = QPushButton("执行关节运动 (PTP)")
        btn_ptp.clicked.connect(self.on_ptp_clicked)
        layout_joints.addWidget(btn_ptp, 6, 0, 1, 3)
        group_joints.setLayout(layout_joints)

        # --- 笛卡尔控制区 (LIN) ---
        group_cartesian = QGroupBox("笛卡尔空间控制 (LIN 直线)")
        layout_cartesian = QHBoxLayout()
        
        self.spin_x = QDoubleSpinBox(); self.spin_x.setRange(-1.0, 1.0); self.spin_x.setSingleStep(0.05)
        self.spin_y = QDoubleSpinBox(); self.spin_y.setRange(-1.0, 1.0); self.spin_y.setSingleStep(0.05)
        self.spin_z = QDoubleSpinBox(); self.spin_z.setRange(-1.0, 1.0); self.spin_z.setSingleStep(0.05)
        
        layout_cartesian.addWidget(QLabel("X(m):")); layout_cartesian.addWidget(self.spin_x)
        layout_cartesian.addWidget(QLabel("Y(m):")); layout_cartesian.addWidget(self.spin_y)
        layout_cartesian.addWidget(QLabel("Z(m):")); layout_cartesian.addWidget(self.spin_z)

        btn_lin = QPushButton("执行直线运动 (LIN)")
        btn_lin.clicked.connect(self.on_lin_clicked)
        
        cartesian_vbox = QVBoxLayout()
        cartesian_vbox.addLayout(layout_cartesian)
        cartesian_vbox.addWidget(btn_lin)
        group_cartesian.setLayout(cartesian_vbox)

        # 拼装主界面
        main_layout.addWidget(group_joints)
        main_layout.addWidget(group_cartesian)
        self.setLayout(main_layout)

    # 按钮回调：发送 PTP 指令
    def on_ptp_clicked(self):
        joint_values = [slider.value() / 100.0 for slider in self.sliders]
        self.ros_node.send_joint_target(joint_values)

    # 按钮回调：发送 LIN 指令
    def on_lin_clicked(self):
        x = self.spin_x.value()
        y = self.spin_y.value()
        z = self.spin_z.value()
        self.ros_node.send_cartesian_target(x, y, z)


def main(args=None):
    rclpy.init(args=args)
    
    # 初始化 ROS 2 节点
    ros_node = URGUIControllerNode()
    
    # 启动 ROS 2 监听线程
    ros_thread = ROS2Thread(ros_node)
    ros_thread.start()

    # 启动 Qt 界面
    app = QApplication(sys.argv)
    gui = URControlPanel(ros_node)
    gui.show()
    
    # 等待界面关闭
    exit_code = app.exec_()
    
    # 界面关闭后清理 ROS 2 节点
    rclpy.shutdown()
    ros_thread.wait()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
