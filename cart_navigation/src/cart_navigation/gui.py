#!/usr/bin/env python3
import sys
import yaml
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from rclpy.action import ActionClient
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, QLabel,
                             QMessageBox, QCheckBox)
from PyQt5.QtCore import Qt, QThread
from pathlib import Path
import os
import math

# ------------------- Follow Me Node ------------------- #
class FollowMeNode(Node):
    def __init__(self):
        super().__init__('follow_me_node')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.target_distance = None
        self.target_angle = None
        self.target_locked = False
        self.last_angle_error = 0
        self.integral = 0
        self.kp = 0.8
        self.ki = 0.1
        self.kd = 0.05
        self.linear_kp = 0.6
        self.desired_distance = 0.4
        self.max_linear_speed = 0.4
        self.max_angular_speed = 2.0

    def lidar_callback(self, msg):
        min_distance = float('inf')
        min_angle = 0
        fov_limit = math.radians(15)
        min_valid_range = 0.02
        max_valid_range = 3.0
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > fov_limit:
                continue
            if distance < min_valid_range or distance > max_valid_range:
                continue
            if distance < min_distance:
                min_distance = distance
                min_angle = angle
        if min_distance < float('inf'):
            self.target_distance = min_distance
            self.target_angle = min_angle
            self.target_locked = True
        else:
            self.target_locked = False

    def control_loop(self):
        if not self.target_locked:
            self.cmd_pub.publish(Twist())
            return
        msg = Twist()
        distance_error = self.target_distance - self.desired_distance
        linear_velocity = self.linear_kp * distance_error
        msg.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_velocity))
        angle_error = self.target_angle
        self.integral += angle_error
        derivative = angle_error - self.last_angle_error
        angular_velocity = self.kp * angle_error + self.ki * self.integral + self.kd * derivative
        self.last_angle_error = angle_error
        msg.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_velocity))
        self.cmd_pub.publish(msg)

# ------------------- ROS2 Thread ------------------- #
class ROS2Thread(QThread):
    def __init__(self):
        super().__init__()
        self.node = None
        self.executor = None

    def run(self):
        self.node = FollowMeNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        try:
            self.executor.spin()
        except Exception:
            pass
        finally:
            if self.node:
                self.executor.remove_node(self.node)
                self.node.destroy_node()

    def stop(self):
        if self.executor:
            self.executor.shutdown()

# ------------------- GUI ------------------- #
class WaypointNavigator(QWidget):
    def __init__(self, waypoint_file, map_origin):
        super().__init__()
        self.setWindowTitle("Waypoint Navigator")
        self.multi_section_mode = False
        self.selected_sections = []

        if not os.path.exists(waypoint_file):
            QMessageBox.critical(self, "Error", f"YAML file not found at {waypoint_file}")
            sys.exit(1)

        self.waypoints = self.load_waypoints(waypoint_file, map_origin)
        self.layout = QVBoxLayout()
        self.layout.addWidget(QLabel("Cart Screen:", alignment=Qt.AlignCenter))

        self.toggle_multi_btn = QPushButton("Multi-Section Mode OFF")
        self.toggle_multi_btn.clicked.connect(self.toggle_multi_section_mode)
        self.layout.addWidget(self.toggle_multi_btn)

        start_nav_btn = QPushButton("Start")
        start_nav_btn.clicked.connect(self.start_navigation)
        self.layout.addWidget(start_nav_btn)

        follow_me_on_btn = QPushButton("Follow Me ON")
        follow_me_on_btn.clicked.connect(self.start_follow_me)
        self.layout.addWidget(follow_me_on_btn)

        follow_me_off_btn = QPushButton("Follow Me OFF")
        follow_me_off_btn.clicked.connect(self.stop_follow_me)
        self.layout.addWidget(follow_me_off_btn)

        detect_on_btn = QPushButton("Detect ON")
        detect_on_btn.clicked.connect(self.start_detection)
        self.layout.addWidget(detect_on_btn)

        detect_off_btn = QPushButton("Detect OFF")
        detect_off_btn.clicked.connect(self.stop_detection)
        self.layout.addWidget(detect_off_btn)

        self.layout.addWidget(QLabel("Sections:", alignment=Qt.AlignCenter))
        self.section_buttons = {}
        for section in self.waypoints.keys():
            btn = QPushButton(section.capitalize())
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, s=section: self.handle_section_click(s))
            self.section_buttons[section] = btn
            self.layout.addWidget(btn)

        self.setLayout(self.layout)

        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node('waypoint_gui_node')
        self.single_nav_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.multi_nav_client = ActionClient(self.node, NavigateThroughPoses, '/navigate_through_poses')

        self.follow_me_thread = None
        self.detect_process = None
        self.executing_multi_nav = False

    def toggle_multi_section_mode(self):
        self.multi_section_mode = not self.multi_section_mode
        self.toggle_multi_btn.setText("Multi-Section Mode ON" if self.multi_section_mode else "Single Section Mode")
        self.selected_sections.clear()
        for btn in self.section_buttons.values():
            btn.setChecked(False)
        self.executing_multi_nav = False

    def handle_section_click(self, section):
        btn = self.section_buttons[section]
        if self.multi_section_mode:
            if btn.isChecked():
                if section not in self.selected_sections:
                    self.selected_sections.append(section)
            else:
                if section in self.selected_sections:
                    self.selected_sections.remove(section)
            self.navigate_through_selected()
        else:
            if btn.isChecked():
                self.navigate_to(section)
            else:
                print(f"Cancelled {section} navigation")

    def load_waypoints(self, file_path, origin):
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        adjusted_waypoints = {}
        if isinstance(data, dict) and 'waypoints' in data:
            for key, wp in data['waypoints'].items():
                if 'pose' in wp and 'orientation' in wp:
                    pos = list(map(float, wp['pose']))
                    orient = list(map(float, wp['orientation']))
                    adjusted_waypoints[key] = [{
                        'pose': {
                            'position': {'x': pos[0], 'y': pos[1], 'z': pos[2]},
                            'orientation': {'x': orient[0], 'y': orient[1], 'z': orient[2], 'w': orient[3]}
                        },
                        'speed': 0.5
                    }]
        return adjusted_waypoints

    def start_navigation(self):
        try:
            subprocess.Popen(["ros2", "launch", "cart_navigation", "navigation.launch.py"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("Navigation started.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error launching navigation: {e}")

    def start_follow_me(self):
        if self.follow_me_thread is None:
            self.follow_me_thread = ROS2Thread()
            self.follow_me_thread.start()
            print("Follow Me ON.")

    def stop_follow_me(self):
        if self.follow_me_thread:
            self.follow_me_thread.stop()
            self.follow_me_thread.quit()
            self.follow_me_thread.wait()
            self.follow_me_thread = None
            print("Follow Me OFF.")

    def start_detection(self):
        script_dir = Path(__file__).resolve().parent.parent.parent / "detection"
        detect_script = script_dir / "detect.py"
        if self.detect_process is None and detect_script.exists():
            self.detect_process = subprocess.Popen(["python3", str(detect_script)],
                                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("Detection ON.")

    def stop_detection(self):
        if self.detect_process:
            self.detect_process.terminate()
            self.detect_process.wait()
            self.detect_process = None
            print("Detection OFF.")

    def navigate_to(self, section):
        if not self.single_nav_client.wait_for_server(timeout_sec=5.0):
            QMessageBox.critical(self, "Error", f"NavigateToPose server not available for {section}.")
            return
        for wp in self.waypoints.get(section, []):
            pos = wp['pose']['position']
            orient = wp['pose']['orientation']
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = pos['x']
            goal_msg.pose.pose.position.y = pos['y']
            goal_msg.pose.pose.orientation.z = orient['z']
            goal_msg.pose.pose.orientation.w = orient['w']
            future = self.single_nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, future)
            print(f"Navigating to: {section}")

    def navigate_through_selected(self):
        if not self.selected_sections:
            return
        if not self.multi_nav_client.wait_for_server(timeout_sec=5.0):
            QMessageBox.critical(self, "Error", "NavigateThroughPoses server not available.")
            return
        goal = NavigateThroughPoses.Goal()
        for section in self.selected_sections:
            for wp in self.waypoints.get(section, []):
                pos = wp['pose']['position']
                orient = wp['pose']['orientation']
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.pose.position.x = pos['x']
                pose.pose.position.y = pos['y']
                pose.pose.orientation.z = orient['z']
                pose.pose.orientation.w = orient['w']
                goal.poses.append(pose)
        future = self.multi_nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        print("Cart heading to multiple sections:", self.selected_sections)

# ------------------- Main ------------------- #
if __name__ == "__main__":
    script_dir = Path(__file__).resolve().parent.parent.parent
    waypoint_file = str(script_dir / "waypoints.yaml")
    map_origin = (0.0, 0.0)
    app = QApplication(sys.argv)
    window = WaypointNavigator(waypoint_file, map_origin)
    window.resize(500, 500)
    window.show()
    sys.exit(app.exec_())