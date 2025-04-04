#!/usr/bin/env python3
import sys
import yaml
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QMessageBox
from PyQt5.QtCore import Qt, QThread
import os
import math
import signal

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

    def lidar_callback(self, msg):
        min_distance = float('inf')
        min_index = -1
        min_angle = 0

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > math.radians(180):
                continue
            if msg.range_min < distance < 2.0 and distance < min_distance:
                min_distance = distance
                min_index = i
                min_angle = angle

        if min_index != -1:
            if not self.target_locked:
                self.target_distance = min_distance
                self.target_angle = min_angle
                self.target_locked = True
                self.get_logger().info(f"Target LOCKED at {min_distance:.2f}m, angle {math.degrees(min_angle):.1f}°")
            else:
                self.target_distance = min_distance
                self.target_angle = min_angle
        else:
            self.target_locked = False
            self.get_logger().info("Target lost. Looking for a new target.")

    def control_loop(self):
        if not self.target_locked:
            self.get_logger().info("No target locked. Stopping.")
            self.cmd_pub.publish(Twist())
            return

        msg = Twist()
        distance_buffer = 0.05

        if self.target_distance < 0.55 - distance_buffer:
            msg.linear.x = 0.2
        elif self.target_distance > 0.55 + distance_buffer:
            msg.linear.x = 0.2
        else:
            msg.linear.x = 0.0

        angle_error = self.target_angle
        self.integral += angle_error
        derivative = angle_error - self.last_angle_error
        angular_velocity = self.kp * angle_error + self.ki * self.integral + self.kd * derivative
        self.last_angle_error = angle_error

        msg.angular.z = angular_velocity
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
        except Exception as e:
            print(f"[ROS2Thread] Exception: {e}")
        finally:
            if self.node:
                self.executor.remove_node(self.node)
                self.node.destroy_node()
            print("[ROS2Thread] Follow Me node shutdown complete.")

    def stop(self):
        if self.executor:
            self.executor.shutdown()

# ------------------- GUI ------------------- #
class WaypointNavigator(QWidget):
    def __init__(self, waypoint_file, map_origin):
        super().__init__()
        self.setWindowTitle("Waypoint Navigator")

        if not os.path.exists(waypoint_file):
            QMessageBox.critical(self, "Error", f"YAML file not found at {waypoint_file}")
            sys.exit(1)

        self.waypoints = self.load_waypoints(waypoint_file, map_origin)
        self.layout = QVBoxLayout()
        self.layout.addWidget(QLabel("Cart Screen:", alignment=Qt.AlignCenter))

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

        if self.waypoints and any(self.waypoints.values()):
            self.layout.addWidget(QLabel("Sections:", alignment=Qt.AlignCenter))
            for section in self.waypoints.keys():
                button = QPushButton(section.capitalize())
                button.clicked.connect(lambda checked, s=section: self.navigate_to(s))
                self.layout.addWidget(button)
        else:
            self.layout.addWidget(QLabel("No valid waypoints found.", alignment=Qt.AlignCenter))

        self.setLayout(self.layout)

        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node('waypoint_gui_node')
        self.action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')

        self.follow_me_thread = None
        self.detect_process = None

    def load_waypoints(self, file_path, origin):
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)

        adjusted_waypoints = {}
        origin_x, origin_y = origin

        if isinstance(data, dict) and 'waypoints' in data:
            for key, wp in data['waypoints'].items():
                if 'position' in wp and 'orientation' in wp:
                    pos = wp['position']
                    orient = wp['orientation']
                    adjusted_waypoints[key] = [{
                        'pose': {
                            'position': {
                                'x': pos['x'] + origin_x,
                                'y': pos['y'] + origin_y,
                                'z': 0.0
                            },
                            'orientation': {
                                'x': 0.0,
                                'y': 0.0,
                                'z': orient['z'],
                                'w': orient['w']
                            }
                        }
                    }]
        return adjusted_waypoints

    def start_navigation(self):
        print("🚀 Launching navigation stack...")
        try:
            subprocess.Popen(["ros2", "launch", "cart_navigation", "navigation.launch.py"])
            QMessageBox.information(self, "Navigation Stack", "Navigation stack launched.")
        except Exception as e:
            print(f"❌ Error launching navigation: {e}")
            QMessageBox.critical(self, "Error", f"Error launching navigation: {e}")

    def start_follow_me(self):
        print("🚶 Starting Follow Me...")
        if self.follow_me_thread is None:
            self.follow_me_thread = ROS2Thread()
            self.follow_me_thread.start()
        else:
            print("Follow Me ON.")

    def stop_follow_me(self):
        print("❌ Stopping Follow Me...")
        if self.follow_me_thread:
            self.follow_me_thread.stop()
            self.follow_me_thread.quit()
            self.follow_me_thread.wait()
            self.follow_me_thread = None
            print("✅ Follow Me stopped.")
        else:
            print("Follow Me OFF.")

    def start_detection(self):
        print("🔍 Starting Detection...")
        if self.detect_process is None:
            self.detect_process = subprocess.Popen(["python3", "/home/amen/ros2_ws/src/marketcart/cart_navigation/detection/detect.py"])
        else:
            print("Detection ON.")

    def stop_detection(self):
        print("❌ Stopping Detection...")
        if self.detect_process:
            self.detect_process.terminate()
            self.detect_process.wait()
            self.detect_process = None
            print("✅ Detection stopped.")
        else:
            print("Detection OFF.")

    def navigate_to(self, section):
        section_name = section.capitalize()

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            QMessageBox.critical(self, "Error", f"NavigateToPose action server not available for {section_name}.")
            return

        print(f"\n🔄 Navigating to: {section_name}")

        for i, wp in enumerate(self.waypoints.get(section, []), 1):
            pos = wp['pose']['position']
            orient = wp['pose']['orientation']

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = pos['x']
            goal_msg.pose.pose.position.y = pos['y']
            goal_msg.pose.pose.orientation.z = orient['z']
            goal_msg.pose.pose.orientation.w = orient['w']

            print(f"[{section_name}] Sending waypoint {i}: x={pos['x']:.2f}, y={pos['y']:.2f}")
            future = self.action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result():
                print(f"[{section_name}] ✅ Goal {i} accepted.")
            else:
                print(f"[{section_name}] ❌ Goal {i} failed.")

        QMessageBox.information(self, "Navigation", f"🧭 Cart heading to: {section_name}")

# ------------------- Main ------------------- #
if __name__ == "__main__":
    waypoint_file = "/home/amen/ros2_ws/src/marketcart/cart_navigation/waypoints.yaml"
    map_origin = (-10.1, -9.98)

    app = QApplication(sys.argv)
    window = WaypointNavigator(waypoint_file, map_origin)
    window.resize(400, 500)
    window.show()
    sys.exit(app.exec_())
