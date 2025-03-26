#!/usr/bin/env python3
import sys
import yaml
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QMessageBox
from PyQt5.QtCore import Qt
import os

class WaypointNavigator(QWidget):
    def __init__(self, waypoint_file, map_origin):
        super().__init__()
        self.setWindowTitle("Waypoint Navigator")

        if not os.path.exists(waypoint_file):
            QMessageBox.critical(self, "Error", f"YAML file not found at {waypoint_file}")
            sys.exit(1)

        self.waypoints = self.load_waypoints(waypoint_file, map_origin)
        self.layout = QVBoxLayout()
        self.layout.addWidget(QLabel("Start navigation and select a section:", alignment=Qt.AlignCenter))

        start_nav_btn = QPushButton("Start")
        start_nav_btn.clicked.connect(self.start_navigation)
        self.layout.addWidget(start_nav_btn)

        rclpy.init()
        self.node = rclpy.create_node('waypoint_gui_node')
        self.action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')

        # Add buttons for each section
        if self.waypoints and any(self.waypoints.values()):
            sections_label = QLabel("sections:")
            sections_label.setAlignment(Qt.AlignCenter)
            self.layout.addWidget(sections_label)

            for section, points in self.waypoints.items():
                if points:
                    button = QPushButton(section.capitalize())
                    button.clicked.connect(lambda checked, s=section: self.navigate_to(s))
                    self.layout.addWidget(button)
        else:
            print("YAML file format error or no 'waypoints' dictionary found or no valid points.")
            self.layout.addWidget(QLabel("No valid waypoints found in the YAML file.", alignment=Qt.AlignCenter))

        self.setLayout(self.layout)

    def load_waypoints(self, file_path, origin):
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)

        adjusted_waypoints = {}
        origin_x, origin_y = origin

        if isinstance(data, dict) and 'waypoints' in data and isinstance(data['waypoints'], dict):
            for key, value in data['waypoints'].items():
                if isinstance(value, list) and value:
                    adjusted_waypoints[key] = []
                    for wp in value:
                        if isinstance(wp, dict) and 'pose' in wp and 'position' in wp['pose']:
                            pos = wp['pose']['position']
                            wp['pose']['position']['x'] = pos['x'] + origin_x
                            wp['pose']['position']['y'] = pos['y'] + origin_y
                            adjusted_waypoints[key].append(wp)
                else:
                    print(f"Section '{key}' has no valid waypoint list.")
        else:
            print("YAML file format error or no 'waypoints' dictionary found.")

        return adjusted_waypoints

    def start_navigation(self):
        print("Launching navigation stack...")
        try:
            subprocess.Popen(["ros2", "launch", "hardware_navigation", "navigation.launch.py"])
            QMessageBox.information(self, "Navigation Stack", "Navigation stack launched.")
        except Exception as e:
            print(f"Error launching navigation stack: {e}")
            QMessageBox.critical(self, "Error", f"Error launching navigation: {e}")

    def navigate_to(self, section):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            QMessageBox.critical(self, "Error", "NavigateToPose action server not available.")
            return

        for wp in self.waypoints.get(section, []):
            pos = wp['pose']['position']
            orient = wp['pose']['orientation']

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = pos['x']
            goal_msg.pose.pose.position.y = pos['y']
            goal_msg.pose.pose.orientation.z = orient['z']
            goal_msg.pose.pose.orientation.w = orient['w']

            future = self.action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result():
                print(f"Goal accepted for section: {section}")
            else:
                print(f"Goal was rejected or failed for section: {section}")

        QMessageBox.information(self, "Navigation", f"cart is going to {section}.")

if __name__ == "__main__":
    waypoint_file = "/home/amen/ros2_ws/src/marketcart/waypoints.yaml"
    map_origin = (-10.4, -9.98)  # Adjust as per your map

    app = QApplication(sys.argv)
    window = WaypointNavigator(waypoint_file, map_origin)
    window.resize(400, 400)
    window.show()
    sys.exit(app.exec_())
