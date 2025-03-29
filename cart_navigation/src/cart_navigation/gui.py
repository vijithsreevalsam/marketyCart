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

        start_nav_btn = QPushButton("Start Navigation")
        start_nav_btn.clicked.connect(self.start_navigation)
        self.layout.addWidget(start_nav_btn)

        rclpy.init()
        self.node = rclpy.create_node('waypoint_gui_node')
        self.action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')

        # Add buttons for each section
        if self.waypoints and any(self.waypoints.values()):
            sections_label = QLabel("Go to these sections:")
            sections_label.setAlignment(Qt.AlignCenter)
            self.layout.addWidget(sections_label)

            for section in self.waypoints.keys():
                button = QPushButton(section.capitalize())
                button.clicked.connect(lambda checked, s=section: self.navigate_to(s))
                self.layout.addWidget(button)
        else:
            print("YAML file format error or no valid waypoints.")
            self.layout.addWidget(QLabel("No valid waypoints found.", alignment=Qt.AlignCenter))

        self.setLayout(self.layout)

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
                else:
                    print(f"[WARN] Section '{key}' is missing 'position' or 'orientation'.")
        else:
            print("Invalid YAML structure or missing 'waypoints' key.")

        return adjusted_waypoints

    def start_navigation(self):
        print("🚀 Launching navigation stack...")
        try:
            subprocess.Popen(["ros2", "launch", "cart_navigation", "navigation.launch.py"])
            QMessageBox.information(self, "Navigation Stack", "Navigation stack launched.")
        except Exception as e:
            print(f"❌ Error launching navigation: {e}")
            QMessageBox.critical(self, "Error", f"Error launching navigation: {e}")

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

        QMessageBox.information(self, "Navigation", f"🧭 Cart is navigating to: {section_name}")

if __name__ == "__main__":
    waypoint_file = "/home/amen/ros2_ws/src/marketcart/cart_navigation/waypoints.yaml"
    map_origin = (-10.1, -9.98)  # Adjust to match your map.yaml

    app = QApplication(sys.argv)
    window = WaypointNavigator(waypoint_file, map_origin)
    window.resize(400, 400)
    window.show()
    sys.exit(app.exec_())
