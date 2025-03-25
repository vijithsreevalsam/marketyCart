#!/usr/bin/env python3
# NOTE: This GUI script requires ROS 2 and the `rclpy` package.
# Ensure you are running this script inside a ROS 2 environment with the proper setup.

try:
    import rclpy
    from rclpy.node import Node
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped
    import subprocess
    import os
except ModuleNotFoundError as e:
    print("ERROR: This script must be run inside a ROS 2 environment with rclpy installed.")
    print("Please source your ROS 2 setup.bash before running this GUI.")
    raise e

import tkinter as tk
from tkinter import scrolledtext
import math

# These are your section coordinates from the world file
SECTIONS = {
    'Vegetables': (1.5, -7.0, 0.0),
    'Fruits': (1.48372, -3.18258, 0.0),
    'Drinks': (-7.09885, -4.41814, 1.5708),
    'Sweets': (0.40844, 6.91594, 0.0),
    'Dairies': (8.95676, 3.90401, 1.5708),
}

# --- Offset values you can tune here ---
OFFSET_X = 0.0   # Change if your Gazebo world origin doesn't match map origin in X
OFFSET_Y = 0.0   # Change if there's a shift in Y
# ---------------------------------------

class NavClient(Node):
    def __init__(self, log_callback):
        super().__init__('gui_nav_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.log_callback = log_callback

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        goal_msg.pose = goal_pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log_callback("Goal rejected")
            return
        self.log_callback("Goal accepted, navigating...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.log_callback("Navigation completed!")

class GUI:
    def __init__(self, node):
        self.node = node
        self.window = tk.Tk()
        self.window.title("Supermarket Navigation GUI")
        launch_btn = tk.Button(self.window, text="Start Full Navigation System", width=30, command=self.launch_navigation_system)
        launch_btn.pack(pady=10)
        for section, coords in SECTIONS.items():
            button = tk.Button(self.window, text=f"Go to {section}", width=20, command=lambda c=coords, s=section: self.send_goal(s, c))
            button.pack(pady=5)
        self.log_area = scrolledtext.ScrolledText(self.window, width=50, height=10)
        self.log_area.pack(pady=10)

    def launch_navigation_system(self):
        self.log("Launching navigation, RViz, and Gazebo...")
        subprocess.Popen(["ros2", "launch", "cart_navigation", "navigation.launch.py"])

    def send_goal(self, section, coords):
        x, y, yaw = coords

        # Apply offset correction before sending
        x += OFFSET_X
        y += OFFSET_Y

        self.log(f"Sending robot to {section} with offset (x={x}, y={y}, yaw={yaw})")
        self.node.send_goal(x, y, yaw)

    def log(self, message):
        self.log_area.insert(tk.END, message + "\n")
        self.log_area.see(tk.END)

    def run(self):
        self.window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    gui_node = NavClient(log_callback=lambda msg: app.log(msg))
    global app
    app = GUI(gui_node)
    import threading
    thread = threading.Thread(target=rclpy.spin, args=(gui_node,), daemon=True)
    thread.start()
    app.run()
    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
