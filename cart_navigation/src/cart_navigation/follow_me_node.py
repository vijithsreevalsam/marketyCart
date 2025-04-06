#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class FollowMeNode(Node):
    def __init__(self):
        super().__init__('follow_me_node')

        # ROS2 setup
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Target tracking
        self.target_distance = None
        self.target_angle = None
        self.target_locked = False

        # PID for angular control
        self.last_angle_error = 0
        self.integral = 0
        self.kp = 0.8
        self.ki = 0.1
        self.kd = 0.05

        # PID for linear control
        self.linear_kp = 0.6
        self.desired_distance = 0.3
        self.max_linear_speed = 0.25
        self.max_angular_speed = 1.0

    def lidar_callback(self, msg):
        min_distance = float('inf')
        min_angle = 0

        # Parameters
        fov_limit = math.radians(15)     # Only check ±30° in front
        min_valid_range = 0.02            # Too close = likely wall
        max_valid_range = 3.0           # Too far = ignore

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
            self.get_logger().info(f"Target LOCKED at {min_distance:.2f}m, angle {math.degrees(min_angle):.1f}°")
        else:
            self.target_locked = False
            self.get_logger().info("Target lost. Looking for a new target.")

    def control_loop(self):
        if not self.target_locked:
            self.get_logger().info("No target locked. Stopping.")
            self.cmd_pub.publish(Twist())
            return

        msg = Twist()

        # === Linear PID ===
        distance_error = self.target_distance - self.desired_distance
        linear_velocity = self.linear_kp * distance_error
        msg.linear.x = max(-self.max_linear_speed,
                           min(self.max_linear_speed, linear_velocity))

        # === Angular PID ===
        angle_error = self.target_angle
        self.integral += angle_error
        derivative = angle_error - self.last_angle_error
        angular_velocity = self.kp * angle_error + self.ki * self.integral + self.kd * derivative
        self.last_angle_error = angle_error
        msg.angular.z = max(-self.max_angular_speed,
                            min(self.max_angular_speed, angular_velocity))

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
