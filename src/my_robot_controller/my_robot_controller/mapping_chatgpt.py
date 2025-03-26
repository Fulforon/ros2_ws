#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtlebotMappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.get_logger().info("Fast Wall-Following Mapping Node has started.")

        # Publisher to send movement commands
        self._pose_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber to receive LiDAR data
        self._scan_listener = self.create_subscription(LaserScan, "/scan", self.robot_controller, 10)

        # Desired distance from the wall (in meters)
        self._desired_wall_distance = 0.5
        self._front_threshold = 0.7  # Distance to detect obstacles ahead

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()

        # Define sectors for LiDAR data
        left_sector = scan.ranges[85:95]  # Left side (90° ± 5°)
        front_sector = scan.ranges[:5] + scan.ranges[-5:]  # Front side (0° ± 5°)
        left_front_sector = scan.ranges[60:80]  # Front-left (between 60° and 80°)

        # Extract minimum distances in each sector
        left_distance = min(left_sector) if left_sector else float('inf')
        front_distance = min(front_sector) if front_sector else float('inf')
        left_front_distance = min(left_front_sector) if left_front_sector else float('inf')

        # Obstacle Avoidance Logic
        if front_distance < self._front_threshold:  # Obstacle directly ahead
            cmd.linear.x = 0.0
            cmd.angular.z = -1.0  # Turn right aggressively to avoid obstacle
        elif left_front_distance < self._desired_wall_distance:  # Wall turning corner
            cmd.linear.x = 0.3  # Maintain higher forward speed
            cmd.angular.z = -0.8  # Turn right more aggressively to follow wall
        elif left_distance > self._desired_wall_distance + 0.1:  # Too far from the wall
            cmd.linear.x = 0.5  # Move faster
            cmd.angular.z = 0.5  # Turn left more aggressively to get closer to the wall
        elif left_distance < self._desired_wall_distance - 0.1:  # Too close to the wall
            cmd.linear.x = 0.5  # Move faster
            cmd.angular.z = -0.5  # Turn right more aggressively to create space
        else:  # Follow wall
            cmd.linear.x = 0.6  # Maximize forward speed
            cmd.angular.z = 0.0  # Go straight

        # Publish the command
        self._pose_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()