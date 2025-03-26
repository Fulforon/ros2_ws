#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtlebotMappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.get_logger().info("Mapping Node has started.")

        # Publisher to send movement commands
        self._pose_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber to receive LiDAR data
        self._scan_listener = self.create_subscription(LaserScan, "/scan", self.robot_controller, 10)

        # Initialize state variables
        self.previous_turn = None  # Keep track of the last turn direction
        self.backup_counter = 0  # Counter for backup maneuver

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()

        # Define the width of the range for obstacle detection
        detection_width = 5

        # Extract directional distances
        front = min(scan.ranges[:detection_width+1] + scan.ranges[-detection_width:])
        left = min(scan.ranges[90-detection_width:90+detection_width+1])
        right = min(scan.ranges[270-detection_width:270+detection_width+1])

        # Handle perpendicular wall collision with a backup maneuver
        if front < 0.3:  # Very close to an obstacle
            if self.backup_counter < 15:
                cmd.linear.x = -0.1  # Reverse slightly
                cmd.angular.z = 0.0
                self.backup_counter += 1
            else:
                self.backup_counter = 0  # Reset backup counter
                if left > right:  # Turn away from closer side
                    cmd.angular.z = 0.5
                else:
                    cmd.angular.z = -0.5
        elif front < 1.0:  # Obstacle ahead but not too close
            self.backup_counter = 0  # Reset backup counter
            if left > right:  # Prefer turning left if it's more open
                cmd.linear.x = 0.05
                cmd.angular.z = 0.5
                self.previous_turn = "left"
            elif right > left:  # Prefer turning right if it's more open
                cmd.linear.x = 0.05
                cmd.angular.z = -0.5
                self.previous_turn = "right"
            else:  # Equal spaces, alternate turns to prevent loops
                if self.previous_turn == "left":
                    cmd.linear.x = 0.05
                    cmd.angular.z = -0.5
                    self.previous_turn = "right"
                else:
                    cmd.linear.x = 0.05
                    cmd.angular.z = 0.5
                    self.previous_turn = "left"
        else:  # Path is clear ahead
            self.backup_counter = 0  # Reset backup counter
            if abs(left - right) > 0.5:  # Steer towards the more open side slightly
                cmd.angular.z = -0.3 if right > left else 0.3
            else:
                cmd.angular.z = 0.0
            cmd.linear.x = 0.3  # Move forward

        # Publish the command
        self._pose_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()
