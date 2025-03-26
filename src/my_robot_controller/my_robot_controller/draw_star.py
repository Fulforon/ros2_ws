#!/usr/bin/env python3
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DrawStarNode(Node):
    def __init__(self):
        super().__init__("draw_star")
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Draw star node has started")

        # Define the movement and angle instructions
        self.instructions = [
            (2.0, 1.047), (2.0, 1.047), (2, 1.047), (2.0, 1.047),
            (2, 1.047), (2, 1.047)
        ]
        self.current_instruction_index = 0
        self.turtle_pos = (0.0, 0.0)
        self.current_angle = 0.0  # Angle in radians
        
        # Timer setup to control the sequence of movements
        self.timer = self.create_timer(1.0, self.execute_instructions)
        self.moving_forward = False
        self.turning = False
        self.start_time = time.time()

    def execute_instructions(self):
        # If all instructions are completed, stop
        if self.current_instruction_index >= len(self.instructions):
            self.get_logger().info("Finished drawing the star.")
            return

        distance, angle = self.instructions[self.current_instruction_index]
        
        # If we are moving forward, send the forward movement command
        if not self.moving_forward and not self.turning:
            self.get_logger().info(f"Moving forward {distance} units.")
            self.move_forward(distance)
            self.moving_forward = True
            self.start_time = time.time()

        # Check if enough time has passed to complete the forward movement
        elif self.moving_forward and (time.time() - self.start_time) > distance / 2:
            self.get_logger().info(f"Turn by {angle} radians.")
            self.turn(angle)
            self.moving_forward = False
            self.turning = True
            self.start_time = time.time()

        # If turning is done, move to the next instruction
        elif self.turning and (time.time() - self.start_time) > abs(angle) / 2:
            self.turning = False
            self.current_instruction_index += 1
            self.get_logger().info(f"Completed instruction {self.current_instruction_index}.")
    
    def move_forward(self, distance):
        # Set the linear speed to move forward
        msg = Twist()
        msg.linear.x = 1.0  # Move at a constant speed
        self.cmd_vel_pub.publish(msg)
    
    def turn(self, angle):
        # Set the angular velocity to rotate
        msg = Twist()
        msg.angular.z = angle / abs(angle) * 1.0  # Rotate by the given angle
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DrawStarNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
