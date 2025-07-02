#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info("Keyboard Teleop Started. Use arrow keys to move. Press 'q' to quit.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(3)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        move_bindings = {
            '\x1b[A': (1.0, 0.0),   # Up arrow
            '\x1b[B': (-1.0, 0.0),  # Down arrow
            '\x1b[C': (0.0, -1.0),  # Right arrow
            '\x1b[D': (0.0, 1.0),   # Left arrow
        }

        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'q':
                    break
                twist = Twist()
                if key in move_bindings:
                    linear, angular = move_bindings[key]
                    twist.linear.x = linear
                    twist.angular.z = angular
                    self.publisher_.publish(twist)
                else:
                    # Stop if unknown key
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            twist = Twist()  # Stop robot
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
