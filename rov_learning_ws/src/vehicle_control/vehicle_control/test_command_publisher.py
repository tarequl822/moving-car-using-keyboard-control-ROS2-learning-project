#!/usr/bin/env python3
"""
Test Command Publisher Node
---------------------------
Publishes simulated velocity commands to `/vehicle/cmd_vel` (geometry_msgs/msg/Twist)
at a regular interval to test subscriber communication and timeout detection.

Part of the ROS 2 Jazzy Remote-Controlled Vehicle Learning Project.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TestCommandPublisher(Node):
    """
    Automated test publisher that sends movement commands in a loop.
    Used to verify the ROS 2 communication layer without human keyboard input.
    """

    def __init__(self):
        super().__init__('test_command_publisher')

        # Declare configurable parameters
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)

        rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

        # Create publisher for Twist messages
        self.publisher = self.create_publisher(Twist, '/vehicle/cmd_vel', 10)

        # Predefined sequence of test maneuvers: (linear_x, angular_z, label, duration_cycles)
        self.pattern = [
            (self.linear_speed, 0.0, 'FORWARD', 4),
            (0.0, 0.0, 'STOP', 2),
            (0.0, self.angular_speed, 'TURN LEFT', 3),
            (0.0, 0.0, 'STOP', 2),
            (-self.linear_speed, 0.0, 'REVERSE', 4),
            (0.0, 0.0, 'STOP', 2),
            (0.0, -self.angular_speed, 'TURN RIGHT', 3),
            (0.0, 0.0, 'STOP', 2),
        ]

        self.pattern_index = 0
        self.cycle_in_step = 0

        # Periodic timer for publishing
        timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(timer_period, self.publish_command)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Test Command Publisher initialized.')
        self.get_logger().info(f'Publishing to /vehicle/cmd_vel at {rate_hz:.1f}Hz')
        self.get_logger().info('=' * 50)

    def publish_command(self):
        linear_x, angular_z, label, duration = self.pattern[self.pattern_index]

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Publishing [{label}]: Linear X={msg.linear.x:+.2f} m/s, Angular Z={msg.angular.z:+.2f} rad/s'
        )

        self.cycle_in_step += 1
        if self.cycle_in_step >= duration:
            self.cycle_in_step = 0
            self.pattern_index = (self.pattern_index + 1) % len(self.pattern)


def main(args=None):
    rclpy.init(args=args)
    node = TestCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down test publisher. Sending final STOP command.')
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
