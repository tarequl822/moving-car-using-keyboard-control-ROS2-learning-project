#!/usr/bin/env python3
"""
Vehicle Command Monitor Node
----------------------------
Subscribes to `/vehicle/cmd_vel` (geometry_msgs/msg/Twist).
Monitors received velocity commands, formats and logs them, detects stop commands,
and alerts if commands stop arriving (command timeout).

Part of the ROS 2 Jazzy Remote-Controlled Vehicle Learning Project.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VehicleCommandMonitor(Node):
    """
    Monitors velocity commands destined for the vehicle.
    Serves as an essential safety layer and diagnostic tool.
    """

    def __init__(self):
        super().__init__('vehicle_command_monitor')

        # Declare configurable parameters with default values
        self.declare_parameter('timeout_seconds', 1.0)
        self.declare_parameter('check_rate_hz', 5.0)

        self.timeout_seconds = self.get_parameter('timeout_seconds').get_parameter_value().double_value
        check_rate_hz = self.get_parameter('check_rate_hz').get_parameter_value().double_value

        # Create subscriber to the movement command topic
        # Topic: /vehicle/cmd_vel
        # Message Type: Twist (contains linear: [x,y,z] and angular: [x,y,z])
        # Queue Depth: 10 (keeps up to 10 latest messages if processing takes time)
        self.subscription = self.create_subscription(
            Twist,
            '/vehicle/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Internal state tracking
        self.last_cmd_time = None
        self.timeout_reported = False
        self.command_count = 0

        # Create a periodic timer to check for command timeouts
        timer_period = 1.0 / check_rate_hz
        self.timeout_timer = self.create_timer(timer_period, self.check_command_timeout)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Vehicle Command Monitor initialized and listening on /vehicle/cmd_vel')
        self.get_logger().info(f'Timeout threshold: {self.timeout_seconds:.1f}s | Check rate: {check_rate_hz:.1f}Hz')
        self.get_logger().info('=' * 50)

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function executed every time a new Twist message arrives.
        """
        self.command_count += 1
        now = self.get_clock().now()
        self.last_cmd_time = now
        self.timeout_reported = False  # Reset timeout warning flag

        lin_x = msg.linear.x
        lin_y = msg.linear.y
        ang_z = msg.angular.z

        # Check if this is a stop command (all velocities near zero)
        is_stop = (abs(lin_x) < 1e-4 and abs(lin_y) < 1e-4 and abs(ang_z) < 1e-4)

        if is_stop:
            self.get_logger().info(
                f'[CMD #{self.command_count}] [STOP] Linear: ({lin_x:+.2f}, {lin_y:+.2f}) m/s | Angular Z: {ang_z:+.2f} rad/s'
            )
        else:
            # Determine primary motion direction for clear logging
            direction = []
            if lin_x > 0.01:
                direction.append('FORWARD')
            elif lin_x < -0.01:
                direction.append('REVERSE')

            if ang_z > 0.01:
                direction.append('TURN_LEFT')
            elif ang_z < -0.01:
                direction.append('TURN_RIGHT')

            dir_str = '+'.join(direction) if direction else 'MOVE'

            self.get_logger().info(
                f'[CMD #{self.command_count}] [{dir_str}] Linear X: {lin_x:+.2f} m/s | Angular Z: {ang_z:+.2f} rad/s'
            )

    def check_command_timeout(self):
        """
        Periodic safety check: alerts if commands stop arriving.
        In a physical vehicle or ROV, a timeout triggers a hardware or software failsafe.
        """
        if self.last_cmd_time is None:
            # No commands have ever been received yet
            return

        now = self.get_clock().now()
        elapsed_seconds = (now - self.last_cmd_time).nanoseconds / 1e9

        if elapsed_seconds > self.timeout_seconds and not self.timeout_reported:
            self.get_logger().warn(
                f'[TIMEOUT ALERT] No command received for {elapsed_seconds:.2f}s '
                f'(exceeds threshold of {self.timeout_seconds:.1f}s). Vehicle should halt!'
            )
            self.timeout_reported = True


def main(args=None):
    rclpy.init(args=args)
    node = VehicleCommandMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down command monitor.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
