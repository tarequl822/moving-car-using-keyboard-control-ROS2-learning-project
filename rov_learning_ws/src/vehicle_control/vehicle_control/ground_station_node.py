#!/usr/bin/env python3
"""
Ground Station / Keyboard Controller Node
-----------------------------------------
Captures keyboard inputs in a non-blocking terminal loop and publishes
velocity commands (geometry_msgs/msg/Twist) to `/vehicle/cmd_vel`.

Key Controls:
  [W]        : Forward
  [S]        : Backward
  [A]        : Turn Left (positive angular Z)
  [D]        : Turn Right (negative angular Z)
  [SPACE]    : Immediate STOP (zero velocity)
  [+] or [=] : Increase linear and angular speed limits
  [-] or [_] : Decrease linear and angular speed limits
  [Q]        : Safe Exit (publishes STOP and restores terminal)

Safety Features:
  - Non-blocking stdin reading preserves ROS 2 executor responsiveness.
  - Periodic heartbeat publisher prevents watchdog starvation while active.
  - Configurable idle timeout automatically zeros velocities if operator is inactive.
  - Publishes zero-velocity STOP message on shutdown or interrupt.
  - Safe terminal restoration ensures the shell is not corrupted.

Part of the ROS 2 Jazzy Remote-Controlled Vehicle Learning Project.
"""

import os
import sys
import select
import termios
import tty
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


BANNER = """
==============================================================
   GROUND STATION - REMOTE VEHICLE KEYBOARD CONTROLLER
==============================================================
  Controls:
      [W] : Move Forward          [+] : Increase Speed Limit
  [A] [S] [D] : Left / Back / Right   [-] : Decrease Speed Limit
    [SPACE]   : Immediate STOP        [Q] : Safe Shutdown
--------------------------------------------------------------
  Target Topic : /vehicle/cmd_vel (geometry_msgs/msg/Twist)
==============================================================
"""


class GroundStationNode(Node):
    """
    ROS 2 Node acting as the operator's ground station.
    Converts keyboard inputs into velocity vectors and publishes them.
    """

    def __init__(self):
        super().__init__('ground_station_node')

        # -------------------------------------------------------------
        # Configurable Parameters
        # -------------------------------------------------------------
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('max_linear_speed', 1.0)       # m/s
        self.declare_parameter('max_angular_speed', 2.0)      # rad/s
        self.declare_parameter('linear_step', 0.1)           # m/s change per keypress
        self.declare_parameter('angular_step', 0.2)          # rad/s change per keypress
        self.declare_parameter('idle_timeout_seconds', 0.8)  # stop if no key pressed in this duration
        self.declare_parameter('vehicle_name', 'learning_vehicle')

        # Retrieve parameter values
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.linear_step = self.get_parameter('linear_step').get_parameter_value().double_value
        self.angular_step = self.get_parameter('angular_step').get_parameter_value().double_value
        self.idle_timeout_seconds = self.get_parameter('idle_timeout_seconds').get_parameter_value().double_value
        self.vehicle_name = self.get_parameter('vehicle_name').get_parameter_value().string_value

        # Current setpoints
        self.target_linear_speed = 0.5   # Base forward/backward step speed
        self.target_angular_speed = 1.0  # Base turn step speed

        # Current commanded velocities
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        # State tracking
        self.last_key_press_time = time.time()
        self.is_running = True
        self.lock = threading.Lock()

        # -------------------------------------------------------------
        # ROS 2 Publisher
        # -------------------------------------------------------------
        self.cmd_publisher = self.create_publisher(Twist, '/vehicle/cmd_vel', 10)

        # Periodic timer for regular command publishing
        # Regular publishing ensures the vehicle's command watchdog does not trigger
        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.publish_command)

        self.get_logger().info(f'Ground Station initialized for "{self.vehicle_name}"')
        self.get_logger().info(
            f'Rate: {self.publish_rate_hz}Hz | Max Linear: {self.max_linear_speed:.1f}m/s | '
            f'Max Angular: {self.max_angular_speed:.1f}rad/s'
        )

    def publish_command(self):
        """
        Periodic timer callback: publishes the active velocity setpoint.
        Enforces idle timeout safety: if operator stops pressing keys,
        decelerate / zero the velocities to prevent runaway.
        """
        with self.lock:
            now = time.time()
            elapsed = now - self.last_key_press_time

            # Idle timeout watchdog: automatically stop vehicle if no key is pressed
            if elapsed > self.idle_timeout_seconds and (self.cmd_linear_x != 0.0 or self.cmd_angular_z != 0.0):
                self.cmd_linear_x = 0.0
                self.cmd_angular_z = 0.0
                self.print_status('IDLE TIMEOUT -> VEHICLE STOPPED')

            msg = Twist()
            msg.linear.x = float(self.cmd_linear_x)
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = float(self.cmd_angular_z)

            self.cmd_publisher.publish(msg)

    def publish_stop(self):
        """Immediately sends a zero-velocity Twist command."""
        stop_msg = Twist()
        self.cmd_publisher.publish(stop_msg)
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

    def print_status(self, action_label: str):
        """Formats and prints the active driving state cleanly in the terminal."""
        sys.stdout.write(
            f"\r\033[K[{action_label:<25}] "
            f"Linear X: {self.cmd_linear_x:+.2f} m/s (Cap: {self.target_linear_speed:.2f}) | "
            f"Angular Z: {self.cmd_angular_z:+.2f} rad/s (Cap: {self.target_angular_speed:.2f})"
        )
        sys.stdout.flush()

    def handle_key(self, key: str):
        """
        Maps a single character input to a motion command or configuration change.
        """
        with self.lock:
            self.last_key_press_time = time.time()
            key_upper = key.upper()

            if key_upper == 'W':
                self.cmd_linear_x = self.target_linear_speed
                self.cmd_angular_z = 0.0
                self.print_status('FORWARD')

            elif key_upper == 'S':
                self.cmd_linear_x = -self.target_linear_speed
                self.cmd_angular_z = 0.0
                self.print_status('REVERSE')

            elif key_upper == 'A':
                self.cmd_angular_z = self.target_angular_speed
                self.print_status('TURN LEFT')

            elif key_upper == 'D':
                self.cmd_angular_z = -self.target_angular_speed
                self.print_status('TURN RIGHT')

            elif key == ' ':
                self.cmd_linear_x = 0.0
                self.cmd_angular_z = 0.0
                self.publish_stop()
                self.print_status('STOP')

            elif key in ('+', '='):
                self.target_linear_speed = min(
                    self.max_linear_speed,
                    round(self.target_linear_speed + self.linear_step, 2)
                )
                self.target_angular_speed = min(
                    self.max_angular_speed,
                    round(self.target_angular_speed + self.angular_step, 2)
                )
                self.print_status('SPEED INCREASED')

            elif key in ('-', '_'):
                self.target_linear_speed = max(
                    self.linear_step,
                    round(self.target_linear_speed - self.linear_step, 2)
                )
                self.target_angular_speed = max(
                    self.angular_step,
                    round(self.target_angular_speed - self.angular_step, 2)
                )
                self.print_status('SPEED DECREASED')

            elif key_upper == 'Q':
                self.publish_stop()
                self.is_running = False
                print('\n\n[Ground Station] Safe shutdown requested by user.')
                return False

            else:
                # Ignore unrecognized keys
                pass

        return True


def keyboard_listener(node: GroundStationNode):
    """
    Worker thread that reads single characters from standard input
    using termios non-blocking raw mode.
    """
    # Check if standard input is an interactive terminal (TTY)
    if not sys.stdin.isatty():
        node.get_logger().error('Standard input is not a TTY. Keyboard controller requires an interactive terminal.')
        return

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        while node.is_running and rclpy.ok():
            # Use select with a 100ms timeout so the loop can periodically check node.is_running
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                char = sys.stdin.read(1)
                # Check for Ctrl+C (character 0x03)
                if char == '\x03':
                    node.publish_stop()
                    node.is_running = False
                    break
                should_continue = node.handle_key(char)
                if not should_continue:
                    break
    finally:
        # Guarantee terminal attributes are restored even if an exception occurs
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = GroundStationNode()

    print(BANNER)
    node.print_status('READY - PRESS W/A/S/D')

    # Run the keyboard listener in a background daemon thread
    # This prevents blocking the ROS 2 executor from processing timers and callbacks!
    input_thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    input_thread.start()

    try:
        while node.is_running and rclpy.ok():
            # Spin with a short timeout to handle ROS 2 timer callbacks smoothly
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()
        print('[Ground Station] Node terminated cleanly. Terminal restored.\n')


if __name__ == '__main__':
    main()
