#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /mnt/e/meriner_ros/rov_learning_ws/install/setup.bash

echo "=== Starting Vehicle Command Monitor in background ==="
ros2 run vehicle_control vehicle_command_monitor > /tmp/monitor.log 2>&1 &
MON_PID=$!
sleep 2

echo "=== Publishing FORWARD command via ros2 topic pub ==="
ros2 topic pub --once /vehicle/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

sleep 0.5

echo "=== Publishing TURN LEFT command via ros2 topic pub ==="
ros2 topic pub --once /vehicle/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.8}}'

sleep 0.5

echo "=== Publishing STOP command via ros2 topic pub ==="
ros2 topic pub --once /vehicle/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

echo "=== Waiting 1.5 seconds to trigger timeout check ==="
sleep 1.5

kill $MON_PID 2>/dev/null || true
wait $MON_PID 2>/dev/null || true

echo "=== Vehicle Command Monitor Log Output ==="
cat /tmp/monitor.log
