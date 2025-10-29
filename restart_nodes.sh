#!/bin/bash
# =========================================
# ROS2 노드 종료 후 재실행 스크립트
# =========================================
# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source ~/ros2_ws/install/setup.bash

# --------------------------
# 1️⃣ 실행 중인 관련 ROS2 노드 종료
# --------------------------
echo "Stopping ROS2 nodes..."
pkill -f "motor_node"
pkill -f "ultrasonic_node"
pkill -f "opencv_cam_main"

# --------------------------
# 2️⃣ 각 노드 재실행
# --------------------------
echo "Starting ROS2 nodes..."

sleep 3

# motor_node
ros2 run gpio_node motor_node &

sleep 3

# ultrasonic_node
ros2 run gpio_node ultrasonic_node &

sleep 3

# opencv_cam_main (예: index 1, 640x480, 30fps)
ros2 run opencv_cam opencv_cam_main --ros-args -p index:=3 -p width:=640 -p height:=480 -p fps:=30 &

echo "All ROS2 nodes restarted."

