#!/bin/bash
# ==========================================
# ROS2 워크스페이스 빌드 + 노드 실행 스크립트
# 실행 시: ./build_ros2_ws.sh
# ==========================================

set -e  # 오류 발생 시 스크립트 중단

# 워크스페이스 위치
WS_DIR=~/ros2_ws

echo "==> 워크스페이스로 이동: $WS_DIR"
cd $WS_DIR

# 기존 빌드 제거 (선택)
echo "==> 기존 빌드, install, log 삭제"
rm -rf build/ install/ log/

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 워크스페이스 빌드
echo "==> colcon 빌드 시작"
colcon build --symlink-install

# 빌드 완료 후 환경 설정
echo "==> 환경 설정"
source install/setup.bash

echo "==> 빌드 완료! ROS2 워크스페이스가 준비되었습니다."

# ==========================================
# 노드 실행
# ==========================================
# echo "==> motor_node 실행"
# ros2 run gpio_node motor_node &

# echo "==> ultrasonic_node 실행"
# ros2 run gpio_node ultrasonic_node &

# echo "==> opencv_cam_main 실행 (index=1, 640x480, 30fps)"
# ros2 run opencv_cam opencv_cam_main --ros-args -p index:=1 -p width:=640 -p height:=480 -p fps:=30 &

# echo "==> 모든 노드 실행 완료!"

