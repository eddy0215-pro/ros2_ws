#!/bin/bash
# ==========================================
# ROS2 Humble + colcon + opencv_cam/ros2_shared 의존 패키지 설치 스크립트
# 실행: ./install_ros2_deps.sh
# ==========================================

set -e  # 오류 발생 시 중단

echo "==> 패키지 목록 업데이트"
apt update
apt upgrade -y

echo "==> 필수 빌드 도구 설치"
apt install -y build-essential cmake git wget curl unzip python3-colcon-common-extensions python3-pip python3-rosdep python3-vcstool

apt install -y python3-rpi.gpio

echo "==> ROS2 Humble 데스크탑 및 의존 패키지 설치 (이미 설치되어 있으면 생략 가능)"
apt install -y ros-humble-desktop python3-rosdep python3-rosinstall-generator python3-vcstool build-essential

echo "==> GPIO 제어용 패키지 설치"
apt install -y python3-rpi.gpio

echo "==> OpenCV 및 v4l2 / CSI 관련 패키지 설치"
apt install -y libopencv-dev python3-opencv v4l-utils v4l2loopback-dkms v4l2loopback-utils
apt install -y ros-humble-camera-calibration-parsers

echo "==> Python 패키지 설치 (ROS2 + OpenCV용)"
pip3 install --user numpy

echo "==> rosdep 초기화 및 업데이트"
rosdep init || true
rosdep update

echo "==> 설치 완료!"

