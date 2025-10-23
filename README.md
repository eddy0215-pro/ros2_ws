docker run -d \
  --name ros2_humble \
  --network host \
  --device=/dev/video0:/dev/video0 \
  --device=/dev/video1:/dev/video1 \
  -v ~/ros2_ws:/root/ros2_ws \
  --restart unless-stopped \
  ros:humble-ros-core

apt install -y \
  ros-humble-camera-calibration-parsers \
  ros-humble-camera-info-manager \
  ros-humble-launch-testing-ament-cmake \
  ros-humble-image-pipeline

apt install -y python3-colcon-common-extensions python3-pip

cd ~/ros2_ws/src
git clone https://github.com/clydemcqueen/opencv_cam.git
git clone https://github.com/ptrmu/ros2_shared.git
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

cd ~/opencv_cam_ws/install
source setup.bash
ros2 run opencv_cam opencv_cam_main --ros-args --param index:=0

ros2 run opencv_cam opencv_cam_main --ros-args \
  -p device:="/dev/video0" \
  -p width:=640 \
  -p height:=480 \
  -p fps:=30

apt install -y build-essential cmake
apt install python3-colcon-common-extensions -y

====
docker rm ros2_humble
docker run -it \
  --name ros2_humble \
  --network host \
  --privileged \
  --device=/dev/video0:/dev/video0 \
  --device=/dev/video1:/dev/video1 \
  --device=/dev/gpiomem:/dev/gpiomem \
  -v /sys/class/gpio:/sys/class/gpio \
  -v /sys/devices:/sys/devices \
  -v ~/ros2_ws:/root/ros2_ws \
  ros:humble-ros-core \
  bash

apt update  
apt install -y python3-colcon-common-extensions python3-opencv python3-rpi.gpio \
               python3-pip ros-humble-cv-bridge ros-humble-image-transport \
               ros-humble-rclpy

apt install -y ros-humble-camera-calibration-parsers ros-humble-camera-info-manager

cd ~/ros2_ws  
colcon build  
source install/setup.bash  

docker start ros2_humble  
docker exec -it ros2_humble bash  


root@dsm:/# cat ros_entrypoint.sh  
#!/bin/bash  
set -e  

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

# 추가: 워크스페이스 환경 로드
source "/root/ros2_ws/install/setup.bash"

# motor_node 자동 실행
ros2 run gpio_node motor_node &

# 초음파 노드만
ros2 run gpio_node ultrasonic_node &

# camera_node 실행 (백그라운드)
ros2 run opencv_cam opencv_cam_main --ros-args --param index:=1 &

# 컨테이너 실행 명령 그대로 수행
exec "$@"
