#!/bin/bash
echo "========================================="
echo "   KHỞI ĐỘNG HỆ THỐNG TỰ HÀNH (NAV 2)     "
echo "========================================="

# Source môi trường làm việc
source /opt/ros/humble/setup.bash
cd ~/robot_ws
source ~/robot_ws/install/setup.bash

# Cơ chế an toàn: Bắt tín hiệu Ctrl+C để tắt toàn bộ tiến trình con
trap "echo 'Đang tắt toàn bộ các node Nav2...'; kill 0" EXIT

echo "[1/5] Khởi động TF (Robot State Publisher)..."
ros2 launch mecanum_description state_publisher.launch.py &
sleep 2

echo "[2/5] Khởi động Lidar A1M8..."
ros2 launch sllidar_ros2 sllidar_a1_launch.py &
sleep 2

echo "[3/5] Khởi động Base Controller (Giao tiếp STM32)..."
ros2 run mecanum_controller base_controller &
sleep 2

echo "[4/5] Khởi động Laser Odometry (rf2o)..."
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &
sleep 3

echo "[5/5] Khởi động Nav2 Stack..."
ros2 launch nav2_bringup bringup_launch.py autostart:=true map:=/home/raspberry_nhat/robot_ws/maps/map_101h1.yaml params_file:=/home/raspberry_nhat/robot_ws/nav2_config/nav2_params.yaml &

echo "========================================="
echo " HỆ THỐNG NAV 2 ĐÃ SẴN SÀNG!             "
echo "  Mở RViz2 trên PC và cấp Initial Pose. "
echo " Nhấn Ctrl + C để dừng toàn bộ.        "
echo "========================================="
wait