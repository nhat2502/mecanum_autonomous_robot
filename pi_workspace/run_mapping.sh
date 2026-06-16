#!/bin/bash
echo "========================================="
echo " KHỞI ĐỘNG HỆ THỐNG MAPPING (CARTOGRAPHER) "
echo "========================================="

# Source môi trường làm việc
source /opt/ros/humble/setup.bash
cd ~/robot_ws
source ~/robot_ws/install/setup.bash

# Cơ chế an toàn: Bắt tín hiệu Ctrl+C để tắt toàn bộ tiến trình con
trap "echo 'Đang tắt toàn bộ các node...'; kill 0" EXIT

echo "[1/6] Khởi động TF (Robot State Publisher)..."
ros2 launch mecanum_description state_publisher.launch.py &
sleep 2

echo "[2/6] Khởi động Lidar A1M8..."
ros2 launch sllidar_ros2 sllidar_a1_launch.py &
sleep 2

echo "[3/6] Khởi động Base Controller (Giao tiếp STM32)..."
ros2 run mecanum_controller base_controller &
sleep 2

echo "[4/6] Khởi động Laser Odometry (rf2o)..."
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &
sleep 3

echo "[5/6] Khởi động Cartographer Lõi..."
ros2 run cartographer_ros cartographer_node -configuration_directory ~/robot_ws/cartographer_config -configuration_basename mecanum_2d.lua --ros-args -r odom:=/odom_rf2o -r scan:=/scan &
sleep 2

echo "[6/6] Khởi động Node xuất bản đồ Grid..."
ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0 &

echo "========================================="
echo " TẤT CẢ CÁC NODE ĐÃ CHẠY THÀNH CÔNG!     "
echo " 👉 Nhấn Ctrl + C để dừng toàn bộ.      "
echo "========================================="
wait