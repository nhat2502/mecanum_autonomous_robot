import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math

def get_pose(x, y, yaw, navigator):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    
    # Chuyển đổi góc Yaw sang Quaternion
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    print("🚀 Khởi động PA 1: Chạy Waypoint Hình chữ nhật 3.2m x 4.0m...")
    
    # Kích thước: X = 3.2m (đi thẳng), Y = 4.0m (rẽ trái)
    waypoints = [
        get_pose(3.2, 0.0, 0.0, navigator),           # Điểm 1: Tiến thẳng 3.2m (Mũi xe 0 độ)
        get_pose(3.2, 4.0, math.pi/2, navigator),     # Điểm 2: Rẽ trái đi dọc trục Y 4.0m (Mũi xe 90 độ)
        get_pose(0.0, 4.0, math.pi, navigator),       # Điểm 3: Rẽ trái đi ngược trục X 3.2m (Mũi xe 180 độ)
        get_pose(0.0, 0.0, -math.pi/2, navigator)     # Điểm 4: Rẽ trái lùi về điểm xuất phát (Mũi xe -90 độ)
    ]

    navigator.followWaypoints(waypoints)

    # Vòng lặp chờ phản hồi
    while not navigator.isTaskComplete():
        pass 

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Đã hoàn thành lộ trình Waypoint!")
    elif result == TaskResult.CANCELED:
        print("Nhiệm vụ bị hủy!")
    elif result == TaskResult.FAILED:
        print("Lỗi: Xe không thể đến được mục tiêu.")
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()