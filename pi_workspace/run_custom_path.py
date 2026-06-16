import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
import math

class CustomPathClient(Node):
    def __init__(self):
        super().__init__('custom_path_client')
        self.path_pub = self.create_publisher(Path, '/custom_global_plan', 10)
        self.action_client = ActionClient(self, FollowPath, 'follow_path')
        
        # Tạo quỹ đạo 1 lần duy nhất vào bộ nhớ
        self.custom_path = self.generate_path(length=3.2, width=4.0, step=0.05)
        
        # TẠO TIMER: Phát lại quỹ đạo này mỗi 1 giây để RViz2 luôn nhìn thấy
        self.timer = self.create_timer(1.0, self.publish_path)

    def publish_path(self):
        # Cập nhật thời gian mới nhất để RViz2 không báo lỗi Timeout
        self.custom_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.custom_path)

    def send_goal(self):
        self.get_logger().info('Đang kết nối tới DWB Controller...')
        self.action_client.wait_for_server()
        
        goal_msg = FollowPath.Goal()
        goal_msg.path = self.custom_path  # Gửi quỹ đạo đã lưu
        goal_msg.controller_id = 'FollowPath'

        self.get_logger().info('🚀 Đã phát lệnh chạy quỹ đạo Custom 3.2m x 4.0m!')
        self.action_client.send_goal_async(goal_msg)

    def make_pose(self, path_msg, x, y, yaw):
        p = PoseStamped()
        p.header = path_msg.header
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        
        # Chuyển đổi góc Yaw (Radian) sang Quaternion
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def generate_path(self, length=3.2, width=4.0, step=0.05):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Số lượng điểm trên mỗi cạnh (sử dụng round để tránh lỗi sai số dấu phẩy động)
        steps_x = int(round(length / step))
        steps_y = int(round(width / step))
        
        # Cạnh 1: Tiến thẳng 3.2m theo trục X (Mũi xe 0 rad)
        for i in range(0, steps_x + 1):
            path.poses.append(self.make_pose(path, i * step, 0.0, 0.0))
            
        # Cạnh 2: Rẽ trái đi 4.0m theo trục Y (Mũi xe pi/2 rad = 90 độ)
        for i in range(1, steps_y + 1):
            path.poses.append(self.make_pose(path, length, i * step, math.pi / 2.0))
            
        # Cạnh 3: Rẽ trái đi lùi ngược 3.2m theo trục X (Mũi xe pi rad = 180 độ)
        for i in range(1, steps_x + 1):
            path.poses.append(self.make_pose(path, length - (i * step), width, math.pi))
            
        # Cạnh 4: Rẽ trái đi xuống 4.0m về gốc tọa độ (Mũi xe -pi/2 rad = -90 độ)
        for i in range(1, steps_y + 1):
            path.poses.append(self.make_pose(path, 0.0, width - (i * step), -math.pi / 2.0))
            
        return path

def main():
    rclpy.init()
    node = CustomPathClient()
    node.send_goal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()