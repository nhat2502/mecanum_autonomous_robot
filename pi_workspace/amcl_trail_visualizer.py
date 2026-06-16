import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
import math

class AmclTrailVisualizer(Node):
    def __init__(self):
        super().__init__('amcl_trail_visualizer')
        
        # Lắng nghe vị trí tuyệt đối từ AMCL
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.amcl_callback, 
            10
        )
        
        # Phát ra một Path chứa toàn bộ lịch sử di chuyển
        self.pub = self.create_publisher(Path, '/amcl_trail', 10)
        
        # Khởi tạo bản tin Path trống
        self.trail_path = Path()
        self.trail_path.header.frame_id = 'map'
        
        # Biến lưu trữ điểm trước đó để lọc nhiễu (chỉ vẽ khi xe có di chuyển)
        self.last_x = None
        self.last_y = None

    def amcl_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        # Lọc nhiễu: Chỉ thêm điểm mới nếu robot đã di chuyển ít nhất 2cm
        if self.last_x is not None and self.last_y is not None:
            dist = math.hypot(current_x - self.last_x, current_y - self.last_y)
            if dist < 0.02:
                return
                
        self.last_x = current_x
        self.last_y = current_y

        # Chuyển đổi định dạng để đưa vào Path
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose  # Lấy luôn cả góc hướng của xe
        
        # Thêm điểm mới vào đuôi quỹ đạo
        self.trail_path.poses.append(pose_stamped)
        self.trail_path.header.stamp = self.get_clock().now().to_msg()
        
        # Phát đường đi lên RViz2
        self.pub.publish(self.trail_path)

def main():
    rclpy.init()
    node = AmclTrailVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()