import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import struct
import math

class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        
        # Cấu hình cổng UART chân cứng GPIO
        self.declare_parameter('serial_port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f"Đã kết nối STM32 tại {port} với baudrate {baud}")
        except Exception as e:
            self.get_logger().error(f"Lỗi mở cổng Serial: {e}")
            return

        # --- PHẦN LẮNG NGHE LỆNH (SUBSCRIBER) ---
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # --- PHẦN PHÁT DỮ LIỆU ODOMETRY THÔ (PUBLISHER) ---
        # Tên Topic đã được đổi thành 'odom_raw'
        self.odom_pub = self.create_publisher(Odometry, 'odom_encoder', 10)
        
        # Biến tích lũy vị trí
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Vòng lặp đọc dữ liệu UART với tần số 20Hz (0.05s)
        self.timer = self.create_timer(0.05, self.read_serial_data)

    def cmd_vel_callback(self, msg):
        # Đóng gói 14 bytes gửi xuống STM32 theo chuẩn Little Endian
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        omega = float(msg.angular.z)
        try:
            packet = struct.pack('<BfffB', 0xAA, vx, vy, omega, 0x55)
            self.serial_conn.write(packet)
        except Exception as e:
            self.get_logger().error(f"Lỗi gửi dữ liệu Serial: {e}")

    def read_serial_data(self):
        while self.serial_conn.in_waiting >= 14:
            first_byte = self.serial_conn.read(1)
            if first_byte == b'\xBB': 
                data = self.serial_conn.read(13)
                if len(data) == 13 and data[12] == 0x66: 
                    # Đọc dữ liệu thô từ STM32
                    vx_stm, vy_stm, omega_stm = struct.unpack('<fff', data[0:12])
                    
                    # LẬT NGƯỢC LẠI ĐỂ BÁO CÁO ĐÚNG CHO ROS 2
                    vx_real = vx_stm
                    vy_real = vy_stm
                    omega_real = omega_stm
                    
                    self.publish_odometry(vx_real, vy_real, omega_real)
                else:
                    self.serial_conn.reset_input_buffer()

    def publish_odometry(self, vx, vy, omega):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Tính toán động học thuận dời vị trí cho xe Mecanum
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_th = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th

        # Chuyển đổi góc xoay (Euler) sang Quaternion cho ROS 2
        q_w = math.cos(self.theta / 2.0)
        q_z = math.sin(self.theta / 2.0)

        # Đóng gói và phát bản tin Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'serial_conn') and node.serial_conn.is_open:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()