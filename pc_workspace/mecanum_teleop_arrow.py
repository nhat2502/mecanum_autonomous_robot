import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
========================================
ĐIỀU KHIỂN ROBOT MECANUM BẰNG MŨI TÊN
========================================
[Mũi tên Lên]    : Tiến tới
[Mũi tên Xuống]  : Lùi lại
[Mũi tên Trái]   : Xoay tại chỗ sang Trái
[Mũi tên Phải]   : Xoay tại chỗ sang Phải

[Phím 'A']       : Trượt ngang sang Trái (Strafe)
[Phím 'D']       : Trượt ngang sang Phải (Strafe)

[Phím SPACE]     : Phanh khẩn cấp
CTRL-C để thoát
========================================
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b': # Xử lý mã phím mũi tên
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('mecanum_arrow_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    speed = 0.2  # Vận tốc tịnh tiến (m/s) - Để 0.15 đi chậm vẽ map cho nét
    turn = 0.4    # Vận tốc xoay (rad/s)
    
    print(msg)
    twist = Twist()
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            
            if key == '\x1b[A':   # Mũi tên Lên
                twist.linear.x, twist.linear.y, twist.angular.z = speed, 0.0, 0.0
            elif key == '\x1b[B': # Mũi tên Xuống
                twist.linear.x, twist.linear.y, twist.angular.z = -speed, 0.0, 0.0
            elif key == '\x1b[C': # Mũi tên Phải
                twist.linear.x, twist.linear.y, twist.angular.z = 0.0, 0.0, -turn
            elif key == '\x1b[D': # Mũi tên Trái
                twist.linear.x, twist.linear.y, twist.angular.z = 0.0, 0.0, turn
            elif key in ['a', 'A']: # Trượt Trái
                twist.linear.x, twist.linear.y, twist.angular.z = 0.0, speed, 0.0
            elif key in ['d', 'D']: # Trượt Phải
                twist.linear.x, twist.linear.y, twist.angular.z = 0.0, -speed, 0.0
            elif key == ' ':      # Phanh
                twist.linear.x, twist.linear.y, twist.angular.z = 0.0, 0.0, 0.0
            elif key == '\x03':   # Ctrl+C
                break
                
            if key != '':
                pub.publish(twist)
                
    except Exception as e:
        print(e)
    finally:
        # Dừng xe khi thoát
        twist.linear.x, twist.linear.y, twist.angular.z = 0.0, 0.0, 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
