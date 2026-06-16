import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Lấy đường dẫn tới file URDF
    urdf_file = os.path.join(
        get_package_share_directory('mecanum_description'),'urdf', 'mecanum.urdf'
    )
    
    # Đọc nội dung file URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Cấu hình Node robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    return LaunchDescription([
        rsp_node
    ])
