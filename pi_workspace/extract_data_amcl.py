import sqlite3
import csv
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import math

def extract_experiment_data(db3_file_path, is_custom=False):
    conn = sqlite3.connect(db3_file_path)
    cursor = conn.cursor()

    # 1. Trích xuất /amcl_pose
    cursor.execute("SELECT id FROM topics WHERE name='/amcl_pose'")
    amcl_id = cursor.fetchone()[0]
    cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={amcl_id}")
    amcl_rows = cursor.fetchall()
    amcl_type = get_message('geometry_msgs/msg/PoseWithCovarianceStamped')

    with open('amcl_poses.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time_ns', 'X', 'Y', 'Yaw'])
        for t, data in amcl_rows:
            msg = deserialize_message(data, amcl_type)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            # Đổi Quaternion sang Yaw (Radian)
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            writer.writerow([t, x, y, yaw])

    # 2. Trích xuất Path đặt (/plan hoặc /custom_global_plan)
    path_topic = '/custom_global_plan' if is_custom else '/plan'
    cursor.execute(f"SELECT id FROM topics WHERE name='{path_topic}'")
    path_id = cursor.fetchone()[0]
    cursor.execute(f"SELECT data FROM messages WHERE topic_id={path_id} LIMIT 1")
    path_data = cursor.fetchone()[0]
    path_type = get_message('nav_msgs/msg/Path')
    path_msg = deserialize_message(path_data, path_type)

    with open('reference_path.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['X_ref', 'Y_ref', 'Yaw_ref'])
        for pose in path_msg.poses:
            x_r = pose.pose.position.x
            y_r = pose.pose.position.y
            q_r = pose.pose.orientation
            yaw_r = math.atan2(2.0 * (q_r.w * q_r.z + q_r.x * q_r.y), 1.0 - 2.0 * (q_r.y * q_r.y + q_r.z * q_r.z))
            writer.writerow([x_r, y_r, yaw_r])

    print("✅ Đã trích xuất xong amcl_poses.csv và reference_path.csv!")
    conn.close()

# HƯỚNG DẪN CHẠY:
# Đối với trường hợp 1 (Waypoint), sửa đường dẫn file .db3 và đặt is_custom=False
# Đối với trường hợp 2 (Custom Path), sửa đường dẫn file .db3 và đặt is_custom=True
extract_experiment_data('bag_waypoint/bag_waypoint_0.db3', is_custom=False)
