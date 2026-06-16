import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

from langchain_ollama import ChatOllama
from langchain.agents import tool
from rosa import ROSA
from rosa.prompts import RobotSystemPrompts

# --- BIẾN TOÀN CỤC ---
node = None
vel_publisher = None
nav_client = None

# Tọa độ các điểm trong kho
LOCATIONS = {
    "tram_sac": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "ke_a": {"x": 2.5, "y": 1.5, "z": 0.707, "w": 0.707},
    "ke_b": {"x": 2.5, "y": -1.5, "z": -0.707, "w": 0.707}
}

# --- CÔNG CỤ (TOOLS) CHO LLM ---
@tool
def move_mecanum(vel_x: float, vel_y: float, angular_z: float) -> str:
    """
    Điều khiển vận tốc của robot Mecanum.
    - vel_x: Tiến (+) / lùi (-).
    - vel_y: Trượt ngang trái (+) / phải (-). Dùng để lách qua khe hẹp.
    - angular_z: Xoay tại chỗ.
    """
    global vel_publisher
    twist = Twist()
    twist.linear.x = float(vel_x)
    twist.linear.y = float(vel_y) 
    twist.angular.z = float(angular_z)
    vel_publisher.publish(twist)
    return f"Đã gửi lệnh vận tốc: X={vel_x}, Y={vel_y}, Góc={angular_z}"

@tool
def stop_robot() -> str:
    """Dừng robot khẩn cấp."""
    global vel_publisher
    twist = Twist()
    vel_publisher.publish(twist)
    return "Đã phanh robot."

@tool
def navigate_to_location(location_name: str) -> str:
    """Đưa robot đến một vị trí xác định tự động bằng Nav2 (như: tram_sac, ke_a, ke_b)."""
    global nav_client, node
    loc_key = location_name.lower().replace(" ", "_")
    
    if loc_key not in LOCATIONS:
        return f"Lỗi: Không tìm thấy '{location_name}'."

    target = LOCATIONS[loc_key]
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = target["x"]
    goal_msg.pose.pose.position.y = target["y"]
    goal_msg.pose.pose.orientation.z = target["z"]
    goal_msg.pose.pose.orientation.w = target["w"]

    nav_client.send_goal_async(goal_msg)
    return f"Đã gửi tọa độ mục tiêu '{location_name}' cho hệ thống dẫn đường Nav2."

# --- HÀM MAIN ---
def main():
    global node, vel_publisher, nav_client
    rclpy.init()
    node = rclpy.create_node("ubuntu_llm_commander")

    # Khởi tạo giao tiếp ROS 2
    vel_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    print("Đang khởi động kết nối với Ollama...")
    
    # Kết nối với Ollama trên Ubuntu PC
    llm = ChatOllama(
        model="qwen2.5:3b",
        temperature=0.0, # Giữ ở mức 0 để AI suy luận logic, không sáng tạo lung tung
        base_url="http://localhost:11434"
    )

    prompt = RobotSystemPrompts()
    prompt.embodiment = (
        "Bạn là trí tuệ nhân tạo điều khiển robot kho bãi. "
        "Giới hạn cơ thể: Khung xe 24x15x7cm, khoảng sáng gầm 6cm. Bánh mecanum đường kính 6cm cho phép trượt ngang (strafing). Lidar đặt cách mặt đất 20cm. "
        "Hãy ưu tiên dùng chức năng trượt ngang khi cần lách qua không gian chật hẹp. "
        "Nhiệm vụ: Dịch lệnh tiếng Việt của con người thành các hành động gọi tool tương ứng."
    )

    agent = ROSA(
        ros_version=2,
        llm=llm,
        tools=[move_mecanum, stop_robot, navigate_to_location],
        prompts=prompt,
    )

    print("\n[HỆ THỐNG SẴN SÀNG]")
    print("Giao tiếp với robot. Gõ 'exit' để thoát.\n")

    try:
        while rclpy.ok():
            user_msg = input("Người dùng: ")
            if user_msg.lower() == "exit":
                break
            
            # Gửi lệnh cho LLM xử lý
            response = agent.invoke(user_msg)[0]
            if isinstance(response, dict) and "text" in response:
                print(f"AI: {response['text']}\n")
            else:
                print(f"AI: {response}\n")
                
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        agent.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
