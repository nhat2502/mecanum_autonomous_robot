import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped 
from nav2_msgs.action import NavigateToPose
import time
import math
import threading
import queue
from langchain_groq import ChatGroq
from langchain.agents import tool, AgentExecutor, create_tool_calling_agent
from langchain_core.prompts import ChatPromptTemplate
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped 
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String  

# --- GLOBAL VARIABLES ---
node = None
vel_publisher = None
nav_client = None
progress_pub = None
initial_pose_pub = None
# Biến lưu tọa độ thực tế của xe
current_x = 0.0
current_y = 0.0
current_theta = 0.0

action_queue = queue.Queue()       # Hàng đợi chứa các chuỗi lệnh
current_goal_handle = None         # Lưu trạng thái Nav2 hiện tại để có thể STOP khẩn cấp

# Map Database
LOCATIONS = {
    "home_base": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "charger": {"x": 0.5, "y": 0.0, "z": 0.0, "w": 1.0},
    "shelf_a": {"x": 1.2, "y": -0.4, "z": 0.707, "w": 0.707},
    "shelf_b": {"x": 0.4, "y": 0.4, "z": -0.707, "w": 0.707}
}

# ==========================================
# ROS 2 CALLBACKS
# ==========================================

# ==========================================
# TASK QUEUE WORKER
# ==========================================
def notify_web(text):
    """Helper: Print to terminal and publish to Web via /ai_progress topic"""
    print(text)
    if progress_pub is not None:
        msg = String()
        msg.data = text
        progress_pub.publish(msg)

def task_worker_thread():
    """Background thread to fetch tasks from the Queue and execute via Nav2 sequentially"""
    global current_goal_handle, nav_client
    
    while rclpy.ok():
        try:
            task = action_queue.get(timeout=1.0)
        except queue.Empty:
            continue

        notify_web(f"▶️ [EXECUTING]: {task['desc']}...")

        if task.get('type') == 'wait':
            time.sleep(task['duration'])
            notify_web(f"✅ [COMPLETED]: Finished waiting for {task['duration']} seconds.")
            action_queue.task_done()
            continue
            
        goal_event = threading.Event()

        def goal_response_callback(future):
            global current_goal_handle
            goal_handle = future.result()
            if not goal_handle.accepted:
                notify_web(f"❌ [REJECTED]: Nav2 cannot execute '{task['desc']}'!")
                goal_event.set()
                return

            current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(get_result_callback)

        def get_result_callback(future):
            global current_goal_handle
            status = future.result().status
            notify_web(f"✅ [FINISHED]: '{task['desc']}' (Mã: {status})")
            current_goal_handle = None
            goal_event.set()

        send_goal_future = nav_client.send_goal_async(task['goal_msg'])
        send_goal_future.add_done_callback(goal_response_callback)

        goal_event.wait() 
        action_queue.task_done()
#===========================================
def amcl_pose_callback(msg):
    """Update the robot's absolute position from the /amcl_pose topic"""
    global current_x, current_y, current_theta
    
    # Get X, Y coordinates
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    
    # Chuyển đổi Quaternion sang góc Euler (Yaw - độ)
    q = msg.pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    current_theta = math.degrees(math.atan2(siny_cosp, cosy_cosp))

# ==========================================
# LANGCHAIN TOOLS
# ==========================================

@tool
def get_robot_info(dummy: str = "") -> str:
    """Use this tool when the user asks about the robot's specs, dimensions, or capabilities."""
    info = (
        "Robot Name: Meca (Mecanum AI AMR).\n"
        "Dimensions: Length 24cm, Width 15cm, Height 7cm. Ground Clearance: 6cm.\n"
        "Drivetrain: 4 Mecanum wheels, capable of strafing."
    )
    return info

@tool
def get_location_names(dummy: str = "") -> str:
    """Use when you need to know the list of saved locations or coordinates in the warehouse."""
    return f"Saved locations: {', '.join(LOCATIONS.keys())}"

@tool
def get_current_location(dummy: str = "") -> str:
    """Use this tool when the user asks 'where are you', 'your location', or 'current pose'."""
    global current_x, current_y, current_theta
    return f"My exact real-time pose is: X = {current_x:.2f} meters, Y = {current_y:.2f} meters, Theta = {current_theta:.2f} degrees."

@tool
def save_current_location(location_name: str) -> str:
    """Saves the robot's current real-time pose as a new named location in the database.
    Use this when the user says 'save this location as X', 'remember this pose as Y', or 'mark this spot'.
    """
    global current_x, current_y, current_theta, LOCATIONS
    
    loc_key = location_name.lower().replace(" ", "_")
    yaw_rad = math.radians(current_theta)
    q_z = math.sin(yaw_rad / 2.0)
    q_w = math.cos(yaw_rad / 2.0)
    
    LOCATIONS[loc_key] = {
        "x": current_x,
        "y": current_y,
        "z": q_z,
        "w": q_w
    }
    
    return f"Successfully saved the current pose (X={current_x:.2f}m, Y={current_y:.2f}m) as '{location_name}'."
@tool
def set_initial_pose(x: float, y: float, yaw_degrees: float) -> str:
    """Set the robot's initial pose estimate (2D Pose Estimate) for AMCL localization."""
    global node, initial_pose_pub
    
    yaw_rad = math.radians(float(yaw_degrees))
    q_z = math.sin(yaw_rad / 2.0)
    q_w = math.cos(yaw_rad / 2.0)
    
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = q_z
    msg.pose.pose.orientation.w = q_w
    
    # Cấu hình ma trận hiệp phương sai (Covariance) cơ bản
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853892326654787
    
    initial_pose_pub.publish(msg)
    return f"Successfully set initial 2D pose to X={x}, Y={y}, Yaw={yaw_degrees} degrees."

@tool
def launch_rviz(dummy: str = "") -> str:
    """Launch the RViz2 visualization and mapping interface."""
    try:
        # Lệnh mở RViz2 chuẩn của Nav2
        cmd = "ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz"
        subprocess.Popen(cmd, shell=True)
        return "RViz2 has been launched successfully on the host machine."
    except Exception as e:
        return f"Failed to launch RViz2: {str(e)}"

@tool
def wait_time(seconds: float) -> str:
    """Use this tool when the user explicitly asks the robot to wait, pause, or rest for a specific amount of time.
    Also use this tool to add a default wait time of 2.0 seconds BETWEEN any two movement or rotation actions if the user doesn't specify a wait time.
    """
    global action_queue
    duration = float(seconds)
    
    # Đưa nhiệm vụ chờ vào hàng đợi với cờ type='wait'
    action_queue.put({
        'desc': f"Waiting for {duration} seconds", 
        'type': 'wait', 
        'duration': duration
    })
    return f"Added [Waiting for {duration} seconds] to the task queue."

@tool
def navigate_to_location(location_name: str) -> str:
    """Automatically navigate the robot to a predefined location using Nav2."""
    global node, action_queue
    loc_key = location_name.lower().replace(" ", "_")
    if loc_key not in LOCATIONS:
        return f"Error: No data found for location '{location_name}'."

    target = LOCATIONS[loc_key]
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = float(target["x"])
    goal_msg.pose.pose.position.y = float(target["y"])
    goal_msg.pose.pose.orientation.z = float(target["z"])
    goal_msg.pose.pose.orientation.w = float(target["w"])

    # Đưa vào hàng đợi
    action_queue.put({'desc': f"Going to {location_name}", 'goal_msg': goal_msg})
    return f"Added task [Going to '{location_name}'] to the action queue."

@tool
def move_distance(distance_x, distance_y) -> str: 
    """Move the robot a specific distance in meters."""
    global node, current_x, current_y, current_theta, action_queue
    dx, dy = float(distance_x), float(distance_y)
    
    yaw_rad = math.radians(current_theta)
    goal_x = current_x + (dx * math.cos(yaw_rad)) - (dy * math.sin(yaw_rad))
    goal_y = current_y + (dx * math.sin(yaw_rad)) + (dy * math.cos(yaw_rad))
    q_z, q_w = math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)
    
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = goal_x
    goal_msg.pose.pose.position.y = goal_y
    goal_msg.pose.pose.orientation.z = q_z
    goal_msg.pose.pose.orientation.w = q_w

    action_queue.put({'desc': f"Moving X={dx}m, Y={dy}m", 'goal_msg': goal_msg})
    return f"Added task [Moving X={dx}m, Y={dy}m] to the action queue."

@tool
def rotate_in_place(angle_degrees: float) -> str:
    """Rotate the robot in place by a specific angle in degrees."""
    global node, current_x, current_y, current_theta, action_queue
    angle = float(angle_degrees)
    goal_theta = current_theta + angle
    goal_yaw_rad = math.radians(goal_theta)
    q_z, q_w = math.sin(goal_yaw_rad / 2.0), math.cos(goal_yaw_rad / 2.0)
    
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = current_x
    goal_msg.pose.pose.position.y = current_y
    goal_msg.pose.pose.orientation.z = q_z
    goal_msg.pose.pose.orientation.w = q_w

    action_queue.put({'desc': f"Rotating {angle} degrees", 'goal_msg': goal_msg})
    return f"Added task [Rotating {angle} degrees] to the action queue."

@tool
def move_shape(shape_name: str) -> str:
    """Drive the robot in a specific shape. Supported: 'square', 'triangle'."""
    shape = shape_name.lower()
    if shape == 'square':
        for _ in range(4):
            move_distance.invoke({"distance_x": 0.5, "distance_y": 0.0})
            wait_time.invoke({"seconds": 1.0}) # NGHỈ 1 GIÂY
            rotate_in_place.invoke({"angle_degrees": 90.0})
            wait_time.invoke({"seconds": 1.0}) # NGHỈ 1 GIÂY
        return "Added sequence of 8 actions to follow a square shape to the task queue."
    elif shape == 'triangle':
        for _ in range(3):
            move_distance.invoke({"distance_x": 0.5, "distance_y": 0.0})
            wait_time.invoke({"seconds": 1.0}) # NGHỈ 1 GIÂY
            rotate_in_place.invoke({"angle_degrees": 120.0})
            wait_time.invoke({"seconds": 1.0}) # NGHỈ 1 GIÂY
        return "Added sequence of 6 actions to follow a triangle shape to the task queue."
    return f"Error: Unknown shape '{shape_name}'."

@tool
def stop_robot(dummy: str = "") -> str:
    """Stop the robot immediately and clear all pending task sequences."""
    global vel_publisher, current_goal_handle, action_queue

    # 1. Xóa sạch hàng đợi
    with action_queue.mutex:
        action_queue.queue.clear()

    # 2. Hủy lệnh Nav2 đang chạy (nếu có)
    if current_goal_handle is not None:
        current_goal_handle.cancel_goal_async()
        current_goal_handle = None

    # 3. Phanh gấp bánh xe
    twist = Twist()
    vel_publisher.publish(twist)

    return "EMERGENCY STOP: Robot stopped and all pending action sequences cleared!"



# ==========================================
# MAIN NODE
# ==========================================
def main():
    global node, vel_publisher, nav_client
    rclpy.init()
    node = rclpy.create_node("ai_brain_node_en")

    # Nếu bạn chạy mô phỏng bằng Gazebo
    # from rclpy.parameter import Parameter
    # node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    vel_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10) 
    nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")   
    # Khởi tạo bộ lắng nghe topic /amcl_pose
    node.create_subscription(
        PoseWithCovarianceStamped, 
        "/amcl_pose", 
        amcl_pose_callback, 
        qos_profile_sensor_data
    )
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # BẬT LUỒNG QUẢN LÝ CHUỖI NHIỆM VỤ
    worker_thread = threading.Thread(target=task_worker_thread, daemon=True)
    worker_thread.start()

    print("[System] Connecting to Groq Cloud API natively...")
    
    llm = ChatGroq(
        temperature=0.1, 
        groq_api_key="  ", 
        model="llama-3.3-70b-versatile" 
    )

    tools = [
        get_robot_info, get_location_names, get_current_location, 
        stop_robot, navigate_to_location, save_current_location,
        move_distance, rotate_in_place, move_shape,
        wait_time
    ]
    
    prompt = ChatPromptTemplate.from_messages([
        ("system", "You are 'Meca', a highly intelligent warehouse robot assistant. "
                   "Always answer in friendly, concise ENGLISH. "
                   "If user asks to move/strafe left, right, forward or backward, use 'move_distance'. "
                   "If user asks to rotate or turn around, use 'rotate_in_place'. "
                   "If user asks to draw a shape, use 'move_shape'. "
                   "If user asks to remember or save the current location/pose, use 'save_current_location'. "
                   "CRITICAL RULE FOR SEQUENCES: If the user asks for a sequence of multiple actions (e.g., 'go to A then go to B'), "
                   "you MUST insert a 'wait_time' tool between each movement action. "
                   "If the user specifies a wait time (e.g., 'wait 3 seconds'), use that duration. "
                   "If the user does NOT specify a wait time, you MUST use a default duration of 2.0 seconds for the 'wait_time' tool between movements."
                   "If you use a tool, explain briefly what you did to the user."),
        ("human", "{input}"),
        ("placeholder", "{agent_scratchpad}"),
    ])

    agent = create_tool_calling_agent(llm, tools, prompt)
    agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=False)

    print("\n=========================================")
    print("[SYSTEM] PURE LANGCHAIN AI BRAIN BOOTED!")
    print("=========================================\n")

    try:
        while rclpy.ok():
            user_msg = input("User: ")
            if user_msg.lower() in ["exit", "quit"]:
                break
            
            try:
                response = agent_executor.invoke({"input": user_msg})
                print(f"Meca: {response['output']}\n")
            except Exception as e:
                print(f"[Error]: {e}\n")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()