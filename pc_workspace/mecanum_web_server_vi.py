# 1. Standard Library Imports
import math
import queue
import subprocess
import threading
import time

# 2. Web Server & API Framework (FastAPI / Uvicorn / Pydantic)
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

# 3. AI & LLM Framework (LangChain / Groq)
from langchain.agents import AgentExecutor, create_tool_calling_agent, tool
from langchain_core.prompts import ChatPromptTemplate
from langchain_groq import ChatGroq

# 4. ROS 2 & Robotics Infrastructure
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

# ==========================================
# GLOBAL VARIABLES
# ==========================================
node = None
vel_publisher = None
nav_client = None
agent_executor = None
progress_pub = None

# Current real-time coordinates of the robot
current_x = 0.0
current_y = 0.0
current_theta = 0.0

action_queue = queue.Queue()       # Queue for task sequences
current_goal_handle = None         # Stores current Nav2 state for Emergency Stop

# Map Database
LOCATIONS = {
    "home_base": {"x": 0.001, "y": 0.001, "z": 0.0, "w": 1.0},
    "charger": {"x": 0.5, "y": 0.0, "z": 0.0, "w": 1.0},
    "shelf_a": {"x": 1.2, "y": -0.4, "z": 0.0, "w": 1.0},
    "shelf_b": {"x": 0.4, "y": 0.4, "z": -0.707, "w": 0.707}
}

# ==========================================
# ROS 2 CALLBACKS & WORKER
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

        notify_web(f"▶️ [ĐANG THỰC THI]: {task['desc']}...")

        # IF IT IS A WAIT TASK
        if task.get('type') == 'wait':
            time.sleep(task['duration'])
            notify_web(f"✅ [HOÀN THÀNH]: Đã chờ xong {task['duration']} giây.")
            action_queue.task_done()
            continue
            
        # IF IT IS A NAV TASK
        goal_event = threading.Event()

        def goal_response_callback(future):
            global current_goal_handle
            goal_handle = future.result()
            if not goal_handle.accepted:
                notify_web(f"❌ [TỪ CHỐI]: Nav2 không thể thực hiện lệnh '{task['desc']}'!")
                goal_event.set() # Unlock
                return

            current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(get_result_callback)

        def get_result_callback(future):
            global current_goal_handle
            status = future.result().status
            notify_web(f"✅ [HOÀN THÀNH]: '{task['desc']}' (Mã trạng thái: {status})")
            current_goal_handle = None
            goal_event.set() # Unlock for the next task

        send_goal_future = nav_client.send_goal_async(task['goal_msg'])
        send_goal_future.add_done_callback(goal_response_callback)

        goal_event.wait() 
        action_queue.task_done()

def amcl_pose_callback(msg):
    """Update absolute pose from /amcl_pose topic"""
    global current_x, current_y, current_theta
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
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
    return "Tên Robot: Meca (Mecanum AI AMR).\nKích thước: Dài 24cm, Rộng 15cm, Cao 7cm. Gầm cao: 6cm.\nHệ truyền động: 4 bánh Mecanum, có khả năng di chuyển trượt ngang."

@tool
def get_location_names(dummy: str = "") -> str:
    """Use when you need to know the list of saved locations or coordinates in the warehouse."""
    return f"Các vị trí đã lưu trên bản đồ: {', '.join(LOCATIONS.keys())}"

@tool
def get_current_location(dummy: str = "") -> str:
    """Use this tool when the user asks 'where are you', 'your location', or 'current pose'."""
    global current_x, current_y, current_theta
    return f"Vị trí hiện tại của tôi trên bản đồ là: X = {current_x:.2f} mét, Y = {current_y:.2f} mét, Góc xoay = {current_theta:.2f} độ."

@tool
def save_current_location(location_name: str) -> str:
    """Saves the robot's current real-time pose as a new named location in the database."""
    global current_x, current_y, current_theta, LOCATIONS
    loc_key = location_name.lower().replace(" ", "_")
    yaw_rad = math.radians(current_theta)
    LOCATIONS[loc_key] = {"x": current_x, "y": current_y, "z": math.sin(yaw_rad / 2.0), "w": math.cos(yaw_rad / 2.0)}
    return f"Đã lưu thành công vị trí hiện tại (X={current_x:.2f}m, Y={current_y:.2f}m) với tên là '{location_name}'."

@tool
def wait_time(seconds: float) -> str:
    """Use this tool when the user explicitly asks the robot to wait, pause, or rest for a specific amount of time.
    Also use this tool to add a default wait time of 2.0 seconds BETWEEN any two movement or rotation actions if the user doesn't specify a wait time."""
    global action_queue
    duration = float(seconds)
    action_queue.put({'desc': f"Chờ đợi trong {duration} giây", 'type': 'wait', 'duration': duration})
    return f"Đã thêm nhiệm vụ [Chờ {duration} giây] vào hàng đợi."

@tool
def navigate_to_location(location_name: str) -> str:
    """Automatically navigate the robot to a predefined location using Nav2."""
    global node, action_queue
    loc_key = location_name.lower().replace(" ", "_")
    if loc_key not in LOCATIONS:
        return f"Lỗi: Không tìm thấy dữ liệu tọa độ cho điểm đến '{location_name}'."
    target = LOCATIONS[loc_key]
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = float(target["x"])
    goal_msg.pose.pose.position.y = float(target["y"])
    goal_msg.pose.pose.orientation.z = float(target["z"])
    goal_msg.pose.pose.orientation.w = float(target["w"])
    action_queue.put({'desc': f"Tự động dẫn đường đến '{location_name}'", 'goal_msg': goal_msg})
    return f"Đã thêm nhiệm vụ [Đi đến '{location_name}'] vào hàng đợi."

@tool
def navigate_to_coordinates(x: float, y: float, yaw_degrees: float) -> str:
    """Use this tool when the user asks the robot to navigate or go to specific numerical absolute coordinates (X, Y, and optionally Yaw/Angle)."""
    global node, action_queue
    
    yaw_rad = math.radians(float(yaw_degrees))
    q_z = math.sin(yaw_rad / 2.0)
    q_w = math.cos(yaw_rad / 2.0)
    
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"  
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    
    goal_msg.pose.pose.position.x = float(x)
    goal_msg.pose.pose.position.y = float(y)
    goal_msg.pose.pose.orientation.z = q_z
    goal_msg.pose.pose.orientation.w = q_w
    
    action_queue.put({
        'desc': f"Dẫn đường tới tọa độ (X={x}, Y={y}, Góc={yaw_degrees}°)", 
        'goal_msg': goal_msg
    })
    return f"Đã thêm nhiệm vụ [Đi tới X={x}, Y={y}, Góc={yaw_degrees}°] vào hàng đợi."

@tool
def move_distance(distance_x, distance_y) -> str: 
    """Move the robot a specific distance in meters relative to its current position.
    Positive X: Move Forward. Negative X: Move Backward.
    Positive Y: Strafe Left. Negative Y: Strafe Right.
    """
    global node, action_queue
    dx, dy = float(distance_x), float(distance_y)
    
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "base_link" 
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    
    goal_msg.pose.pose.position.x = dx
    goal_msg.pose.pose.position.y = dy
    
    goal_msg.pose.pose.orientation.z = 0.0
    goal_msg.pose.pose.orientation.w = 1.0

    action_queue.put({'desc': f"Di chuyển tương đối: X={dx}m, Y={dy}m", 'goal_msg': goal_msg})
    return f"Đã thêm nhiệm vụ [Trượt tương đối: X={dx}m, Y={dy}m] vào hàng đợi."

@tool
def rotate_in_place(angle_degrees: float) -> str:
    """Rotate the robot in place by a specific angle in degrees."""
    global node, action_queue
    
    angle = float(angle_degrees)
    sign = 1.0 if angle >= 0 else -1.0
    remaining_angle = abs(angle)
    chunks_added = 0
    
    while remaining_angle > 0.01:  
        step = min(remaining_angle, 90.0) 
        
        goal_yaw_rad = math.radians(sign * step)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "base_link"
        goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = 0.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(goal_yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(goal_yaw_rad / 2.0)

        action_queue.put({'desc': f"Xoay tại chỗ nhịp {chunks_added + 1}: {sign * step} độ", 'goal_msg': goal_msg})
        
        remaining_angle -= step
        chunks_added += 1

    return f"Đã thêm chuỗi nhiệm vụ ({chunks_added} bước) để xoay {angle_degrees} độ vào hàng đợi."

@tool
def move_shape(shape_name: str) -> str:
    """Drive the robot in a specific shape. Supported: 'square', 'triangle'."""
    shape = shape_name.lower()
    if shape == 'square' or shape == 'vuông' or shape == 'hình vuông':
        for _ in range(4):
            move_distance.invoke({"distance_x": 0.5, "distance_y": 0.0})
            wait_time.invoke({"seconds": 1.0}) 
            rotate_in_place.invoke({"angle_degrees": 90.0})
            wait_time.invoke({"seconds": 1.0}) 
        return "Đã thêm chuỗi 8 hành động để di chuyển theo Hình Vuông vào hàng đợi."
    elif shape == 'triangle' or shape == 'tam giác' or shape == 'hình tam giác':
        for _ in range(3):
            move_distance.invoke({"distance_x": 0.5, "distance_y": 0.0})
            wait_time.invoke({"seconds": 1.0}) 
            rotate_in_place.invoke({"angle_degrees": 120.0})
            wait_time.invoke({"seconds": 1.0}) 
        return "Đã thêm chuỗi 6 hành động để di chuyển theo Hình Tam Giác vào hàng đợi."
    return f"Lỗi: Không biết cách di chuyển theo hình dạng '{shape_name}'."

@tool
def stop_robot(dummy: str = "") -> str:
    """Stop the robot immediately and clear all pending task sequences."""
    global vel_publisher, current_goal_handle, action_queue
    with action_queue.mutex:
        action_queue.queue.clear()
    if current_goal_handle is not None:
        current_goal_handle.cancel_goal_async()
        current_goal_handle = None
    vel_publisher.publish(Twist())
    return "DỪNG KHẨN CẤP: Robot đã phanh lại và xóa toàn bộ hàng đợi!"

# ==========================================
# FASTAPI SERVER SETUP
# ==========================================
app = FastAPI()

app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"],
)

class ChatMessage(BaseModel):
    message: str

class TeleopCommand(BaseModel):
    linear_x: float
    linear_y: float
    angular_z: float

@app.on_event("startup")
def startup_event():
    """Start ROS 2, Threads, and LangChain AI Brain when Server boots"""
    global node, vel_publisher, nav_client, agent_executor, progress_pub
    
    # Initialize ROS 2
    rclpy.init()
    node = rclpy.create_node("ai_brain_web_backend")
    vel_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    progress_pub = node.create_publisher(String, "/ai_progress", 10)
    nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", amcl_pose_callback, qos_profile_sensor_data)
    
    # Start Threads for ROS 2 and Task Queue
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=task_worker_thread, daemon=True).start()

    # Initialize LangChain AI
    print("[Hệ thống] Đang kết nối tới Groq Cloud API...")
    llm = ChatGroq(temperature=0.1, groq_api_key="  ", model="llama-3.3-70b-versatile")
    
    tools = [
        get_robot_info, get_location_names, get_current_location, 
        stop_robot, navigate_to_location, save_current_location,
        move_distance, rotate_in_place, move_shape, wait_time,
        navigate_to_coordinates
    ]
    
    prompt = ChatPromptTemplate.from_messages([
        ("system", "Bạn là 'Meca', một trợ lý robot nhà kho thông minh. "
                   "Luôn luôn trả lời bằng Tiếng Việt thân thiện, súc tích. "
                   "Bạn phải hiểu các lệnh tiếng Việt của người dùng và gọi các công cụ (tools) tương ứng: "
                   "- 'tiến', 'lùi', 'sang trái', 'sang phải', 'trượt' -> dùng 'move_distance'. "
                   "- 'xoay', 'quay', 'quay trái', 'quay phải' -> dùng 'rotate_in_place'. "
                   "- 'đi tới', 'đến', 'tới kệ' -> dùng 'navigate_to_location' (nếu có tên điểm) hoặc 'navigate_to_coordinates' (nếu là tọa độ x, y cụ thể). "
                   "- 'vẽ hình', 'chạy theo hình' -> dùng 'move_shape'. "
                   "- 'lưu vị trí', 'nhớ chỗ này' -> dùng 'save_current_location'. "
                   "QUY TẮC QUAN TRỌNG CHO CHUỖI LỆNH: Nếu người dùng yêu cầu một chuỗi hành động liên tiếp (ví dụ: 'đi đến kệ A sau đó quay 90 độ'), "
                   "bạn BẮT BUỘC phải chèn công cụ 'wait_time' vào giữa mỗi hành động di chuyển. "
                   "Nếu người dùng có nhắc đến thời gian chờ (ví dụ: 'đợi 3 giây'), hãy dùng thời gian đó. "
                   "Nếu không nhắc đến thời gian chờ, bạn BẮT BUỘC dùng thời gian mặc định là 2.0 giây cho công cụ 'wait_time' ở giữa các bước."
                   "Sau khi sử dụng công cụ, hãy tóm tắt ngắn gọn bằng Tiếng Việt cho người dùng biết bạn vừa đưa các lệnh gì vào hàng đợi."),
        ("human", "{input}"),
        ("placeholder", "{agent_scratchpad}"),
    ])

    agent = create_tool_calling_agent(llm, tools, prompt)
    agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=False)
    
    print("\n=========================================")
    print("[HỆ THỐNG] API SERVER & BỘ NÃO AI ĐÃ KHỞI ĐỘNG!")
    print("=========================================\n")

@app.on_event("shutdown")
def shutdown_event():
    """Gracefully shutdown ROS 2 when closing Server"""
    global node
    node.destroy_node()
    rclpy.shutdown()

@app.post("/chat")
async def chat_with_ai(chat: ChatMessage):
    global agent_executor
    try:
        # Nhận diện cả tiếng Việt và Anh cho lệnh dừng khẩn cấp
        if chat.message.lower() in ["stop", "dừng", "dừng lại", "halt"]:
            stop_robot.invoke({})
            return {"reply": "🛑 ĐÃ KÍCH HOẠT DỪNG KHẨN CẤP! Mọi hoạt động đã bị hủy."}

        # Feed prompt into LangChain
        response = agent_executor.invoke({"input": chat.message})
        return {"reply": response['output']}
    except Exception as e:
        return {"reply": f"[Lỗi hệ thống AI]: {str(e)}"}

@app.post("/launch-rviz")
async def launch_rviz():
    try:
        cmd = "ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz"
        subprocess.Popen(cmd, shell=True)
        return {"status": "Đang mở RViz2 với cấu hình Nav2!"}
    except Exception as e:
        return {"status": f"Lỗi: {e}"}

@app.post("/teleop")
async def teleop_robot(cmd: TeleopCommand):
    """API Receives manual control commands from Web and forwards to ROS 2"""
    global vel_publisher
    try:
        twist = Twist()
        twist.linear.x = float(cmd.linear_x)
        twist.linear.y = float(cmd.linear_y)
        twist.angular.z = float(cmd.angular_z)
        vel_publisher.publish(twist)
        return {"status": "ok"}
    except Exception as e:
        return {"status": "error", "message": str(e)}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)