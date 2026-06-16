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
initial_pose_pub = None

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
    "shelf_b": {"x": 0.4, "y": 0.4, "z": -0.707, "w": 0.707},
    "shelf_c": {"x": 1.7, "y": 0.4, "z": 0.707, "w": 0.707} 
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

        notify_web(f"▶️ [EXECUTING]: {task['desc']}...")

        # IF IT IS A WAIT TASK
        if task.get('type') == 'wait':
            time.sleep(task['duration'])
            notify_web(f"✅ [COMPLETED]: Waited for {task['duration']} seconds.")
            action_queue.task_done()
            continue
            
        # IF IT IS A NAV TASK
        goal_event = threading.Event()

        def goal_response_callback(future):
            global current_goal_handle
            goal_handle = future.result()
            if not goal_handle.accepted:
                notify_web(f"❌ [REJECTED]: Nav2 cannot execute '{task['desc']}'!")
                goal_event.set() # Unlock
                return

            current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(get_result_callback)

        def get_result_callback(future):
            global current_goal_handle
            status = future.result().status
            notify_web(f"✅ [COMPLETED]: '{task['desc']}' (Status Code: {status})")
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
def get_robot_info() -> str:
    """Use this tool when the user asks about the robot's specs, dimensions, or capabilities."""
    return "Robot Name: Meca (Mecanum AI AMR).\nDimensions: Length 24cm, Width 15cm, Height 19cm. Ground Clearance: 6cm.\nDrivetrain: 4 Mecanum wheels, capable of strafing."

@tool
def get_location_names() -> str:
    """Use this tool when the user asks for the list of saved locations, points of interest, or where the robot can go."""
    global LOCATIONS
    return f"Saved locations: {', '.join(LOCATIONS.keys())}"

@tool
def get_current_location() -> str:
    """Use this tool when the user asks 'where are you', 'your location', or 'current pose'."""
    global current_x, current_y, current_theta
    return f"My exact real-time pose is: X = {current_x:.2f} meters, Y = {current_y:.2f} meters, Theta = {current_theta:.2f} degrees."

@tool
def get_location_coordinates(location_name: str) -> str:
    """Use this tool whenever the user asks about a specific location name (e.g., 'shelf_a', 'shelf_b', 'charger') to get its details, info, position, or exact coordinates."""
    global LOCATIONS
    loc_key = location_name.lower().replace(" ", "_")
    
    if loc_key in LOCATIONS:
        target = LOCATIONS[loc_key]
        x = target["x"]
        y = target["y"]
        z = target["z"]
        w = target["w"]
        
        siny_cosp = 2 * (w * z)
        cosy_cosp = 1 - 2 * (z * z)
        yaw_degrees = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        
        return f"Coordinates of '{location_name}': X = {x:.3f} meters, Y = {y:.3f} meters, Yaw = {yaw_degrees:.1f} degrees."
    else:
        return f"Location '{location_name}' not found in the database."

@tool
def save_current_location(location_name: str) -> str:
    """Saves the robot's current real-time pose as a new named location in the database."""
    global current_x, current_y, current_theta, LOCATIONS
    loc_key = location_name.lower().replace(" ", "_")
    yaw_rad = math.radians(current_theta)
    LOCATIONS[loc_key] = {"x": current_x, "y": current_y, "z": math.sin(yaw_rad / 2.0), "w": math.cos(yaw_rad / 2.0)}
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
    
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853892326654787
    
    initial_pose_pub.publish(msg)
    return f"Successfully set initial 2D pose to X={x}, Y={y}, Yaw={yaw_degrees} degrees."

@tool
def wait_time(seconds: float) -> str:
    """Use this tool to add a waiting or pause time for the robot."""
    global action_queue
    duration = float(seconds)
    action_queue.put({'desc': f"Waiting for {duration} seconds", 'type': 'wait', 'duration': duration})
    return f"Added task [Wait {duration} seconds] to the execution queue."

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
    action_queue.put({'desc': f"Navigating to {location_name}", 'goal_msg': goal_msg})
    return f"Added task [Navigate to '{location_name}'] to the execution queue."

@tool
def navigate_to_coordinates(x: float, y: float, yaw_degrees: float) -> str:
    """Use this tool when the user asks the robot to navigate to specific absolute numerical coordinates (X, Y, Yaw)."""
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
        'desc': f"Navigating to coords (X={x}, Y={y}, Yaw={yaw_degrees}°)", 
        'goal_msg': goal_msg
    })
    return f"Added task [Navigate to X={x}, Y={y}, Yaw={yaw_degrees}°] to the execution queue."

@tool
def move_distance(distance_x: float, distance_y: float) -> str: 
    """Move the robot a specific distance in meters relative to its current position.
    Positive X: Forward. Negative X: Backward.
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

    action_queue.put({'desc': f"Relative move: Forward {dx}m, Left {dy}m", 'goal_msg': goal_msg})
    return f"Added task [Relative move: X={dx}m, Y={dy}m] to the execution queue."

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

        action_queue.put({'desc': f"Rotating step {chunks_added + 1}: {sign * step} degrees", 'goal_msg': goal_msg})
        
        remaining_angle -= step
        chunks_added += 1

    return f"Added task sequence ({chunks_added} steps) to rotate {angle_degrees} degrees in the exact specified direction."

@tool
def move_shape(shape_name: str) -> str:
    """Drive the robot in a specific shape. Supported: 'square', 'triangle'."""
    shape = shape_name.lower()
    if shape == 'square':
        for _ in range(4):
            move_distance.invoke({"distance_x": 0.5, "distance_y": 0.0})
            wait_time.invoke({"seconds": 1.0}) 
            rotate_in_place.invoke({"angle_degrees": 90.0})
            wait_time.invoke({"seconds": 1.0}) 
        return "Added a sequence of 8 actions for a Square to the execution queue."
    elif shape == 'triangle':
        for _ in range(3):
            move_distance.invoke({"distance_x": 0.5, "distance_y": 0.0})
            wait_time.invoke({"seconds": 1.0}) 
            rotate_in_place.invoke({"angle_degrees": 120.0})
            wait_time.invoke({"seconds": 1.0}) 
        return "Added a sequence of 6 actions for a Triangle to the execution queue."
    return f"Error: Do not know how to draw shape '{shape_name}'."

@tool
def stop_robot() -> str:
    """Stop the robot immediately and clear all pending task sequences."""
    global vel_publisher, current_goal_handle, action_queue
    with action_queue.mutex:
        action_queue.queue.clear()
    if current_goal_handle is not None:
        current_goal_handle.cancel_goal_async()
        current_goal_handle = None
    vel_publisher.publish(Twist())
    return "EMERGENCY STOP: Robot stopped and task sequence cleared!"

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
    global node, vel_publisher, nav_client, agent_executor, progress_pub, initial_pose_pub
    
    # Initialize ROS 2
    rclpy.init()
    node = rclpy.create_node("ai_brain_web_backend")
    initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    vel_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    progress_pub = node.create_publisher(String, "/ai_progress", 10)
    nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", amcl_pose_callback, qos_profile_sensor_data)
    
    # Start Threads for ROS 2 and Task Queue
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=task_worker_thread, daemon=True).start()

    # Initialize LangChain AI
    print("[System] Connecting to Groq Cloud API natively...")
    llm = ChatGroq(temperature=0.1, groq_api_key="  ", model="llama-3.3-70b-versatile")
    
    tools = [
        get_robot_info, get_location_names, get_current_location, get_location_coordinates,
        stop_robot, navigate_to_location, save_current_location,
        move_distance, rotate_in_place, move_shape, wait_time,
        navigate_to_coordinates, set_initial_pose
    ]
    
    prompt = ChatPromptTemplate.from_messages([
        ("system", "You are 'Meca', a highly intelligent warehouse robot assistant. Answer in concise ENGLISH.\n"
                   "CHAT RULE: If the user is just greeting you (e.g., 'hello', 'hi'), respond with a friendly, short greeting WITHOUT calling any tools.\n"
                   "CRITICAL RULE 1: If the user provides 3 numbers (e.g., '0 0 0'), it means ONE single location (X=0, Y=0, Yaw=0). You MUST use 'navigate_to_coordinates' EXACTLY ONCE. DO NOT treat it as a sequence.\n"
                   "CRITICAL RULE 2: DO NOT repeat the same tool call multiple times. Only use 'wait_time' between clearly DIFFERENT actions.\n"
                   "CRITICAL RULE 3: You are a Mecanum robot capable of strafing. To move left, right, forward, or backward, JUST use 'move_distance' directly. DO NOT use 'rotate_in_place' before moving unless explicitly asked to rotate.\n"
                   "CRITICAL RULE 4: For a sequence of multiple actions, you MUST insert a 'wait_time' tool between each movement (default 2.0 seconds if not specified).\n"
                   "CRITICAL RULE 5 FOR TOOL CALLING: DO NOT output any conversational text before calling a tool. Call the tool directly without saying anything first. After the tool completes, you can output a short message to conclude."),
        ("human", "{input}"),
        ("placeholder", "{agent_scratchpad}"),
    ])

    agent = create_tool_calling_agent(llm, tools, prompt)
    agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=False)
    
    print("\n=========================================")
    print("[SYSTEM] API SERVER + AI BRAIN BOOTED!")
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
        if chat.message.lower() in ["stop", "stop robot", "halt"]:
            stop_robot.invoke({"run": ""})
            return {"reply": "EMERGENCY STOP ACTIVATED! The robot has stopped."}

        response = agent_executor.invoke({"input": chat.message})
        return {"reply": response['output']}
    except Exception as e:
        return {"reply": f"[AI System Error]: {str(e)}"}

@app.post("/launch-rviz")
async def launch_rviz():
    try:
        cmd = "ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz"
        subprocess.Popen(cmd, shell=True)
        return {"status": "Launched RViz2 with Nav2 configuration!"}
    except Exception as e:
        return {"status": f"Error: {e}"}

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