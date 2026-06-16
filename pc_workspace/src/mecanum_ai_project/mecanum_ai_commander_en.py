import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

from langchain_groq import ChatGroq
from langchain.agents import tool, AgentExecutor, create_tool_calling_agent
from langchain_core.prompts import ChatPromptTemplate

# --- GLOBAL VARIABLES ---
node = None
vel_publisher = None
nav_client = None

# Map Database
LOCATIONS = {
    "charger": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "shelf_a": {"x": 2.5, "y": 1.5, "z": 0.707, "w": 0.707},
    "shelf_b": {"x": 2.5, "y": -1.5, "z": -0.707, "w": 0.707}
}

# ==========================================
# PURE LANGCHAIN TOOLS (FIXED FOR GROQ)
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
    """Use this tool when the user asks 'where are you' or 'what is your current location'."""
    return "I am currently at the 'charger'."

@tool
def move_mecanum(vel_x: float, vel_y: float, angular_z: float) -> str:
    """Control the velocity of the Mecanum robot. 
    Set vel_x > 0 to move forward (e.g., to move 1 meter ahead).
    Set vel_y > 0 to strafe left.
    """
    global vel_publisher
    twist = Twist()
    twist.linear.x = float(vel_x)
    twist.linear.y = float(vel_y) 
    twist.angular.z = float(angular_z)
    vel_publisher.publish(twist)
    return f"Started moving with velocity X={vel_x}, Y={vel_y}, Z={angular_z}. Note: I will keep moving until stop_robot is called."

@tool
def stop_robot(dummy: str = "") -> str:
    """Stop the robot immediately."""
    global vel_publisher
    twist = Twist()
    vel_publisher.publish(twist)
    return "Robot stopped successfully."

@tool
def navigate_to_location(location_name: str) -> str:
    """Automatically navigate the robot to a predefined location using Nav2."""
    global nav_client, node
    loc_key = location_name.lower().replace(" ", "_")
    
    if loc_key not in LOCATIONS:
        return f"Error: No data found for location '{location_name}'."

    target = LOCATIONS[loc_key]
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = target["x"]
    goal_msg.pose.pose.position.y = target["y"]
    goal_msg.pose.pose.orientation.z = target["z"]
    goal_msg.pose.pose.orientation.w = target["w"]

    nav_client.send_goal_async(goal_msg)
    return f"Navigating robot to '{location_name}'."

# ==========================================
# MAIN NODE
# ==========================================
def main():
    global node, vel_publisher, nav_client
    rclpy.init()
    node = rclpy.create_node("ai_brain_node_en")

    vel_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    print("[System] Connecting to Groq Cloud API natively...")
    
    # 1. KẾT NỐI LLM
    llm = ChatGroq(
        temperature=0.1, 
        groq_api_key="  ", 
        model="llama-3.3-70b-versatile" 
    )

    # 2. KHAI BÁO TOOLS
    tools = [get_robot_info, get_location_names, get_current_location, move_mecanum, stop_robot, navigate_to_location]

    # 3. TẠO PROMPT CHUẨN CỦA LANGCHAIN
    prompt = ChatPromptTemplate.from_messages([
        ("system", "You are 'Meca', a highly intelligent warehouse robot assistant. "
                   "Always answer in friendly, concise ENGLISH. "
                   "If asked to move ahead, use 'move_mecanum' with positive vel_x. "
                   "If you use a tool, explain briefly what you did to the user."),
        ("human", "{input}"),
        ("placeholder", "{agent_scratchpad}"),
    ])

    # 4. TẠO AGENT SIÊU TỐC
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
                # GỌI AGENT ĐƠN GIẢN VÀ AN TOÀN
                response = agent_executor.invoke({"input": user_msg})
                print(f"Meca: {response['output']}\n")
            except Exception as e:
                print(f"[Error]: {e}\n")
                
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()