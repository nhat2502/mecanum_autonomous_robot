# Integration of Large Language Models (LLM) and SLAM Algorithms for Localization, Navigation, and Decision-Making in Omnidirectional Mobile Robots

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Llama 3.3](https://img.shields.io/badge/AI-Llama_3.3_(Groq)-0466C8?style=for-the-badge)
![Raspberry Pi](https://img.shields.io/badge/Hardware-Raspberry_Pi_4-C51A4A?style=for-the-badge&logo=Raspberry-Pi)

> **Project Description:** This is the official source code for the Graduation Thesis in Robotics & Automation Engineering. The project presents a fully autonomous system for a warehouse robot utilizing a 4-wheel Mecanum chassis. The core architecture integrates Artificial Intelligence (LLM Llama 3.3) to translate natural language commands into a structured JSON task queue, collaborating with the ROS 2 framework (Nav2, Cartographer) to execute real-time localization, navigation, and dynamic obstacle avoidance.

**[View Project Demo Video on Google Drive](https://drive.google.com/drive/folders/1FGJKuNfUkUo9YEt7l6B-tnkr-01AXbxe?usp=drive_link)**

## Core Features

* **Artificial Intelligence (AI Agent):** Utilizes Natural Language Processing (NLP) powered by Llama 3.3 via the Groq API to automatically extract user intent and determine behavioral sequences.
* **Simultaneous Localization and Mapping (SLAM):** Implements the Google Cartographer algorithm combined with the AMCL particle filter to build static maps and determine (x, y, θ) coordinates with high precision.
* **Autonomous Navigation (Nav2):** Executes global trajectory calculation (A*/Dijkstra) and local velocity control (DWB Local Planner) for automated dynamic obstacle avoidance.
* **Omnidirectional Control (Mecanum):** A low-level microcontroller (STM32 Nucleo) solves inverse kinematics and communicates with the Raspberry Pi 4 via Serial, enabling lateral movement and zero-radius turns.

---

## Repository Structure

The project employs a distributed ROS 2 architecture, sharing the computational load between an embedded worker and a master station:

* **`pi_workspace/` (Robot Worker - Running on RPi 4):**
  * Reads raw sensor data (RPLiDAR A1).
  * Contains bash scripts for automated startup (`run_mapping.sh`, `run_navigation.sh`).
  * Hardware drivers for serial communication and motor control board output.
* **`pc_workspace/` (AI & Master Station - Running on PC Ubuntu):**
  * Integrates the Nav2 framework and Rviz2 configurations.
  * AI Agent Node (LLM Commander) interfacing with the Groq API.
  * Data extraction scripts and trajectory tracking error analysis.
* **`meca-dashboard/` (Web GUI Dashboard):**
  * Web application built with React/Vite.
  * Visually displays the map, AMCL coordinates, system status, and a natural language command input interface.

---

## Getting Started

### 1. Robot Operation (On Raspberry Pi 4)
Connect to the local Wi-Fi network and SSH into the Raspberry Pi:
```bash
ssh ubuntu@192.168.1.13
# Password: ubuntu
cd ~/pi_workspace

```

**Option 1: Mapping Mode**
Open 2 terminals on the Pi:

```bash
# Terminal 1: Run SLAM core and hardware connection
./run_mapping.sh

# Terminal 2: Manual teleoperation to build the map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

**Option 2: Autonomous Navigation Mode (Nav2 & AMCL)**

```bash
# Launch localization and navigation system
./run_navigation.sh

```

**Option 3: Custom Trajectory Tracking (Figure-8)**

```bash
python3 run_custom_path.py

```

### 2. AI and Master Station Operation (On PC Ubuntu)

Open a Terminal on the PC to launch the AI Agent and Rviz2 interface:

```bash
cd ~/pc_workspace
colcon build
source install/setup.bash
# Launch Rviz2 to monitor Nav2
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False map:=<path_to_map_file.yaml>
# Launch AI Agent to listen for commands
ros2 run mecanum_ai_project llm_commander

```

### 3. Web Interface Initialization (Dashboard)

Open another Terminal on the PC:

```bash
cd ~/meca-dashboard
npm install
npm run dev

```

---

## Technical Analysis & Evaluation

* **Simulation & Evaluation:** The system's accuracy was verified through trajectory tracking experiments. All logs comparing actual versus theoretical paths are saved as `.csv` files, with tracking errors visualized and plotted via Python scripts.
* **Power Management:** The 18650 Lishen battery pack configuration was rigorously calculated regarding discharge rates to ensure stable power delivery to the STM32 microcontroller, four JGB37-520 DC motors, and the Raspberry Pi 4 board over extended operation periods.

---

## Author

* **Lê Văn Nhật**
* Graduation Thesis in Robotics & Automation Engineering.

```

```
