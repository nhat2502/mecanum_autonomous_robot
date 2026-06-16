```markdown
# 🚀 Tích hợp Mô hình Ngôn ngữ Lớn (LLM) và SLAM trong Định vị, Dẫn đường cho Robot Di động Đa hướng (Mecanum)

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Llama 3.3](https://img.shields.io/badge/AI-Llama_3.3_(Groq)-0466C8?style=for-the-badge)
![Raspberry Pi](https://img.shields.io/badge/Hardware-Raspberry_Pi_4-C51A4A?style=for-the-badge&logo=Raspberry-Pi)

> **Mô tả dự án:** Đây là hệ thống điều khiển tự chủ hoàn chỉnh cho robot kho bãi sử dụng khung gầm 4 bánh Mecanum. Hệ thống ứng dụng sức mạnh của trí tuệ nhân tạo (LLM Llama 3.3) để dịch các lệnh bằng ngôn ngữ tự nhiên của con người thành chuỗi tác vụ có cấu trúc (JSON Task Queue), sau đó cấp phát cho hệ điều hành ROS 2 thực thi quá trình di chuyển, né vật cản và thực hiện nhiệm vụ qua khung điều hướng Nav2.


## 🌟 Tính năng Cốt lõi
* **Trí tuệ nhân tạo (AI Agent):** Xử lý ngôn ngữ tự nhiên (NLP) bằng Llama 3.3 thông qua Groq API, bóc tách ý định và ra quyết định chuỗi hành vi cho robot.
* **Định vị & Lập bản đồ (SLAM):** Sử dụng lõi thuật toán Google Cartographer kết hợp bộ lọc hạt AMCL để xây dựng bản đồ tĩnh và xác định tọa độ không gian với sai số cấp centimet.
* **Dẫn đường tự chủ (Nav2):** Tính toán quỹ đạo toàn cục (A*/Dijkstra) và điều khiển vận tốc cục bộ (DWB Local Planner) giúp luồn lách qua các chướng ngại vật động.
* **Điều khiển Đa hướng (Omnidirectional):** Giải bài toán động học cho khung gầm 4 bánh Mecanum, tối ưu hóa không gian chật hẹp (trượt ngang, xoay tại chỗ).

---

## 📂 Cấu trúc Kho lưu trữ (Repository Structure)

Dự án sử dụng kiến trúc ROS 2 phân tán (Distributed Architecture), chia tải xử lý giữa Máy tính nhúng (Pi 4) và Máy trạm (PC):

* **`pi_workspace/` (Robot Worker - Chạy trên Raspberry Pi 4):**
  * Giao tiếp phần cứng cấp thấp.
  * Node đọc dữ liệu cảm biến LiDAR (`sllidar_ros2`).
  * Driver xuất xung điều khiển vi điều khiển và động cơ Mecanum (`mecanum_controller`).
* **`pc_workspace/` (AI & Master Station - Chạy trên PC Ubuntu):**
  * Tích hợp khung điều hướng Nav2 và thuật toán Google Cartographer (`nav2_config`, `cartographer_config`).
  * Nơi chứa Tác tử AI gọi API Llama 3.3 (`mecanum_ai_project`).
  * Giao diện theo dõi thông số AMCL và đo lường hành trình.
* **`meca-dashboard/` (Giao diện người dùng Web):**
  * Ứng dụng Web viết bằng React/Vite.
  * Cung cấp Dashboard trực quan để giám sát trạng thái pin, vị trí robot trên bản đồ và khung chat ra lệnh bằng tiếng Việt.

---

## 🛠️ Hướng dẫn Khởi chạy (Getting Started)

### 1. Khởi động phần cứng (Trên Raspberry Pi 4)
Đảm bảo Pi 4 đã kết nối chung mạng nội bộ với PC. Mở Terminal SSH vào Pi:
```bash
cd ~/pi_workspace
colcon build
source install/setup.bash
# Khởi chạy LiDAR và Base Controller
ros2 launch mecanum_description state_publisher.launch.py

```

### 2. Khởi động AI và Dẫn đường (Trên PC Ubuntu)

Mở Terminal trên PC:

```bash
cd ~/pc_workspace
colcon build
source install/setup.bash
# Bật Nav2 và Rviz2
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False map:=<đường_dẫn_file_map.yaml>
# Bật AI Agent lắng nghe lệnh
ros2 run mecanum_ai_project llm_commander

```

### 3. Khởi động Giao diện Web (Dashboard)

```bash
cd ~/meca-dashboard
npm install
npm run dev

```



## ✍️ Tác giả

* **Lê Văn Nhật**
* Đồ án Tốt nghiệp chuyên ngành Kỹ thuật Robot & Tự động hóa.

```

```
