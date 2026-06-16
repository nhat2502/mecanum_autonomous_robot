
# 🚀 Tích hợp Mô hình Ngôn ngữ Lớn (LLM) và Thuật toán SLAM trong Định vị, Dẫn đường và Ra quyết định cho Robot Di động Đa hướng

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Llama 3.3](https://img.shields.io/badge/AI-Llama_3.3_(Groq)-0466C8?style=for-the-badge)
![Raspberry Pi](https://img.shields.io/badge/Hardware-Raspberry_Pi_4-C51A4A?style=for-the-badge&logo=Raspberry-Pi)

> **Mô tả dự án:** Đây là mã nguồn chính thức cho Đồ án Tốt nghiệp chuyên ngành Kỹ thuật Robot & Tự động hóa. Dự án xây dựng một hệ thống tự chủ hoàn chỉnh cho robot kho bãi sử dụng khung gầm 4 bánh Mecanum. Lõi hệ thống kết hợp trí tuệ nhân tạo (LLM Llama 3.3) để dịch ngôn ngữ tự nhiên thành chuỗi tác vụ JSON, phối hợp cùng hệ điều hành ROS 2 (Nav2, Cartographer) để thực thi định vị, di chuyển và né vật cản theo thời gian thực.
*https://drive.google.com/drive/folders/1FGJKuNfUkUo9YEt7l6B-tnkr-01AXbxe?usp=drive_link*
## 🌟 Tính năng Cốt lõi
* **Trí tuệ nhân tạo (AI Agent):** Xử lý ngôn ngữ tự nhiên (NLP) bằng Llama 3.3 thông qua Groq API, tự động bóc tách ý định người dùng và ra quyết định chuỗi hành vi.
* **Định vị & Lập bản đồ (SLAM):** Thuật toán Google Cartographer kết hợp bộ lọc hạt AMCL giúp xây dựng bản đồ tĩnh và xác định tọa độ $(x,y,\theta)$ với độ chính xác cao.
* **Dẫn đường tự chủ (Nav2):** Tính toán quỹ đạo toàn cục (A*/Dijkstra) và điều khiển vận tốc cục bộ (DWB Local Planner), tự động luồn lách qua chướng ngại vật động.
* **Điều khiển Đa hướng (Mecanum):** Vi điều khiển cấp thấp (STM32 Nucleo) giải bài toán động học nghịch, giao tiếp với Raspberry Pi 4 qua Serial, cho phép robot trượt ngang và xoay tại chỗ (Zero-radius turn).

---

## 📂 Cấu trúc Kho lưu trữ (Repository Structure)

Dự án sử dụng kiến trúc ROS 2 phân tán, chia tải xử lý giữa Máy tính nhúng (Raspberry Pi 4) và Máy trạm (PC Ubuntu):

* **`pi_workspace/` (Robot Worker - Chạy trên RPi 4):**
  * Đọc dữ liệu cảm biến (RPLiDAR A1).
  * Chứa các file bash script tự động hóa khởi chạy (`run_mapping.sh`, `run_navigation.sh`).
  * Driver giao tiếp và xuất xung xuống mạch điều khiển động cơ.
* **`pc_workspace/` (AI & Master Station - Chạy trên PC):**
  * Tích hợp khung điều hướng Nav2 và cấu hình Rviz2.
  * Node Tác tử AI (LLM Commander) giao tiếp với Groq API.
  * Các script trích xuất dữ liệu, vẽ đồ thị phân tích sai số quỹ đạo.
* **`meca-dashboard/` (Giao diện Web GUI):**
  * Ứng dụng Web viết bằng React/Vite.
  * Hiển thị trực quan bản đồ, tọa độ AMCL, trạng thái hệ thống và khung nhập lệnh tiếng Việt.

---

## 🛠️ Hướng dẫn Khởi chạy (Getting Started)

### 1. Vận hành Robot (Trên Raspberry Pi 4)
Kết nối chung mạng Wi-Fi và SSH vào Raspberry Pi:
```bash
ssh ubuntu@192.168.1.13
# Mật khẩu: ubuntu
cd ~/pi_workspace

```

**Lựa chọn 1: Chế độ Lập bản đồ (Mapping)**
Mở 2 terminal trên Pi:

```bash
# Terminal 1: Chạy lõi SLAM và kết nối phần cứng
./run_mapping.sh

# Terminal 2: Điều khiển robot chạy thủ công để vẽ map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

**Lựa chọn 2: Chế độ Dẫn đường & Tự chủ (Navigation & AMCL)**

```bash
# Khởi chạy hệ thống định vị và dẫn đường
./run_navigation.sh

```

**Lựa chọn 3: Chạy kiểm thử quỹ đạo hình số 8 (Custom Path)**

```bash
python3 run_custom_path.py

```

### 2. Vận hành AI và Trạm điều khiển (Trên PC Ubuntu)

Mở Terminal trên máy tính PC để bật AI Agent nhận lệnh và giao diện Rviz2:

```bash
cd ~/pc_workspace
colcon build
source install/setup.bash
# Bật Rviz2 theo dõi Nav2
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False map:=<đường_dẫn_file_map.yaml>
# Bật AI Agent lắng nghe lệnh
ros2 run mecanum_ai_project llm_commander

```

### 3. Bật Giao diện Web (Dashboard)

Mở một Terminal khác trên PC:

```bash
cd ~/meca-dashboard
npm install
npm run dev

```

---

## 🔬 Tính toán Kỹ thuật & Đánh giá

* **Mô phỏng & Đánh giá:** Hệ thống đã được kiểm chứng thông qua việc bám quỹ đạo thiết kế trước (Trajectory Tracking). Toàn bộ dữ liệu logs (đường dẫn thực tế vs lý thuyết) được lưu thành file `.csv` và xuất đồ thị phân tích sai số thông qua script Python.
* **Năng lượng:** Cấu hình pin 18650 Lishen được tính toán dòng xả cẩn thận để gánh toàn bộ tải của vi điều khiển STM32, 4 động cơ DC JGB37-520 và bo mạch Raspberry Pi 4.

---

## ✍️ Tác giả

* **Lê Văn Nhật**
* Đồ án Tốt nghiệp chuyên ngành Kỹ thuật Robot & Tự động hóa.

```

```
