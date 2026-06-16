import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

def normalize_angle(angle):
    """
    Chuẩn hóa góc về khoảng [-pi, pi] để tính toán sai số xoay.
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

def calculate_errors(actual_df, reference_df):
    """ Hàm tính toán Sai số Vị trí (CTE) và xuất mảng Đáp ứng Góc """
    actual_pts = actual_df[['X', 'Y']].values
    ref_pts = reference_df[['X_ref', 'Y_ref']].values
    
    actual_yaws = actual_df['Yaw'].values
    ref_yaws = reference_df['Yaw_ref'].values
    
    # 1. TÍNH SAI SỐ VỊ TRÍ
    distances = cdist(actual_pts, ref_pts, metric='euclidean')
    closest_indices = np.argmin(distances, axis=1)
    
    # =========================================================
    # VÁ LỖI VÒNG LẶP KÍN (CLOSED-LOOP FIX)
    # Ngăn tình trạng xe về đích bị map nhầm mảng góc về điểm xuất phát
    n_actual = len(closest_indices)
    n_ref = len(ref_pts)
    for j in range(n_actual):
        # Nếu xe đang ở 20% thời gian cuối, mà thuật toán lại gán nhầm về 20% điểm đầu tiên
        if j > n_actual * 0.8 and closest_indices[j] < n_ref * 0.2:
            closest_indices[j] = n_ref - 1 # Ép lấy thông số của điểm cuối cùng
    # =========================================================

    min_distances = distances[np.arange(len(actual_pts)), closest_indices]
    rmse_pos = np.sqrt(np.mean(min_distances**2))
    
    # 2. XỬ LÝ ĐÁP ỨNG GÓC HƯỚNG (Chỉ lấy giá trị để vẽ, không tính RMSE nữa)
    actual_yaws_deg = []
    ref_yaws_deg = []
    
    for i, closest_idx in enumerate(closest_indices):
        yaw_robot = actual_yaws[i]
        yaw_path = ref_yaws[closest_idx]
        
        # Lưu lại giá trị góc để vẽ đồ thị đáp ứng
        actual_yaws_deg.append(np.degrees(yaw_robot))
        ref_yaws_deg.append(np.degrees(yaw_path))
        
    return rmse_pos, actual_yaws_deg, ref_yaws_deg

def main():
    print("⏳ Đang nạp dữ liệu và tính toán...")
    
    # ==============================================================
    # 🛠️ TÙY CHỈNH FILE TẠI ĐÂY
    # ==============================================================
    # Thay đổi 3 biến này tương ứng với đợt thực nghiệm bạn muốn vẽ
    
    PATH_FILE = 'path_custom.csv'               # Tên file quỹ đạo lý thuyết
    AMCL_FILE = 'amcl_poses_waypoint2.csv'         # Tên file AMCL thực tế
    CASE_NAME = 'Trường hợp dùng quỹ đạo từ bộ planer A*'           # Tên hiển thị trên tiêu đề biểu đồ
    
    # Gợi ý đổi tên cho các trường hợp khác:
    # Trường hợp 1: PATH_FILE = 'path_waypoint.csv', AMCL_FILE = 'amcl_poses_waypoint.csv', CASE_NAME = 'Nav2 Waypoint 1'
    # Trường hợp 2: PATH_FILE = 'path_waypoint2.csv', AMCL_FILE = 'amcl_poses_waypoint2.csv', CASE_NAME = 'Nav2 Waypoint 2'
    # ==============================================================

    try:
        ref_path = pd.read_csv(PATH_FILE)
        amcl_data = pd.read_csv(AMCL_FILE)
    except FileNotFoundError as e:
        print(f"❌ LỖI: Không tìm thấy file dữ liệu! Chi tiết: {e}")
        return

    # Tính toán
    rmse_pos, act_yaw, ref_yaw = calculate_errors(amcl_data, ref_path)

    print("\n" + "="*50)
    print(f"📊 KẾT QUẢ ĐÁNH GIÁ ĐỊNH VỊ: {CASE_NAME}")
    print("="*50)
    print(f"Vị trí (RMSE)   : {rmse_pos:.4f} m")
    print("="*50 + "\n")

    # ================= VẼ BIỂU ĐỒ =================
    try:
        plt.style.use('ggplot')
    except OSError:
        pass 

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))
    
    # ---------------- BỨC 1: QUỸ ĐẠO X-Y ----------------
    ax1.plot(ref_path['X_ref'].values, ref_path['Y_ref'].values, 'k--', label='Quỹ đạo mong muốn', linewidth=2.5)
    ax1.plot(amcl_data['X'].values, amcl_data['Y'].values, 'b-', label=f'Thực tế (RMSE Vị trí: {rmse_pos:.3f}m)', alpha=0.8, linewidth=2.0)
    
    ax1.set_title(f'Đánh giá Quỹ đạo Không gian 2D - {CASE_NAME}', fontsize=14, fontweight='bold', pad=15)
    ax1.set_xlabel('Trục X (m)', fontsize=12)
    ax1.set_ylabel('Trục Y (m)', fontsize=12)
    ax1.legend(loc='best', fontsize=11, frameon=True, shadow=True)
    ax1.axis('equal') 

    # ---------------- BỨC 2: ĐÁP ỨNG GÓC HƯỚNG ----------------
    time_steps = np.arange(len(act_yaw))
    
    # Vẽ đường Setpoint
    ax2.plot(time_steps, ref_yaw, 'k--', label='Góc hướng mong muốn', linewidth=2.5)
    
    # Vẽ Đáp ứng (Đã lược bỏ RMSE)
    ax2.plot(time_steps, act_yaw, 'b-', label='Đáp ứng thực tế', alpha=0.8, linewidth=2.0)
    
    ax2.set_title(f'Đánh giá Đáp ứng Góc Hướng - {CASE_NAME}', fontsize=14, fontweight='bold', pad=15)
    ax2.set_xlabel('Thời gian / Số lượng mẫu thu thập', fontsize=12)
    ax2.set_ylabel('Góc hướng thực tế (Độ)', fontsize=12)
    
    # Đặt giới hạn trục Y từ -190 đến 190 độ để dễ nhìn các điểm neo
    ax2.set_ylim([-190, 190])
    ax2.set_yticks([-180, -90, 0, 90, 180])
    
    ax2.legend(loc='best', fontsize=11, frameon=True, shadow=True)

    plt.tight_layout(pad=3.0)
    
    # Tự động lưu file ảnh theo tên Case Name để không bị đè file
    save_name = f'Analysis_{CASE_NAME.replace(" ", "_")}.png'
    plt.savefig(save_name, dpi=300, bbox_inches='tight')
    print(f"✅ Đã xuất biểu đồ thành công ra file: {save_name}")
    plt.show()

if __name__ == '__main__':
    main()