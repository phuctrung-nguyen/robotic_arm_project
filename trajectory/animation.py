"""
Mô phỏng 3D (animation) robot RPRR thực hiện chu trình hàn dọc trục Z.

Quỹ đạo: Home -> B (dừng mồi hồ quang) -> A (dừng ngắt hồ quang) -> Home,
mỗi chặng dùng biên dạng LSPB. Animation phát đúng tốc độ thời gian thực.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# =================================================================
# 1. THÔNG SỐ CƠ KHÍ ROBOT RPRR
# =================================================================
D0 = 0.084
D1 = 0.2175
A2 = 0.25
D3 = 0.105
D4 = 0.262
DT = 0.01

# Các điểm mốc trong không gian Cartesian (hàn dọc trục Z)
P_HOME = np.array([0.1518, -0.2730, 0.7698])
P_B = np.array([0.1589, -0.1543, 0.7782])  # điểm bắt đầu hàn (trên cao)
P_A = np.array([0.1589, -0.1543, 0.5782])  # điểm kết thúc hàn (dưới thấp)


# =================================================================
# 2. ĐỘNG HỌC NGƯỢC VÀ VỊ TRÍ CÁC KHÂU
# =================================================================
def inverse_kinematics(x, y, z):
    """Động học ngược theo bảng DH: vị trí mũi hàn (x, y, z) -> biến khớp [q1, q2, q3]."""
    r = np.sqrt(x ** 2 + y ** 2)
    q1 = np.arctan2(D3, np.sqrt(r ** 2 - D3 ** 2)) + np.arctan2(y, x)
    cos_q3 = (x * np.cos(q1) + y * np.sin(q1) - A2) / D4
    cos_q3 = np.clip(cos_q3, -1.0, 1.0)
    sin_q3 = np.sqrt(1 - cos_q3 ** 2)
    q3 = np.arctan2(sin_q3, cos_q3)
    q2 = z - D4 * sin_q3 - D0 - D1
    return np.array([q1, q2, q3])


def get_robot_links(q1, q2, q3):
    """Tọa độ các điểm nút của khung xương robot (theo chuỗi DH), dùng để vẽ."""
    c1, s1 = np.cos(q1), np.sin(q1)
    z1 = D0 + D1 + q2                                         # độ cao tay đòn A2
    p0 = np.array([0, 0, 0])                                  # chân đế
    p1 = np.array([0, 0, D0 + D1])                            # khớp q1
    p2 = np.array([0, 0, z1])                                 # cuối hành trình tịnh tiến q2
    p3 = np.array([A2 * c1, A2 * s1, z1])                     # cuối tay đòn A2
    p4 = p3 + np.array([D3 * s1, -D3 * c1, 0])                # khớp q3 (lệch ngang D3)
    p5 = p4 + D4 * np.array([c1 * np.cos(q3), s1 * np.cos(q3), np.sin(q3)])  # gốc O4
    return np.vstack([p0, p1, p2, p3, p4, p5])


# =================================================================
# 3. QUY HOẠCH QUỸ ĐẠO LSPB 3 CHẶNG + THỜI GIAN DỪNG
# =================================================================
def generate_lspb(dist, V_max, A_max, dt):
    """Biên dạng quãng đường s(t) dạng LSPB trên quãng đường ``dist``."""
    if dist < 1e-5:
        return np.array([]), np.array([])
    if V_max ** 2 / A_max > dist:
        V_max = np.sqrt(dist * A_max)
    t_b = V_max / A_max
    t_c = (dist - V_max * t_b) / V_max
    t_f = 2 * t_b + t_c

    t = np.arange(0, t_f, dt)
    s = np.zeros_like(t)

    for i, ti in enumerate(t):
        if ti < t_b:
            s[i] = 0.5 * A_max * ti ** 2
        elif ti < t_b + t_c:
            s[i] = 0.5 * A_max * t_b ** 2 + V_max * (ti - t_b)
        else:
            tau = ti - t_b - t_c
            s[i] = (dist - 0.5 * A_max * t_b ** 2) + V_max * tau - 0.5 * A_max * tau ** 2
            if s[i] > dist:
                s[i] = dist
    return s, t


def lspb_profile(p_start, p_end, V_max, A_max, dt):
    """Quỹ đạo LSPB trên đoạn thẳng 3D từ ``p_start`` đến ``p_end``."""
    dist = np.linalg.norm(p_end - p_start)
    if dist < 1e-5:
        return np.array([p_start]), np.array([0])
    s, t = generate_lspb(dist, V_max, A_max, dt)
    u = (p_end - p_start) / dist
    pts = p_start + s[:, np.newaxis] * u
    return pts, t


pts1, t1 = lspb_profile(P_HOME, P_B, V_max=0.05, A_max=0.10, dt=DT)
pts2, t2 = lspb_profile(P_B, P_A, V_max=0.01, A_max=0.05, dt=DT)
pts3, t3 = lspb_profile(P_A, P_HOME, V_max=0.05, A_max=0.10, dt=DT)

dwell_b_pts = np.tile(P_B, (int(0.5 / DT), 1))
dwell_a_pts = np.tile(P_A, (int(1.0 / DT), 1))

P_total = np.vstack([pts1, dwell_b_pts, pts2, dwell_a_pts, pts3])
Q_total = np.array([inverse_kinematics(p[0], p[1], p[2]) for p in P_total])

total_frames = len(P_total)
T_total = np.arange(total_frames) * DT

# Mốc thời gian các giai đoạn, suy ra từ độ dài từng đoạn
T_REACH_B = T_total[len(pts1) - 1]
T_LEAVE_B = T_total[len(pts1) + len(dwell_b_pts) - 1]
T_REACH_A = T_total[len(pts1) + len(dwell_b_pts) + len(pts2) - 1]
T_LEAVE_A = T_total[len(pts1) + len(dwell_b_pts) + len(pts2) + len(dwell_a_pts) - 1]
TIME_FINISH = T_total[-1]

print("=" * 55)
print("BẢNG THỐNG KÊ THỜI GIAN CHU TRÌNH HÀN")
print("=" * 55)
print(f"[1] Mỏ hàn chạm điểm B (Bắt đầu Dwell mồi hồ quang): {T_REACH_B:.3f} s")
print(f"[2] Mỏ hàn rời điểm B  (Bắt đầu rê mỏ hàn đi hàn)   : {T_LEAVE_B:.3f} s")
print(f"[3] Mỏ hàn chạm điểm A (Bắt đầu Dwell ngắt hồ quang): {T_REACH_A:.3f} s")
print(f"[4] Mỏ hàn rời điểm A  (Rút mỏ hàn về vị trí Home)  : {T_LEAVE_A:.3f} s")
print(f"[5] Hoàn thành toàn bộ chu trình                      : {TIME_FINISH:.3f} s")
print("=" * 55)

# =================================================================
# 4. ĐỒ HỌA MÔ PHỎNG 3D (TỐC ĐỘ THỜI GIAN THỰC 1:1)
# =================================================================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
fig.canvas.manager.set_window_title(f'Mô phỏng Robot RPRR - {TIME_FINISH:.2f}s')

ax.plot(P_total[:, 0], P_total[:, 1], P_total[:, 2], 'k--', alpha=0.4, label='Đường quỹ đạo thiết kế')
robot_arm, = ax.plot([], [], [], 'o-', color='#1f77b4', lw=5, markersize=8, label='Khung xương Robot')
welding_trail, = ax.plot([], [], [], '-', color='#d62728', lw=3, label='Mối hàn thực tế')
status_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes, fontsize=12, weight='bold')

ax.scatter(*P_HOME, color='green', s=100, label='HOME')
ax.scatter(*P_B, color='orange', s=100, label='B (Bắt đầu hàn)')
ax.scatter(*P_A, color='purple', s=100, label='A (Kết thúc hàn)')

ax.set_xlim(-0.1, 0.4)
ax.set_ylim(-0.4, 0.1)
ax.set_zlim(0, 0.8)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title("MÔ PHỎNG QUY HOẠCH QUỸ ĐẠO CHU TRÌNH HÀN TRỤC Z", fontsize=13, weight='bold')
ax.legend(loc='lower left')
ax.view_init(elev=20, azim=50)

trail_x, trail_y, trail_z = [], [], []

# STEP = 2 với interval = 20 ms -> phát đúng tốc độ thời gian thực
STEP = 2


def update(frame):
    """Cập nhật khung hình: tư thế robot, vết hàn và dòng trạng thái."""
    idx = frame * STEP
    if idx >= total_frames:
        idx = total_frames - 1

    q1, q2, q3 = Q_total[idx]
    t = T_total[idx]

    links = get_robot_links(q1, q2, q3)
    ee_pos = links[-1]

    robot_arm.set_data(links[:, 0], links[:, 1])
    robot_arm.set_3d_properties(links[:, 2])

    if t <= T_REACH_B:
        status = "🤖 Đang tiếp cận điểm hàn B (Không tải)..."
        color = "blue"
    elif T_REACH_B < t <= T_LEAVE_B:
        status = "⏱️ DWELL: Đang phanh dừng chờ mồi hồ quang tại B..."
        color = "goldenrod"
    elif T_LEAVE_B < t <= T_REACH_A:
        status = "🔥 ĐANG HÀN: Rê mỏ hàn thẳng đứng dọc trục Z!"
        color = "red"
        # Vẽ vết hàn trong lúc đang hàn
        trail_x.append(ee_pos[0])
        trail_y.append(ee_pos[1])
        trail_z.append(ee_pos[2])
    elif T_REACH_A < t <= T_LEAVE_A:
        status = "⏱️ DWELL: Đang dừng ngắt hồ quang và điền đầy miệng hàn tại A..."
        color = "orange"
    else:
        status = "🚀 Đã xong nhiệm vụ, đang rút mỏ hàn về HOME!"
        color = "purple"

    welding_trail.set_data(trail_x, trail_y)
    welding_trail.set_3d_properties(trail_z)

    status_text.set_text(f"Thời gian: {t:.2f}s / {TIME_FINISH:.2f}s | Trạng thái: {status}")
    status_text.set_color(color)
    return robot_arm, welding_trail, status_text


ani = animation.FuncAnimation(fig, update, frames=total_frames // STEP, interval=20, blit=False, repeat=False)
plt.show()
