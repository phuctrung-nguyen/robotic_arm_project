"""
Phân tích không gian làm việc và chỉ số manipulability của robot hàn RPRR.

Các bước:
    1. Đọc tham số DH từ ``config/dh_params.yaml``.
    2. Lấy mẫu Monte Carlo các khớp trong giới hạn và tính động học thuận.
    3. In thông số biên của không gian làm việc.
    4. Lưu các điểm ra ``data/workspace_points.npy``.
    5. Tính chỉ số manipulability w (tìm vùng kỳ dị).
    6. Vẽ toàn bộ kết quả và lưu ra ``data/ket_qua.png``.

Chạy:
    python main.py
"""

import os

from src.kinematics import Robot
from src.workspace_calculator import compute_workspace, print_summary, save_points
from src.utils import plot_all
from src.jacobian import manipulability

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "config", "dh_params.yaml")
DATA_DIR = os.path.join(BASE_DIR, "data")


def main():
    # 1) Đọc cấu hình robot
    robot = Robot(CONFIG_PATH)

    # 2) Tính workspace (lấy mẫu ngẫu nhiên + FK); seed=42 để kết quả lặp lại được
    ws = compute_workspace(robot, seed=42)

    # 3) In thông số biên
    print_summary(ws)

    # 4) Lưu điểm
    save_points(ws, out_dir=DATA_DIR)

    # 5) Tính manipulability w (tìm vùng kỳ dị)
    w = manipulability(robot, ws["q"])
    print(f"Manipulability w: {w.min():.5f}  ->  {w.max():.5f}")

    # 6) Vẽ tất cả trong một cửa sổ và lưu hình
    plot_all(ws, w, save_path=os.path.join(DATA_DIR, "ket_qua.png"))


if __name__ == "__main__":
    main()
