"""
Tính không gian làm việc (workspace) của robot bằng phương pháp Monte Carlo.

- ``compute_workspace()``: lấy mẫu ngẫu nhiên 4 khớp trong giới hạn -> động học
  thuận -> tọa độ (x, y, z) của đầu mỏ hàn.
- ``print_summary()``    : in thông số biên (r, z, x, y).
- ``save_points()``      : lưu các điểm ra file ``.npy``.
"""

import os
import numpy as np


def compute_workspace(robot, n_samples=None, seed=None):
    """
    Args:
        robot: đối tượng ``Robot`` (từ ``kinematics.py``).
        n_samples: số điểm lấy mẫu (mặc định lấy từ file cấu hình).
        seed: cố định seed để kết quả lặp lại được.

    Returns:
        dict ``{x, y, z, r, q}``; ``q`` là danh sách 4 mảng biến khớp
        (dùng để tính manipulability).
    """
    if n_samples is None:
        n_samples = robot.n_samples
    if seed is not None:
        np.random.seed(seed)

    limits = robot.get_limits()  # [(min, max)] x 4, đơn vị rad / m

    # Lấy mẫu ngẫu nhiên từng khớp trong giới hạn
    q = [np.random.uniform(lo, hi, n_samples) for (lo, hi) in limits]

    x, y, z = robot.forward_kinematics(q)
    r = np.sqrt(x ** 2 + y ** 2)

    return {"x": x, "y": y, "z": z, "r": r, "q": q}


def print_summary(ws):
    """In thông số biên của workspace."""
    x, y, z, r = ws["x"], ws["y"], ws["z"], ws["r"]
    print("============ THONG SO WORKSPACE ============")
    print(f"So diem          : {x.size}")
    print(f"Ban kinh ngang r : {r.min():.3f}  ->  {r.max():.3f}  m")
    print(f"Chieu cao      z : {z.min():.3f}  ->  {z.max():.3f}  m")
    print(f"X                : {x.min():.3f}  ->  {x.max():.3f}  m")
    print(f"Y                : {y.min():.3f}  ->  {y.max():.3f}  m")
    print("============================================")


def save_points(ws, out_dir="data", filename="workspace_points.npy"):
    """Lưu các điểm (x, y, z) ra file ``.npy`` để dùng lại sau."""
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, filename)
    pts = np.column_stack([ws["x"], ws["y"], ws["z"]])
    np.save(path, pts)
    print(f"Da luu {pts.shape[0]} diem vao: {path}")
    return path
