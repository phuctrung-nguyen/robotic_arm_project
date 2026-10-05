"""
Động học thuận của robot hàn 4 bậc tự do (R-P-R-R).

- ``Robot.dh_matrix()``         : ma trận biến đổi thuần nhất Denavit-Hartenberg.
- ``Robot.forward_kinematics()``: tính vị trí đầu mỏ hàn từ biến khớp.

Tham số được đọc từ ``config/dh_params.yaml``. Mọi hàm đều nhận được mảng
numpy làm đầu vào để chạy Monte Carlo nhanh (vector hóa).
"""

import numpy as np
import yaml


class Robot:
    """Mô hình động học robot RPRR dựng từ file cấu hình DH."""

    def __init__(self, config_path):
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        self.base_offset_d = cfg["base_offset_d"]
        self.joints = cfg["joints"]
        self.torch_offset = np.array(cfg["torch_offset_mm"]) / 1000.0  # mm -> m
        self.torch_offset = np.append(self.torch_offset, 1.0)          # tọa độ thuần nhất [x, y, z, 1]
        self.n_samples = cfg["n_samples"]

    def get_limits(self):
        """Trả về giới hạn các khớp ``[(min, max), ...]`` theo đơn vị tính toán (rad / m)."""
        limits = []
        for j in self.joints:
            lo, hi = j["limit"]
            if j["type"] == "revolute":
                limits.append((np.deg2rad(lo), np.deg2rad(hi)))
            else:  # prismatic
                limits.append((lo, hi))
        return limits

    @staticmethod
    def dh_matrix(theta, d, a, alpha):
        """Ma trận DH 4x4. Các tham số có thể là số vô hướng hoặc mảng numpy."""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        ones = np.ones_like(ct)
        shape = ones.shape
        T = np.zeros(shape + (4, 4))
        T[..., 0, 0] = ct; T[..., 0, 1] = -st * ca; T[..., 0, 2] =  st * sa; T[..., 0, 3] = a * ct  # noqa: E702
        T[..., 1, 0] = st; T[..., 1, 1] =  ct * ca; T[..., 1, 2] = -ct * sa; T[..., 1, 3] = a * st  # noqa: E702
        T[..., 2, 1] = sa; T[..., 2, 2] = ca;       T[..., 2, 3] = d * ones                         # noqa: E702
        T[..., 3, 3] = 1.0
        return T

    def forward_kinematics(self, q):
        """
        Động học thuận: biến khớp -> vị trí đầu mỏ hàn.

        Args:
            q: 4 phần tử ``[q1, q2, q3, q4]``; mỗi phần tử là số vô hướng
               hoặc mảng numpy cùng kích thước.

        Returns:
            ``(x, y, z)`` của đầu mỏ hàn trong hệ tọa độ gốc.
        """
        q = list(q)
        q1_shape = np.asarray(q[0]) * 1.0  # ép về float / mảng
        ones = np.ones_like(q1_shape)

        # Khâu 0: đế cố định
        T = self.dh_matrix(0.0 * ones, self.base_offset_d, 0.0, 0.0)

        # Lần lượt nhân 4 khâu động
        for i, j in enumerate(self.joints):
            qi = np.asarray(q[i]) * 1.0
            if j["type"] == "revolute":
                theta = qi + np.deg2rad(j["theta_offset"])
                d = j["d"]
            else:  # prismatic
                theta = np.deg2rad(j["theta"]) * ones
                d = qi + j["d_offset"]
            a = j["a"]
            alpha = np.deg2rad(j["alpha"])
            T = T @ self.dh_matrix(theta, d, a, alpha)

        # Ghép offset đầu mỏ hàn (hệ O4 -> hệ gốc)
        p = T @ self.torch_offset
        return p[..., 0], p[..., 1], p[..., 2]
