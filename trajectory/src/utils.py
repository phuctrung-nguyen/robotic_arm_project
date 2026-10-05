"""
Các hàm vẽ đồ thị không gian làm việc và manipulability.

- ``plot_all()``: workspace 3D, hình chiếu bằng (XY), mặt cắt dọc (R-Z) và
  workspace tô màu theo chỉ số manipulability w, trong cùng một cửa sổ.
"""

import os
import numpy as np
import matplotlib.pyplot as plt


def plot_all(ws, w, max_points=20000, save_path=None):
    """
    Gộp tất cả đồ thị vào một cửa sổ duy nhất.

    - Hàng trên: workspace (3D, XY, R-Z).
    - Hàng dưới: manipulability tô màu theo w (R-Z, XY).
    """
    x, y, z, r = ws["x"], ws["y"], ws["z"], ws["r"]

    n = x.size
    if n > max_points:
        idx = np.random.choice(n, max_points, replace=False)
        x, y, z, r, w = x[idx], y[idx], z[idx], r[idx], w[idx]

    fig = plt.figure(figsize=(15, 9))

    # ===== Hàng trên: workspace =====
    ax1 = fig.add_subplot(231, projection="3d")
    ax1.scatter(x, y, z, s=1, c=z, cmap="viridis", alpha=0.3)
    ax1.set_title("Workspace 3D")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")

    ax2 = fig.add_subplot(232)
    ax2.scatter(x, y, s=1, alpha=0.2, c="tab:blue")
    ax2.set_title("Workspace - nhin tu tren (XY)")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.axis("equal")
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(233)
    ax3.scatter(r, z, s=1, alpha=0.2, c="tab:red")
    ax3.set_title("Workspace - mat cat (R-Z)")
    ax3.set_xlabel("R (m)")
    ax3.set_ylabel("Z (m)")
    ax3.grid(True, alpha=0.3)

    # ===== Hàng dưới: manipulability =====
    ax4 = fig.add_subplot(234)
    sc4 = ax4.scatter(r, z, s=2, c=w, cmap="viridis", alpha=0.5)
    ax4.set_title("Manipulability (R-Z)")
    ax4.set_xlabel("R (m)")
    ax4.set_ylabel("Z (m)")
    ax4.grid(True, alpha=0.3)
    fig.colorbar(sc4, ax=ax4, label="w")

    ax5 = fig.add_subplot(235)
    sc5 = ax5.scatter(x, y, s=2, c=w, cmap="viridis", alpha=0.5)
    ax5.set_title("Manipulability (XY)")
    ax5.set_xlabel("X (m)")
    ax5.set_ylabel("Y (m)")
    ax5.axis("equal")
    ax5.grid(True, alpha=0.3)
    fig.colorbar(sc5, ax=ax5, label="w")

    plt.tight_layout()

    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=110)
        print(f"Da luu hinh vao: {save_path}")

    plt.show()
