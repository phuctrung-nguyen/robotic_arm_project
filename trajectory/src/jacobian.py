"""
Jacobian vị trí và chỉ số manipulability (phát hiện điểm kỳ dị).

- ``position_jacobian()``: Jacobian vị trí J_v (3x4) của đầu mỏ hàn, tính bằng
  sai phân trung tâm dựa trên động học thuận có sẵn.
- ``manipulability()``   : w = sqrt(det(J_v · J_v^T)).
  w lớn -> robot khéo léo; w ≈ 0 -> gần điểm kỳ dị (singularity).
"""

import numpy as np


def position_jacobian(robot, q, h=1e-6):
    """
    Tính Jacobian vị trí J_v (3x4) bằng sai phân trung tâm.

    Cột j là độ thay đổi của (x, y, z) khi nhích khớp j một lượng nhỏ h:
        J_v[:, j] = (FK(q + h·e_j) - FK(q - h·e_j)) / (2h)

    Args:
        robot: đối tượng ``Robot``.
        q: 4 phần tử ``[q1, q2, q3, q4]``; mỗi phần tử là mảng numpy (N,) hoặc số vô hướng.
        h: bước sai phân.

    Returns:
        Mảng J có shape ``(..., 3, 4)``.
    """
    q = [np.asarray(qi) * 1.0 for qi in q]  # ép về float / mảng
    shape = q[0].shape
    J = np.zeros(shape + (3, 4))

    for j in range(4):
        qp = [qi.copy() for qi in q]
        qp[j] = qp[j] + h
        qm = [qi.copy() for qi in q]
        qm[j] = qm[j] - h

        xp, yp, zp = robot.forward_kinematics(qp)
        xm, ym, zm = robot.forward_kinematics(qm)

        J[..., 0, j] = (xp - xm) / (2 * h)  # dx/dqj
        J[..., 1, j] = (yp - ym) / (2 * h)  # dy/dqj
        J[..., 2, j] = (zp - zm) / (2 * h)  # dz/dqj

    return J


def manipulability(robot, q):
    """
    Chỉ số manipulability w = sqrt(det(J_v · J_v^T)).

    J_v có kích thước 3x4 (không vuông) nên không lấy det(J) trực tiếp được;
    J_v · J_v^T là ma trận 3x3 nên lấy định thức được.

    Returns:
        Mảng w (cùng kích thước với số mẫu). w càng nhỏ càng gần kỳ dị; w = 0 là kỳ dị.
    """
    J = position_jacobian(robot, q)    # (..., 3, 4)
    JJt = J @ np.swapaxes(J, -1, -2)   # (..., 3, 3)
    det = np.linalg.det(JJt)
    det = np.clip(det, 0.0, None)      # chống sai số âm nhỏ
    return np.sqrt(det)
