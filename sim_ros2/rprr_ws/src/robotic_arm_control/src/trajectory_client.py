#!/usr/bin/env python3
"""
Node ROS 2 phát quỹ đạo hàn cho robot RPRR trong Gazebo.

Quỹ đạo Cartesian LSPB Home -> B -> dừng -> A -> dừng -> Home được tính trước,
chuyển sang không gian khớp bằng động học ngược, rồi phát lần lượt từng điểm
(chu kỳ 10 ms) xuống các topic ``/q1_cmd_pos`` ... ``/q4_cmd_pos``.

Trước chu trình hàn, node đọc tư thế hiện tại từ ``/joint_states`` và đưa robot
về HOME bằng một đoạn chuyển động êm trong không gian khớp, rồi giữ tại HOME
một chút cho các bộ PID ổn định. Chạy xong chu trình, node tự thoát.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINT_NAMES = ('q1', 'q2', 'q3', 'q4')
# Giới hạn vận tốc khớp trong URDF (rad/s, m/s, rad/s, rad/s)
JOINT_VEL_LIMITS = np.array([0.5, 0.4, 0.5, 0.8])
# Đoạn đưa về HOME chỉ dùng tối đa nửa giới hạn vận tốc
APPROACH_VEL_SCALE = 0.5
APPROACH_MIN_DURATION = 2.0  # s
HOME_SETTLE_DURATION = 1.0   # s


class TrajectoryDirectPublisher(Node):
    """Phát trực tiếp vị trí đặt của từng khớp theo quỹ đạo đã tính sẵn."""

    def __init__(self):
        super().__init__('trajectory_direct_publisher')

        # Publisher vị trí đặt cho từng khớp trong mô phỏng
        self._pub_q1 = self.create_publisher(Float64, '/q1_cmd_pos', 10)
        self._pub_q2 = self.create_publisher(Float64, '/q2_cmd_pos', 10)
        self._pub_q3 = self.create_publisher(Float64, '/q3_cmd_pos', 10)
        self._pub_q4 = self.create_publisher(Float64, '/q4_cmd_pos', 10)
        self._sub_joint_states = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10)

        self.DT = 0.01

        # Chu trình hàn (N, 3) cho q1, q2, q3; q4 luôn bằng 0
        self.Q_profile = self._calculate_joint_space_trajectory()
        # Toàn bộ điểm sẽ phát (N, 4): đưa về HOME + giữ + chu trình hàn;
        # chỉ tạo được khi đã biết tư thế ban đầu
        self._commands = None
        self._q_current = None
        self.current_index = 0
        self.done = False

        self._timer = self.create_timer(self.DT, self.timer_callback)
        self.get_logger().info(
            f'Chu trình hàn gồm {len(self.Q_profile)} điểm. '
            'Đang chờ /joint_states để đưa robot về HOME...')

    def _joint_state_callback(self, msg):
        """Lưu tư thế khớp hiện tại (q1..q4) từ ``/joint_states``."""
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINT_NAMES):
            self._q_current = np.array([positions[name] for name in JOINT_NAMES])

    def _build_commands(self, q_start):
        """Ghép đoạn đưa về HOME (đa thức bậc 3), đoạn giữ tại HOME và chu trình hàn."""
        q_home = np.append(self.Q_profile[0], 0.0)
        delta = q_home - q_start
        # Vận tốc đỉnh của đa thức bậc 3 là 1.5 * delta / T
        duration = max(APPROACH_MIN_DURATION,
                       np.max(1.5 * np.abs(delta) / (APPROACH_VEL_SCALE * JOINT_VEL_LIMITS)))
        tau = np.arange(0.0, duration, self.DT) / duration
        blend = 3 * tau ** 2 - 2 * tau ** 3
        approach = q_start + blend[:, np.newaxis] * delta
        settle = np.tile(q_home, (int(round(HOME_SETTLE_DURATION / self.DT)), 1))
        cycle = np.column_stack([self.Q_profile, np.zeros(len(self.Q_profile))])
        self.get_logger().info(
            f'Đưa robot về HOME trong {duration:.2f} s, giữ {HOME_SETTLE_DURATION:.1f} s, '
            f'rồi chạy chu trình hàn {(len(cycle) - 1) * self.DT:.2f} s.')
        return np.vstack([approach, settle, cycle])

    def _calculate_joint_space_trajectory(self):
        """Tính trước toàn bộ quỹ đạo khớp ``(N, 3)`` cho q1, q2, q3."""
        D0, D1, A2, D3, D4 = 0.084, 0.2175, 0.25, 0.105, 0.262
        P_Home = np.array([0.1518, -0.2730, 0.7698])
        P_B = np.array([0.1589, -0.1543, 0.7782])
        P_A = np.array([0.1589, -0.1543, 0.5782])

        def generate_lspb(P_start, P_end, V_max, A_max, dt):
            dist = np.linalg.norm(P_end - P_start)
            if dist < 1e-5:
                return np.empty((0, 3))
            if V_max ** 2 / A_max > dist:
                V_max = np.sqrt(dist * A_max)
            t_b = V_max / A_max
            t_c = (dist - V_max * t_b) / V_max
            t = np.arange(0, 2 * t_b + t_c, dt)
            s = np.zeros_like(t)
            for i, ti in enumerate(t):
                if ti < t_b:
                    s[i] = 0.5 * A_max * ti ** 2
                elif ti < t_b + t_c:
                    s[i] = 0.5 * A_max * t_b ** 2 + V_max * (ti - t_b)
                else:
                    tau = ti - t_b - t_c
                    s[i] = ((0.5 * A_max * t_b ** 2 + V_max * t_c)
                            + (V_max * tau - 0.5 * A_max * tau ** 2))
            direction = (P_end - P_start) / dist
            return P_start + s[:, np.newaxis] * direction

        def generate_dwell(P_stay, duration, dt):
            t = np.arange(0, duration, dt)
            return np.tile(P_stay, (len(t), 1))

        def inverse_kinematics(x, y, z):
            # Động học ngược theo bảng DH (khớp với URDF)
            r = np.sqrt(x ** 2 + y ** 2)
            q1 = np.arctan2(D3, np.sqrt(r ** 2 - D3 ** 2)) + np.arctan2(y, x)
            cos_q3 = np.clip((x * np.cos(q1) + y * np.sin(q1) - A2) / D4, -1.0, 1.0)
            q3 = np.arctan2(np.sqrt(1 - cos_q3 ** 2), cos_q3)
            q2 = z - D4 * np.sin(q3) - D0 - D1
            return q1, q2, q3

        p1 = generate_lspb(P_Home, P_B, V_max=0.05, A_max=0.1, dt=self.DT)
        pd_b = generate_dwell(P_B, duration=0.5, dt=self.DT)
        p2 = generate_lspb(P_B, P_A, V_max=0.01, A_max=0.05, dt=self.DT)
        pd_a = generate_dwell(P_A, duration=1.0, dt=self.DT)
        p3 = generate_lspb(P_A, P_Home, V_max=0.05, A_max=0.1, dt=self.DT)

        P_total = np.vstack([p1, pd_b, p2, pd_a, p3])

        N_points = len(P_total)
        Q_profile = np.zeros((N_points, 3))

        for i in range(N_points):
            x, y, z = P_total[i]
            q1, q2, q3 = inverse_kinematics(x, y, z)
            Q_profile[i] = [q1, q2, q3]

        return Q_profile

    def timer_callback(self):
        """Phát điểm quỹ đạo kế tiếp; dừng timer khi hết quỹ đạo."""
        if self._commands is None:
            if self._q_current is None:
                return
            self._commands = self._build_commands(self._q_current)

        if self.current_index >= len(self._commands):
            self.get_logger().info(
                'Hoàn thành! Robot đã đi từ Home -> B -> Dwell -> A -> Dwell '
                '-> Rút về Home thành công!')
            self._timer.cancel()
            self.done = True
            return

        q1, q2, q3, q4 = self._commands[self.current_index]

        msg_q1 = Float64(data=float(q1))
        msg_q2 = Float64(data=float(q2))
        msg_q3 = Float64(data=float(q3))
        msg_q4 = Float64(data=float(q4))

        self._pub_q1.publish(msg_q1)
        self._pub_q2.publish(msg_q2)
        self._pub_q3.publish(msg_q3)
        self._pub_q4.publish(msg_q4)

        self.current_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryDirectPublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
