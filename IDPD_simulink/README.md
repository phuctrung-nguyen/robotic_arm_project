# Hướng dẫn chạy mô phỏng Simulink

Dự án này bao gồm mô hình Simulink và các tệp dữ liệu cần thiết. Để đảm bảo mô phỏng chạy chính xác, bạn vui lòng thực hiện đúng theo các bước sau:

## Các bước thực hiện

1. **Khởi tạo thông số (Quan trọng):**
   - Mở phần mềm MATLAB.
   - Điều hướng (Navigate) đến thư mục chứa các tệp dự án này.
   - Chạy tệp **`initialize.m`** bằng cách gõ lệnh sau vào Command Window hoặc click chuột phải vào file rồi chọn *Run*:
     ```matlab
     initialize
     ```
   - Bước này sẽ nạp các biến và tham số cần thiết vào *MATLAB Workspace*.

2. **Chạy mô hình Simulink:**
   - Sau khi đã chạy xong `initialize.m`, hãy mở tệp **`controller_simulation.slx`**.
   - Nhấn nút **Run** trong giao diện Simulink để bắt đầu mô phỏng.

---
**Lưu ý:** Nếu bạn không chạy `initialize.m` trước, mô hình Simulink sẽ báo lỗi do thiếu dữ liệu đầu vào hoặc tham số hệ thống chưa được định nghĩa.

## Dữ liệu quỹ đạo đặt

`Vitridat.mat`, `Vantocdat.mat`, `Giatocdat.mat` (vị trí, vận tốc, gia tốc khớp đặt; mỗi file 5 × 2961: hàng 0 là
thời gian, hàng 1–3 là q1..q3, hàng 4 là q4 = 0) được sinh bởi `03_dong_hoc_quy_dao_python/JointSpace.py`. Muốn đổi
quỹ đạo thì sửa script đó rồi chạy lại `python JointSpace.py`. Đây cũng chính là quỹ đạo mà node ROS 2
`trajectory_client.py` phát trong Gazebo.

- Robot xuất phát tại HOME: `initialize.m` đọc điểm đầu của `Vitridat.mat` vào biến `q_init`, dùng làm điều kiện đầu
  của khối tích phân `robot/qthuc`.
- Sau khi quỹ đạo kết thúc (29.60 s), các khối From File giữ nguyên vị trí cuối, vận tốc và gia tốc đặt bằng 0, nên
  robot đứng yên tại HOME tới hết thời gian mô phỏng (35 s).

## Kết quả mô phỏng

Với Kp = 500, Kd = 40 (MATLAB R2024a, solver ode45):

| Chỉ tiêu | Giá trị |
|---|---|
| Sai số khớp lớn nhất (0–35 s) | q1: 1.2·10⁻³ rad, q2: 0.23 mm, q3: 8.5·10⁻⁴ rad |
| Sai số mũi hàn lớn nhất | 0.25 mm (cả chu trình), 0.05 mm (đoạn hàn B → A) |
| Lực/mô-men điều khiển lớn nhất | q1: 0.13 N·m, q2: 35.4 N (chủ yếu giữ trọng lực), q3: 0.14 N·m |
