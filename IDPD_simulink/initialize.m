% INITIALIZE  Khởi tạo tham số hình học, động lực học và bộ điều khiển
%             cho robot hàn 4 bậc tự do RPRR.
%
%   Chạy file này TRƯỚC khi mở và chạy mô hình controller_simulation.slx.
%   Kết quả: các biến global và struct 'p' trong MATLAB Workspace.

clear;
clear global; % Xóa các biến global cũ nếu có
global g d0 d1 a2 d3 d4 ...
       xc1 yc1 zc1 xc2 yc2 zc2 xc3 yc3 zc3 xc4 yc4 zc4 ...
       m1 m2 m3 m4 IC1 IC2 IC3 IC4 ...
       Ixx1 Ixy1 Ixz1 Iyy1 Iyz1 Izz1 ...
       Ixx2 Ixy2 Ixz2 Iyy2 Iyz2 Izz2 ...
       Ixx3 Ixy3 Ixz3 Iyy3 Iyz3 Izz3 ...
       Ixx4 Ixy4 Ixz4 Iyy4 Iyz4 Izz4
global Kp Kd

% Hệ số bộ điều khiển IDPD
Kp = diag([500, 500, 500, 500]);
Kd = diag([40,  40,  40,  40]);

% =========================================================================
% 1. GIA TỐC TRỌNG TRƯỜNG
% =========================================================================
g = 9.81; % [m/s^2]

% =========================================================================
% 2. THAM SỐ HÌNH HỌC ĐỘNG HỌC [m]
% =========================================================================
d0 = 0.084;
d1 = 0.2175;
a2 = 0.25;
d3 = 0.105;
d4 = 0.262;

% Tọa độ tâm khối lượng (CoM) từng khâu so với hệ trục khâu
xc1 = 0.0;       yc1 = 0.0;       zc1 = -0.06624;
xc2 = -0.0871;   yc2 = 0.0;       zc2 = 0.02104;
xc3 = 0.0;       yc3 = -0.00053;  zc3 = 0.0354;
xc4 = -0.00226;  yc4 = 0.0;       zc4 = -0.04753;

% =========================================================================
% 3. KHỐI LƯỢNG CÁC KHÂU [kg]
% =========================================================================
m1 = 5.91726;
m2 = 2.81241;
m3 = 0.47967;
m4 = 0.23944;

% =========================================================================
% 4. TENSOR QUÁN TÍNH CÁC KHÂU [kg*m^2] (xuất từ SolidWorks)
% =========================================================================
% Khâu 1
IC1 = [ 0.15474082,  0,           0;
        0,           0.15474082,  0;
        0,           0,           0.01090799 ];
% Khâu 2
IC2 = [ 0.00468339,  0,           0.00517658;
        0,           0.03089511,  0;
        0.00517658,  0,           0.02785621 ];
% Khâu 3
IC3 = [ 0.00095903,  0,           0;
        0,           0.00102455,  0.00000954;
        0,           0.00000954,  0.00019716 ];
% Khâu 4
IC4 = [  0.00083898,  0,          -0.00008676;
         0,           0.00086920,  0;
        -0.00008676,  0,           0.00004376 ];

% =========================================================================
% 5. TÁCH PHẦN TỬ TENSOR QUÁN TÍNH
% =========================================================================
Ixx1=IC1(1,1); Ixy1=IC1(1,2); Ixz1=IC1(1,3); Iyy1=IC1(2,2); Iyz1=IC1(2,3); Izz1=IC1(3,3);
Ixx2=IC2(1,1); Ixy2=IC2(1,2); Ixz2=IC2(1,3); Iyy2=IC2(2,2); Iyz2=IC2(2,3); Izz2=IC2(3,3);
Ixx3=IC3(1,1); Ixy3=IC3(1,2); Ixz3=IC3(1,3); Iyy3=IC3(2,2); Iyz3=IC3(2,3); Izz3=IC3(3,3);
Ixx4=IC4(1,1); Ixy4=IC4(1,2); Ixz4=IC4(1,3); Iyy4=IC4(2,2); Iyz4=IC4(2,3); Izz4=IC4(3,3);

disp('==> Da khoi tao thanh cong cac tham so robot!');

% =========================================================================
% 6. ĐÓNG GÓI TOÀN BỘ THAM SỐ VÀO STRUCT 'p' CHO SIMULINK
% =========================================================================
p.g = g; p.a2 = a2; p.d3 = d3; p.d4 = d4;
p.m1 = m1; p.m2 = m2; p.m3 = m3; p.m4 = m4;
p.xc1 = xc1; p.xc2 = xc2; p.xc3 = xc3; p.xc4 = xc4;
p.yc1 = yc1; p.yc3 = yc3; p.yc4 = yc4;
p.zc2 = zc2; p.zc3 = zc3; p.zc4 = zc4;
p.Ixx3 = Ixx3; p.Ixx4 = Ixx4; p.Iyy2 = Iyy2; p.Iyy3 = Iyy3; p.Iyy4 = Iyy4;
p.Izz1 = Izz1; p.Izz3 = Izz3; p.Izz4 = Izz4;
p.Ixy3 = Ixy3; p.Ixy4 = Ixy4; p.Ixz3 = Ixz3; p.Ixz4 = Ixz4; p.Iyz3 = Iyz3; p.Iyz4 = Iyz4;
disp('==> Da dong goi thanh cong struct p!');

% =========================================================================
% 7. TƯ THẾ BAN ĐẦU CỦA ROBOT = ĐIỂM ĐẦU QUỸ ĐẠO ĐẶT (HOME)
% =========================================================================
% Dùng làm điều kiện đầu của khối tích phân robot/qthuc, để robot xuất phát
% đúng tại HOME thay vì tại q = 0 (tránh giật và mô-men lớn lúc t = 0).
traj = load(fullfile(fileparts(mfilename('fullpath')), 'Vitridat.mat'));
q_init = traj.Vitridat(2:5, 1);
clear traj;
fprintf('==> Tu the ban dau q_init = [%s]\n', num2str(q_init.', '%.4f  '));
