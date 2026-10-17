%% 圆形轨迹跟踪效果绘图
% 前提：仿真已运行，Scope 已开启 Logging（或手动勾选 DataLogging）

% --- 如果 Scope 没有开启 Logging，在 Scope 属性里勾选 "Log data to workspace" ---
% 变量名建议设为:
%   x Scope    → ScopeData3
%   y Scope    → ScopeData4
%   yaw Scope  → ScopeData5
%   z Scope    → ScopeData

% ===== 方法1：从 ScopeData 读取（推荐） =====
% 假设 ScopeData3=x实际, ScopeData4=y实际, ScopeData=z实际
if exist('ScopeData3', 'var') && exist('ScopeData4', 'var')
    t_x  = ScopeData3.time;
    x_actual = ScopeData3.signals.values;
    y_actual = ScopeData4.signals.values;
    z_actual = ScopeData1.signals.values;
    % 生成期望圆形轨迹
    R = 2;  omega = 0.2;
    x_desired = R * cos(omega * t_x);
    y_desired = R * sin(omega * t_x);
    z_desired = 2;
end

% ===== 方法2：从 out 结构体读取 =====
% 如果模型启用了 Single simulation output，用 out.logsout 或 out.yout

% ===== 图1: XY 平面轨迹对比 =====
figure('Name', '圆形轨迹跟踪', 'Position', [100, 100, 900, 400]);

subplot(1,2,1);
plot(x_desired, y_desired, 'b--', 'LineWidth', 1.5); hold on;
plot(x_actual, y_actual, 'r-', 'LineWidth', 1.5);
xlabel('X (m)'); ylabel('Y (m)');
title('XY 平面圆形轨迹跟踪');
legend('期望轨迹', '实际轨迹', 'Location', 'best');
axis equal; grid on;

% ===== 图1右: 跟踪误差随时间变化 =====
subplot(1,2,2);
e_x = x_actual - x_desired;
e_y = y_actual - y_desired;
plot(t_x, e_x, 'k-', 'LineWidth', 1.2); hold on;
plot(t_x, e_y, 'g-', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('跟踪误差 (m)');
title('位置跟踪误差');
legend('e_x', 'e_y', 'Location', 'best');
grid on;

% ===== 图2: 各自由度单独对比 =====
figure('Name', '各自由度跟踪', 'Position', [100, 550, 900, 500]);

subplot(2,2,1);
plot(t_x, x_desired, 'b--', t_x, x_actual, 'r-', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('X (m)'); legend('期望', '实际'); grid on;

subplot(2,2,2);
plot(t_x, y_desired, 'b--', t_x, y_actual, 'r-', 'LineWidth', 1.2);
xlabel('时间 (s)'); ylabel('Y (m)'); legend('期望', '实际'); grid on;

% ===== 图3: 3D 轨迹 =====
if exist('ScopeData', 'var')
    z_actual = ScopeData.signals.values;
    figure('Name', '3D轨迹');
    plot3(x_actual, y_actual, z_actual, 'r-', 'LineWidth', 1.5); hold on;
    plot3(x_desired, y_desired, 2*ones(size(t_x)), 'b--', 'LineWidth', 1);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D Trajectory Tracking');
    legend('Actual Trajectory', 'Desired Trajectory', 'Location', 'best');
    grid on; axis equal;
end

% ===== 误差统计 =====
fprintf('===== 跟踪误差统计 =====\n');
fprintf('X 方向: 均值=%.4f m, 最大=%.4f m, RMS=%.4f m\n', ...
    mean(abs(e_x)), max(abs(e_x)), rms(e_x));
fprintf('Y 方向: 均值=%.4f m, 最大=%.4f m, RMS=%.4f m\n', ...
    mean(abs(e_y)), max(abs(e_y)), rms(e_y));