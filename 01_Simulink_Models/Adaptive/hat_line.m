%% ========== ROV 自适应反步法收敛性分析绘图 ==========
% 前提：已仿真 100s，以下变量已存入 Workspace：
%   t      - 时间向量 (ToWorkspace, SaveFormat=Array)
%   c1_hat_data - c1_hat 积分器输出 (标量)
%   c2_hat_data - c2_hat 积分器输出 (6x1 向量)
%   ScopeData3  - x 实际位置 (Scope 日志)
%   ScopeData4  - y 实际位置 (Scope 日志)
%   ScopeData  - z 实际位置 (Scope 日志)
%   ScopeData5  - yaw 实际角度 (Scope 日志)

close all;

%% --- 圆形轨迹参数 ---
R = 2;  omega = 0.2;
t_vec = t(:);                 % 时间列向量
N = length(t_vec);

% 期望轨迹
x_d = R * cos(omega * t_vec);
y_d = R * sin(omega * t_vec);
z_d = 2 * ones(N, 1);

% 实际轨迹（从 Scope 取）
x_act = ScopeData3.signals.values(:);
y_act = ScopeData4.signals.values(:);
z_act = ScopeData.signals.values(:);

% 同步长度
L = min([length(x_d), length(x_act), length(y_act), length(z_act)]);
t_plot = t_vec(1:L);
x_d = x_d(1:L);  y_d = y_d(1:L);  z_d = z_d(1:L);
x_act = x_act(1:L);  y_act = y_act(1:L);  z_act = z_act(1:L);

% 跟踪误差
e_x = x_act - x_d;
e_y = y_act - y_d;
e_z = z_act - z_d;

%% --- 自适应参数 ---
c1 = c1_hat_data(:);
c2_all = c2_hat_data;         % [6 x N] 或 [1 x N]
if size(c2_all,1) > size(c2_all,2)
    c2_all = c2_all';         % 转为行
end

% 截断到相同长度
Lc = min(L, min(size(c1,1), size(c2_all,2)));
t_plot = t_plot(1:Lc);
c1 = c1(1:Lc);
c2_all = c2_all(:, 1:Lc);

%% ===== 图1: 自适应参数收敛性 (核心图) =====
figure('Name', '自适应参数收敛性', 'Position', [50, 450, 900, 420]);

% --- (a) c1_hat ---
subplot(1,2,1);
plot(t_plot, c1, 'b-', 'LineWidth', 1.5);
xlabel('time (s)');
ylabel('$\hat{c}_1$', 'Interpreter', 'latex');
grid on; hold on;
yline(-3, 'r--', 'LineWidth', 1);
legend('$\hat{c}_1$', '$\hat{c}_1(0)$ = $-3$', ...
    'Interpreter', 'latex', 'Location', 'best');

%% ===== (a) 标题：拆分为数学+中文 =====
ax_a = gca;
t1a = text(0.35, 1.02, '(a) $\dot{\hat{c}}_1$', ...
    'Interpreter', 'latex', 'FontSize', 11, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
    'Units', 'normalized');
drawnow;
text(0.43, 1.02, 'Scalar Adaptive Gain', ...
    'FontSize', 11, 'FontName', 'SimSun', ...
    'VerticalAlignment', 'bottom', 'Units', 'normalized');

% --- (b) c2_hat (6维) ---
subplot(1,2,2);
dof_labels = {'surge', 'sway', 'heave', 'roll', 'pitch', 'yaw'};
colors = lines(6);
for i = 1:6
    plot(t_plot, c2_all(i,:), 'Color', colors(i,:), 'LineWidth', 1.2); hold on;
end
yline(5, 'k--', 'LineWidth', 1);
xlabel('time (s)');
ylabel('$\hat{\mathbf{c}}_2$', 'Interpreter', 'latex');
legend([dof_labels, {'$\hat{c}_2(0)$ = $5$'}], 'Interpreter', 'latex', 'Location', 'best');
grid on;

%% ===== (b) 标题：拆分为数学+中文 =====
ax_b = gca;
t1b = text(0.4, 1.02, '(b) $\dot{\hat{\mathbf{c}}}_2$ (6 DOF)', ...
    'Interpreter', 'latex', 'FontSize', 11, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
    'Units', 'normalized');
drawnow;
text(0.58, 1.02, 'Adaptive Gain Vector', ...
    'FontSize', 11, 'FontName', 'SimSun', ...
    'VerticalAlignment', 'bottom', 'Units', 'normalized');
% 加文字标注收敛现象
%annotation('textbox', [0.15, 0.15, 0.25, 0.08], ...
 %   'String', {'c1_hat: 初始调整后趋于稳态', ...
  %            'c2_hat: 6个通道各自有界收敛'}, ...
   % 'Interpreter', 'latex', 'FontSize', 10, 'EdgeColor', 'none');

%% ===== 图2: 位置跟踪误差 =====
figure('Name', '跟踪误差收敛性', 'Position', [50, 50, 900, 500]);

% --- (a) 位置误差 ---
subplot(2,2,1);
plot(t_plot, e_x, 'k-', 'LineWidth', 1); hold on;
plot(t_plot, e_y, 'y-', 'LineWidth', 1);
plot(t_plot, e_z, 'g-', 'LineWidth', 1);
xlabel('time (s)');  ylabel('Position error (m)');
title('(a) Position tracking error ', 'Interpreter', 'latex');
legend('$e_x$', '$e_y$', '$e_z$', 'Interpreter', 'latex');
grid on;

% --- (b) XY 平面轨迹 ---
subplot(2,2,2);
plot(x_d, y_d, 'b--', 'LineWidth', 1.5); hold on;
plot(x_act, y_act, 'r-', 'LineWidth', 1.2);
plot(x_act(1), y_act(1), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
xlabel('X (m)');  ylabel('Y (m)');
title('(b) XY 平面圆形轨迹跟踪');
legend('期望轨迹 (R=2 m)', '实际轨迹', '起点', 'Location', 'best');
axis equal;  grid on;

% --- (c) XYZ 3D 轨迹 ---
subplot(2,2,3);
plot3(x_d, y_d, z_d, 'b--', 'LineWidth', 1.2); hold on;
plot3(x_act, y_act, z_act, 'r-', 'LineWidth', 1.5);
xlabel('X (m)');  ylabel('Y (m)');  zlabel('Z (m)');
title('(c) 3D 轨迹跟踪');  grid on;
legend('期望', '实际', 'Location', 'best');

% --- (d) 误差范数 ---
subplot(2,2,4);
e_norm_pos = sqrt(e_x.^2 + e_y.^2 + e_z.^2);
plot(t_plot, e_norm_pos, 'k-', 'LineWidth', 1.5);
xlabel('时间 (s)');  ylabel('$\|\tilde{\eta}\|$ (m)', 'Interpreter', 'latex');
title('(d) 位置误差 2-范数');
grid on;

%% ===== 图3: 稳态性能统计 =====
% 取 t > 30s 的稳态数据
idx_ss = t_plot > 30;
t_ss = t_plot(idx_ss);

fprintf('\n========== 稳态性能统计 (t > 30 s) ==========\n');
fprintf('指标                |  数值\n');
fprintf('--------------------|------------------\n');

% 位置误差 RMS
e_x_rms = rms(e_x(idx_ss));
e_y_rms = rms(e_y(idx_ss));
e_z_rms = rms(e_z(idx_ss));
fprintf('e_x RMS (m)         |  %.4f\n', e_x_rms);
fprintf('e_y RMS (m)         |  %.4f\n', e_y_rms);
fprintf('e_z RMS (m)         |  %.4f\n', e_z_rms);

% c1_hat 稳态值
c1_final = mean(c1(idx_ss));
fprintf('c1_hat Steady-State Mean     |  %.4f\n', c1_final);

% c2_hat 各通道稳态值
fprintf('\n--- c2_hat Per-Channel Steady-State Mean ---\n');
for i = 1:6
    c2_mean = mean(c2_all(i, idx_ss));
    fprintf('  DOF %d (%-5s)     |  %.4f\n', i, dof_labels{i}, c2_mean);
end

% 收敛速度：误差进入 5% 包络的时间
e_norm = sqrt(e_x.^2 + e_y.^2 + e_z.^2);
e_steady = mean(e_norm(idx_ss));
threshold = max(0.05 * R, e_steady * 1.5);  % 5% 半径 或 稳态×1.5
idx_conv = find(e_norm < threshold, 1, 'first');
if ~isempty(idx_conv)
    fprintf('\n收敛时间 (进入 %.2f m 包络):  %.2f s\n', threshold, t_plot(idx_conv));
end

%% ===== 图4a: c1_hat 自适应参数变化率 =====
figure('Name', 'c1-hat Rate of Change', 'Position', [150, 200, 700, 500]);
ctmp = lines(6);

dc1_dt = [0; diff(c1)] ./ [1; diff(t_plot)];
dc1_smooth = movmean(abs(dc1_dt), max(50, round(length(t_plot)*0.1)));

semilogy(t_plot, dc1_smooth, '-', 'Color', ctmp(1,:), 'LineWidth', 2);
xlabel('time (s)');
ylabel('$\dot{\hat{c}}_1$', 'Interpreter', 'latex');
ylim([1e-10, 1e-3]);
grid on;

%% ===== 标题：(a)\dot{\hat{c}}_1 变化率 =====
ax = gca;

% 数学符号 — 用 text()（有Extent属性）
t1 = text(...
    0.5, ...                                   % 水平居中（归一化）
    1.01, ...                                  % 轴框顶部上方
    '(a)$\dot{\hat{c}}_1$', ...
    'Interpreter', 'latex', ...
    'FontSize', 12, ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'bottom', ...
    'Units', 'normalized');

drawnow;                                       % 刷新获取Extent

% 中文 — 固定偏移量，不用Extent
t2 = text(...
    0.58, ...                                  % 固定在居中点右侧（根据效果微调此值）
    1.015, ...
    'Rate of Change', ...
    'FontSize', 12, ...
    'FontName', 'SimSun', ...
    'VerticalAlignment', 'bottom', ...
    'Units', 'normalized');

%% ===== 图4b: c2_hat 各自由度导数包络 =====
figure('Name', 'c2-hat Per-Degree-of-Freedom Rate of Change', 'Position', [200, 80, 1000, 700]);
ctmp = lines(6);
dof_labels_cn = {'surge', 'sway', 'heave', 'roll', 'pitch', 'yaw'};

for i = 1:6
    dc2_raw = [0, diff(c2_all(i,:))] ./ [1, diff(t_plot')];
    Nwin = max(20, round(length(t_plot)*0.03));
    dc2_smooth = envelope(abs(dc2_raw), Nwin, 'peak');
    
    subplot(2, 3, i);
    semilogy(t_plot, dc2_smooth, '-', 'Color', ctmp(i,:), 'LineWidth', 1.5);
    
    xlabel('time (s)');
    ylabel(sprintf('$|\\dot{\\hat{c}}_2^{(%d)}|$', i), 'Interpreter', 'latex');
    title(sprintf('(%s) %s', char('a'+i-1), dof_labels_cn{i}), 'FontSize', 11);
    grid on;
    ylim([1e-5, 1e2]);
end

sgtitle('Adaptive Parameter Rate of Change — Per-Degree-of-Freedom, Per-Channel Convergence Verification', ...
        'FontSize', 13, 'FontWeight', 'bold');
a

fprintf('\n========== 绘图完成 ==========\n');
fprintf('图1: 自适应参数 c1_hat, c2_hat 的收敛曲线\n');
fprintf('图2: 跟踪误差 + XY/3D 轨迹\n');
fprintf('图3: (控制台输出) 稳态性能统计\n');
fprintf('图4: 自适应参数导数 → 0 (收敛性佐证)\n');
