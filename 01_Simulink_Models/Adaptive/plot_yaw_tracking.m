%% ===== Yaw 艏摇跟踪时程曲线 =====
% 使用前确保仿真已运行，ScopeData5 已开启 Data Logging
% 期望值：无（仅观察实际艏摇角响应）

clearvars -except t ScopeData5;
close all;

%% ===== 读取数据 =====
t_vec = t(:);

if exist('ScopeData5','var')
    psi_act = ScopeData5.signals.values(:);
else
    error('ScopeData5 不存在，请在 Yaw Scope 中启用 Data Logging');
end

% 统一长度
L = min([length(t_vec), length(psi_act)]);
t_plot = t_vec(1:L);
psi_act = psi_act(1:L);

psi_deg = psi_act * 180/pi;   % 转为度

%% ===== 绘图 =====
figure('Name','Yaw艏摇跟踪','Position',[200,300,800,420]);

plot(t_plot, psi_deg, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('艏摇角 (deg)');
title('Yaw 艏摇角时程曲线');
grid on;

%% ===== 统计 =====
idx_ss = t_plot > max(t_plot) * 0.5;   % 后50%为稳态

fprintf('\n========== Yaw 艏摇统计 ==========\n');
fprintf('稳态均值 (deg)  |  %.4f\n', mean(psi_deg(idx_ss)));
fprintf('稳态 RMS (deg)  |  %.4f\n', rms(psi_deg(idx_ss)));
fprintf('最大值 (deg)    |  %.4f\n', max(psi_deg));
fprintf('最小值 (deg)    |  %.4f\n', min(psi_deg));
fprintf('峰峰值 (deg)    |  %.4f\n', max(psi_deg)-min(psi_deg));
fprintf('==================================\n');
