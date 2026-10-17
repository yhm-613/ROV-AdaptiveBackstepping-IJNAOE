%% ===== Z 深度 & Yaw 艏摇 跟踪误差时程曲线 =====
% 使用前确保仿真已运行，且以下变量在 Workspace 中：
%   t           - 时间向量
%   ScopeData   - z 实际位置 (Scope, Data Logging)
%   ScopeData5  - yaw 艏向 (Scope, Data Logging)
% 期望轨迹：z 阶跃 0→2m, psi 恒定 0

clearvars -except t ScopeData ScopeData5 ScopeData3 ScopeData4 ...
                   c1_hat_data c2_hat_data;
close all;

%% ===== 读取数据 =====
t_vec = t(:);
N     = length(t_vec);

% 期望轨迹
z_d   = zeros(N,1);
z_d(t_vec >= 0) = 2;      % 阶跃 0→2 m
psi_d = zeros(N,1);        % 艏向期望 0

% 实际轨迹
if exist('ScopeData','var')
    z_act = ScopeData.signals.values(:);
else
    error('ScopeData 不存在，请在 Z Scope 中启用 Data Logging');
end

if exist('ScopeData5','var')
    psi_act = ScopeData5.signals.values(:);
else
    error('ScopeData5 不存在，请在 Yaw Scope 中启用 Data Logging');
end

% 统一长度
L = min([N, length(z_act), length(psi_act)]);
t_plot = t_vec(1:L);
z_d    = z_d(1:L);      z_act    = z_act(1:L);
psi_d  = psi_d(1:L);    psi_act  = psi_act(1:L);

% 跟踪误差
e_z   = z_act - z_d;
e_psi = psi_act - psi_d;

%% ===== 绘图 =====
figure('Name','Z深度与Yaw艏摇跟踪误差','Position',[100,200,1000,420]);

% 左：Z 深度误差
subplot(1,2,1);
yyaxis left;
plot(t_plot, z_d, 'b--', 'LineWidth', 1.2); hold on;
plot(t_plot, z_act, 'b-', 'LineWidth', 1.5);
ylabel('Z 深度 (m)');  ylim([-0.5, 2.8]);

yyaxis right;
plot(t_plot, e_z, 'r-', 'LineWidth', 1.5);
ylabel('深度误差 (m)');

xlabel('时间 (s)');
title('Z 深度跟踪 (阶跃 0→2 m)');
legend('期望 z_d','实际 z','误差 e_z','Location','best');
grid on;

% 右：Yaw 艏摇误差
subplot(1,2,2);
e_psi_deg = e_psi * 180/pi;   % 转为度

yyaxis left;
plot(t_plot, psi_d*180/pi, 'b--', 'LineWidth', 1.2); hold on;
plot(t_plot, psi_act*180/pi, 'b-', 'LineWidth', 1.5);
ylabel('艏摇角 (deg)');

yyaxis right;
plot(t_plot, e_psi_deg, 'r-', 'LineWidth', 1.5);
ylabel('艏摇误差 (deg)');

xlabel('时间 (s)');
title('Yaw 艏摇跟踪 (期望 \psi_d=0)');
legend('期望 \psi_d','实际 \psi','误差 e_\psi','Location','best');
grid on;

%% ===== 误差统计 =====
idx_ss = t_plot > max(t_plot) * 0.5;   % 后50%为稳态

fprintf('\n========== Z深度 & Yaw艏摇 跟踪误差统计 ==========\n');
fprintf('指标                 |    数值\n');
fprintf('---------------------------------------|------------\n');
fprintf('Z 深度 稳态 RMS (m)       |  %.4f\n', rms(e_z(idx_ss)));
fprintf('Z 深度 最大误差 (m)        |  %.4f\n', max(abs(e_z)));
fprintf('Z 深度 稳态均值 (m)        |  %.4f\n', mean(e_z(idx_ss)));
fprintf('Yaw 艏摇 稳态 RMS (deg)   |  %.4f\n', rms(e_psi_deg(idx_ss)));
fprintf('Yaw 艏摇 最大误差 (deg)    |  %.4f\n', max(abs(e_psi_deg)));
fprintf('Yaw 艏摇 稳态均值 (deg)    |  %.4f\n', mean(e_psi_deg(idx_ss)));
fprintf('===================================================\n');
