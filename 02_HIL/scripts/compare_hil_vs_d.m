% compare_hil_vs_d.m -- HIL 真板 vs PC 回环 D 组 逐自由度对比
ROOT = fileparts(fileparts(mfilename('fullpath')));
import_dir = fullfile('out','figures'); if ~exist(import_dir,'dir'), mkdir(import_dir); end; clear import_dir;
D = load(fullfile(ROOT, 'out', 'discretization_check.mat'));
dD = D.res.D_ode4_10m_disc;          % 4001 点, 0-40s
S  = load(fullfile(ROOT, 'out', 'hil_board_run.mat'));
M  = S.M;  BR = S.BR;
n  = size(M, 1);
tH = (0:n-1) * 0.01;

% HIL 布局: [eta(1:6); nu(7:12); eta_d(13:18); nu_d(19:24)] (圆轨迹已验证)
etaH  = M(:, 1:6);    nuH = M(:, 7:12);
eta_dH = M(:, 13:18);
uH    = BR(:, 1:6);   linkH = BR(:, 7);

tD = dD.t;  etaD = dD.eta;  uD = dD.u;  eta_dD = dD.eta_d;
% 对齐时间: 取 0-20s
idx20 = tD <= 20 + 1e-9;
fprintf('D组: %d 点(0-40s), 取 0-20s 共 %d 点;  HIL: %d 点\n', ...
    numel(tD), sum(idx20), n);

% ---- 逐 2s 窗口 RMS 对比表 ----
labels = {'x','y','z','roll','pitch','yaw'};
fprintf('\n=== 跟踪误差 RMS (|eta-eta_d|)  D组 vs HIL ===\n');
fprintf('窗口        |   x: D/HIL   |   y: D/HIL   |   z: D/HIL   | yaw(deg): D/HIL\n');
for w = 1:2:20
    iH = tH >= w-2 & tH < w;
    iD = tD >= w-2 & tD < w;
    eH = etaH(iH,:) - eta_dH(iH,:);
    eD = etaD(iD,:) - eta_dD(iD,:);
    fprintf('t=%2d-%2d s | %5.2f / %5.2f | %5.2f / %5.2f | %5.2f / %5.2f | %6.1f / %6.1f\n', ...
        w-2, w, sqrt(mean(eD(:,1).^2)), sqrt(mean(eH(:,1).^2)), ...
        sqrt(mean(eD(:,2).^2)), sqrt(mean(eH(:,2).^2)), ...
        sqrt(mean(eD(:,3).^2)), sqrt(mean(eH(:,3).^2)), ...
        sqrt(mean(eD(:,6).^2))*180/pi, sqrt(mean(eH(:,6).^2))*180/pi);
end

% ---- u 对比: 前 5s 两侧状态应接近, 若 u 从一开始就差 -> 板上控制器输出不同 ----
fprintf('\n=== u 对比 (前 10 步, 逐拍) ===\n');
fprintf(' t     | u1: D / HIL        | u2: D / HIL        | u4: D / HIL\n');
for k = 1:10
    fprintf('%.2f | %8.2f / %8.2f | %8.2f / %8.2f | %8.2f / %8.2f\n', ...
        tH(k), uD(k,1), uH(k,1), uD(k,2), uH(k,2), uD(k,4), uH(k,4));
end
du = max(abs(uD(1:n, :) - uH), [], 2);
fprintf('\nu 逐拍最大偏差: 首超 5V @ t=%.2f, 首超 20V @ t=%.2f, 首超 50V @ t=%.2f\n', ...
    tH(find(du > 5, 1)), tH(find(du > 20, 1)), tH(find(du > 50, 1)));

% ---- eta 分歧起点 ----
de = max(abs(etaD(1:n, :) - etaH), [], 2);
fprintf('eta 逐拍最大偏差: 首超 0.1 @ t=%.2f, 首超 0.5 @ t=%.2f, 首超 2 @ t=%.2f\n', ...
    tH(find(de > 0.1, 1)), tH(find(de > 0.5, 1)), tH(find(de > 2, 1)));

% ---- 图 ----
labels6 = {'x [m]','y [m]','z [m]','roll [rad]','pitch [rad]','yaw [deg]'};
fig = figure('Position', [40 40 1250 850], 'Visible', 'off');
for k = 1:6
    subplot(3, 2, k);
    plot(tD(idx20), eta_dD(idx20,k), 'k--', 'LineWidth', 1); hold on;
    plot(tD(idx20), etaD(idx20,k), 'b-', 'LineWidth', 1);
    plot(tH, etaH(:,k), 'r-', 'LineWidth', 1);
    ylabel(labels6{k}); grid on; xlim([0 20]);
    if k == 1
        legend('\eta_d', 'D组回环(收敛)', 'HIL真板', 'Location', 'best');
    end
end
xlabel('t [s]');
sgtitle('HIL(真板) vs PC回环D组: eta 跟踪对比');
saveas(fig, 'out\figures\HIL_vs_回环D组_eta对比.png');

fig2 = figure('Position', [40 40 1250 600], 'Visible', 'off');
cols = {'u1','u2','u3','u4','u5','u6'};
for k = 1:6
    subplot(3, 2, k);
    plot(tD(idx20), uD(idx20,k), 'b-', 'LineWidth', 0.8); hold on;
    plot(tH, uH(:,k), 'r-', 'LineWidth', 0.8);
    ylabel(cols{k}); grid on; xlim([0 20]); ylim([-110 110]);
    if k == 1, legend('D组回环', 'HIL真板', 'Location', 'best'); end
end
xlabel('t [s]');
sgtitle('推进器指令对比: D组回环(蓝) vs HIL真板(红)');
saveas(fig2, 'out\figures\HIL_vs_回环D组_u对比.png');
disp('CMP_HIL_DONE');
