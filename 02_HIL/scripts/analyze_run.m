% analyze_run.m -- HIL 真板 20s 闭环完整分析 (列布局已确认: 1-6 eta, 7-12 nu, 13-18 eta_d, 19-24 nu_d)
import_dir = fullfile('out','figures'); if ~exist(import_dir,'dir'), mkdir(import_dir); end; clear import_dir;
ROOT = fileparts(fileparts(mfilename('fullpath')));
S = load(fullfile(ROOT, 'out', 'hil_board_run.mat'));
M = S.M;  BR = S.BR;
n = size(M, 1);
t = (0:n-1) * 0.01;      % T 日志被模型级 1000 点截断, 定步长直接重构
fprintf('步数=%d  t=[%.2f, %.2f]\n', n, t(1), t(end));

eta   = M(:, 1:6);
nu    = M(:, 7:12);
eta_d = M(:, 13:18);
uc    = BR(:, 1:6);
link  = BR(:, 7);

fprintf('[LINK] ok=%d timeout=%d badcrc=%d serial_fail=%d\n', ...
    sum(link==0), sum(link==1), sum(link==2), sum(link>=3));
e = eta - eta_d;
fprintf('[ETA]  各轴 max|.|: %s\n', mat2str(round(max(abs(eta)),4)));
fprintf('[ERR]  RMS : %s\n', mat2str(round(sqrt(mean(e.^2,1)),4)));
fprintf('[ERR]  max : %s\n', mat2str(round(max(abs(e)),4)));
fprintf('[U]    max : %s\n', mat2str(round(max(abs(uc)),3)));
sat_any = sum(any(abs(uc) >= 99.99, 2));
fprintf('[U]    任一轴饱和步数=%d / %d (%.1f%%)\n', sat_any, n, 100*sat_any/n);

fprintf('\n[演化] 每 2s 窗口:  x RMS | y RMS | z RMS | yaw RMS(deg) | 饱和步数\n');
for w = 1:2:20
    idx = t >= w-2 & t < w;
    fprintf('  t=%4.1f-%4.1f : %7.3f %7.3f %7.3f %8.2f %4d\n', ...
        w-2, w, ...
        sqrt(mean(e(idx,1).^2)), sqrt(mean(e(idx,2).^2)), ...
        sqrt(mean(e(idx,3).^2)), sqrt(mean(abs(e(idx,6))).^2)*180/pi, ...
        sum(any(abs(uc(idx,:)) >= 99.99, 2)));
end

labels = {'x [m]','y [m]','z [m]','roll [rad]','pitch [rad]','yaw [deg]'};
fig = figure('Position', [50 50 1200 800], 'Visible', 'off');
for k = 1:6
    subplot(3, 2, k);
    plot(t, eta_d(:,k), 'b--', 'LineWidth', 1); hold on;
    plot(t, eta(:,k), 'r-', 'LineWidth', 1);
    ylabel(labels{k}); grid on;
    if k == 1, legend('\eta_d (ref)', '\eta (HIL真板)', 'Location', 'best'); end
end
xlabel('t [s]');
sgtitle('HIL 真板闭环: \eta 跟踪 (20s, 圆轨迹)');
saveas(fig, 'out\figures\HIL闭环_eta跟踪_20s.png');

fig2 = figure('Position', [50 50 1200 500], 'Visible', 'off');
plot(t, uc, 'LineWidth', 1); grid on;
xlabel('t [s]'); ylabel('u_{cmd} [V]');
legend('u1','u2','u3','u4','u5','u6', 'Location', 'best');
title('HIL 推进器指令 (板上控制器输出)');
saveas(fig2, 'out\figures\HIL闭环_u指令_20s.png');
disp('ANALYZE_DONE');
