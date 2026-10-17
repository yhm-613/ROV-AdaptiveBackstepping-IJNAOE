% analyze_hil_paper.m -- 论文 HIL 工况完整分析 (A/B/C/D + D组回环对照)
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
warning('off','all');
FIGD = fullfile(ROOT, 'out', 'figures');
if ~exist(FIGD, 'dir'), mkdir(FIGD); end

% ------- D组回环 (PC 离散控制器参考) -------
Dg = load(fullfile(ROOT,'out','discretization_check.mat'));
fprintf('=== D-group fields ===\n');
disp(fieldnames(Dg.res));
r = Dg.res.D_ode4_10m_disc;
disp(fieldnames(r));
fprintf('D horizon: t=[%.2f %.2f] n=%d\n', r.t(1), r.t(end), numel(r.t));

% ------- HIL 工况 -------
S = struct();
ids = {'A','B','C','D'};
names = {'circle_nominal','circle_const_current','circle_vary_current','step_nominal'};
for k = 1:4
    L = load(fullfile(ROOT,'out',sprintf('hil_paper_%s.mat',ids{k})));
    M = double(L.M); BR = double(L.BR); n = size(M,1);
    t = (0:n-1) * 0.01;                  % 定步长 0.01, 时间轴重构
    S.(ids{k}) = struct('t',t, ...
        'eta',M(:,1:6), 'nu',M(:,7:12), 'eta_d',M(:,13:18), ...
        'ucmd',BR(:,1:6), 'link',BR(:,7));
end

% ================= 圆轨迹指标 (与论文 Table 4 / Table 7 同口径) ==========
fprintf('\n=== CIRCLE scenarios: full-run RMS (paper metric) ===\n');
fprintf('%-24s %10s %10s %10s\n', 'case', 'RMS_xy', 'RMS_z', 'Total');
ref = {
  'PC ode45 (paper Tab.4 adaptive)', 0.3208, 0.3532, 0.4771
  'HIL A: nominal',   '', '', ''
  'HIL B: const 0.3', '', '', ''
  'HIL C: vary',      '', '', ''
};
for k = 1:3
    id = ids{k}; d = S.(id);
    e = d.eta - d.eta_d;
    rxy = sqrt(mean(e(:,1).^2 + e(:,2).^2));
    rz  = sqrt(mean(e(:,3).^2));
    rt  = sqrt(mean(e(:,1).^2 + e(:,2).^2 + e(:,3).^2));
    fprintf('%-24s %10.4f %10.4f %10.4f\n', names{k}, rxy, rz, rt);
end
fprintf('%-24s %10.4f %10.4f %10.4f  (paper, PC ode45)\n', ref{1}, ref{2}, ref{3}, ref{4});

% growth vs A
eA = S.A.eta - S.A.eta_d;
rA = [sqrt(mean(eA(:,1).^2+eA(:,2).^2)), sqrt(mean(eA(:,3).^2)), ...
      sqrt(mean(eA(:,1).^2+eA(:,2).^2+eA(:,3).^2))];
fprintf('\ngrowth vs A: ');
for k = 2:3
    d = S.(ids{k}); e = d.eta - d.eta_d;
    rr = [sqrt(mean(e(:,1).^2+e(:,2).^2)), sqrt(mean(e(:,3).^2)), ...
          sqrt(mean(e(:,1).^2+e(:,2).^2+e(:,3).^2))];
    fprintf('%s: %+0.1f%%/%+0.1f%%/%+0.1f%%  ', names{k}, ...
        (rr(1)/rA(1)-1)*100, (rr(2)/rA(2)-1)*100, (rr(3)/rA(3)-1)*100);
end
fprintf('\n');

% link health
for k = 1:3
    d = S.(ids{k});
    fprintf('%s link: ok=%d to=%d badcrc=%d fail=%d\n', ids{k}, ...
        sum(d.link==0), sum(d.link==1), sum(d.link==2), sum(d.link>=3));
end

% ================= 阶跃指标 (与论文 Table 3 同口径) =================
fprintf('\n=== STEP scenario D: transient metrics ===\n');
d = S.D; e = d.eta - d.eta_d;
fprintf('%-28s %10s %10s %10s  (paper adaptive: 7.37/8.55/10.91)\n', 'rise 10-90% (s)', 'x', 'y', 'z');
for ch = 1:3
    yv = d.eta(:,ch); yf = yv(end);
    % 上升至稳态 90% (阶跃 2m, 稳态取 t=100 附近均值)
    yss = mean(yv(d.t > 90));
    i10 = find(yv >= 0.1*yss, 1); i90 = find(yv >= 0.9*yss, 1);
    if isempty(i10) || isempty(i90), tr = NaN; else, tr = d.t(i90) - d.t(i10); end
    fprintf('%-28s %10.2f', '', tr); fprintf('  [ch%d]', ch);
    % 打印在一行: 重写
end
fprintf('\n');
for ch = {'x','y','z'}
end
chans = [1 2 3]; lbl = {'x','y','z'};
tr = zeros(1,3); ts = zeros(1,3); mp = zeros(1,3); er = zeros(1,3);
for j = 1:3
    ch = chans(j);
    yv = d.eta(:,ch); yss = mean(yv(d.t > 90));
    i10 = find(yv >= 0.1*yss, 1); i90 = find(yv >= 0.9*yss, 1);
    if isempty(i10) || isempty(i90), tr(j) = NaN; else, tr(j) = d.t(i90) - d.t(i10); end
    band = 0.02 * abs(yss);
    outside = find(abs(yv - yss) > band, 1, 'last');
    if isempty(outside), ts(j) = 0; else, ts(j) = d.t(outside); end
    mp(j) = max(0, (max(yv) - yss) / abs(yss) * 100);
    h = d.t > 80;
    er(j) = sqrt(mean(e(h,ch).^2));
end
fprintf('rise (s):      x=%.2f y=%.2f z=%.2f   (paper adaptive 7.37/8.55/10.91)\n', tr);
fprintf('settle (s):    x=%.2f y=%.2f z=%.2f   (paper adaptive 15.48/23.34/28.98)\n', ts);
fprintf('overshoot %%:   x=%.2f y=%.2f z=%.2f   (paper adaptive 0.4/0.1/0.0)\n', mp);
fprintf('steady RMS:    x=%.4f y=%.4f z=%.4f (paper adaptive 0.0147/0.0178/0.0202)\n', er);
fprintf('u max: %s\n', mat2str(round(max(abs(d.ucmd)),2)));

% ================= 图 1: HIL 圆轨迹跟踪总览 (A) =================
d = S.A; e = d.eta - d.eta_d;
fig = figure('Visible','off','Position',[100 100 860 640], 'Color','w');
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile; hold on; grid on; box on;
th = linspace(0, 2*pi, 200);
plot(2*cos(th), 2*sin(th), 'k--', 'LineWidth', 0.8);
plot(d.eta_d(:,1), d.eta_d(:,2), 'Color', [0.4 0.4 0.4], 'LineWidth', 0.7);
plot(d.eta(:,1), d.eta(:,2), 'b-', 'LineWidth', 1.1);
xlabel('x (m)'); ylabel('y (m)');
title('(a) HIL horizontal path');
legend({'reference circle','desired','HIL adaptive'}, 'Location','northeast');

nexttile; hold on; grid on; box on;
plot(d.t, d.eta_d(:,3), 'k--', d.t, d.eta(:,3), 'b-', 'LineWidth', 1);
xlabel('t (s)'); ylabel('z (m)');
title('(b) depth');

nexttile; hold on; grid on; box on;
plot(d.t, e(:,1), 'r', d.t, e(:,2), 'g', d.t, e(:,3), 'b', 'LineWidth', 0.9);
xlabel('t (s)'); ylabel('error (m)');
title('(c) tracking error');
legend({'e_x','e_y','e_z'}, 'Location','northeast');

nexttile; hold on; grid on; box on;
% D-group overlay (if horizon covers)
plot(d.t, d.ucmd(:,1), 'r', d.t, d.ucmd(:,6), 'm', 'LineWidth', 0.9);
xlabel('t (s)'); ylabel('thruster command');
title('(d) thruster commands u_1, u_6');
legend({'u_1','u_6'}, 'Location','northeast');
title(tl, 'HIL adaptive backstepping: circular trajectory (STM32F407 in the loop, 100 s)', 'FontWeight','bold');
exportgraphics(fig, fullfile(FIGD, 'fig_hil_tracking.pdf'), 'ContentType','vector');
fprintf('[FIG] fig_hil_tracking.pdf\n');

% ================= 图 2: 海流影响对比 (A vs B vs C) =================
fig = figure('Visible','off','Position',[100 100 860 420], 'Color','w');
tl = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
cols = [0 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19];
nexttile; hold on; grid on; box on;
for k = 1:3
    dd = S.(ids{k}); ee = dd.eta - dd.eta_d;
    plot(dd.t, sqrt(ee(:,1).^2 + ee(:,2).^2), 'LineWidth', 0.9, 'Color', cols(k,:));
end
xlabel('t (s)'); ylabel('||e_{xy}|| (m)');
title('(a) horizontal error norm');
legend({'still water','0.3 m/s constant','0.2-0.3 m/s varying'}, 'Location','northeast');
nexttile; hold on; grid on; box on;
for k = 1:3
    dd = S.(ids{k}); ee = dd.eta - dd.eta_d;
    plot(dd.t, ee(:,3), 'LineWidth', 0.9, 'Color', cols(k,:));
end
xlabel('t (s)'); ylabel('e_z (m)');
title('(b) depth error');
title(tl, 'HIL circular tracking under ocean-current loading', 'FontWeight','bold');
exportgraphics(fig, fullfile(FIGD, 'fig_hil_current.pdf'), 'ContentType','vector');
fprintf('[FIG] fig_hil_current.pdf\n');

% ================= 图 3: HIL vs PC 回环 D 组 =================
if numel(r.t) >= 2000
    fig = figure('Visible','off','Position',[100 100 860 420], 'Color','w');
    tl = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    Tcmp = min(S.A.t(end), r.t(end));
    m = S.A.t <= Tcmp;
    nexttile; hold on; grid on; box on;
    plot(r.t, r.eta(:,1)-r.eta_d(:,1), 'k--', 'LineWidth', 0.9);
    plot(S.A.t(m), (S.A.eta(m,1)-S.A.eta_d(m,1)), 'b-', 'LineWidth', 0.9);
    xlabel('t (s)'); ylabel('e_x (m)');
    title('(a) surge error: PC loopback vs HIL');
    legend({'PC discrete loopback','HIL board'}, 'Location','northeast');
    nexttile; hold on; grid on; box on;
    plot(r.t, r.u(:,1), 'k--', 'LineWidth', 0.9);
    plot(S.A.t(m), S.A.ucmd(m,1), 'b-', 'LineWidth', 0.9);
    xlabel('t (s)'); ylabel('u_1');
    title('(b) thruster command u_1');
    title(tl, 'HIL vs PC discrete-controller loopback', 'FontWeight','bold');
    exportgraphics(fig, fullfile(FIGD, 'fig_hil_vs_pc.pdf'), 'ContentType','vector');
    fprintf('[FIG] fig_hil_vs_pc.pdf (horizon %.0f s)\n', Tcmp);
end

% ================= 摘要文本 =================
fid = fopen(fullfile(ROOT,'out','hil_paper_summary.txt'), 'w');
fprintf(fid, 'HIL PAPER RESULTS SUMMARY\n');
fprintf(fid, 'circle full-run RMS (xy, z, total):\n');
for k = 1:3
    d0 = S.(ids{k}); ee = d0.eta - d0.eta_d;
    fprintf(fid, '  %s: %.4f / %.4f / %.4f\n', names{k}, ...
        sqrt(mean(ee(:,1).^2+ee(:,2).^2)), sqrt(mean(ee(:,3).^2)), ...
        sqrt(mean(ee(:,1).^2+ee(:,2).^2+ee(:,3).^2)));
end
fprintf(fid, 'step: rise %.2f/%.2f/%.2f  settle %.2f/%.2f/%.2f  overshoot %.2f/%.2f/%.2f  steadyRMS %.4f/%.4f/%.4f\n', ...
    tr, ts, mp, er);
fclose(fid);
fprintf('[OUT] summary written\n');
fprintf('ANALYSIS_DONE\n');
