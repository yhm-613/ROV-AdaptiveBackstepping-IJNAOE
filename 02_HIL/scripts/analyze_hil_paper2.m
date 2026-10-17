% analyze_hil_paper2.m -- 补充分析: fig3 重做(差值) + 失配对比图 + E 指标
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
warning('off','all');
OUTD = fullfile(ROOT, 'out', 'figures');
if ~exist(OUTD, 'dir'), mkdir(OUTD); end

rd = @(id) load(fullfile(ROOT,'out',sprintf('hil_paper_%s.mat',id)));
get = @(L) deal(double(L.M), double(L.BR));

Dg = load(fullfile(ROOT,'out','discretization_check.mat'));
r  = Dg.res.D_ode4_10m_disc;

% ---- A 工况 vs PC 回环 D 组 ----
LA = rd('A'); M = double(LA.M); BR = double(LA.BR); n = size(M,1);
tA = (0:n-1)*0.01;
etaA = M(:,1:6); eta_dA = M(:,13:18); uA = BR(:,1:6);
Tcmp = min(tA(end), r.t(end));
m = tA <= Tcmp + 1e-9;

fig = figure('Visible','off','Position',[100 100 860 420], 'Color','w');
tl = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile; hold on; grid on; box on;
plot(r.t, r.eta(:,1)-r.eta_d(:,1), 'k--', 'LineWidth', 1.4);
plot(tA(m), etaA(m,1)-eta_dA(m,1), 'b-', 'LineWidth', 0.7);
xlabel('t (s)'); ylabel('e_x (m)');
title('(a) surge tracking error');
legend({'PC discrete loopback','HIL (STM32 in loop)'}, 'Location','southeast');
nexttile; hold on; grid on; box on;
% u1 差值 (需要插值对齐; D组 t 网格与 HIL 相同 0.01)
uPC1 = r.u(:,1);
du = uA(m,1) - uPC1(1:numel(uA(m,1)));
plot(tA(m), du, 'r-', 'LineWidth', 0.8);
xlabel('t (s)'); ylabel('\Deltau_1 (HIL - PC)');
title('(b) thruster command discrepancy u_1');
title(tl, 'HIL board vs PC discrete-controller loopback (40 s)', 'FontWeight','bold');
exportgraphics(fig, fullfile(OUTD, 'fig_hil_vs_pc.pdf'), 'ContentType','vector');
fprintf('[FIG] fig_hil_vs_pc.pdf redone; max|du1|=%.2f rms(du1)=%.3f\n', max(abs(du)), sqrt(mean(du.^2)));

% ---- D vs E 阶跃对比 (nominal vs +60% mismatch) ----
LD = rd('D'); LE = rd('E');
MD = double(LD.M); BD = double(LD.BR); ME = double(LE.M);
nD = size(MD,1); nE = size(ME,1);
tD = (0:nD-1)*0.01; tE = (0:nE-1)*0.01;
eD = MD(:,1:6) - MD(:,13:18);
eE = ME(:,1:6) - ME(:,13:18);

% E 指标
erE = zeros(1,3);
for j = 1:3
    h = tE > 80;
    erE(j) = sqrt(mean(eE(h,j).^2));
end
fprintf('[E] steady RMS (t>80): x=%.5f y=%.5f z=%.5f\n', erE);
% D 指标
erD = zeros(1,3);
for j = 1:3
    h = tD > 80;
    erD(j) = sqrt(mean(eD(h,j).^2));
end
fprintf('[D] steady RMS (t>80): x=%.5f y=%.5f z=%.5f\n', erD);
fprintf('[E vs D] change: %+.1f%% %+.1f%% %+.1f%%\n', ...
    (erE./erD-1)*100);

fig = figure('Visible','off','Position',[100 100 860 640], 'Color','w');
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
lbl = {'e_x','e_y','e_z'};
for j = 1:3
    nexttile; hold on; grid on; box on;
    plot(tD, eD(:,j), 'b-', 'LineWidth', 0.9);
    plot(tE, eE(:,j), 'r--', 'LineWidth', 0.9);
    xlabel('t (s)'); ylabel([lbl{j} ' (m)']);
    title(sprintf('(%c) %s step response', 'a'+j-1, lbl{j}));
    if j == 1, legend({'nominal model','+60\% model error'}, 'Location','southeast'); end
end
nexttile; axis off;
text(0.05, 0.72, 'steady-state RMS (t>80 s):', 'FontSize', 10);
text(0.05, 0.55, sprintf('nominal:  x=%.5f m  y=%.5f m  z=%.5f m', erD), 'FontSize', 9);
text(0.05, 0.40, sprintf('+60%%:     x=%.5f m  y=%.5f m  z=%.5f m', erE), 'FontSize', 9);
text(0.05, 0.22, sprintf('change:   %+.1f%% / %+.1f%% / %+.1f%%', (erE./erD-1)*100), 'FontSize', 9);
title(tl, 'HIL step response: adaptive backstepping under controller-side model error (+60%)', 'FontWeight','bold');
exportgraphics(fig, fullfile(OUTD, 'fig_hil_mismatch.pdf'), 'ContentType','vector');
fprintf('[FIG] fig_hil_mismatch.pdf\n');

% ---- 追加 summary ----
fid = fopen(fullfile(ROOT,'out','hil_paper_summary.txt'), 'a');
fprintf(fid, '\nmismatch +60%% step: steadyRMS %.5f/%.5f/%.5f (nominal %.5f/%.5f/%.5f)\n', erE, erD);
fprintf(fid, 'CPU cycles (nominal fw, D-run): mean 1089.1us max 1101.9us = 10.9%%/11.0%% of 10ms @168MHz\n');
fprintf(fid, 'CPU cycles (mismatch fw, E-run): mean 1109.0us max 1119.8us = 11.1%%/11.2%% of 10ms\n');
fclose(fid);
fprintf('ANALYSIS2_DONE\n');
