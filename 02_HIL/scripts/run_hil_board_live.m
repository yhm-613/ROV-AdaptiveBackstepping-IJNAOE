% run_hil_board_live.m -- 真板在环 20s 闭环【实拍版】( ode4 定步长 0.01, COM14 @460800 )
% 与 run_hil_board.m 同一算法与工况, 区别:
%   1) Simulink 模型窗口【可见】并置前, 仿真期间信号线动画流动 -> 供全景照拍摄
%   2) 打开根层全部 Scope 窗口
%   3) 结束后自动弹出 2 张论文级图 (eta 跟踪 / u 指令) 并保持在前台
% 注意: 运行前先软复位板子 (自适应律状态清零)
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
addpath(genpath(fullfile(ROOT, 'pc')));
warning('off', 'all');
warning('on', 'hil_bridge_mfun:fail');

% --- 基础工作区 (与 build_pc_models 同口径) ---
Sv = load(fullfile(ROOT, 'rovmodel75kg.mat'));
fn = fieldnames(Sv);
for ii = 1:numel(fn), assignin('base', fn{ii}, Sv.(fn{ii})); end
assignin('base', 'M_nom', Sv.M); assignin('base', 'Ma_nom', Sv.Ma);
assignin('base', 'DL_nom', Sv.DL);
assignin('base', 'CTRL_DISCRETE', 1);   % 真控制器在板上 (离散 10ms)
assignin('base', 'TS_CTRL', 0.01);

mdl = 'rov_hil_host';
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(mdl);
set_param(mdl, 'StopTime', '20');
set_param(mdl, 'SolverType', 'Fixed-step');
set_param(mdl, 'Solver', 'ode4');
set_param(mdl, 'FixedStep', '0.01');
set_param(mdl, 'SaveTime', 'on', 'TimeSaveName', 'TS_t');
try
    set_param(mdl, 'LimitDataPoints', 'off', 'MaxDataPoints', 'inf');
catch, end
fprintf('[RUN] %s %s 步长 %s  StopTime=%s  port=COM14 @460800\n', ...
    get_param(mdl, 'SolverType'), get_param(mdl, 'Solver'), ...
    get_param(mdl, 'FixedStep'), get_param(mdl, 'StopTime'));

% --- 打开模型窗口 (实拍: 仿真时信号线动画可见) ---
open_system(mdl);
try
    set_param(mdl, 'ZoomFactor', 'FitSystem');
catch, end

% --- 强制 HIL Bridge 离散化: 每步恰好一次调用 ---
try
    set_param([mdl '/HIL Bridge'], 'SampleTime', '0.01');
    fprintf('[RUN] HIL Bridge SampleTime = 0.01 (离散化, 1 call/step)\n');
catch
    fprintf('[RUN] !! HIL Bridge 无 SampleTime 参数, 需插 ZOH\n');
end

% --- 打开根层全部 Scope 窗口 (实拍: 曲线实时滚动) ---
blks = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Scope');
for ii = 1:numel(blks)
    try, open_system(blks{ii}); fprintf('[RUN] 打开 %s\n', blks{ii}); catch, end
end

% --- 挂临时探针 (不保存模型) ---
add_probe = @(src, nm) add_ts_probe(mdl, src, nm);
add_probe('Mux_hil', 'TS_mux24');        % [eta;nu;eta_d;nu_d] 24x1
add_probe('HIL Bridge', 'TS_bridge');    % [u(6);link] 7x1

fprintf('[RUN] === 仿真开始: 现在可以拍全景照 (板子+ST-Link+USB-TTL+本屏幕) ===\n');
t_wall = tic;
try
    out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    fprintf('[RUN] 仿真完成, 墙钟耗时 %.1f s\n', toc(t_wall));
catch err
    fprintf('[RUN] 仿真异常: %s\n', err.message);
    close_system(mdl, 0);
    rethrow(err);
end
close_system(mdl, 0);   % 丢弃, 不写回 .slx

% --- 统计 ---
T  = double(out.TS_t);
M  = double(out.TS_mux24);
BR = double(out.TS_bridge);
% TS_t 可能受模型 LimitDataPoints 截断 (只存最后 1000 点): 按步长重建完整时间轴
if numel(T) ~= size(M, 1)
    fprintf('[RUN] TS_t 被截断 (%d/%d), 按 0.01 s 步长重建时间轴\n', numel(T), size(M, 1));
    T = reshape((0:size(M, 1) - 1)' * 0.01, [], 1);
end
eta   = M(:, 1:6);   nu = M(:, 7:12);
eta_d = M(:, 13:18);
ucmd  = BR(:, 1:6);  link = BR(:, 7);

fprintf('[STAT] 步数=%d  t=[%.2f, %.2f] s\n', numel(T), T(1), T(end));
fprintf('[STAT] link: ok=%d  timeout=%d  badcrc=%d  serial_fail=%d\n', ...
    sum(link == 0), sum(link == 1), sum(link == 2), sum(link >= 3));
e = eta - eta_d;
fprintf('[STAT] 跟踪误差 RMS: %s\n', mat2str(round(sqrt(mean(e.^2, 1)), 5)));
fprintf('[STAT] u_cmd max|.|: %s (饱和限 100)\n', mat2str(round(max(abs(ucmd)), 3)));

save(fullfile(ROOT, 'out', 'hil_board_run.mat'), 'T', 'M', 'BR');
fprintf('[OUT] 已存 out/hil_board_run.mat\n');

% --- 弹出实拍收尾图: 跟踪曲线 + 推力指令 (保持前台) ---
fg1 = figure('Name', 'HIL 20s 圆轨迹闭环 - eta 跟踪 (真板 STM32F407)', ...
    'Color', 'w', 'Position', [80 380 780 520]);
lbl = {'x (m)', 'y (m)', 'z (m)', '\phi (rad)', '\theta (rad)', '\psi (rad)'};
for k = 1:6
    subplot(2, 3, k); hold on; grid on
    plot(T, eta_d(:, k), 'k--', 'LineWidth', 1.1);
    plot(T, eta(:, k), 'r-', 'LineWidth', 0.9);
    title(lbl{k}, 'FontSize', 9); xlabel('t (s)')
end
[lg, ic] = unique([eta_d(:,1), eta_d(:,2)], 'rows', 'stable');
subplot(2, 3, 1), legend({'参考', '板载闭环'}, 'FontSize', 7)

fg2 = figure('Name', 'HIL 20s 圆轨迹闭环 - 推力指令 u (真板 STM32F407)', ...
    'Color', 'w', 'Position', [880 380 780 520]);
lblu = {'u_1 (V)', 'u_2 (V)', 'u_3 (V)', 'u_4 (V)', 'u_5 (V)', 'u_6 (V)'};
for k = 1:6
    subplot(2, 3, k); hold on; grid on
    plot(T, ucmd(:, k), 'b-', 'LineWidth', 0.9);
    yl = ylim; plot(T, 100 * ones(size(T)), 'r--'); plot(T, -100 * ones(size(T)), 'r--');
    title(lblu{k}, 'FontSize', 9); xlabel('t (s)')
end

figure(fg1);
fprintf('RUN_DONE  (两张结果图已弹出并保持打开, 可继续拍摄)\n');

% ====================================================================
function add_ts_probe(mdl, src, nm)
tap = [mdl '/' nm];
try, delete_line(mdl, [src '/1'], [nm '/1']); catch, end
try, delete_block(tap); catch, end
add_block('simulink/Sinks/To Workspace', tap, 'VariableName', nm, ...
    'SaveFormat', 'Array', 'SampleTime', '-1');
dp = get_param(tap, 'DialogParameters');
if isfield(dp, 'LimitDataPoints')
    set_param(tap, 'LimitDataPoints', 'off');
elseif isfield(dp, 'MaxDataPoints')
    set_param(tap, 'MaxDataPoints', 'inf');
end
add_line(mdl, [src '/1'], [nm '/1'], 'autorouting', 'on');
end
