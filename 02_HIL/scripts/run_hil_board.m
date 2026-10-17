% run_hil_board.m -- 真板在环 20s 闭环 ( ode4 定步长 0.01, COM11 @460800 )
% 探针临时挂载, 不写回 .slx 文件
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
addpath(genpath(fullfile(ROOT, 'pc')));
warning('off', 'all');
warning('on', 'hil_bridge_mfun:fail');   % 放行桥函数的失败原因

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
fprintf('[RUN] %s %s 步长 %s  StopTime=%s  port=COM14 @460800\n', ...
    get_param(mdl, 'SolverType'), get_param(mdl, 'Solver'), ...
    get_param(mdl, 'FixedStep'), get_param(mdl, 'StopTime'));

% --- 强制 HIL Bridge 离散化: 每步恰好一次调用 (RK4 子阶段不再重复调桥) ---
% R2024b 把 Interpreted MATLAB Fcn 当连续直通块, ode4 每步评估 4 次,
% 相当于以 400Hz 给板子发 setpoint, 板上自适应律被 4 倍速率激励导致发散
try
    set_param([mdl '/HIL Bridge'], 'SampleTime', '0.01');
    fprintf('[RUN] HIL Bridge SampleTime = 0.01 (离散化, 1 call/step)\n');
catch
    fprintf('[RUN] !! HIL Bridge 无 SampleTime 参数, 需插 ZOH\n');
end

% --- 挂临时探针 (不保存模型) ---
add_probe = @(src, nm) add_ts_probe(mdl, src, nm);
add_probe('Mux_hil', 'TS_mux24');        % [eta;nu;eta_d;nu_d] 24x1
add_probe('HIL Bridge', 'TS_bridge');    % [u(6);link] 7x1

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
eta   = M(:, 1:6);   nu = M(:, 7:12);
eta_d = M(:, 13:18);
ucmd  = BR(:, 1:6);  link = BR(:, 7);

fprintf('[STAT] 步数=%d  t=[%.2f, %.2f] s\n', numel(T), T(1), T(end));
fprintf('[STAT] link: ok=%d  timeout=%d  badcrc=%d  serial_fail=%d\n', ...
    sum(link == 0), sum(link == 1), sum(link == 2), sum(link >= 3));
fprintf('[STAT] eta 各轴 max|.|: %s\n', mat2str(round(max(abs(eta)), 4)));
fprintf('[STAT] eta_d 各轴 max|.|: %s\n', mat2str(round(max(abs(eta_d)), 4)));
e = eta - eta_d;
fprintf('[STAT] 跟踪误差 RMS: %s\n', mat2str(round(sqrt(mean(e.^2, 1)), 5)));
fprintf('[STAT] 跟踪误差 max|.|: %s\n', mat2str(round(max(abs(e)), 4)));
fprintf('[STAT] u_cmd max|.|: %s (饱和限 100)\n', mat2str(round(max(abs(ucmd)), 3)));

save(fullfile(ROOT, 'out', 'hil_board_run.mat'), 'T', 'M', 'BR');
fprintf('[OUT] 已存 out/hil_board_run.mat\n');
fprintf('RUN_DONE\n');

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
