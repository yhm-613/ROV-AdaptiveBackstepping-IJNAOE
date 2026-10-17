% run_hil_step.m -- 工况 D: 阶跃 2m 100s (真板 HIL)
% 修正: 注释块用路径直接取句柄; 关模型级 LimitDataPoints
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
addpath(genpath(fullfile(ROOT, 'pc')));
warning('off', 'all');
warning('on', 'hil_bridge_mfun:fail');

% 板卡软复位 (自适应律状态清零): 运行前按一次板上 RESET 键, 或调用本机复位脚本

Sv = load(fullfile(ROOT, 'rovmodel75kg.mat'));
fn = fieldnames(Sv);
for ii = 1:numel(fn), assignin('base', fn{ii}, Sv.(fn{ii})); end
assignin('base', 'M_nom', Sv.M); assignin('base', 'Ma_nom', Sv.Ma);
assignin('base', 'DL_nom', Sv.DL);
assignin('base', 'CTRL_DISCRETE', 1);
assignin('base', 'TS_CTRL', 0.01);
assignin('base', 'HIL_CURR_PROF', 0);
assignin('base', 'HIL_CURR_BETA', pi/4);

mdl = 'rov_hil_host';
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(mdl);

% --- 模型级数据日志不截断 ---
try
    set_param(mdl, 'LimitDataPoints', 'off');
    fprintf('[CFG] LimitDataPoints off\n');
catch e
    fprintf('[CFG] %s\n', e.message);
end

wire_current(mdl);
rewire_step(mdl);
fprintf('[SETUP] 阶跃参考已接好 (x,y,z=2m, nu_d=0), 海流=0\n');

% --- 软复位板子 ---
[st, out] = system([RESET ' 2>&1']);
fprintf('[RESET] status=%d\n%s\n', st, strtrim(out));
clear('hil_bridge_host', 'hil_bridge_mfun');
pause(1);

set_param(mdl, 'StopTime', '100');
set_param(mdl, 'SolverType', 'Fixed-step');
set_param(mdl, 'Solver', 'ode4');
set_param(mdl, 'FixedStep', '0.01');
set_param(mdl, 'SaveTime', 'on', 'TimeSaveName', 'TS_t');
try, set_param([mdl '/HIL Bridge'], 'SampleTime', '0.01'); catch, end
add_probe(mdl, 'Mux_hil', 'TS_mux24');
add_probe(mdl, 'HIL Bridge', 'TS_bridge');

t_wall = tic;
out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
fprintf('[RUN D] 完成, 墙钟 %.1f s\n', toc(t_wall));
close_system(mdl, 0);

T  = double(out.TS_t);
M  = double(out.TS_mux24);
BR = double(out.TS_bridge);
eta = M(:, 1:6);  eta_d = M(:, 13:18);
ucmd = BR(:, 1:6);  link = BR(:, 7);
e = eta - eta_d;

fprintf('[STAT D] T=%d M=%d BR=%d t=[%.2f %.2f] link ok=%d to=%d badcrc=%d fail=%d\n', ...
    numel(T), size(M,1), size(BR,1), T(1), T(end), sum(link==0), sum(link==1), sum(link==2), sum(link>=3));
fprintf('[STAT D] RMS 全程 : %s\n', mat2str(round(sqrt(mean(e.^2,1)), 5)));
h = T > 80;
fprintf('[STAT D] RMS 末段 : %s (t>80s)\n', mat2str(round(sqrt(mean(e(h,:).^2,1)), 5)));
fprintf('[STAT D] u max|.|: %s\n', mat2str(round(max(abs(ucmd)), 2)));

save(fullfile(ROOT, 'out', 'hil_paper_D.mat'), 'T', 'M', 'BR');
fprintf('STEP_DONE\n');

% =====================================================================
function wire_current(mdl)
hSum  = blk_by_sid(mdl, '1265');
hEuler= blk_by_sid(mdl, '1229');
set_param(hSum, 'Inputs', '+---+');
ck = [mdl '/curr_clock'];
try, delete_block(ck); catch, end
add_block('simulink/Sources/Clock', ck, 'Position', [560 300 590 330]);
mx = [mdl '/curr_mux'];
try, delete_block(mx); catch, end
add_block('simulink/Signal Routing/Mux', mx, 'Inputs', '2', ...
    'Position', [590 240 595 330]);
im = [mdl '/curr_force'];
try, delete_block(im); catch, end
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', im, ...
    'MATLABFcn', 'hil_current_force', 'Position', [650 255 750 305]);
pe = get_param(hEuler, 'PortHandles');
pc = get_param(ck,    'PortHandles');
pm = get_param(mx,    'PortHandles');
pi_= get_param(im,    'PortHandles');
pS = get_param(hSum,  'PortHandles');
add_line(mdl, pe.Outport(1),  pm.Inport(1));
add_line(mdl, pc.Outport(1),  pm.Inport(2));
add_line(mdl, pm.Outport(1),  pi_.Inport(1));
add_line(mdl, pi_.Outport(1), pS.Inport(5));
end

% =====================================================================
function rewire_step(mdl)
% 注释块 find_system 看不见 -> 用路径句柄
hMux = get_param([mdl '/Mux'], 'Handle');
hXd  = get_param([mdl '/x '], 'Handle');
hYd  = get_param([mdl '/y '], 'Handle');
hMd  = blk_by_sid(mdl, '1859');

pxd = get_param(hXd, 'PortHandles');
pyd = get_param(hYd, 'PortHandles');
pm  = get_param(hMux, 'PortHandles');

% 若原来连着 x_d/y_d 正弦, 先断开 (字符串形式最可靠)
for dl = {'x_d','y_d'}
    for pp = 1:2
        try
            delete_line(mdl, [dl{1} '/1'], ['Mux/' num2str(pp)]);
            fprintf('[REWIRE] 已断开 %s -> Mux/%d\n', dl{1}, pp);
        catch
        end
    end
end

set_param(hXd, 'Commented', 'off');
set_param(hYd, 'Commented', 'off');
add_line(mdl, pxd.Outport(1), pm.Inport(1));
add_line(mdl, pyd.Outport(1), pm.Inport(2));

% nu_d = 0
zc = [mdl '/step_zeros6'];
try, delete_block(zc); catch, end
add_block('simulink/Sources/Constant', zc, 'Value', 'zeros(6,1)', ...
    'Position', [380 660 440 690]);
pz  = get_param(zc,  'PortHandles');
pmd = get_param(hMd, 'PortHandles');
srcs = {'etad_dot_src1','etad_dot_src2','etad_dot_src3','etad_dot_src4','etad_dot_src5','etad_dot_src6'};
for pp = 1:6
    try
        delete_line(mdl, [srcs{pp} '/1'], ['etad_dot/' num2str(pp)]);
    catch
    end
end
for pp = 1:6
    add_line(mdl, pz.Outport(1), pmd.Inport(pp));
end
end

% =====================================================================
function h = blk_by_sid(mdl, sid)
hs = find_system(mdl, 'SearchDepth', 1, 'Type', 'block');
h = [];
for k = 1:numel(hs)
    if strcmp(char(string(get_param(hs{k}, 'SID'))), sid)
        h = hs{k};
        return
    end
end
error('SID %s not found in %s', sid, mdl);
end

% =====================================================================
function add_probe(mdl, src, nm)
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
