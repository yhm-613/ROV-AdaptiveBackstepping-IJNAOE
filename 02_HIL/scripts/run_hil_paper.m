% run_hil_paper.m -- 论文 HIL 工况批处理 (单次 MATLAB 会话跑 4 个工况)
%   A: 圆轨迹 100 s, 无海流   (对应论文 4.2)
%   B: 圆轨迹 100 s, 恒定海流 0.3 m/s @45deg (对应论文 4.3 profile 1)
%   C: 圆轨迹 100 s, 缓变海流 0.2+0.1sin(0.05t) (对应论文 4.3 profile 2)
%   D: 阶跃 2 m 100 s, 无海流 (对应论文 4.1)
% 每个工况前用 SWD 软复位板子清自适应状态; 模型编辑只在内存中, 不写回 .slx
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
addpath(genpath(fullfile(ROOT, 'pc')));
warning('off', 'all');
warning('on', 'hil_bridge_mfun:fail');

% 板卡软复位 (自适应律状态清零): 运行前按一次板上 RESET 键, 或调用本机复位脚本

% --- 基础工作区 (与 build_pc_models 同口径) ---
Sv = load(fullfile(ROOT, 'rovmodel75kg.mat'));
fn = fieldnames(Sv);
for ii = 1:numel(fn), assignin('base', fn{ii}, Sv.(fn{ii})); end
assignin('base', 'M_nom', Sv.M); assignin('base', 'Ma_nom', Sv.Ma);
assignin('base', 'DL_nom', Sv.DL);
assignin('base', 'CTRL_DISCRETE', 1);
assignin('base', 'TS_CTRL', 0.01);
assignin('base', 'HIL_CURR_BETA', pi/4);

mdl = 'rov_hil_host';

% ================= 公共配置: 注入海流载荷块 (一次性, 内存中) ==============
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(mdl);
wire_current(mdl);
fprintf('[SETUP] 海流注入块已挂到 Sum1 (第5口, +---+)\n');

% ================= 工况 A/B/C: 圆轨迹 (共用同一内存模型, 海流接线保留) ==
scen = struct( ...
    'A', struct('prof', 0, 'ref', 'circle', 'stop', '100'), ...
    'B', struct('prof', 1, 'ref', 'circle', 'stop', '100'), ...
    'C', struct('prof', 2, 'ref', 'circle', 'stop', '100'));

ids = fieldnames(scen);
for k = 1:numel(ids)
    id  = ids{k};
    cfg = scen.(id);
    run_one(mdl, id, cfg, RESET, ROOT);
end
close_system(mdl, 0);   % 圆轨迹组结束, 丢弃内存编辑

% ================= 工况 D: 阶跃 (重新加载 + 重接参考) =================
load_system(mdl);
wire_current(mdl);
rewire_step(mdl);
fprintf('[SETUP] 参考已切换为阶跃 (x,y,z=2m, nu_d=0)\n');
run_one(mdl, 'D', struct('prof', 0, 'ref', 'step', 'stop', '100'), RESET, ROOT);
close_system(mdl, 0);

fprintf('ALL_DONE\n');

% =====================================================================
function run_one(mdl, id, cfg, RESET, ROOT)
fprintf('\n======== 工况 %s (prof=%d, ref=%s, T=%s s) ========\n', ...
    id, cfg.prof, cfg.ref, cfg.stop);

% --- 软复位板子 (清自适应状态), SWD 不占串口 ---
[st, out] = system([RESET ' 2>&1']);
fprintf('[RESET] status=%d\n%s\n', st, strtrim(out));

% --- 清桥函数持久状态 (串口句柄/统计), 重建干净开局 ---
clear('hil_bridge_host', 'hil_bridge_mfun');
pause(1);

assignin('base', 'HIL_CURR_PROF', cfg.prof);

set_param(mdl, 'StopTime', cfg.stop);
set_param(mdl, 'SolverType', 'Fixed-step');
set_param(mdl, 'Solver', 'ode4');
set_param(mdl, 'FixedStep', '0.01');
set_param(mdl, 'SaveTime', 'on', 'TimeSaveName', 'TS_t');
try
    set_param([mdl '/HIL Bridge'], 'SampleTime', '0.01');
catch
end

% --- 挂探针 (幂等) ---
add_probe(mdl, 'Mux_hil', 'TS_mux24');
add_probe(mdl, 'HIL Bridge', 'TS_bridge');

t_wall = tic;
try
    out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    fprintf('[RUN %s] 完成, 墙钟 %.1f s\n', id, toc(t_wall));
catch err
    fprintf('[RUN %s] 异常: %s\n', id, err.message);
    save(fullfile(ROOT, 'out', sprintf('hil_paper_%s.mat', id)), 'err');
    close_system(mdl, 0);
    return
end

T  = double(out.TS_t);
M  = double(out.TS_mux24);
BR = double(out.TS_bridge);
eta = M(:, 1:6);  nu = M(:, 7:12);  eta_d = M(:, 13:18);
ucmd = BR(:, 1:6);  link = BR(:, 7);
e = eta - eta_d;

fprintf('[STAT %s] 步数=%d t=[%.2f %.2f] link ok=%d to=%d badcrc=%d fail=%d\n', ...
    id, numel(T), T(1), T(end), sum(link==0), sum(link==1), sum(link==2), sum(link>=3));
fprintf('[STAT %s] RMS 全程 : %s\n', id, mat2str(round(sqrt(mean(e.^2,1)), 5)));
h = T > 80;
fprintf('[STAT %s] RMS 末段 : %s (t>80s)\n', id, mat2str(round(sqrt(mean(e(h,:).^2,1)), 5)));
fprintf('[STAT %s] u max|.|: %s\n', id, mat2str(round(max(abs(ucmd)), 2)));

save(fullfile(ROOT, 'out', sprintf('hil_paper_%s.mat', id)), 'T', 'M', 'BR');
fprintf('[OUT %s] 已存 out/hil_paper_%s.mat\n', id, id);
% 不 close_system: 圆轨迹 A/B/C 共用内存模型 (海流接线保留), 由外层控制关闭
end

% =====================================================================
function wire_current(mdl)
% Sum1 (Inputs='+---') 增加第5口 '+' 接海流等效载荷
hSum  = blk_by_sid(mdl, '1265');
hEuler= blk_by_sid(mdl, '1229');   % eta 输出 (6x1)
set_param(hSum, 'Inputs', '+---+');

% Clock: t
ck = [mdl '/curr_clock'];
try, delete_block(ck); catch, end
add_block('simulink/Sources/Clock', ck, 'Position', [560 300 590 330]);
% Mux: [eta(6); t] -> IM Fcn 内部取 psi=eta(6), t
mx = [mdl '/curr_mux'];
try, delete_block(mx); catch, end
add_block('simulink/Signal Routing/Mux', mx, 'Inputs', '2', ...
    'Position', [590 240 595 330]);
% Interpreted MATLAB Fcn
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
% Mux 17: in1<-x_d(sin), in2<-y_d(sin) 改接 x/y 阶跃块 (SID 1266/1267)
% etad_dot Mux 1859 六个输入全部改接 zeros(6,1)
hMux  = blk_by_sid(mdl, '17');
hXd   = blk_by_sid(mdl, '1844');
hYd   = blk_by_sid(mdl, '1845');
hX    = blk_by_sid(mdl, '1266');
hY    = blk_by_sid(mdl, '1267');
hMd   = blk_by_sid(mdl, '1859');   % etad_dot Mux

pm  = get_param(hMux, 'PortHandles');
pxd = get_param(hXd,  'PortHandles');
pyd = get_param(hYd,  'PortHandles');
px  = get_param(hX,   'PortHandles');
py  = get_param(hY,   'PortHandles');

delete_line(mdl, pxd.Outport(1), pm.Inport(1));
delete_line(mdl, pyd.Outport(1), pm.Inport(2));
set_param(hX, 'Commented', 'off');
set_param(hY, 'Commented', 'off');
add_line(mdl, px.Outport(1), pm.Inport(1));
add_line(mdl, py.Outport(1), pm.Inport(2));

% nu_d = 0
zc = [mdl '/step_zeros6'];
try, delete_block(zc); catch, end
add_block('simulink/Sources/Constant', zc, 'Value', 'zeros(6,1)', ...
    'Position', [380 660 440 690]);
pz = get_param(zc,  'PortHandles');
pmd = get_param(hMd, 'PortHandles');
for pp = 1:6
    try
        delete_line(mdl, ['etad_dot_src' num2str(pp) '/1'], ['etad_dot/' num2str(pp)]);
    catch
        % 输入3-6 本来就是常量 0, 名字相同仍可删; 万一失败用句柄方式跳过
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
