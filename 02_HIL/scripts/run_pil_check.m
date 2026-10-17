%% run_pil_check.m -- PiL 等效性验证：Simulink 部署模型 vs 生成的 C 代码
%
%  1. 读 out/hosttest_sp.csv  [t eta(6) nu(6) eta_d(6) nu_d(6)]
%     （由 run_discretization_check.m 从一个「状态有界」的收敛场景导出）
%  2. 以它作为外部输入灌进 rov_ctrl_deploy.slx，跑一遍，记录 u
%     -> out/hosttest_u_sim.csv
%  3. C 侧回放（firmware/hil_target_hosttest.c）产生 out/hosttest_u_c.csv
%  4. scripts/compare_hosttest.py 比对：Simulink 离散模型 与 生成代码 必须逐点一致
%
%  为什么用「回放」而不是闭环：PiL 的判据是「同一输入序列 -> 同一输出序列」，
%  逐点比对。把闭环里记录下来的 (eta, nu) 序列回放给两侧控制器，就完全隔离了
%  被控对象积分器的差异，比的是控制器本身。两侧的离散状态 (c1_hat, c2_hat,
%  UnitDelay) 初值一致，因此 u 应当逐点可复现。
%
%  ⚠ 关于取输出的踩坑（勿回退）：
%     · 'SaveFormat' 不是合法的 block_diagram 参数 -> 报「参数设置无效」
%     · set_param(<line>,'DataLogging','on') 也不行 -> 报「line 没有名为
%       'DataLogging' 的参数」
%     · 可靠做法：在根饱和块 Sat 的输出上「分支」挂一个 To Workspace，
%       仿真结束后立刻 close_system(MODEL,0) —— 磁盘上的 .slx 不被改动
clear; close all;
ROOT  = 'E:\yanjiusheng\wb_hil';
MODEL = 'rov_ctrl_deploy';
TS    = 0.01;

cd(ROOT);
fid = fopen(fullfile(ROOT,'out','pil_check.log'),'w','n','UTF-8');
cobj = onCleanup(@() fclose(fid)); %#ok<NASGU>
warning('off','all');

spFile = fullfile(ROOT,'out','hosttest_sp.csv');
if ~exist(spFile,'file')
    fprintf(fid,'[ERR] 找不到 %s，请先运行 run_discretization_check.m\n', spFile);
    fprintf(2,   '[ERR] 找不到 %s，请先运行 run_discretization_check.m\n', spFile);
    fprintf('DONE\n'); return
end

Sv = load(fullfile(ROOT,'rovmodel75kg.mat'));
fn = fieldnames(Sv);
for ii = 1:numel(fn), assignin('base', fn{ii}, Sv.(fn{ii})); end
assignin('base','M_nom',Sv.M); assignin('base','Ma_nom',Sv.Ma);
assignin('base','DL_nom',Sv.DL);

M = readmatrix(spFile);
M = M(~any(~isfinite(M),2), :);
M = sortrows(M, 1);
[~, ia] = unique(M(:,1)); M = M(ia,:);
fprintf(fid,'读入 setpoint 序列 %d 行，t = %.3f .. %.3f s\n', size(M,1), M(1,1), M(end,1));
fprintf(1,   '读入 setpoint 序列 %d 行，t = %.3f .. %.3f s\n', size(M,1), M(1,1), M(end,1));

% 重采样到 10 ms 网格（模型是定步长离散，输入必须是网格上的点）
% 列数自适应：[t, eta(6), nu(6), eta_d(6), nu_d(6)] = 25 列
tEnd  = M(end,1);
nCol  = size(M,2);
nSig  = nCol - 1;
tg    = (0:TS:tEnd)';
sp    = zeros(numel(tg), nCol);
sp(:,1) = tg;
for c = 1:nSig
    sp(:,1+c) = interp1(M(:,1), M(:,1+c), tg, 'linear', 'extrap');
end
sp(~isfinite(sp)) = 0;
assignin('base','sp_data', sp);
fprintf(fid,'外部输入矩阵 = %d 点 x %d 列（t + %d 个信号）\n', size(sp,1), nCol, nSig);

% ---- 灌进部署模型 ----
if bdIsLoaded(MODEL), close_system(MODEL,0); end
load_system(MODEL);
set_param(MODEL,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
                'FixedStep',num2str(TS),'StopTime',num2str(tEnd), ...
                'LoadExternalInput','on','ExternalInput','sp_data');
set_param(MODEL,'SignalLogging','on','SignalLoggingName','logsout');

% 在根饱和块输出上分支挂一个 To Workspace（内存态，不保存 .slx）
hSat = [];
for nmSat = {'Sat','Saturation (+/-100V)','Saturation'}
    try
        hSat = get_param([MODEL '/' nmSat{1}], 'Handle');
        fprintf(fid,'取到根饱和块 = %s\n', nmSat{1});
        break
    catch
    end
end
madeTW = false;
try
    if isempty(hSat), error('找不到根饱和块'); end
    phS = get_param(hSat,'PortHandles');
    add_block('built-in/ToWorkspace', [MODEL '/u_pil_log'], ...
        'VariableName','u_pil','MaxDataPoints','inf','SaveFormat','Array', ...
        'SampleTime','-1','Position',[860 260 930 300]);
    add_line(MODEL, phS.Outport(1), ...
        get_param([MODEL '/u_pil_log'],'PortHandles').Inport(1), 'autorouting','on');
    madeTW = true;
    fprintf(fid,'已在根饱和块输出分支 To Workspace u_pil\n');
catch e
    fprintf(fid,'[WARN] 挂 To Workspace 失败: %s\n', e.message);
end

try
    so = sim(MODEL);
    [T, U] = pullU(so);
    if isempty(T)
        error('取不到模型输出（u_pil / logsout / yout 均为空）');
    end
    n = min(numel(T), size(U,1));
    U = U(1:n, 1:min(6,size(U,2)));
    writematrix([T(1:n), U], fullfile(ROOT,'out','hosttest_u_sim.csv'));
    fprintf(fid,'[OK] 部署模型仿真完成，%d 点 x %d 列 -> out/hosttest_u_sim.csv\n', ...
            n, size(U,2));
    fprintf(1,   '[OK] 部署模型仿真完成，%d 点 -> out/hosttest_u_sim.csv\n', n);
catch e
    fprintf(fid,'[ERR] 部署模型仿真: %s\n', e.message);
    if isprop(e,'cause') && ~isempty(e.cause)
        for kk=1:numel(e.cause)
            if ~isempty(e.cause{kk})
                fprintf(fid,'    cause%d: %s\n',kk,e.cause{kk}.message);
            end
        end
    end
    fprintf(2,'[ERR] 部署模型仿真失败: %s\n', e.message);
end

% ---- 第二遍：输入信号先量化到 float32 再跑 ----
% 串口协议里 setpoint 是 float32（hil_setpoint_t 的字段全是 float）。C 侧
% rov_ctrl_deploy_U.eta[i] = (real_T)g_sp.eta[i] 这一步把 double 收窄成 float，
% 输入因此少了约 7 位有效数字；而 Simulink 侧喂的是完整 double。
% 为了把「代码生成链路是否可信」与「传输格式精度」这两件事分开，
% 这里再跑一遍「输入同样被 float32 量化」的 Simulink，得到 hosttest_u_sim_f32.csv：
%   · c  vs sim_f32  -> 应位对齐（<1e-9），证明代码生成无误差
%   · sim vs sim_f32 -> float32 传输引入的指令偏差（HIL 链路的真实代价）
%
% ⚠ 只量化前 24 个信号列，第 1 列（时间）必须保持 double。
%   踩过的坑：一开始写成 sp32 = double(single(sp)) 把时间列也量化了，
%   single(23.8) = 23.799999237...，Simulink 于是按这张被扭曲的时间轴做线性
%   插值，等效给输入注入约 1e-4 量级的相对扰动（θ 最大可达 7.6e-5），
%   再被控制器里增益 100 的微分项放大，就冒出了 5e-3 V 的假偏差。
try
    sp32 = sp;
    sp32(:, 2:end) = double(single(sp(:, 2:end)));
    assignin('base','sp_data', sp32);
    so2  = sim(MODEL);
    [T2, U2] = pullU(so2);
    if ~isempty(T2)
        n2 = min(numel(T2), size(U2,1));
        U2 = U2(1:n2, 1:min(6,size(U2,2)));
        writematrix([T2(1:n2), U2], fullfile(ROOT,'out','hosttest_u_sim_f32.csv'));
        fprintf(fid,'[OK] float32 量化输入复跑完成，%d 点 -> out/hosttest_u_sim_f32.csv\n', n2);
        fprintf(1,   '[OK] float32 量化输入复跑完成，%d 点 -> out/hosttest_u_sim_f32.csv\n', n2);
    else
        fprintf(fid,'[WARN] float32 量化输入复跑取不到输出\n');
    end
catch e
    fprintf(fid,'[WARN] float32 量化输入复跑失败: %s\n', e.message);
end

% 无论成败都丢弃内存态改动，保证磁盘 .slx 不被污染
try, close_system(MODEL, 0); catch, end
if madeTW, fprintf(fid,'[OK] 已丢弃内存态的 u_pil_log，磁盘 .slx 未改动\n'); end

fprintf(fid,'DONE\n');
fprintf('DONE\n');

% ================= 局部函数 =================
function [T, U] = pullU(so)
% 依次尝试：新挂的 To Workspace(u_pil) -> logsout -> yout -> base 工作区
T = []; U = [];
for nm = {'u_pil','u_out','u_cmd'}
    v = [];
    try, v = so.get(nm{1}); catch, end
    if isempty(v), try, v = evalin('base', nm{1}); catch, end, end
    [T, U] = unpack(v);
    if ~isempty(T), return; end
end
try
    lg = so.get('logsout');
    if ~isempty(lg)
        for k = 1:lg.numElements
            [T, U] = unpack(lg.getElement(k));
            if ~isempty(T), return; end
        end
    end
catch
end
try
    v = so.yout;
    if ~isempty(v)
        for k = 1:v.numElements
            [T, U] = unpack(v.getElement(k));
            if ~isempty(T), return; end
        end
    end
catch
end
end

function [T, U] = unpack(v)
T = []; U = [];
if isempty(v), return; end
if isstruct(v) && isfield(v,'signals') && isfield(v,'time')
    T = double(v.time); U = double(v.signals.values);
elseif isa(v,'timeseries')
    T = double(v.Time); U = squeeze(double(v.Data));
elseif isa(v,'Simulink.SimulationData.Signal')
    [T, U] = unpack(v.Values);
    return
elseif isa(v,'Simulink.SimulationData.Dataset')
    for k = 1:v.numElements
        [T, U] = unpack(v.getElement(k));
        if ~isempty(T), return; end
    end
    return
else
    return
end
if isempty(T) || ~isnumeric(U), T = []; U = []; return; end
T = T(:);
if size(U,1) ~= numel(T) && size(U,2) == numel(T), U = U.'; end
if size(U,1) ~= numel(T), T = []; U = []; return; end
end
