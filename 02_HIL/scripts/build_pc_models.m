%% build_pc_models.m -- 阶段2：PC 端模型（回环验证 + 串口 HIL 主机）
%
%  产出两个模型：
%    rov_hil_loopback.slx : 原模型 + 离散控制器(取自 rov_ctrl_deploy)
%                           用一个开关在「连续控制器 / 离散控制器」之间切换，
%                           配合求解器设置可分离「控制器离散化误差」与
%                           「定步长积分误差」两个来源。
%    rov_hil_host.slx     : 同上，但把控制器换成串口 HIL Bridge。
%
%  采用「共位步长」配置：PC 端求解步长 = 控制周期（默认 10 ms）。
%  理由见 USE_ZOH 处注释。
%
%  日志: out/build_pc_models.log
clear;
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
SRC  = 'nonlinear_closed_backstepping_adaptive_circle';
LOOP = 'rov_hil_loopback';
HOST = 'rov_hil_host';
DEP  = 'rov_ctrl_deploy';

fid = fopen(fullfile(ROOT,'out','build_pc_models.log'),'w','n','UTF-8');
cobj = onCleanup(@() fclose(fid)); %#ok<NASGU>
warning('off','all');

try
    cd(ROOT);
    addpath(fullfile(ROOT,'src'));      % 原模型在 src/ 下
    Sv = load(fullfile(ROOT,'rovmodel75kg.mat'));
    fn = fieldnames(Sv);
    for ii = 1:numel(fn), assignin('base', fn{ii}, Sv.(fn{ii})); end
    assignin('base','M_nom', Sv.M); assignin('base','Ma_nom', Sv.Ma);
    assignin('base','DL_nom', Sv.DL);
    assignin('base','CTRL_DISCRETE', 1);
    assignin('base','TS_CTRL', 0.01);
    L(fid, '参数已载入: %s\n', strjoin(fn', ', '));

    if bdIsLoaded(DEP), close_system(DEP, 0); end
    load_system(DEP);

    build_one(fid, ROOT, SRC, LOOP, DEP, false);   % 纯 Simulink 回环
    build_one(fid, ROOT, SRC, HOST, DEP, true);    % 串口 HIL 主机

catch ME
    fprintf(fid, '\n!!! 未捕获错误: %s\n', ME.message);
    for kk = 1:numel(ME.stack)
        fprintf(fid, '    @ %s : line %d\n', ME.stack(kk).name, ME.stack(kk).line);
    end
end

fprintf('DONE\n');

% =====================================================================
function build_one(fid, ROOT, SRC, DST, DEP, useBridge)
L(fid, '\n========================================\n=== 构建 %s ===\n', DST);
cd(ROOT);
try, close_system(DST, 0); catch, end
sf = fullfile(ROOT, [DST '.slx']);
if exist(sf,'file'), delete(sf); end

% ---- 1. 复制原模型 ----
try
    close_system(SRC, 0);
catch
end
load_system(SRC);
save_system(SRC, DST);
try, close_system(SRC, 0); catch, end
if ~bdIsLoaded(DST), load_system(DST); end
L(fid, '[OK] 复制 %s -> %s\n', SRC, DST);
mH = get_param(DST, 'Handle');

% ---- 2. 断开 Backstepping Controller -> Inverse Thruster Matrix ----
hInvT = findByName(mH, 'Gain', 'Inverse Thruster Matrix');
if isempty(hInvT)
    L(fid, '[ERR] 找不到 Inverse Thruster Matrix\n');
else
    phInv = get_param(hInvT, 'PortHandles');
    clearLine(phInv.Inport(1), fid, 'Inverse Thruster Matrix[1]');
end

% ---- 2b. 去掉过程噪声源，换成常数 0 ----
% 原模型的 Random Number(SampleTime=0.1) -> Gain([1;1;...]) -> Add[1] 注入 plant
% 加速度通道。在定步长求解器下，0.1 s 离散信号直接驱动连续 Add 会触发
% Simulink 的「数据完整性」报错；而且做离散化误差研究本来就应该去掉随机
% 噪声，才能得到确定性、可复现的对比结果。这里把噪声链换成 Constant 0，
% 保持拓扑不变（Add 仍是 2 输入）。
L(fid, '\n--- 去过程噪声 ---\n');
hRnd = findByName(mH, 'RandomNumber', 'Random Number');
hGin = [];
szs  = find_system(mH, 'SearchDepth',1, 'BlockType','Gain');
for k = 1:numel(szs)
    ph = get_param(szs(k),'PortHandles');
    ln = get_param(ph.Inport(1),'Line');
    if numel(ln)==1 && ln>0
        try
            if get_param(get_param(ln,'SrcBlockHandle'),'BlockType') == "RandomNumber"
                hGin = szs(k);
            end
        catch
        end
    end
end
if ~isempty(hGin)
    phG = get_param(hGin,'PortHandles');
    ln  = get_param(phG.Outport(1),'Line');
    dstP = [];
    if numel(ln)==1 && ln>0
        dstP = get_param(ln,'DstPortHandle');
    end
    try, delete_block(hGin); catch e, L(fid,'[WARN] 删 Gain: %s\n', e.message); end
    L(fid, '[OK] 删除噪声 Gain\n');
    if ~isempty(hRnd), try, delete_block(hRnd); catch, end; L(fid,'[OK] 删除 Random Number\n'); end
    if ~isempty(dstP) && dstP(1) > 0
        clearLine(dstP(1), fid, 'Add[1]');
    end
    mk(fid, 'built-in/Constant', [DST '/noise_off'], ...
       {'Value','zeros(6,1)','Position',[110 40 150 60]});
    if ~isempty(dstP) && dstP(1) > 0
        try
            add_line(DST, get_param([DST '/noise_off'],'PortHandles').Outport(1), dstP(1), ...
                     'autorouting','on');
            L(fid, '[OK] noise_off -> Add[1]\n');
        catch e
            L(fid, '[FAIL] noise_off 连线: %s\n', e.message);
        end
    else
        L(fid, '[WARN] 未捕获到 Gain 的下游端口，噪声链可能未清干净\n');
    end
else
    L(fid, '[SKIP] 未找到噪声 Gain\n');
end

% ---- 3. 选择开关 + 模式常量 ----
mk(fid, 'built-in/Constant', [DST '/CTRL_DISCRETE'], ...
   {'Value','CTRL_DISCRETE','Position',[520 40 580 60]});
mk(fid, 'built-in/Switch',   [DST '/ctrl_sel'], ...
   {'Criteria','u2 >= Threshold','Threshold','0.5', ...
    'Position',[900 300 950 400]});

if useBridge
    % ---- 串口桥：Mux(24) -> Interpreted MATLAB Function -> Demux ----
    mk(fid, 'built-in/Mux', [DST '/Mux_hil'], ...
       {'Inputs','24','Position',[560 120 565 320]});
    mk(fid, 'simulink/User-Defined Functions/Interpreted MATLAB Function', ...
       [DST '/HIL Bridge'], ...
       {'MATLABFcn','hil_bridge_mfun','OutputDimensions','7', ...
        'Position',[620 180 740 260]});
    mk(fid, 'built-in/Demux', [DST '/Demux_hil'], ...
       {'Position',[790 170 795 270]});
    try
        set_param([DST '/Demux_hil'], 'Outputs', '[6 1]');
        L(fid, '[OK] Demux_hil Outputs = [6 1]\n');
    catch e
        L(fid, '[FAIL] Demux_hil Outputs: %s\n', e.message);
    end

    wd(fid, DST, 'Euler  Transformation1/1', 'Mux_hil/1');
    wd(fid, DST, 'Integrator/1',            'Mux_hil/2');
    wd(fid, DST, 'Mux/1',                   'Mux_hil/3');
    % 第 4 组（19..24）原来是 nu_d_zero 常量 0；改成解析参考速度，
    % 与 rov_ctrl_deploy 的 nu_d 入口口径一致（协议本来就带 nu_d 字段）。
    buildEtadDot(fid, DST, mH);
    wd(fid, DST, 'etad_dot/1',              'Mux_hil/4');
    wd(fid, DST, 'Mux_hil/1',               'HIL Bridge/1');
    wd(fid, DST, 'HIL Bridge/1',            'Demux_hil/1');
    wd(fid, DST, 'Demux_hil/1',             'ctrl_sel/1');
else
    % ---- 离散控制器（从 rov_ctrl_deploy 复制，保留原作接口）----
    cp(fid, [DEP '/Controller'], [DST '/Controller_D'], [700 120 880 340]);
    hCD = get_param([DST '/Controller_D'], 'Handle');
    ips = find_system(hCD, 'SearchDepth', 1, 'BlockType', 'Inport');
    for j = 1:numel(ips), set_param(ips(j), 'PortDimensions', '6'); end
    ops = find_system(hCD, 'SearchDepth', 1, 'BlockType', 'Outport');
    for j = 1:numel(ops), set_param(ops(j), 'PortDimensions', '6'); end
    L(fid, '[OK] Controller_D 端口维度=6（In %d / Out %d）\n', numel(ips), numel(ops));

    % ---- 误差信号符号对账 ----
    % 原模型：Sum2 (Inputs='-+') = -Mux(=η_d) + Kinematics(=η) -> err = η - η_d
    % 必须确认送进 Controller_D/2 的就是这个信号，否则矫正项整项变号。
    try
        hSum2 = [];
        ks = find_system(mH, 'SearchDepth', 1, 'BlockType', 'Sum');
        for k = 1:numel(ks)
            if strcmp(nmz(get_param(ks(k),'Name')), 'Sum2'); hSum2 = ks(k); break; end
        end
        if isempty(hSum2)
            L(fid, '[WARN] 找不到 Sum2，无法做误差符号对账\n');
        else
            L(fid, '  Sum2 Inputs=%s  (in1<-%s, in2<-%s)\n', ...
                get_param(hSum2,'Inputs'), srcName(hSum2,1), srcName(hSum2,2));
            if strcmp(get_param(hSum2,'Inputs'), '-+')
                L(fid, '[OK] 误差符号正确：err = eta - eta_d，与 rov_ctrl_deploy/Controller 约定一致\n');
            else
                L(fid, '[FAIL] Sum2 符号 = %s，期望 ''-+''！\n', get_param(hSum2,'Inputs'));
            end
        end
    catch e
        L(fid, '[WARN] 误差符号对账异常: %s\n', e.message);
    end

    % ---- 控制器入口连线 ----
    % 曾尝试插 Zero-Order Hold 做显式速率转换（便于被控对象用 1e-3 细步长），
    % 实测在反馈回路里 Simulink 仍报「数据完整性」错误。改为「共位步长」：
    % PC 端求解步长 = 控制周期，不存在速率混合，也不需要 ZOH。
    % 依据：ode4 @0.01 + 连续控制器收敛正常（out/discretization_check.log D 组），
    % 说明 10 ms 的 ode4 对被控对象已足够。
    L(fid, '[INFO] 控制器入口直连（共位步长配置，不使用 ZOH）\n');
    wd(fid, DST, 'Mux/1',        'Controller_D/1');
    wd(fid, DST, 'Sum2/1',       'Controller_D/2');
    wd(fid, DST, 'Integrator/1', 'Controller_D/3');
    % 第 4 个入口 = 参考速度 eta_d_dot（解析构造，替代原数值微分器）
    buildEtadDot(fid, DST, mH);
    wd(fid, DST, 'etad_dot/1',   'Controller_D/4');
    wd(fid, DST, 'Controller_D/1', 'ctrl_sel/1');
end

% 连续控制器 -> 开关第 3 输入
wd(fid, DST, 'Backstepping Controller/1', 'ctrl_sel/3');
wd(fid, DST, 'CTRL_DISCRETE/1',           'ctrl_sel/2');
% 注意：'Inverse Thruster Matrix' 块名尾部带一个空格，按名字连线会报
% "Simulink 对象名称无效"，必须用端口句柄
if ~isempty(hInvT)
    wdH(fid, DST, 'ctrl_sel/1', hInvT, 1);
end

% ---- 4. 观测 ----
% u_log 必须接在 ±100 V 饱和「之后」—— 那才是 MCU 真正下发的推力指令电压，
% 也正是 rov_ctrl_deploy 的根输出 u。接在 ctrl_sel/1 上拿到的是未饱和的
% 广义力，与 PiL 比对基准根本不是同一个物理量。
mk(fid, 'built-in/ToWorkspace', [DST '/u_log'], ...
   {'VariableName','u_cmd','SaveFormat','Structure With Time', ...
    'MaxDataPoints','inf','Position',[1020 320 1080 340]});
hSat = [];
sats = find_system(mH, 'SearchDepth', 1, 'BlockType', 'Saturate');
if ~isempty(sats), hSat = sats(1); end
if isempty(hSat)
    L(fid, '[WARN] 找不到饱和块，u_log 退回 ctrl_sel/1（比对口径将不一致）\n');
    wd(fid, DST, 'ctrl_sel/1', 'u_log/1');
else
    L(fid, '[INFO] u_log 接在饱和块 <%s> 输出（= MCU 下发口径）\n', nmz(get_param(hSat,'Name')));
    connectH(fid, DST, hSat, [DST '/u_log'], 1);
end

mk(fid, 'built-in/ToWorkspace', [DST '/etad_log'], ...
   {'VariableName','eta_d_log','SaveFormat','Structure With Time', ...
    'MaxDataPoints','inf','Position',[1020 380 1080 400]});
wd(fid, DST, 'Mux/1', 'etad_log/1');

% 参考速度也要落到 out/*.csv，供 PiL 回放使用
mk(fid, 'built-in/ToWorkspace', [DST '/etad_dot_log'], ...
   {'VariableName','eta_d_dot_log','SaveFormat','Structure With Time', ...
    'MaxDataPoints','inf','Position',[1020 430 1080 450]});
wd(fid, DST, 'etad_dot/1', 'etad_dot_log/1');

% 统一已有 To Workspace 的名字与格式（原模型用了带空格的变量名），
% 并务必关掉默认的「Limit data points to last: 1000」——否则短序列会
% 与 eta_log/nu_log 长度不一致，后续重采样直接崩。
tw = find_system(mH, 'SearchDepth', 1, 'BlockType', 'ToWorkspace');
for k = 1:numel(tw)
    nmb = nmz(get_param(tw(k), 'Name'));
    ph  = get_param(tw(k), 'PortHandles');
    ln  = get_param(ph.Inport(1), 'Line');
    sblk = '';
    if numel(ln) == 1 && ln > 0
        try, sblk = nmz(getfullname(get_param(ln,'SrcBlockHandle'))); catch, end
    end
    newName = '';
    if contains(sblk, 'Euler') || contains(sblk, 'Transformation1'), newName = 'eta_log';
    elseif contains(sblk, 'Integrator'),                              newName = 'nu_log';
    elseif contains(sblk, 'Clock'),                                   newName = 't_log';
    end
    try, set_param(tw(k), 'MaxDataPoints', 'inf'); catch, end
    if ~isempty(newName)
        try
            set_param(tw(k), 'VariableName', newName, 'SaveFormat', 'Structure With Time');
            L(fid, '[OK] ToWorkspace %s -> %s (MaxDataPoints=inf)\n', nmb, newName);
        catch e
            L(fid, '[WARN] ToWorkspace %s: %s\n', nmb, e.message);
        end
    else
        try, set_param(tw(k), 'SaveFormat', 'Structure With Time'); catch, end
        % 这里不是「跳过」——MaxDataPoints / SaveFormat 上面已经设好了，
        % 只是因为源块不是 eta/nu/时间，所以保持它自己的变量名（u_cmd 等）
        L(fid, '[OK] ToWorkspace %s 保持原名 (MaxDataPoints=inf, 源: %s)\n', nmb, sblk);
    end
end

% ---- 5. 模型配置 ----
set_param(DST, 'StopTime', '20');
set_param(DST, 'SolverType','Variable-step', 'Solver','ode45', ...
               'MaxStep','1e-3', 'AbsTol','1e-6', 'RelTol','1e-4');
try
    set_param(DST, 'SystemTargetFile','grt.tlc', 'TargetLang','C');
catch e
    L(fid, '[WARN] 目标文件: %s\n', e.message);
end
L(fid, '[OK] 默认求解器 ode45（运行脚本会按需改成 ode4）\n');

% ---- 6. 编译自检 ----
try
    set_param(DST, 'SimulationCommand','update');
    L(fid, '[OK] Update Diagram 通过\n');
catch e
    L(fid, '[ERR] Update Diagram: %s\n', e.message);
    if isprop(e,'cause') && ~isempty(e.cause)
        for kk = 1:numel(e.cause), L(fid,'    cause%d: %s\n', kk, e.cause{kk}.message); end
    end
end

save_system(DST);
L(fid, '[OK] 已保存 %s\n', fullfile(ROOT, [DST '.slx']));
end

% =====================================================================
function L(fid, varargin), fprintf(fid, varargin{:}); end
function s = nmz(p), s = strtrim(regexprep(p, '\s+', ' ')); end
function s = srcName(blkH, port)
% 第 port 个输入端口的上游块名（归一化空白）；未连线返回 '(none)'
s = '(none)';
try
    ph = get_param(blkH, 'PortHandles');
    if port > numel(ph.Inport), return; end
    ln = get_param(ph.Inport(port), 'Line');
    if numel(ln) ~= 1 || ln <= 0, return; end
    sh = get_param(ln, 'SrcBlockHandle');
    s  = nmz(get_param(sh, 'Name'));
catch
end
end
function h = findByName(sysH, bt, tname)
h = [];
ks = find_system(sysH, 'SearchDepth', 1, 'BlockType', bt);
for k = 1:numel(ks)
    if strcmp(nmz(get_param(ks(k),'Name')), tname); h = ks(k); return; end
end
end
function clearLine(portH, fid, tag)
try
    ln = get_param(portH, 'Line');
    g = 0;
    while ~isempty(ln) && numel(ln)==1 && ln > 0 && g < 10
        delete_line(ln); ln = get_param(portH,'Line'); g = g + 1;
    end
    L(fid, '[OK] 断开 %s 上的连线\n', tag);
catch e
    L(fid, '[WARN] clearLine %s: %s\n', tag, e.message);
end
end
function mk(fid, lib, dstp, pv)
try
    add_block(lib, dstp, pv{:});
    L(fid, '[OK] 新建 %s\n', dstp);
catch e
    L(fid, '[FAIL] 新建 %s : %s\n', dstp, e.message);
end
end
function cp(fid, srcp, dstp, pos)
try
    add_block(srcp, dstp, 'Position', pos);
    L(fid, '[OK] 复制 -> %s\n', dstp);
catch e
    L(fid, '[FAIL] 复制 %s : %s\n', dstp, e.message);
end
end
function wd(fid, sys, s, t)
try
    add_line(sys, s, t, 'autorouting', 'on');
    L(fid, '[OK] %s -> %s\n', s, t);
catch e
    L(fid, '[FAIL] %s -> %s : %s\n', s, t, e.message);
end
end
function buildEtadDot(fid, DST, mH)
% 解析构造参考速度 etad_dot（6x1），替代原模型里对 eta_d 数值微分的 Derivative 块。
%   · Sin 块  A*sin(w*t+phi)  ->  Sin 块  (A*w)*sin(w*t+phi+pi/2)  = (A*w)cos(w*t+phi)
%     用 Sine Wave 加 pi/2 相位而不是 Cosine Wave 块：Simulink 库里没有
%     'simulink/Sources/Cosine Wave' 这个路径，加 pi/2 等价且稳妥。
%   · Step 块                 ->  Constant 0（阶跃的导数是冲激，工程上取 0）
% 对应关系：Mux 的第 k 个输入就是 eta_d 的第 k 个分量。
try
    hMux = findByName(mH, 'Mux', 'Mux');
    if isempty(hMux)
        L(fid, '[FAIL] buildEtadDot: 找不到 Mux\n'); return
    end
    ph = get_param(hMux, 'PortHandles');
    nIn = numel(ph.Inport);

    mk(fid, 'built-in/Mux', [DST '/etad_dot'], ...
       {'Inputs', num2str(nIn), 'Position', [520 420 525 560]});

    for k = 1:nIn
        ln = get_param(ph.Inport(k), 'Line');
        srcNameStr = '(none)'; sb = [];
        if numel(ln) == 1 && ln > 0
            sb = get_param(ln, 'SrcBlockHandle');
            srcNameStr = nmz(get_param(sb, 'Name'));
        end
        dstp = sprintf('%s/etad_dot_src%d', DST, k);
        bt = '';
        if ~isempty(sb), bt = get_param(sb, 'BlockType'); end

        if strcmp(bt, 'Sin')
            % 参数可能是表达式（例如 pi/2），str2double 会得到 NaN，必须求值
            A = evalNum(get_param(sb, 'Amplitude'));
            W = evalNum(get_param(sb, 'Frequency'));
            P = evalNum(get_param(sb, 'Phase'));
            mk(fid, 'simulink/Sources/Sine Wave', dstp, ...
               {'Amplitude', num2str(A*W), 'Frequency', num2str(W), ...
                'Phase', num2str(P + pi/2), 'Bias', '0', ...
                'Position', [420 400+35*k 450 420+35*k]});
            L(fid, '[OK] eta_d_dot[%d] <- d/dt Sin <%s>: A=%g w=%g phi=%g -> A*w=%g\n', ...
                k, srcNameStr, A, W, P, A*W);
        else
            mk(fid, 'built-in/Constant', dstp, ...
               {'Value', '0', 'Position', [420 400+35*k 450 420+35*k]});
            L(fid, '[OK] eta_d_dot[%d] <- 0   (源 <%s> 类型 %s，阶跃/常值导数为 0)\n', ...
                k, srcNameStr, bt);
        end
        wd(fid, DST, sprintf('etad_dot_src%d/1', k), sprintf('etad_dot/%d', k));
    end
catch e
    L(fid, '[FAIL] buildEtadDot: %s\n', e.message);
end
end

function v = evalNum(s)
% 参数可能是数字字面量，也可能是 'pi/2' 这类表达式；求值失败退回 0
v = str2double(s);
if isnan(v)
    try, v = evalin('base', s); catch, v = 0; end
end
if isnan(v), v = 0; end
v = double(v);
end
function connectH(fid, sys, srcH, dstPath, port)
% 两端都用句柄连线：源块句柄 -> 目标块路径的第 port 个输入端口
try
    sph = get_param(srcH, 'PortHandles');
    dh  = get_param(dstPath, 'Handle');
    dph = get_param(dh, 'PortHandles');
    add_line(sys, sph.Outport(1), dph.Inport(port), 'autorouting', 'on');
    L(fid, '[OK] (句柄)%s -> %s[%d]\n', get_param(srcH,'Name'), get_param(dh,'Name'), port);
catch e
    L(fid, '[FAIL] connectH %s -> %s[%d] : %s\n', get_param(srcH,'Name'), dstPath, port, e.message);
end
end
function wdH(fid, sys, s, dstH, port)
% 两端都用端口句柄连接（块名可能含首尾空格 / 斜杠，字符串形式不可用；
% 而且 add_line 不允许一端字符串一端句柄混用）
try
    sp = strsplit(s, '/');
    if numel(sp) >= 2
        bname = strjoin(sp(1:end-1), '/');
        pnum  = str2double(sp{end});
    else
        bname = s; pnum = 1;
    end
    sh  = get_param([sys '/' bname], 'Handle');
    sph = get_param(sh,  'PortHandles');
    dph = get_param(dstH, 'PortHandles');
    add_line(sys, sph.Outport(pnum), dph.Inport(port), 'autorouting', 'on');
    L(fid, '[OK] %s -> %s[%d] (句柄)\n', s, get_param(dstH,'Name'), port);
catch e
    L(fid, '[FAIL] %s -> [%d] : %s\n', s, port, e.message);
end
end
