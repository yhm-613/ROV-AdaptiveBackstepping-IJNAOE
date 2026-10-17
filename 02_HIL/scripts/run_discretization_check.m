%% run_discretization_check.m -- 离散化误差的三段式分解 + PiL 数据导出
%
%  三个场景（全部采用共位步长：求解步长 = 控制周期）：
%    A_ode45_cont    ode45 变步长(MaxStep 1e-3) + 连续控制器   -> 论文基线
%    C_ode4_10m_cont ode4 定步长 0.01          + 连续控制器
%    D_ode4_10m_disc ode4 定步长 0.01          + 离散10ms控制器
%
%  于是两个差异量各自对应一个误差来源：
%    A -> C : 定步长 + 10 ms 粗步长积分误差
%    C -> D : 控制器 10 ms 离散化误差        <== 半实物仿真的核心代价
%
%  为什么不做「ode4 1e-3 + 离散控制器」：
%    该配置下 Controller_D 的连续输入（Mux/Sum2/Integrator）与内部 0.01 离散块
%    速率不匹配，Simulink 报「数据完整性」错误；在反馈回路里插 Zero-Order Hold
%    也解决不了（Sum 属直通块）。既然 C 组已证明 10 ms ode4 对被控对象足够，
%    就统一采用共位步长，既避免速率混合，也正好就是 HIL 的真实配置。
%
%  输出: out/discretization_check.log / .mat
%        out/hosttest_sp.csv     [t eta(6) nu(6) eta_d(6)]  <- PiL 回放激励
%        out/hosttest_u_ref.csv  [t u(6)]                   <- 连续控制器参考输出
clear; close all;
ROOT  = 'E:\yanjiusheng\wb_hil';
MODEL = 'rov_hil_loopback';
TEND  = 40;
TGRID = 0:0.01:TEND;

cd(ROOT);
fid = fopen(fullfile(ROOT,'out','discretization_check.log'),'w','n','UTF-8');
cobj = onCleanup(@() fclose(fid)); %#ok<NASGU>
warning('off','all');

Sv = load(fullfile(ROOT,'rovmodel75kg.mat'));
fn = fieldnames(Sv);
for ii = 1:numel(fn), assignin('base', fn{ii}, Sv.(fn{ii})); end
assignin('base','M_nom',Sv.M); assignin('base','Ma_nom',Sv.Ma);
assignin('base','DL_nom',Sv.DL);
assignin('base','TS_CTRL',0.01);

if bdIsLoaded(MODEL), close_system(MODEL,0); end
load_system(MODEL);

cases = {
%  名称                SolverType        Solver    FixedStep  MaxStep  CtrlDisc
    'A_ode45_cont',   'Variable-step', 'ode45',  '0.01',    '1e-3',  0;
    'C_ode4_10m_cont','Fixed-step',    'ode4',   '0.01',    '',      0;
    'D_ode4_10m_disc','Fixed-step',    'ode4',   '0.01',    '',      1;
};

res = struct();
for k = 1:size(cases,1)
    nm  = cases{k,1};
    fnm = matlab.lang.makeValidName(nm);
    fprintf(fid, '\n===== %s =====\n', nm);
    fprintf(1,    '\n===== %s =====\n', nm);

    set_param(MODEL, 'SolverType', cases{k,2}, 'Solver', cases{k,3});
    try, set_param(MODEL, 'FixedStep', cases{k,4}); catch, end
    if ~isempty(cases{k,5})
        try, set_param(MODEL, 'MaxStep', cases{k,5}); catch, end
    end
    set_param(MODEL, 'StopTime', num2str(TEND));
    assignin('base','CTRL_DISCRETE', cases{k,6});

    res.(fnm).ok = false;
    t0 = tic;
    try
        so   = sim(MODEL);
        eta  = getlog(so, 'eta_log');
        nu   = getlog(so, 'nu_log');
        u    = getlog(so, 'u_cmd');
        edge = getlog(so, 'eta_d_log');
        res.(fnm).t     = double(eta.time);
        res.(fnm).eta   = double(eta.signals.values);
        res.(fnm).nu    = double(nu.signals.values);
        res.(fnm).u     = double(u.signals.values);
        res.(fnm).eta_d = double(edge.signals.values);
        edd = getlog(so, 'eta_d_dot_log');
        res.(fnm).eta_d_dot = double(edd.signals.values);
        res.(fnm).ok    = true;
        fprintf(fid, '[OK] %.1f s | t=%d eta=%s nu=%s u=%s eta_d=%s\n', toc(t0), ...
            numel(res.(fnm).t), mat2str(size(res.(fnm).eta)), mat2str(size(res.(fnm).nu)), ...
            mat2str(size(res.(fnm).u)), mat2str(size(res.(fnm).eta_d)));
        fprintf(1,    '[OK] %.1f s | 采样点 %d\n', toc(t0), numel(res.(fnm).t));
    catch e
        fprintf(fid, '[ERR] %s\n', e.message);
        fprintf(1,    '[ERR] %s\n', e.message);
        if isprop(e,'cause') && ~isempty(e.cause)
            for kk = 1:numel(e.cause)
                if ~isempty(e.cause{kk})
                    fprintf(fid, '    cause%d: %s\n', kk, e.cause{kk}.message);
                end
            end
        end
    end
end

% ---- 统一到公共网格（逐点过滤非有限值）----
names = cases(:,1);
grid  = struct();
for k = 1:numel(names)
    fnm = matlab.lang.makeValidName(names{k});
    if ~res.(fnm).ok, continue; end
    try
        grid.(fnm).eta = rs(res.(fnm).t, res.(fnm).eta,   TGRID);
        grid.(fnm).nu  = rs(res.(fnm).t, res.(fnm).nu,    TGRID);
        grid.(fnm).u   = rs(res.(fnm).t, res.(fnm).u,     TGRID);
        grid.(fnm).e   = rs(res.(fnm).t, res.(fnm).eta_d, TGRID);
        % 真正的跟踪误差 = 实测 η − 期望 ηd。
        % 注意：res.eta_d 本身在 A/C/D 三档里是同一个参考信号，拿它做差会得到
        % 2.5e-17 这种毫无意义的数（曾经把这一列误标成"跟踪 e RMS"）。
        grid.(fnm).trk = rs(res.(fnm).t, res.(fnm).eta - res.(fnm).eta_d, TGRID);
    catch e
        fprintf(fid, '[ERR] 重采样 %s 失败: %s\n', fnm, e.message);
    end
end

% ---- 对比表 ----
hdr = {'场景','η max|Δ|','η RMS Δ','ν max|Δ|','u max|Δ|','u RMS|Δ|','跟踪 e RMS(末10s)'};
fl  = '%-18s %12s %12s %12s %12s %12s %16s\n';
try
    fprintf(fid, '\n\n===== 对比（A -> C -> D 逐级分解）=====\n');
    fprintf(1,    '\n\n===== 对比（A -> C -> D 逐级分解）=====\n');
    fprintf(fid, fl, hdr{:}); fprintf(1, fl, hdr{:});
    base = 'A_ode45_cont';
    fB   = matlab.lang.makeValidName(base);
    for k = 1:numel(names)
        fnm = matlab.lang.makeValidName(names{k});
        if ~isfield(grid, fnm), continue; end
        if strcmp(fnm, fB)
            fprintf(fid, fl, names{k},'-','-','-','-','-','-');
            fprintf(1,    fl, names{k},'-','-','-','-','-','-');
            continue
        end
        if ~isfield(grid, fB), continue; end
        dE = grid.(fnm).eta - grid.(fB).eta;
        dV = grid.(fnm).nu  - grid.(fB).nu;
        dU = grid.(fnm).u   - grid.(fB).u;
        dT = grid.(fnm).trk - grid.(fB).trk;
        sel = false(size(dT,1),1); sel(max(1,end-999):end) = true;
        v = {mx(dE), rm(dE), mx(dV), mx(dU), rm(dU), rm(dT(sel,:))};
        % ⚠ 必须显式转成字符串再喂给 %s：fprintf 的 %s 遇到「整数值的 double」
        %   会按 char() 转换（例如 200 -> 'È'），把表格打乱
        vs = cellfun(@num2str, v, 'UniformOutput', false);
        fprintf(fid, fl, names{k}, vs{1}, vs{2}, vs{3}, vs{4}, vs{5}, vs{6});
        fprintf(1,    fl, names{k}, vs{1}, vs{2}, vs{3}, vs{4}, vs{5}, vs{6});
    end
    fprintf(fid, ['\n说明：\n' ...
        '  · u max|Δ| 会被「饱和翻转」主导 —— 通道一旦贴住 ±100 V，很小的广义力\n' ...
        '    差异就会让两侧一个 +100、一个 -100，单点差值直接到 200 V。这**不是**\n' ...
        '    控制精度问题，所以必须同时看 u RMS|Δ|。\n' ...
        '  · 跟踪 e RMS(末10s) = RMS(η − ηd) 在最后 10 s 的差值，这才是真正的\n' ...
        '    轨迹跟踪精度口径（与论文表里的调节时间/稳态 RMS 一致）。\n']);
catch e
    fprintf(fid, '[ERR] 生成对比表失败: %s\n', e.message);
end

save(fullfile(ROOT,'out','discretization_check.mat'), 'res','grid','TGRID','cases');
fprintf(fid, '\n[OK] 已保存 out/discretization_check.mat\n');

% ---- 导出 PiL / C 代码回放数据 ----
%   hosttest_sp.csv      [t eta(6) nu(6) eta_d(6) nu_d(6)]  <- 共同激励序列
%   hosttest_u_ref.csv   [t u(6)] 来自 C 组（连续控制器）-> 量化 10 ms 离散化代价
%   hosttest_u_sim.csv   [t u(6)] 来自 D 组（Simulink 离散控制器）
%                        -> 与生成 C 代码的输出逐点比对（判据①：代码生成链路可信度）
%
%   为什么 D 组的 u 可以直接当作「部署模型输出」：
%     rov_hil_loopback 里 Controller_D 就是从 rov_ctrl_deploy/Controller 复制的同一
%     个子系统，且 u_log 接在 Saturation (+/-100V) 之后 —— 与 rov_ctrl_deploy 的
%     根输出 u 是同一条链路、同一物理量（原模型：
%     Controller -> Inverse Thruster Matrix -> Thruster Matrix -> Saturation）。
%     D 组虽然跑的是闭环，但 (eta, nu) 已被逐点记录；回放时按同一序列喂给控制器，
%     离散状态 (c1_hat, c2_hat, UnitDelay) 初值也一致，故 u 应逐点可复现。
try
    src = 'C_ode4_10m_cont';   % 激励与连续参考都取自 C 组
    sim_ = 'D_ode4_10m_disc';  % Simulink 离散控制器输出
    fS  = matlab.lang.makeValidName(src);
    fD  = matlab.lang.makeValidName(sim_);
    if ~res.(fS).ok
        fprintf(fid, '[WARN] %s 未收敛，跳过回放数据导出\n', src);
    else
        nS = min([size(res.(fS).eta,1), size(res.(fS).nu,1), ...
                  size(res.(fS).eta_d,1), size(res.(fS).u,1), ...
                  size(res.(fS).eta_d_dot,1), numel(res.(fS).t)]);
        % 25 列 = t + eta(6) + nu(6) + eta_d(6) + nu_d(6)
        % 列顺序必须与 rov_ctrl_deploy 的根 Inport 端口号一致：
        %   eta(Port1) nu(Port2) eta_d(Port3) nu_d(Port4)
        M = [res.(fS).t(1:nS), res.(fS).eta(1:nS,1:6), res.(fS).nu(1:nS,1:6), ...
             res.(fS).eta_d(1:nS,1:6), res.(fS).eta_d_dot(1:nS,1:6)];
        writematrix(M, fullfile(ROOT,'out','hosttest_sp.csv'));
        writematrix([res.(fS).t(1:nS), res.(fS).u(1:nS,1:6)], ...
                    fullfile(ROOT,'out','hosttest_u_ref.csv'));
        fprintf(fid, '[OK] 回放激励 + 连续 u 参考 取自 %s，%d 行\n', src, nS);
        fprintf(1,    '[OK] 回放激励 + 连续 u 参考 取自 %s，%d 行\n', src, nS);
        if res.(fD).ok
            nD = min([numel(res.(fD).t), size(res.(fD).u,1), nS]);
            writematrix([res.(fD).t(1:nD), res.(fD).u(1:nD,1:6)], ...
                        fullfile(ROOT,'out','hosttest_u_sim.csv'));
            fprintf(fid, '[OK] 离散 u 参考 取自 %s，%d 行\n', sim_, nD);
            fprintf(1,    '[OK] 离散 u 参考 取自 %s，%d 行\n', sim_, nD);
        else
            fprintf(fid, '[WARN] %s 未收敛，hosttest_u_sim.csv 未导出\n', sim_);
        end
    end
catch e
    fprintf(fid, '[ERR] 导出回放数据: %s\n', e.message);
end

fprintf(fid, '\nDONE\n');
fprintf(1,    '\nDONE\n');

% ================= 局部函数（必须在脚本末尾）=================
function vo = rs(t, v, tg)
t = t(:);
if numel(t) ~= size(v,1)
    error('长度不一致: numel(t)=%d, size(v)=%s', numel(t), mat2str(size(v)));
end
ok = isfinite(t) & all(isfinite(v), 2);
t = t(ok); v = v(ok,:);
if numel(t) < 2
    vo = nan(numel(tg), size(v,2)); return
end
[t, ia] = unique(t);
v  = v(ia,:);
vo = interp1(t, v, tg, 'linear', 'extrap');
vo(~isfinite(vo)) = NaN;
end

function v = getlog(so, name)
v = [];
try, v = so.get(name); catch, end
if isempty(v), v = evalin('base', name); end
if isempty(v), error('取不到日志变量 %s', name); end
end

function y = mx(x), x = x(isfinite(x)); if isempty(x), y = NaN; else, y = max(abs(x)); end, end
function y = rm(x), x = x(isfinite(x)); if isempty(x), y = NaN; else, y = sqrt(mean(x.^2)); end, end
