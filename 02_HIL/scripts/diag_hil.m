%% diag_hil.m -- 控制周期 T_s 扫描（共位步长）：连续 vs 离散 并排对照
%
%  目的：把「控制器 10 ms 离散化」这一项的代价单独测出来。
%  做法：固定求解器步长 = 控制周期（共位，无速率混合、无 ZOH），
%        在每一个 T_s 下各跑两遍：
%              CTRL_DISCRETE = 0  -> 原连续控制器
%              CTRL_DISCRETE = 1  -> 离散控制器（取自 rov_ctrl_deploy）
%        两者唯一差别就是控制器是否离散，所以差值就是纯离散化代价。
%
%  判读：
%    · 若「离散组」在所有 T_s 都发散，而「连续组」都收敛 -> 离散化实现有结构性问题
%    · 若「离散组」只在 T_s 大于某个值时发散            -> 采样率限制，T_s 需取更小
%    · 若两者在同一 T_s 一起发散                        -> 是被控对象积分太粗，与离散无关
%
%  只读不写模型（不调用 save_system）
%  日志: out/diag_hil.log / .mat
clear; close all;
ROOT  = 'E:\yanjiusheng\wb_hil';
MODEL = 'rov_hil_loopback';
TEND  = 20;

cd(ROOT);
fid = fopen(fullfile(ROOT,'out','diag_hil.log'),'w','n','UTF-8');
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

hCD  = get_param([MODEL '/Controller_D'],'Handle');
allb = find_system(hCD, 'LookUnderMasks','all','FollowLinks','on','Type','block');
nTs  = 0;
for j = 1:numel(allb)
    if hasTs(allb(j)), nTs = nTs + 1; end
end
fprintf(fid, 'Controller_D 内块总数 = %d，其中带 SampleTime 参数 = %d\n', numel(allb), nTs);
fprintf(fid, '被控对象共位步长 = 控制周期；StopTime = %d s\n\n', TEND);

hdr = '%-9s %-11s %-8s %-13s %-13s %-11s %s\n';
fld = '%-9s %-11s %-8s %-13.4g %-13.4g %-11.4g %s\n';
fprintf(fid, hdr, 'Ts [s]','控制器','结果','eta max','u max','失败时刻','说明');
fprintf(1,    hdr, 'Ts [s]','控制器','结果','eta max','u max','失败时刻','说明');

TsList = [0.01 0.005 0.002 0.001];
sweep  = {};

for k = 1:numel(TsList)
    Ts = TsList(k);
    st = sprintf('%.6g', Ts);

    % 控制器侧全部改成 Ts
    for j = 1:numel(allb)
        try, set_param(allb(j),'SampleTime',st); catch, end
    end
    % 被控对象共位步长
    set_param(MODEL,'SolverType','Fixed-step','Solver','ode4', ...
                   'FixedStep', st, 'StopTime', num2str(TEND));

    for disc = [1 0]
        assignin('base','CTRL_DISCRETE', disc);
        if disc == 1, tag = '离散10ms类'; else, tag = '连续'; end
        tag = regexprep(tag, '10ms类', sprintf('%gms', Ts*1000));

        [verdict, etaMax, uMax, tfail, msg] = runone(MODEL);
        fprintf(fid, fld, st, tag, verdict, etaMax, uMax, tfail, msg);
        fprintf(1,    fld, st, tag, verdict, etaMax, uMax, tfail, msg);
        sweep{end+1} = struct('Ts',Ts,'disc',disc,'verdict',verdict, ...
            'etaMax',etaMax,'uMax',uMax,'tfail',tfail,'msg',msg); %#ok<SAGROW>
    end
    fprintf(fid, '\n'); fprintf(1, '\n');
end

save(fullfile(ROOT,'out','diag_hil.mat'),'sweep','TsList');
fprintf(fid, '[OK] 已保存 out/diag_hil.mat\n');
fprintf(fid, '\nDONE\n');
fprintf(1,    '\nDONE\n');

% ================= 局部函数（必须在脚本末尾）=================
function [verdict, etaMax, uMax, tfail, msg] = runone(MODEL)
verdict = '?'; etaMax = NaN; uMax = NaN; tfail = NaN; msg = '';
try
    so = sim(MODEL, 'CaptureErrors', 'on');
catch
    try
        so = sim(MODEL);
    catch e
        verdict = '发散(报错)';
        msg = e.message;
        if numel(msg) > 55, msg = [msg(1:55) '...']; end
        return
    end
end

eta = gl(so,'eta_log');
if isempty(eta) || ~isstruct(eta) || ~isfield(eta,'signals')
    verdict = '无数据'; msg = '取不到 eta_log'; return
end
e2  = double(eta.signals.values);
t2  = double(eta.time);
if isempty(e2), verdict = '无数据'; msg = 'eta_log 为空'; return; end

fin = isfinite(e2);
if any(fin(:)), etaMax = max(abs(e2(fin))); end
bad = find(~all(isfinite(e2),2), 1, 'first');
if isempty(bad)
    verdict = '收敛';
    if isfield(eta,'time') && numel(t2) > 0 && t2(end) < 19.99
        verdict = '提前结束'; msg = sprintf('t_end=%.2f', t2(end));
    end
else
    verdict = '发散'; tfail = t2(min(bad,numel(t2)));
    msg = '出现非有限值';
end

u = gl(so,'u_cmd');
if isstruct(u) && isfield(u,'signals')
    u2 = double(u.signals.values); finu = isfinite(u2);
    if any(finu(:)), uMax = max(abs(u2(finu))); end
end
end

function tf = hasTs(h)
tf = false;
try
    get_param(h, 'SampleTime');
    tf = true;
catch
end
end

function v = gl(so, name)
v = [];
try, v = so.get(name); catch, end
if isempty(v), try, v = evalin('base', name); catch, end, end
end
