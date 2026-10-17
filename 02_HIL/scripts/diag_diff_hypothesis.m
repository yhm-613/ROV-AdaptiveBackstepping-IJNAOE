%% diag_diff_hypothesis.m -- 判决：离散差分器是不是发散根因
%
%  假设：Controller_D 里两个离散差分器
%          Derivative  : 对参考 yd 求导 -> eta_d_dot
%          Derivative1 : 对 alpha1 求导 -> alpha1_dot  （乘惯量矩阵 M，增益被放大）
%        在连续模型里用的是精确 du/dt；换成 (u[k]-u[k-1])/Ts 后，
%        高频增益无穷大，且 ICPrevScaledInput 默认 0 导致第一步出现
%        |yd(0)|/Ts 的尖峰。这与「任何 T_s 都发散」一致。
%
%  做法：用 Discrete Derivative 的掩码参数 gainval 逐项关断（gainval=0 -> 输出 0），
%        看哪一项关掉后系统恢复收敛。T_s = 0.01 共位步长。
%
%  日志: out/diag_diff.log
clear; close all;
ROOT  = 'E:\yanjiusheng\wb_hil';
MODEL = 'rov_hil_loopback';
TEND  = 20;
cd(ROOT);
fid = fopen(fullfile(ROOT,'out','diag_diff.log'),'w','n','UTF-8');
cobj = onCleanup(@() fclose(fid)); %#ok<NASGU>
warning('off','all');

Sv = load(fullfile(ROOT,'rovmodel75kg.mat'));
fn = fieldnames(Sv);
for ii = 1:numel(fn), assignin('base', fn{ii}, Sv.(fn{ii})); end
assignin('base','M_nom',Sv.M); assignin('base','Ma_nom',Sv.Ma);
assignin('base','DL_nom',Sv.DL);
assignin('base','CTRL_DISCRETE',1);
assignin('base','TS_CTRL',0.01);

if bdIsLoaded(MODEL), close_system(MODEL,0); end
load_system(MODEL);
set_param(MODEL,'SolverType','Fixed-step','Solver','ode4','FixedStep','0.01', ...
               'StopTime',num2str(TEND));

hCD    = get_param([MODEL '/Controller_D'],'Handle');
allb   = find_system(hCD,'LookUnderMasks','all','FollowLinks','on','Type','block');
% 先把控制器内所有块采样时间锁 0.01
for j = 1:numel(allb)
    try, set_param(allb(j),'SampleTime','0.01'); catch, end
end

hD1 = get_param([MODEL '/Controller_D/Derivative'],  'Handle');
hD2 = get_param([MODEL '/Controller_D/Derivative1'], 'Handle');
fprintf(fid,'Derivative  类型=%s  mask=%s\n', get_param(hD1,'BlockType'), ...
        strjoin(get_param(hD1,'MaskNames')', ', '));
fprintf(fid,'读取 gainval: Derivative=%s  Derivative1=%s\n', ...
        get_param(hD1,'gainval'), get_param(hD2,'gainval'));
fprintf(fid,'读取 ICPrevScaledInput: Derivative=%s  Derivative1=%s\n\n', ...
        get_param(hD1,'ICPrevScaledInput'), get_param(hD2,'ICPrevScaledInput'));

variants = {
%  名称                        gD   gD1
    'V0_基线(两个都开)',        1,   1;
    'V1_关掉Derivative',        0,   1;
    'V2_关掉Derivative1',       1,   0;
    'V3_两个都关',              0,   0;
    'V4_基线再跑一遍(可复现)',   1,   1;
};

hdr = '%-24s %-10s %-13s %-13s %-11s %s\n';
fprintf(fid, hdr, '变体','结果','eta max','yaw 末值','失败时刻','说明');
fprintf(1,    hdr, '变体','结果','eta max','yaw 末值','失败时刻','说明');

for v = 1:size(variants,1)
    nm = variants{v,1};
    try, set_param(hD1,'gainval', num2str(variants{v,2})); catch e
        fprintf(fid,'[ERR] 设 gainval(D): %s\n', e.message); end
    try, set_param(hD2,'gainval', num2str(variants{v,3})); catch e
        fprintf(fid,'[ERR] 设 gainval(D1): %s\n', e.message); end

    [verdict, etaMax, uMax, tfail, msg, yawEnd] = runone(MODEL); %#ok<ASGLU>
    fprintf(fid, '%-24s %-10s %-13.4g %-13.4g %-11.4g %s\n', nm, verdict, etaMax, yawEnd, tfail, msg);
    fprintf(1,    '%-24s %-10s %-13.4g %-13.4g %-11.4g %s\n', nm, verdict, etaMax, yawEnd, tfail, msg);
end

fprintf(fid,'\nDONE\n');
fprintf(1,'DONE\n');

% ================= 局部函数 =================
function [verdict, etaMax, uMax, tfail, msg, yawEnd] = runone(MODEL)
verdict='?'; etaMax=NaN; uMax=NaN; tfail=NaN; msg=''; yawEnd=NaN;
try
    so = sim(MODEL,'CaptureErrors','on');
catch e
    verdict='发散(报错)'; msg=e.message; if numel(msg)>50, msg=[msg(1:50) '...']; end; return
end
eta = gl(so,'eta_log');
if isempty(eta) || ~isstruct(eta) || ~isfield(eta,'signals')
    verdict='无数据'; msg='无 eta_log'; return
end
Y = double(eta.signals.values); T = double(eta.time);
if isempty(Y), verdict='无数据'; return; end
fin = isfinite(Y);
if any(fin(:)), etaMax = max(abs(Y(fin))); end
if size(Y,2) >= 6, yawEnd = Y(end,6); end
bad = find(~all(isfinite(Y),2),1,'first');
if isempty(bad)
    verdict='收敛'; msg='跑满 StopTime';
else
    verdict='发散'; tfail=T(min(bad,numel(T))); msg='出现非有限值';
end
u = gl(so,'u_cmd');
if isstruct(u) && isfield(u,'signals')
    U = double(u.signals.values); f2 = isfinite(U);
    if any(f2(:)), uMax = max(abs(U(f2))); end
end
end

function v = gl(so, name)
v = [];
try, v = so.get(name); catch, end
if isempty(v), try, v = evalin('base', name); catch, end, end
end
