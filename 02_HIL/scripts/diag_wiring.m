%% diag_wiring.m -- 精确接线审计（不写模型）
%  目的：确认 rov_hil_loopback 送给 Controller_D 的 3 个信号
%        与 rov_ctrl_deploy/Controller 需要的 (eta_d, err, nu) 是否一致，
%        特别是误差信号 Sum? 的符号方向。
%  坑1：块名可能含首尾空格 / 换行，按名字取会报「Simulink 对象名称无效」
%       -> 一律用 find_system 拿句柄，打印时用 <...> 包住以暴露空白
%  坑2：find_system(名字) 返回 cell，find_system(句柄) 返回 double
%       -> 统一走 fsH() 归一化
clear; close all;
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
fid = fopen(fullfile(ROOT,'out','diag_wiring.log'),'w','n','UTF-8');
cobj = onCleanup(@() fclose(fid)); %#ok<NASGU>
warning('off','all');

Sv = load(fullfile(ROOT,'rovmodel75kg.mat'));
fn = fieldnames(Sv);
for ii = 1:numel(fn), assignin('base', fn{ii}, Sv.(fn{ii})); end
assignin('base','M_nom',Sv.M); assignin('base','Ma_nom',Sv.Ma);
assignin('base','DL_nom',Sv.DL);
addpath(fullfile(ROOT,'src'));

mods = {'rov_ctrl_deploy','rov_hil_loopback','nonlinear_closed_backstepping_adaptive_circle'};
for i = 1:numel(mods)
    m = mods{i};
    fprintf(fid,'\n\n################ %s ################\n', m);
    try
        if bdIsLoaded(m), close_system(m,0); end
        load_system(m);
        fprintf(fid,'[OK] 已载入\n');
    catch e
        fprintf(fid,'[ERR] 载入失败: %s\n', e.message); continue
    end

    try
        fprintf(fid,'\n--- 顶层 Sum / Mux / Demux 接线 ---\n');
        for bt = {'Sum','Mux','Demux'}
            ks = fsH(m, 'SearchDepth', 1, 'BlockType', bt{1});
            for k = 1:numel(ks)
                printSrc(fid, ks(k), bt{1});
            end
        end
    catch e
        fprintf(fid,'[ERR] 顶层扫描: %s\n', e.message);
    end

    try
        fprintf(fid,'\n--- 顶层 Inport / Outport ---\n');
        for bt = {'Inport','Outport'}
            ks = fsH(m, 'SearchDepth', 1, 'BlockType', bt{1});
            for k = 1:numel(ks)
                fprintf(fid,'  <%s> Port=%s dim=%s\n', get_param(ks(k),'Name'), ...
                        get_param(ks(k),'Port'), get_param(ks(k),'PortDimensions'));
            end
        end
    catch e
        fprintf(fid,'[ERR] 接口扫描: %s\n', e.message);
    end
end

fprintf(fid,'\n\n################ 子系统入口对照（顺序即端口号）################\n');
for i = 1:numel(mods)
    m = mods{i};
    try
        if ~bdIsLoaded(m), continue; end
        ks = fsH(m, 'SearchDepth', 1, 'BlockType', 'SubSystem');
        for k = 1:numel(ks)
            nm = get_param(ks(k), 'Name');
            if ~contains(nmz(nm), 'Controller'), continue; end
            fprintf(fid,'\n--- <%s>  (%s) ---\n', nm, m);
            reportInports(fid, ks(k));
        end
    catch e
        fprintf(fid,'[ERR] %s: %s\n', m, e.message);
    end
end

fprintf(fid,'\nDONE\n');
fprintf(1,   'DONE\n');

% ================= 局部函数（必须在脚本末尾）=================
function s = nmz(p)
s = strtrim(regexprep(p, '\s+', ' '));
end

function hs = fsH(m, varargin)
% 归一化 find_system 的返回：无论 cell 还是 double 都返回 double 句柄行向量
r = find_system(m, varargin{:});
if iscell(r)
    hs = zeros(1, numel(r));
    for k = 1:numel(r), hs(k) = get_param(r{k}, 'Handle'); end
else
    hs = double(r(:))';
    hs(hs <= 0) = [];
end
end

function reportInports(fid, subH)
try
    hs = find_system(subH, 'SearchDepth',1, 'BlockType','Inport');
    if isempty(hs), fprintf(fid,'  (无 Inport)\n'); return; end
    n = numel(hs); pv = zeros(1,n);
    for j = 1:n, pv(j) = str2double(get_param(hs(j),'Port')); end
    [~, ix] = sort(pv); hs = hs(ix);
    for j = 1:n
        fprintf(fid,'  Port %-3s  <%s>   dim=%s\n', get_param(hs(j),'Port'), ...
                get_param(hs(j),'Name'), get_param(hs(j),'PortDimensions'));
    end
catch e
    fprintf(fid,'  [WARN] reportInports: %s\n', e.message);
end
end

function printSrc(fid, h, bt)
nm = get_param(h,'Name');
extra = '';
try, extra = [' Inputs=' get_param(h,'Inputs')]; catch, end
try, extra = [extra ' Outputs=' get_param(h,'Outputs')]; catch, end
fprintf(fid,'\n  <%s>  type=%s%s\n', nm, bt, extra);
try
    ph = get_param(h,'PortHandles');
    for p = 1:numel(ph.Inport)
        fprintf(fid,'     in %d  <-  %s\n', p, srcStr(h, p));
    end
catch e
    fprintf(fid,'     [WARN] %s\n', e.message);
end
end

function s = srcStr(blkH, portIdx)
s = '(未连线)';
try
    ph = get_param(blkH,'PortHandles');
    if portIdx > numel(ph.Inport), s = '(无端口)'; return; end
    ln = get_param(ph.Inport(portIdx),'Line');
    if numel(ln) ~= 1 || ln <= 0, return; end
    sh = get_param(ln,'SrcBlockHandle');
    sp = get_param(ln,'SrcPortHandle');
    s = sprintf('<%s>:%s [%s]', get_param(sh,'Name'), ...
                get_param(sp,'PortNumber'), get_param(sh,'BlockType'));
    try
        if strcmp(get_param(blkH,'BlockType'),'Sum')
            sg = get_param(blkH,'Inputs');
            if numel(sg) >= portIdx, s = [sg(portIdx) ' ' s]; end
        end
    catch
    end
catch e
    s = ['(读连线失败: ' e.message ')'];
end
end
