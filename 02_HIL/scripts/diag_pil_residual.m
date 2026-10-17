%% diag_pil_residual.m -- 定位 C 代码与 Simulink 之间 5.5e-3 V 残差的来源
%
%  思路：把「两侧输入是否真的完全相同」这件事先钉死。
%   1. 导出部署模型的关键块参数（Sat 限幅 / Derivative1 / 离散积分器 IC）
%   2. 复现 run_pil_check 的输入构造，导出 sp（double）与 sp32（float32 量化）
%   3. 逐列统计 float32 量化前后的差异，确认量化确实发生了
clear; close all;
ROOT  = 'E:\yanjiusheng\wb_hil';
MODEL = 'rov_ctrl_deploy';
TS    = 0.01;
cd(ROOT);
fid = fopen(fullfile(ROOT,'out','diag_pil_residual.log'),'w','n','UTF-8');
cobj = onCleanup(@() fclose(fid)); %#ok<NASGU>
warning('off','all');

L = @(varargin) fprintf(fid, varargin{:});
P = @(varargin) fprint2(fid, varargin{:});

Sv = load(fullfile(ROOT,'rovmodel75kg.mat'));
fn = fieldnames(Sv);
for ii = 1:numel(fn), assignin('base', fn{ii}, Sv.(fn{ii})); end
assignin('base','M_nom',Sv.M); assignin('base','Ma_nom',Sv.Ma);
assignin('base','DL_nom',Sv.DL);

if bdIsLoaded(MODEL), close_system(MODEL,0); end
load_system(MODEL);

% ---------- 1. 关键块参数 ----------
P('\n===== 1. 部署模型关键块参数 =====\n');
dumpBlk(fid, MODEL, 'Sat');
dumpBlk(fid, MODEL, 'Controller/Derivative1');
dumpBlk(fid, MODEL, 'Controller/c1_hat');
dumpBlk(fid, MODEL, 'Controller/c2_hat');
dumpBlk(fid, MODEL, 'u');

% 离散导数子系统内部
L('\n--- <S4> Discrete Derivative 内部 ---\n');
hSub = find_system(MODEL, 'LookUnderMasks','all','FollowLinks','on', ...
                   'BlockType','SubSystem','Name','Derivative1');
if ~isempty(hSub)
    kids = find_system(hSub(1), 'LookUnderMasks','all','FollowLinks','on', ...
                       'SearchDepth',1,'Type','Block');
    for q = 1:numel(kids)
        try
            bt = get_param(kids(q),'BlockType');
            nm = get_param(kids(q),'Name');
            L('   [%s] %s', bt, nm);
            switch bt
                case 'SampleTimeMath'
                    L('  K=%s  SampleTime=%s', get_param(kids(q),'K'), ...
                      get_param(kids(q),'SampleTime'));
                case 'UnitDelay'
                    L('  IC=%s  SampleTime=%s', get_param(kids(q),'InitialCondition'), ...
                      get_param(kids(q),'SampleTime'));
            end
            L('\n');
        catch e
            L('   (读取失败 %s)\n', e.message);
        end
    end
end

% ---------- 2. 输入构造（与 run_pil_check 完全一致）----------
P('\n===== 2. 输入构造与 float32 量化 =====\n');
M = readmatrix(fullfile(ROOT,'out','hosttest_sp.csv'));
M = M(~any(~isfinite(M),2), :);
M = sortrows(M, 1);
[~, ia] = unique(M(:,1)); M = M(ia,:);
tEnd = M(end,1); nCol = size(M,2); nSig = nCol-1;
tg   = (0:TS:tEnd)';
sp   = zeros(numel(tg), nCol); sp(:,1) = tg;
for c = 1:nSig
    sp(:,1+c) = interp1(M(:,1), M(:,1+c), tg, 'linear', 'extrap');
end
sp(~isfinite(sp)) = 0;
% ⚠ 只量化信号列，时间列必须保持 double —— 否则 Simulink 会按被扭曲的
%    时间轴做线性插值，凭空注入 ~1e-4 的输入扰动（详见 run_pil_check.m 注释）
sp32 = sp;
sp32(:,2:end) = double(single(sp(:,2:end)));
writematrix(sp32, fullfile(ROOT,'out','hosttest_sp_f32.csv'));

nmCol = {'t','eta1','eta2','eta3','eta4','eta5','eta6', ...
         'nu1','nu2','nu3','nu4','nu5','nu6', ...
         'eta_d1','eta_d2','eta_d3','eta_d4','eta_d5','eta_d6', ...
         'nu_d1','nu_d2','nu_d3','nu_d4','nu_d5','nu_d6'};
L('%-8s %14s %14s %14s\n','列','max|原始|','max|量化误差|','相对');
for c = 1:nCol
    if c == 1
        % 时间列不做 float32 量化：列出来是为了留证（一旦被量化就会出现 1e-6 级偏移）
        L('%-8s %14.6g %14s %14s\n', nmCol{c}, max(abs(sp(:,1))), '(不量化)', '-');
        continue
    end
    d = abs(sp(:,c) - sp32(:,c));
    s = max(abs(sp(:,c)));
    L('%-8s %14.6g %14.6g %14.3e\n', nmCol{c}, s, max(d), ...
      max(d)/max(s,eps));
end

% 复跑：double 输入
set_param(MODEL,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep',num2str(TS),'StopTime',num2str(tEnd), ...
    'LoadExternalInput','on','ExternalInput','sp_data');
assignin('base','sp_data', sp);
soA = sim(MODEL); [TA,UA] = pullY(soA);
assignin('base','sp_data', sp32);
soB = sim(MODEL); [TB,UB] = pullY(soB);
if ~isempty(TA) && ~isempty(TB)
    n = min([numel(TA), numel(TB), size(UA,1), size(UB,1)]);
    dU = UA(1:n,:) - UB(1:n,:);
    P('[OK] double vs float32 输入，n=%d，max|du| = %.6e V\n', n, mx(dU));
else
    P('[ERR] 复跑取不到输出\n');
end
writematrix([TB(1:numel(TB)), UB], fullfile(ROOT,'out','diag_sim_f32.csv'));

close_system(MODEL,0);
P('\nDONE\n');

% ================= 局部函数 =================
function fprint2(fid, varargin)
fprintf(fid, varargin{:});
fprintf(1,    varargin{:});
end

function dumpBlk(fid, MODEL, name)
fprintf(fid, '\n--- %s ---\n', name);
try
    h = get_param([MODEL '/' name], 'Handle');
catch e
    fprintf(fid, '  找不到: %s\n', e.message); return
end
fprintf(fid, '  BlockType = %s\n', get_param(h,'BlockType'));
p = get_param(h, 'DialogParameters');
if isstruct(p)
    f = fieldnames(p);
    for k = 1:numel(f)
        try
            v = get_param(h, f{k});
            if ischar(v)
                fprintf(fid, '    %s = %s\n', f{k}, v);
            elseif isnumeric(v) && numel(v) <= 8
                fprintf(fid, '    %s = %s\n', f{k}, mat2str(v));
            end
        catch
        end
    end
end
end

function [T,U] = pullY(so)
T = []; U = [];
try
    v = so.yout;
    if ~isempty(v)
        for k = 1:v.numElements
            e = v.getElement(k);
            if isa(e,'Simulink.SimulationData.Signal')
                ts = e.Values;
                T = double(ts.Time); U = squeeze(double(ts.Data));
                if size(U,1) ~= numel(T) && size(U,2) == numel(T), U = U.'; end
                if size(U,1) == numel(T), return; end
            end
        end
    end
catch
end
T = []; U = [];
end

function y = mx(x), x = x(isfinite(x)); if isempty(x), y = NaN; else, y = max(abs(x)); end, end
