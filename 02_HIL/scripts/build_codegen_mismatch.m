%% build_codegen_mismatch.m -- 控制器侧模型参数 +60% 的固件代码生成
%  复现论文 5.5 节失配定义: 控制器持有的 M / M_A / D_L 同乘 1.6,
%  被控对象不动 (PC 侧宿主模型运行时用标称值).
%  产物: HIL_board_kit/rov_ctrl_deploy_ert_rtw (构建树直接引用的源)
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
DST  = 'rov_ctrl_deploy';
SC   = 1.6;                    % over-estimation factor
cd(ROOT);
warning('off','all');
% --- 载入标称参数 ---
S = load(fullfile(ROOT,'rovmodel75kg.mat'));
fn = fieldnames(S);
for k = 1:numel(fn), assignin('base', fn{k}, S.(fn{k})); end
assignin('base','M_nom', S.M); assignin('base','Ma_nom', S.Ma);
assignin('base','DL_nom', S.DL);

% --- 内存中覆写 PreLoadFcn: 载入后立即施加缩放, 防 rtwbuild 打开模型时重置 ---
pre = ['S=load(''rovmodel75kg.mat''); ' ...
       'fn=fieldnames(S); for k=1:numel(fn), assignin(''base'',fn{k},S.(fn{k})); end; ' ...
       'M_nom=S.M; Ma_nom=S.Ma; DL_nom=S.DL; clear S fn k; ' ...
       'SC=' num2str(SC) '; v={''m'',''I'',''Ma'',''MRB'',''M'',''DL''}; ' ...
       'for q=1:numel(v), if evalin(''base'',[''exist('''' '' v{q} '''''')''])>0, ' ...
       'assignin(''base'',v{q},SC*evalin(''base'',v{q})); end; end; clear SC v q;'];
if bdIsLoaded(DST), close_system(DST, 0); end
load_system(DST);
set_param(DST, 'PreLoadFcn', pre);

% --- base 工作区直接缩放 (双保险) ---
vs = {'m','I','Ma','MRB','M','DL'};
for q = 1:numel(vs)
    if evalin('base', ['exist(''' vs{q} ''')']) > 0
        oldv = evalin('base', vs{q});
        assignin('base', vs{q}, SC * oldv);
        fprintf('[SCALE] %-4s x%.2f  (norm=%.4g)\n', vs{q}, SC, norm(oldv(:)));
    else
        fprintf('[SCALE] %-4s 不存在, 跳过\n', vs{q});
    end
end

% --- 代码生成配置 (与 build_codegen.m 相同) ---
mH = get_param(DST,'Handle');
cfg = getActiveConfigSet(mH);
opts = {
    'SystemTargetFile',      'ert.tlc';
    'TargetLang',            'C';
    'GenCodeOnly',           'on';
    'GenerateSampleERTMain', 'off';
    'CodeInterfacePackaging','Nonreusable';
    'RootIOFormat',          'Part of model data structure';
    'GenerateComments',      'on';
    'MatFileLogging',        'off';
    'SupportNonFinite',      'on';
    'EnableMemcpy',          'on';
    'MemcpyThreshold',       '8';
    'TargetHWDeviceType',    'ARM Compatible->ARM Cortex-M';
    'ProdHWDeviceType',      'ARM Compatible->ARM Cortex-M';
    'DefaultUnderspecifiedDataType', 'single';
    };
bits = {'ProdBitPerChar',8; 'ProdBitPerShort',16; 'ProdBitPerInt',32; ...
        'ProdBitPerLong',32; 'ProdBitPerFloat',32; 'ProdBitPerDouble',64; ...
        'ProdBitPerPointer',32; 'ProdLongLongMode',1};
for k = 1:size(bits,1)
    try, set_param(cfg, bits{k,1}, num2str(bits{k,2})); catch, end
end
for k = 1:size(opts,1)
    try, set_param(cfg, opts{k,1}, opts{k,2}); catch e, fprintf('[CFG FAIL] %s: %s\n', opts{k,1}, e.message); end
end

% --- 构建目录重定向: pwd 不能包含旧构建文件夹 (pwdContainsBuildDir), 移到中性目录 ---
cgDir = fullfile(tempdir, 'codegen_mismatch_build');
if exist(cgDir, 'dir'), rmdir(cgDir, 's'); end
Simulink.fileGenControl('set', ...
    'CacheFolder',    fullfile(cgDir, 'slprj'), ...
    'CodeGenFolder',  cgDir, ...
    'createDir', true);
cd(tempdir);     % 中性 pwd, 不含任何构建文件夹
fprintf('[CFG] pwd=%s  CodeGenFolder -> %s\n', pwd, cgDir);

if exist(fullfile(ROOT,[DST '_ert_rtw']),'dir')
    try
        rmdir(fullfile(ROOT,[DST '_ert_rtw']),'s');   % 强制全量重建
    catch
        fprintf('[WARN] 旧 ert_rtw 目录清理失败, 继续覆盖式重建\n');
    end
end
t0 = tic;
try
    rtwbuild(DST);
    fprintf('[OK] rtwbuild 用时 %.1f s\n', toc(t0));
catch e
    fprintf('[ERR] rtwbuild: %s\n', e.message);
    for kk = 1:numel(e.stack)
        fprintf('    @ %s : %d\n', e.stack(kk).name, e.stack(kk).line);
    end
    if isprop(e,'cause')
        for kk = 1:numel(e.cause)
            fprintf('  cause: %s\n', e.cause(kk).message);
        end
    end
    close_system(DST, 0);
    error('codegen failed');
end

% --- 校验: 生成代码中应出现缩放后的常数 ---
src = fullfile(cgDir, [DST '_ert_rtw'], [DST '.c']);
if exist(src, 'file')
    copyfile(fullfile(cgDir, [DST '_ert_rtw'], '*'), fullfile(ROOT, [DST '_ert_rtw']));
    fprintf('[OK] 生成代码已覆盖 %s\n', fullfile(ROOT, [DST '_ert_rtw']));
else
    src = fullfile(ROOT, [DST '_ert_rtw'], [DST '.c']);   % 兜底: 直接落在模型目录
    fprintf('[WARN] codegen 目录下未找到, 尝试模型目录\n');
end
txt = fileread(src);
fprintf('[CHECK] 生成代码含 1.6x 质量(120): %d\n', contains(txt, '120'));

close_system(DST, 0);
fprintf('CODEGEN_DONE\n');
