%% build_codegen.m -- 阶段1b：部署模型的采样时间核验 + Embedded Coder 代码生成
%  产物: out/codegen/ 下的 C 源码
%  日志: out/build_codegen.log
clear;
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
DST  = 'rov_ctrl_deploy';

fid = fopen(fullfile(ROOT,'out','build_codegen.log'),'w','n','UTF-8');
cobj = onCleanup(@() fclose(fid)); %#ok<NASGU>
warning('off','all');

try
    cd(ROOT);
    load_system(DST);
    mH = get_param(DST,'Handle');

    % 保险：把参数 mat 全部灌进 base（PreLoadFcn 之外的双保险）
    if exist(fullfile(ROOT,'rovmodel75kg.mat'),'file')
        Sv = load(fullfile(ROOT,'rovmodel75kg.mat'));
        fn = fieldnames(Sv);
        for ii = 1:numel(fn)
            assignin('base', fn{ii}, Sv.(fn{ii}));
        end
        assignin('base','M_nom', Sv.M);
        assignin('base','Ma_nom', Sv.Ma);
        assignin('base','DL_nom', Sv.DL);
        L(fid, '[OK] 已载入参数: %s\n', strjoin(fn', ', '));
    else
        L(fid, '[ERR] 找不到 rovmodel75kg.mat\n');
    end

    % ---------- 1. 控制器内全部块（含掩码下）与采样时间 ----------
    L(fid, '=== 控制器内全部块（LookUnderMasks=all）===\n');
    ctrlH = get_param([DST '/Controller'], 'Handle');
    hs = find_system(ctrlH, 'LookUnderMasks','all', 'FollowLinks','on', 'Type','block');
    L(fid, '  块总数 = %d\n', numel(hs));
    for k = 1:numel(hs)
        bt = get_param(hs(k),'BlockType');
        en = get_param(hs(k),'Mask');
        s  = '?';
        try, s = mat2str(get_param(hs(k),'CompiledSampleTime')); catch, end
        if strcmp(bt,'SubSystem') || strcmp(en,'on')
            L(fid, '  [%-18s] %-42s Ts=%-10s mask=%s\n', bt, ...
                strrep(getfullname(hs(k)), [DST '/'], ''), s, en);
        end
    end
    L(fid, '\n--- 只列状态类块 ---\n');
    for k = 1:numel(hs)
        bt = get_param(hs(k),'BlockType');
        if any(strcmp(bt, {'DiscreteIntegrator','UnitDelay','DiscreteDerivative', ...
                           'Integrator','Derivative','Memory'}))
            s = '?';
            try, s = mat2str(get_param(hs(k),'CompiledSampleTime')); catch, end
            L(fid, '  [%-18s] %-42s Ts=%s\n', bt, ...
                strrep(getfullname(hs(k)), [DST '/'], ''), s);
        end
    end
    L(fid, '\n--- Discrete Derivative 内部结构 ---\n');
    for k = 1:numel(hs)
        if strcmp(get_param(hs(k),'BlockType'),'SubSystem') && ...
           contains(getfullname(hs(k)), 'Derivative')
            L(fid, '  子系统 %s\n', strrep(getfullname(hs(k)), [DST '/'], ''));
            in = find_system(hs(k), 'LookUnderMasks','all', 'FollowLinks','on', 'Type','block');
            for q = 1:numel(in)
                L(fid, '      [%-18s] %s\n', get_param(in(q),'BlockType'), get_param(in(q),'Name'));
            end
        end
    end

    % ---------- 2. 代码生成配置（逐项 try，报出到底哪项不合法）----------
    L(fid, '\n=== 代码生成配置 ===\n');
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
    % 部分安装下 ProdHWDeviceType 不会自动带出位宽，显式再设一遍，
    % 否则 rtwtypes.h 里 real_T 会回落成 double（软件浮点，慢 10 倍以上）
    bits = {'ProdBitPerChar',8; 'ProdBitPerShort',16; 'ProdBitPerInt',32; ...
            'ProdBitPerLong',32; 'ProdBitPerFloat',32; 'ProdBitPerDouble',64; ...
            'ProdBitPerPointer',32; 'ProdLongLongMode',1};
    for k = 1:size(bits,1)
        try
            set_param(cfg, bits{k,1}, num2str(bits{k,2}));
            L(fid, '  [OK]   %-24s = %s\n', bits{k,1}, num2str(bits{k,2}));
        catch e
            L(fid, '  [FAIL] %-24s : %s\n', bits{k,1}, e.message);
        end
    end
    for k = 1:size(opts,1)
        try
            set_param(cfg, opts{k,1}, opts{k,2});
            L(fid, '  [OK]   %-24s = %s\n', opts{k,1}, opts{k,2});
        catch e
            L(fid, '  [FAIL] %-24s = %s : %s\n', opts{k,1}, opts{k,2}, e.message);
        end
    end
    % real_T 说明：Embedded Coder 里 real_T 跟随「模型信号精度」。
    % 只要模型里还有 double 常数（本模型的大量 6x6 常数矩阵就是 double），
    % 整条链就会回落成 real_T=double。这里读回确认设置是否真的生效，
    % 并在日志里给出结论（Cortex-M4F 有硬件单精度 FPU，但 double 也能跑，
    % 只是走软件浮点；10 ms 周期内余量仍然充足）。
    for nmb = {'DefaultUnderspecifiedDataType','ProdBitPerFloat','ProdHWDeviceType'}
        v = '';
        try, v = num2str(get_param(cfg, nmb{1})); catch e, v = ['<err:' e.message '>']; end
        L(fid, '  读回 %-30s = %s\n', nmb{1}, v);
    end

    % ---------- 3. 生成代码 ----------
    L(fid, '\n=== 生成代码 ===\n');
    cgDir = fullfile(ROOT, 'out', 'codegen');
    if exist(cgDir,'dir'), rmdir(cgDir,'s'); end
    mkdir(cgDir);
    if exist(fullfile(ROOT,[DST '_ert_rtw']),'dir')
        rmdir(fullfile(ROOT,[DST '_ert_rtw']),'s');   % 强制全量重建
    end
    t0 = tic;
    try
        rtwbuild(DST);
        L(fid, '[OK] rtwbuild 完成，用时 %.1f s\n', toc(t0));
    catch e
        L(fid, '[ERR] rtwbuild: %s\n', e.message);
        L(fid, '      identifier: %s\n', e.identifier);
        if isprop(e,'cause') && ~isempty(e.cause)
            for kk = 1:numel(e.cause)
                L(fid, '      cause%d: %s\n', kk, e.cause{kk}.message);
                if isprop(e.cause{kk},'cause') && ~isempty(e.cause{kk}.cause)
                    for jj = 1:numel(e.cause{kk}.cause)
                        L(fid, '         sub%d: %s\n', jj, e.cause{kk}.cause{jj}.message);
                    end
                end
            end
        end
    end

    % ---------- 4. 收集产物 ----------
    L(fid, '\n=== 产物清单 ===\n');
    srcDir = fullfile(ROOT, [DST '_ert_rtw']);
    if exist(srcDir,'dir')
        d = dir(fullfile(srcDir, '*'));
        for k = 1:numel(d)
            if ~d(k).isdir
                L(fid, '  %-40s %8d B\n', d(k).name, d(k).bytes);
            end
        end
        copyfile(fullfile(srcDir,'*'), cgDir);
        L(fid, '[OK] 已复制到 %s\n', cgDir);
    else
        L(fid, '[WARN] 未找到目录 %s\n', srcDir);
    end

    % ---------- 5. 打印生成代码的外部接口 ----------
    L(fid, '\n=== 生成代码外部接口 ===\n');
    hFile = fullfile(srcDir, [DST '.h']);
    if exist(hFile,'file')
        L(fid, '%s\n', regexprep(fileread(hFile), '\r\n', '\n'));
    else
        L(fid, '[WARN] 未找到 %s\n', hFile);
    end

catch ME
    fprintf(fid, '\n!!! 未捕获错误: %s\n', ME.message);
    for kk = 1:numel(ME.stack)
        fprintf(fid, '    @ %s : line %d\n', ME.stack(kk).name, ME.stack(kk).line);
    end
end

fprintf('DONE\n');

function L(fid, varargin), fprintf(fid, varargin{:}); end
