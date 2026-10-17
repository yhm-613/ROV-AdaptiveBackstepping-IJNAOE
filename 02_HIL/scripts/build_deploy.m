%% build_deploy.m  -- 阶段1：构建板上部署模型的骨架
%  日志: out/build_deploy.log
clear;
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根   % 不能叫 W：mat 里自带变量 W，assignin 会把它覆盖掉
SRC  = 'nonlinear_closed_backstepping_adaptive_circle';
DST  = 'rov_ctrl_deploy';

fid = fopen(fullfile(ROOT,'out','build_deploy.log'),'w','n','UTF-8');
cobj = onCleanup(@() fclose(fid)); %#ok<NASGU>
warning('off','all');

try
    cd(fullfile(ROOT,'src'));
    load_system(SRC);
    srcH = get_param(SRC, 'Handle');

    Sv = load('rovmodel75kg.mat');
    fn = fieldnames(Sv);
    for ii = 1:numel(fn)
        assignin('base', fn{ii}, Sv.(fn{ii}));
    end
    L(fid, '已载入变量: %s\n', strjoin(fn', ', '));

    % ---- 诊断：顶层块清单 ----
    L(fid, '\n=== 顶层块清单 ===\n');
    kb = find_system(srcH, 'SearchDepth', 1, 'Type', 'block');
    L(fid, '  顶层块总数 = %d\n', numel(kb));
    nmList = get_param(kb, 'Name');
    btList = get_param(kb, 'BlockType');
    for k = 1:numel(kb)
        L(fid, '   %-16s [%s]\n', btList{k}, nmz(nmList{k}));
    end

    % ================= 建模型 =================
    L(fid, '\n=== 阶段1: 骨架 ===\n');
    if bdIsLoaded(DST); close_system(DST, 0); end
    new_system(DST);
    load_system(DST);

    % 注意：需要「路径字符串」的场景必须用模型名调用 find_system（返回 cell）
    subs  = find_system(SRC, 'SearchDepth', 1, 'BlockType', 'SubSystem');
    pCtrl = pick(subs, 'Backstepping Controller');
    pKin  = pick(subs, 'Euler');
    pCor  = pick(subs, 'CRB+CA');
    L(fid, '源子系统: ctrl=%s | kin=%s | cor=%s\n', pCtrl, pKin, pCor);

    cp(fid, pCtrl, [DST '/Controller'], [430 120 640 380]);
    cp(fid, pKin,  [DST '/Kinematics'], [70 100 220 300]);
    cp(fid, pCor,  [DST '/Coriolis'],   [70 400 220 600]);

    kinH  = get_param([DST '/Kinematics'], 'Handle');
    ctrlH = get_param([DST '/Controller'], 'Handle');

    % ---- 控制器剔除观测块 ----
    for bt = {'Scope','ToWorkspace'}
        hs = find_system(ctrlH, 'SearchDepth', 1, 'BlockType', bt{1});
        for k = 1:numel(hs)
            nm = nmz(get_param(hs(k), 'Name'));
            try, delete_block(hs(k)); L(fid, '[OK] 删 %s\n', nm); catch e, L(fid, '[FAIL] %s\n', e.message); end
        end
    end

    % ---- Kinematics 改造 ----
    L(fid, '\n--- Kinematics 改造 ---\n');
    for t = {'1/s','Product1','Saturation1','Terminator'}
        h = findByName(kinH, 'SubSystem',  t{1});
        if isempty(h); h = findByName(kinH, 'Product',    t{1}); end
        if isempty(h); h = findByName(kinH, 'Saturate',   t{1}); end
        if isempty(h); h = findByName(kinH, 'Terminator', t{1}); end
        if isempty(h)
            L(fid, '[SKIP] 未找到 %s\n', t{1});
        else
            try, delete_block(h); L(fid, '[OK] 删块 %s\n', t{1}); catch e, L(fid, '[FAIL] %s : %s\n', t{1}, e.message); end
        end
    end
    ops = find_system(kinH, 'SearchDepth', 1, 'BlockType', 'Outport');
    for k = 1:numel(ops)
        nm = nmz(get_param(ops(k), 'Name'));
        try, delete_block(ops(k)); L(fid, '[OK] 删 Outport [%s]\n', nm); catch e, L(fid, '[FAIL] %s\n', e.message); end
    end
    hVel = findByName(kinH, 'Inport', 'vel');
    if ~isempty(hVel)
        try, set_param(hVel, 'Name', 'eta'); L(fid, '[OK] Inport vel -> eta\n'); catch e, L(fid, '[FAIL] 改名: %s\n', e.message); end
    end
    hDmx = findByName(kinH, 'Demux', 'Demux1');
    try
        ph = get_param(hDmx, 'PortHandles');
        ln = get_param(ph.Inport(1), 'Line');
        while numel(ln) == 1 && ln > 0
            delete_line(ln);
            ln = get_param(ph.Inport(1), 'Line');
        end
        L(fid, '[OK] 清空 Demux1 输入端口\n');
    catch e
        L(fid, '[WARN] 清线: %s\n', e.message);
    end
    try
        add_line(kinH, 'eta/1', 'Demux1/1', 'autorouting', 'on');
        L(fid, '[OK] 连线 eta -> Demux1[1]\n');
    catch e
        L(fid, '[FAIL] 连线: %s\n', e.message);
    end

    % ---- 顶层接口 ----
    L(fid, '\n--- 顶层接口 ---\n');
    mk(fid, 'built-in/Inport',  [DST '/eta'],   {'Port','1','Position',[10 140 40 160]});
    mk(fid, 'built-in/Inport',  [DST '/nu'],    {'Port','2','Position',[10 220 40 240]});
    mk(fid, 'built-in/Inport',  [DST '/eta_d'], {'Port','3','Position',[10 300 40 320]});
    mk(fid, 'built-in/Inport',  [DST '/nu_d'],  {'Port','4','Position',[10 380 40 400]});
    mk(fid, 'built-in/Outport', [DST '/u'],     {'Port','1','Position',[980 220 1010 240]});
    % ⚠ 误差符号（重要，勿改）：
    %   原模型 nonlinear_closed_backstepping_adaptive_circle 中
    %       Sum2 (Inputs='-+') : in1 = -Mux(=η_d),  in2 = +Kinematics(=η)
    %       Sum2 -> Backstepping Controller[2] (Inport 'err')
    %   => 控制器内部约定的位置误差是  e1 = η − η_d （实测值减期望值）。
    %   内部用法可交叉验证：alpha1 = d(yd)/dt + c1_hat*err，而 c1_hat(0) = -3，
    %   故 alpha1 = η̇_d − 3·e1；若把 err 取成 η_d − η，矫正项整项变号。
    %   这里必须与原模型保持完全一致：eta_d 接负端，eta 接正端。
    mk(fid, 'built-in/Sum',     [DST '/e_calc'],{'Inputs','-+','Position',[300 290 330 330]});
    for nmb = {'eta','nu','eta_d','nu_d'}
        try
            set_param([DST '/' nmb{1}], 'PortDimensions', '6');
        catch e
            L(fid, '[FAIL] 设维度 %s : %s\n', nmb{1}, e.message);
        end
    end
    % 新增的入口默认是 'Inherit: auto'，在 DefaultUnderspecifiedDataType='single'
    % 的代码生成配置下会被推成 real32_T(float)，与其它入口的 real_T(double) 不一致
    % （实测 rov_ctrl_deploy.h 里出现过 real32_T nu_d[6]）。显式钉成 double。
    for nmb = {'eta','nu','eta_d','nu_d'}
        try
            set_param([DST '/' nmb{1}], 'OutDataTypeStr', 'double');
        catch e
            L(fid, '[WARN] 设类型 %s : %s\n', nmb{1}, e.message);
        end
    end
    % ---- 控制器接口显式化：From9(tag=V) -> Inport nu ----
    L(fid, '\n--- 控制器接口显式化 ---\n');
    try
        hF9 = findByName(ctrlH, 'From', 'From9');
        if ~isempty(hF9)
            delete_block(hF9);
            L(fid, '[OK] 删除 From9 (tag=V)\n');
        else
            L(fid, '[SKIP] 未找到 From9\n');
        end
        % 删块后 Product6 输入端口 2 上可能仍残留线段，必须先清干净
        nClear = clearPortIn(ctrlH, 'Product6', 2, fid);
        hNU = add_block('built-in/Inport', [DST '/Controller/nu'], ...
            'Port', '3', 'Position', [20 260 50 280]);
        set_param(hNU, 'PortDimensions', '6', 'OutDataTypeStr', 'double');
        add_line(ctrlH, 'nu/1', 'Product6/2', 'autorouting', 'on');
        L(fid, '[OK] Controller 新增 Inport nu -> Product6[2] (清线 %d 条)\n', nClear);
    catch e
        L(fid, '[FAIL] 控制器接口显式化: %s\n', e.message);
    end

    % ---- 参考速度显式化：删除对 eta_d 的数值微分，改用显式 nu_d 输入 ----
    % 背景（实测判决，见 out/diag_diff.log 的 V0..V4）：
    %   原模型用连续 Derivative 块对参考 yd 求导得到 eta_d_dot。建成离散
    %   (u[k]-u[k-1])/Ts 后，参考里的 Step 跳变会被差分器放大成 |Δ|/Ts 的尖峰
    %   （Ts=10 ms 时 200 倍），闭环在阶跃时刻附近发散；且该现象与 Ts 无关，
    %   关掉这一个差分器即恢复（eta max 2.687，与连续控制器 2.605 相当）。
    %   参考速度本来就是解析已知量（PC 端生成轨迹时顺手就有），协议里也早已
    %   预留 nu_d[6] 字段，所以正确做法是把它当显式输入送进来，而不是数值微分。
    L(fid, '\n--- 参考速度显式化（Derivative on eta_d -> Inport nu_d）---\n');
    try
        hDer = findByName(ctrlH, 'Derivative', 'Derivative');
        dest = [];
        if ~isempty(hDer)
            ph = get_param(hDer, 'PortHandles');
            ln = get_param(ph.Outport(1), 'Line');
            if numel(ln) == 1 && ln > 0
                dest = get_param(ln, 'DstPortHandle');
                delete_line(ln);      % 先切断！否则目标端口仍被残余线段占用
            end
            delete_block(hDer);
            L(fid, '[OK] 删除 Controller/Derivative（不再对 eta_d 数值微分）\n');
        else
            L(fid, '[SKIP] 未找到 Controller/Derivative\n');
        end
        hNUD = add_block('built-in/Inport', [DST '/Controller/nu_d'], ...
            'Port', '4', 'Position', [20 380 50 400]);
        set_param(hNUD, 'PortDimensions', '6', 'OutDataTypeStr', 'double');
        if ~isempty(dest) && dest(1) > 0
            add_line(ctrlH, get_param(hNUD,'PortHandles').Outport(1), dest(1), ...
                     'autorouting','on');
            L(fid, '[OK] Controller/nu_d[4] -> 原 Derivative 的下游端口\n');
        else
            L(fid, '[WARN] 未捕获 Derivative 的下游端口\n');
        end
    catch e
        L(fid, '[FAIL] 参考速度显式化: %s\n', e.message);
    end

    % ---- Goto/From 作用域自检 ----
    % 注意：find_system 传「模型名字符串」返回 cell，必须用 hg{k} 取值
    L(fid, '\n--- Goto/From 作用域自检 ---\n');
    hg = find_system(DST, 'LookUnderMasks','all', 'BlockType','Goto');
    for k = 1:numel(hg)
        hk = hg{k};
        L(fid, '  Goto %-52s tag=%-8s vis=%s\n', ...
            strrep(getfullname(hk), [DST '/'], ''), ...
            get_param(hk, 'GotoTag'), get_param(hk, 'TagVisibility'));
    end
    hf = find_system(DST, 'LookUnderMasks','all', 'BlockType','From');
    for k = 1:numel(hf)
        hk = hf{k};
        L(fid, '  From %-52s tag=%s\n', ...
            strrep(getfullname(hk), [DST '/'], ''), get_param(hk, 'GotoTag'));
    end

    L(fid, '[OK] 顶层 Inport 维度设为 6\n');
    % 强制控制器端口维度（同时利于代码生成）
    try
        ipsC = find_system(ctrlH, 'SearchDepth', 1, 'BlockType', 'Inport');
        for j = 1:numel(ipsC), set_param(ipsC(j), 'PortDimensions', '6'); end
        opsC = find_system(ctrlH, 'SearchDepth', 1, 'BlockType', 'Outport');
        for j = 1:numel(opsC), set_param(opsC(j), 'PortDimensions', '6'); end
        set_param([DST '/u'], 'PortDimensions', '6');
        L(fid, '[OK] Controller 端口维度设为 6（In %d / Out %d）\n', numel(ipsC), numel(opsC));
    catch e
        L(fid, '[FAIL] 设控制器端口维度: %s\n', e.message);
    end

    % ---- 推力分配链：取数值后重建（避开含换行的块名）----
    L(fid, '\n--- 推力分配链 ---\n');
    hInvT = []; hTmat = []; hSat = [];
    for k = 1:numel(kb)
        if strcmp(btList{k}, 'Gain')
            if contains(nmz(nmList{k}), 'Inverse');        hInvT = kb(k);
            elseif contains(nmz(nmList{k}), 'Thruster');   hTmat = kb(k);
            end
        elseif strcmp(btList{k}, 'Saturate')
            hSat = kb(k);
        end
    end
    L(fid, '  命中 InvT=%d Tmat=%d Sat=%d\n', ~isempty(hInvT), ~isempty(hTmat), ~isempty(hSat));
    Pval = eval(get_param(hInvT, 'Gain'));
    Tval = eval(get_param(hTmat, 'Gain'));
    L(fid, '[OK] pinv(T) %dx%d, T %dx%d\n', size(Pval,1), size(Pval,2), size(Tval,1), size(Tval,2));
    mk(fid, 'built-in/Gain', [DST '/InvT'], {'Position', [680 200 730 240]});
    mk(fid, 'built-in/Gain', [DST '/Tmat'], {'Position', [750 200 800 240]});
    mk(fid, 'built-in/Saturate', [DST '/Sat'], ...
        {'UpperLimit', get_param(hSat,'UpperLimit'), 'LowerLimit', get_param(hSat,'LowerLimit'), ...
         'Position', [820 200 870 240]});
    % 单独赋值（add_block 传矩阵增益会丢），并复制 Multiplication 模式
    mulInv = get_param(hInvT, 'Multiplication');
    mulT   = get_param(hTmat, 'Multiplication');
    L(fid, '  原块 Multiplication: InvT=%s Tmat=%s\n', mulInv, mulT);
    set_param([DST '/InvT'], 'Gain', mat2str(Pval, 10), 'Multiplication', mulInv);
    set_param([DST '/Tmat'], 'Gain', mat2str(Tval, 10), 'Multiplication', mulT);
    L(fid, '[OK] InvT.Gain 字符数=%d | Tmat.Gain 字符数=%d\n', ...
        numel(get_param([DST '/InvT'],'Gain')), numel(get_param([DST '/Tmat'],'Gain')));

    % ---- 顶层连线 ----
    L(fid, '\n--- 顶层连线 ---\n');
    wd(fid, DST, 'eta/1',       'Kinematics/1');
    wd(fid, DST, 'nu/1',        'Coriolis/1');
    wd(fid, DST, 'nu/1',        'Controller/3');
    wd(fid, DST, 'eta_d/1',     'e_calc/1');
    wd(fid, DST, 'eta/1',       'e_calc/2');
    wd(fid, DST, 'eta_d/1',     'Controller/1');
    wd(fid, DST, 'nu_d/1',      'Controller/4');
    wd(fid, DST, 'e_calc/1',    'Controller/2');
    wd(fid, DST, 'Controller/1','InvT/1');
    wd(fid, DST, 'InvT/1',      'Tmat/1');
    wd(fid, DST, 'Tmat/1',      'Sat/1');
    wd(fid, DST, 'Sat/1',       'u/1');

    % ---- 误差信号符号对账（防止 e_calc 符号被改错）----
    % 拿原模型里真正喂给 Backstepping Controller/2 (err) 的那个 Sum 做基准，
    % 与新建的 e_calc 逐符号比对。这两者语义必须完全一致。
    L(fid, '\n--- 误差信号符号对账 ---\n');
    try
        scrBlk = srcBlockHandle([SRC '/Backstepping Controller'], 2);
        if isempty(scrBlk)
            L(fid, '[WARN] 原模型 Controller/2 未接线，无法对账\n');
        else
            L(fid, '  原模型 上游块 = %s [%s]  Inputs=%s\n', ...
                get_param(scrBlk,'Name'), get_param(scrBlk,'BlockType'), ...
                get_param(scrBlk,'Inputs'));
            refSign = get_param(scrBlk,'Inputs');
            newSign = get_param([DST '/e_calc'], 'Inputs');
            L(fid, '  新建 e_calc                        Inputs=%s\n', newSign);
            if strcmp(refSign, newSign)
                L(fid, '[OK] 误差符号一致（e1 = eta - eta_d）\n');
            else
                L(fid, '[FAIL] 误差符号不一致! 原=%s 新=%s\n', refSign, newSign);
            end
        end
    catch e
        L(fid, '[WARN] 误差符号对账失败: %s\n', e.message);
    end

    % ---- 模型配置 ----
    % PreLoadFcn 必须自包含：把 mat 全部变量灌进 base，模型里的 Gain/Constant
    % 直接引用 I / m / T / DL 等符号，代码生成时才能求值。
    preload = [ ...
        'try, S=load(''rovmodel75kg.mat''); ' ...
        'catch, S=load(''E:\yanjiusheng\wb_hil\rovmodel75kg.mat''); end; ' ...
        'fn=fieldnames(S); for k=1:numel(fn), assignin(''base'',fn{k},S.(fn{k})); end; ' ...
        'M_nom=S.M; Ma_nom=S.Ma; DL_nom=S.DL; clear S fn k;'];
    set_param(DST, 'PreLoadFcn', preload);
    set_param(DST, 'SolverType','Fixed-step', 'Solver','FixedStepDiscrete', ...
                   'FixedStep','0.01', 'StartTime','0', 'StopTime','100');
    set_param(DST, 'SystemTargetFile','ert.tlc', 'TargetLang','C');
    L(fid, '\n[OK] 模型配置完成（PreLoadFcn 已自包含）\n');

    % ---- 编译检查 ----
    L(fid, '\n=== 编译检查 ===\n');
    try
        set_param(DST, 'SimulationCommand', 'update');
        L(fid, '[OK] Update Diagram 通过\n');
    catch e
        L(fid, '[ERR] Update Diagram: %s\n', e.message);
    end

    % ---- 阶段2：离散化（消除连续状态）----
    L(fid, '\n=== 阶段2: 离散化 ===\n');
    ctrlP = [DST '/Controller'];
    load_system('simulink');
    hCdd = find_system('simulink', 'SearchDepth', 2, 'Name', 'Discrete Derivative');
    L(fid, '  库中 Discrete Derivative 块: %d 个\n', numel(hCdd));
    if ~isempty(hCdd); L(fid, '  路径 = %s\n', getfullname(hCdd{1})); end
    for nmb = {'c1_hat','c2_hat'}
        try
            h  = get_param([ctrlP '/' nmb{1}], 'Handle');
            ic = get_param(h, 'InitialCondition');
            replace_block(ctrlP, 'Name', nmb{1}, 'built-in/DiscreteIntegrator', 'noprompt');
            h2 = get_param([ctrlP '/' nmb{1}], 'Handle');
            % 显式写 0.01（而不是 -1 继承）：这样在变步长模型里也能被识别为
            % 周期采样，回环对照仿真才能与定步长配置共存
            set_param(h2, 'InitialCondition', ic, 'SampleTime', '0.01');
            L(fid, '[OK] %s: Integrator -> DiscreteIntegrator (IC=%s, Ts=0.01)\n', nmb{1}, ic);
        catch e
            L(fid, '[FAIL] %s : %s\n', nmb{1}, e.message);
        end
    end
    % 只离散 Derivative1（对 alpha1 求导，属回路内必需的前馈项，实测必须保留）。
    % 对 eta_d 求导的 Derivative 已在上面删除，改由显式 nu_d 输入提供参考速度。
    for nmb = {'Derivative1'}
        try
            if isempty(hCdd); error('库中未找到 Discrete Derivative'); end
            replace_block(ctrlP, 'Name', nmb{1}, getfullname(hCdd{1}), 'noprompt');
            h2 = get_param([ctrlP '/' nmb{1}], 'Handle');
            mnames = get_param(h2, 'MaskNames');
            L(fid, '[OK] %s -> Discrete Derivative; 参数=[%s]\n', nmb{1}, strjoin(mnames, ', '));
            for j = 1:numel(mnames)
                mj = mnames{j};
                if any(strcmpi(mj, {'ts','SampleTime','sampletime'}))
                    try, set_param(h2, mj, '-1'); catch, end
                elseif any(strcmpi(mj, {'gainval','K','Gain','gain'}))
                    try, set_param(h2, mj, '1'); catch, end
                end
            end
            % 掩码未暴露采样时间 -> 直接设内部 UnitDelay，使 Ts = 0.01
            for bt = {'UnitDelay','SampleTimeMath'}
                hd = find_system(h2, 'LookUnderMasks','all','FollowLinks','on', ...
                                     'BlockType', bt{1});
                for q = 1:numel(hd)
                    try, set_param(hd(q), 'SampleTime', '0.01'); catch, end
                end
            end
            L(fid, '[OK] %s 内部 UnitDelay SampleTime = 0.01\n', nmb{1});
        catch e
            L(fid, '[FAIL] %s : %s\n', nmb{1}, e.message);
        end
    end

    % ---- 强制单精度（可选，默认关闭）----
    % 目标：让 rtwtypes.h 里 real_T = float，从而用上 Cortex-M4F 的硬件单精度 FPU。
    % 实测：本模型里 MatLab 子系统内嵌套的 Constant/Variable 常数块无法通过
    % 脚本批量改成 single（会报数据类型冲突），只改根接口会导致上游 double
    % 与下游 single 不匹配，Update Diagram 直接失败。
    % 结论：保持默认 double。Cortex-M4F @168MHz 走软件双精度也只需
    %       ~1-3 ms/周期，10 ms 预算内余量充足。
    % 如果确实要单精度，请在 Simulink 里手动把各常数块与根接口一起改成 single，
    % 然后把这个开关打开。
    FORCE_SINGLE = false;
    if FORCE_SINGLE
        L(fid, '\n=== 强制单精度 ===\n');
        nSingle = 0; nFail = 0;
        for c = {'eta','nu','eta_d'}
            try, set_param([DST '/' c{1}], 'OutDataTypeStr', 'single'); nSingle = nSingle + 1;
            catch e, nFail = nFail + 1; L(fid, '[FAIL] 根 Inport %s: %s\n', c{1}, e.message); end
        end
        try, set_param([DST '/u'], 'OutDataTypeStr', 'single'); nSingle = nSingle + 1;
        catch e, nFail = nFail + 1; L(fid, '[FAIL] 根 Outport u: %s\n', e.message); end
        sBt = {'Constant','Gain','Sum','Product','Saturate','MinMax','Abs','Trigonometry', ...
               'DiscreteIntegrator','Memory','Math','Sign','DotProduct','Concatenate', ...
               'Mux','Demux','Switch','Bias','Sqrt','Lookup','Fcn','Assignment'};
        for kk = 1:numel(sBt)
            hs = find_system(DST, 'LookUnderMasks','none', 'FollowLinks','on', ...
                                  'BlockType', sBt{kk});
            for j = 1:numel(hs)
                ok = false;
                for pn = {'OutDataTypeStr','AccumDataTypeStr','OutputDataTypeStr'}
                    try, set_param(hs(j), pn{1}, 'single'); ok = true; nSingle = nSingle + 1;
                    catch, end
                end
                if ~ok, nFail = nFail + 1; end
            end
        end
        L(fid, '[%s] 单精度改写成功 %d 处，跳过 %d 处\n', ...
            tern(nFail==0,'OK','WARN'), nSingle, nFail);
    else
        L(fid, '\n[SKIP] 强制单精度已关闭（real_T 保持 double，见脚本注释）\n');
    end

    % ---- 连续状态残量扫描 ----    L(fid, '\n=== 连续状态残量扫描 ===\n');
    contBT = {'Integrator','Derivative','TransferFcn','StateSpace','ZeroOrderHold', ...
              'TransportDelay','VariableTransportDelay','SecondOrderIntegrator'};
    nCont = 0;
    for kk = 1:numel(contBT)
        hs = find_system(DST, 'FollowLinks','on', 'LookUnderMasks','all', ...
                              'BlockType', contBT{kk});
        for j = 1:numel(hs)
            L(fid, '[WARN] 残留连续块: %s  [%s]\n', getfullname(hs(j)), contBT{kk});
            nCont = nCont + 1;
        end
    end
    L(fid, '[%s] 连续状态块残留 = %d\n', tern(nCont==0,'OK','WARN'), nCont);

    % ---- 离散化后的二次编译检查 ----
    L(fid, '\n=== 二次编译检查 ===\n');
    try
        set_param(DST, 'SimulationCommand', 'update');
        L(fid, '[OK] Update Diagram 通过 —— 已无连续状态\n');
    catch e
        L(fid, '[ERR] Update Diagram: %s\n', e.message);
        if isprop(e, 'cause') && ~isempty(e.cause)
            for kk = 1:numel(e.cause)
                L(fid, '    cause%d: %s\n', kk, e.cause{kk}.message);
            end
        end
    end

    % ---- 保存 ----
    cd(ROOT);
    save_system(DST);
    L(fid, '\n[OK] 已保存 %s\n', fullfile(ROOT, [DST '.slx']));

catch ME
    fprintf(fid, '\n!!! 未捕获错误: %s\n', ME.message);
    for kk = 1:numel(ME.stack)
        fprintf(fid, '    @ %s : line %d\n', ME.stack(kk).name, ME.stack(kk).line);
    end
end

fprintf('DONE\n');

% ================= 局部函数 =================
function L(fid, varargin), fprintf(fid, varargin{:}); end
function s = tern(c, a, b), if c, s = a; else, s = b; end, end
function s = nmz(p), s = strtrim(regexprep(p, '\s+', ' ')); end
function p = pick(list, kw)
p = '';
for i = 1:numel(list), if contains(list{i}, kw); p = list{i}; return; end, end
end
function h = findByName(sysH, bt, tname)
h = [];
ks = find_system(sysH, 'SearchDepth', 1, 'BlockType', bt);
for k = 1:numel(ks)
    if strcmp(nmz(get_param(ks(k),'Name')), tname); h = ks(k); return; end
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
function n = clearPortIn(sysH, blkname, port, fid)
% 删除子系统内某块指定输入端口上的所有残留连线
n = 0;
h = [];
for bt = {'Product','Sum','Gain','SubSystem','Inport','Mux','Fcn'}
    h = findByName(sysH, bt{1}, blkname);
    if ~isempty(h); break; end
end
if isempty(h)
    L(fid, '[WARN] clearPortIn: 未找到块 %s\n', blkname);
    return
end
try
    ph = get_param(h, 'PortHandles');
    ln = get_param(ph.Inport(port), 'Line');
    guard = 0;
    while ~isempty(ln) && numel(ln) == 1 && ln > 0 && guard < 20
        delete_line(ln);
        ln = get_param(ph.Inport(port), 'Line');
        n = n + 1; guard = guard + 1;
    end
catch e
    L(fid, '[WARN] clearPortIn %s[%d]: %s\n', blkname, port, e.message);
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
function wd(fid, sys, s, t)
try
    add_line(sys, s, t, 'autorouting', 'on');
    L(fid, '[OK] %s -> %s\n', s, t);
catch e
    L(fid, '[FAIL] %s -> %s : %s\n', s, t, e.message);
end
end
function h = srcBlockHandle(blkPath, port)
% 返回 blkPath 第 port 个输入端口的上游块句柄；未连线返回 []
h = [];
try
    ph = get_param(blkPath, 'PortHandles');
    if port > numel(ph.Inport), return; end
    ln = get_param(ph.Inport(port), 'Line');
    if numel(ln) ~= 1 || ln <= 0, return; end
    h = get_param(ln, 'SrcBlockHandle');
catch
end
end
