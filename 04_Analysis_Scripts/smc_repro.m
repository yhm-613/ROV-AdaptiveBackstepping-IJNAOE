% smc_repro.m - SMC full-workflow reproduction on R2023b local machine
% Cases: step mismatch x7, circle nom + over60, noise, sat, current const/slow (x2 MaxStep)
% Refs: results_new_补强/SMC_*.mat, smc_metrics.json
clear; clc;

SRC = 'D:/yanjiusheng/yhm论文/自适应反步/04_程序_SMC_R2023b';
VD  = 'C:/Users/yhm/wb_matrun/smc_variants';
OUT = 'C:/Users/yhm/wb_matrun/repro_out';
if ~exist(VD,'dir'), mkdir(VD); end
if ~exist(OUT,'dir'), mkdir(OUT); end
copyfile(fullfile(SRC,'smc_aug.m'),        fullfile(VD,'smc_aug.m'), 'f');
copyfile(fullfile(SRC,'rovmodel75kg.mat'), fullfile(VD,'rovmodel75kg.mat'), 'f');

diary(fullfile(OUT,'repro_smc.log'));
fprintf('==== SMC R2023b reproduction start %s ====\n', datestr(now));

% ---------- A. step mismatch (7) ----------
fprintf('\n---- A. step mismatch ----\n');
SF = {1.6,'SMC_step_over60'; 1.4,'SMC_step_over40'; 1.2,'SMC_step_over20'; ...
      1.0,'SMC_step_nom';     0.8,'SMC_step_under20'; 0.6,'SMC_step_under40'; ...
      0.4,'SMC_step_under60'};
for i = 1:size(SF,1)
    try
        do_run_smc(SRC, 'smc_step', SF{i,2}, SF{i,1}, 'mismatch', '1e-3');
    catch e
        fprintf('[FAIL] %s : %s\n', SF{i,2}, e.message);
    end
end

% ---------- B. circle nominal + over60 ----------
fprintf('\n---- B. circle nominal/over60 ----\n');
try, do_run_smc(SRC, 'smc_circle', 'SMC_circle_nom',    1.0, 'mismatch', '1e-3'); catch e, fprintf('[FAIL] nom: %s\n', e.message); end
try, do_run_smc(SRC, 'smc_circle', 'SMC_circle_over60', 1.6, 'mismatch', '1e-3'); catch e, fprintf('[FAIL] over60: %s\n', e.message); end

% ---------- C. variants ----------
fprintf('\n---- C. variants ----\n');
VJ = { 'noise', 'smc_circle_noise', 'SMC_circle_noise', '1e-3';
       'sat',   'smc_circle_sat',   'SMC_circle_sat',   '1e-3' };
for i = 1:size(VJ,1)
    try
        build_variant(SRC, 'smc_circle', VJ{i,2}, VJ{i,1});
        do_run_smc(VD, VJ{i,2}, VJ{i,3}, 1.0, VJ{i,1}, VJ{i,4});
    catch e
        fprintf('[FAIL] %s : %s\n', VJ{i,3}, e.message);
    end
end

% ---------- D. current const / slow (MaxStep 5e-4 primary, 1e-3 secondary) ----------
fprintf('\n---- D. current ----\n');
CJ = { 'const', 'smc_circle_const', 'SMC_circle_const', '5e-4';
       'slow',  'smc_circle_slow',  'SMC_circle_slow',  '5e-4' };
for i = 1:size(CJ,1)
    try
        build_current(SRC, 'smc_circle', CJ{i,2}, CJ{i,1});
        do_run_smc(VD, CJ{i,2}, CJ{i,3}, 1.0, CJ{i,1}, CJ{i,4});
        do_run_smc(VD, CJ{i,2}, [CJ{i,3} '_ms1e3'], 1.0, CJ{i,1}, '1e-3');
    catch e
        fprintf('[FAIL] %s : %s\n', CJ{i,3}, e.message);
    end
end

bdclose('all');
fprintf('==== SMC REPRO DONE %s ====\n', datestr(now));
diary off;

%% ================= functions =================
function do_run_smc(dirp, mdl, tag, f, variant, ms)
    bdclose('all');
    cd(dirp);
    clearvars -except tag f variant dirp mdl ms;
    evalin('base', 'load(''rovmodel75kg.mat'');');
    assignin('base', 'RHO_SMC', [10;10;10;2;2;2]);
    assignin('base', 'PHI_SMC', 1.0);
    load_system(mdl);
    set_param(mdl, 'MaxStep', ms);
    tws = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'ToWorkspace');
    for kk = 1:numel(tws)
        set_param(tws{kk}, 'SaveFormat', 'Array', 'MaxDataPoints', 'inf');
    end
    assignin('base', 'M_nom',  evalin('base','M')*f);
    assignin('base', 'Ma_nom', evalin('base','Ma')*f);
    assignin('base', 'DL_nom', evalin('base','DL')*f);
    t0 = tic; ok = true;
    try
        sim(mdl);  %#ok<NASGU>
    catch e
        ok = false;
        fprintf('[FAIL] %s : %s\n', tag, e.message);
    end
    el = toc(t0);
    if ok
        SAVEV = {'t','eout','bout'};
        V = struct();
        for kk = 1:numel(SAVEV)
            nm = SAVEV{kk};
            if evalin('base', sprintf('exist(''%s'',''var'')', nm))
                V.(nm) = evalin('base', nm);
            elseif eval(sprintf('exist(''%s'',''var'')', nm))
                V.(nm) = eval(nm);
            end
        end
        V.meta = struct('tag',tag,'model',mdl,'factor',f,'variant',variant, ...
                        'matlab','R2023b','MaxStep',ms,'dur',el);
        save(fullfile('C:/Users/yhm/wb_matrun/repro_out', [tag '.mat']), '-struct', 'V', '-v7.3');
        if isfield(V,'t')
            fprintf('[OK] %-24s %6.1fs MaxStep=%s n=%d\n', tag, el, ms, numel(V.t));
        else
            fprintf('[OK] %-24s %6.1fs (no t saved!)\n', tag, el);
        end
    end
    bdclose('all');
end

function build_variant(srcdir, mdl, newname, mode)
    % mode: 'sat' (thruster saturation +-100 -> +-50 V)
    %       'noise' (measurement noise injected at controller error input)
    bdclose('all');
    cd(srcdir);
    load_system(mdl);
    newp = fullfile('C:/Users/yhm/wb_matrun/smc_variants', newname);
    if exist([newp '.slx'], 'file'), delete([newp '.slx']); end
    if strcmp(mode, 'sat')
        sats = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Saturate');
        nmod = 0;
        for kk = 1:numel(sats)
            ul = str2double(get_param(sats{kk}, 'UpperLimit'));
            if abs(ul - 100) < 1e-6
                set_param(sats{kk}, 'UpperLimit', '50', 'LowerLimit', '-50');
                nmod = nmod + 1;
            end
        end
        fprintf('variant %s: %d saturation block(s) set to +-50 V\n', newname, nmod);
        assert(nmod >= 1, 'no +-100 V saturation block found');
    else
        csub = '';
        subs = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'SubSystem');
        for kk = 1:numel(subs)
            if ~isempty(regexpi(get_param(subs{kk}, 'Name'), 'controller'))
                csub = subs{kk};
            end
        end
        assert(~isempty(csub), 'controller subsystem not found in %s', mdl);
        ph = get_param(csub, 'PortHandles');
        tgtport = 0;
        for pp = 1:numel(ph.Inport)
            lh0 = get_param(ph.Inport(pp), 'Line');
            if lh0 == -1, continue; end
            sb = get_param(get_param(lh0, 'SrcPortHandle'), 'Parent');
            if strcmp(get_param(sb, 'BlockType'), 'Sum')
                tgtport = pp; break;
            end
        end
        if tgtport == 0, tgtport = 1; end
        lh0 = get_param(ph.Inport(tgtport), 'Line');
        assert(lh0 ~= -1, 'injection line not found');
        sph = get_param(lh0, 'SrcPortHandle');
        vp = 1e-4; va = (0.5*pi/180)^2;
        add_block('simulink/Signal Routing/Mux', [mdl '/MN_Mux6'], 'Inputs', '6', ...
                  'Position', [40 -420 45 -300]);
        for jj = 1:6
            if jj <= 3, pw = vp; else, pw = va; end
            try
                add_block('simulink/Sources/Band-Limited White Noise', ...
                    sprintf('%s/MN_WN%d', mdl, jj), 'NoisePower', sprintf('%.10g', pw), ...
                    'SampleTime', '0.001', 'Seed', '23341', ...
                    'Position', [30 -430+jj*30 70 -410+jj*30]);
            catch
                add_block('simulink/Sources/Band-Limited White Noise', ...
                    sprintf('%s/MN_WN%d', mdl, jj), 'Cov', sprintf('%.10g', pw), ...
                    'Ts', '0.001', 'seed', '23341', ...
                    'Position', [30 -430+jj*30 70 -410+jj*30]);
            end
        end
        add_block('simulink/Math Operations/Sum', [mdl '/MN_Sum'], 'Inputs', '+-', ...
                  'Position', [120 -380 150 -350]);
        hSum = get_param([mdl '/MN_Sum'], 'PortHandles');
        hMux = get_param([mdl '/MN_Mux6'], 'PortHandles');
        for jj = 1:6
            hW = get_param(sprintf('%s/MN_WN%d', mdl, jj), 'PortHandles');
            add_line(mdl, hW.Outport(1), hMux.Inport(jj), 'autorouting', 'smart');
        end
        delete_line(lh0);
        add_line(mdl, sph, hSum.Inport(1), 'autorouting', 'smart');
        add_line(mdl, hMux.Outport(1), hSum.Inport(2), 'autorouting', 'smart');
        add_line(mdl, hSum.Outport(1), ph.Inport(tgtport), 'autorouting', 'smart');
        fprintf('variant %s: noise injected at controller inport %d\n', newname, tgtport);
    end
    save_system(mdl, newp);
    bdclose('all');
end

function build_current(srcdir, mdl, newname, sc)
    % replicate 06_分析脚本/10_海流实验/build_current_models.m injection
    % Fc = DL*vc + DQ*|vc|vc, vc body-frame, beta=45deg; inject into Sum1 port5
    DL1=3.221; DL2=3.291; DQ1=105.3; DQ2=139.6;
    BETA = 0.785398;
    bdclose('all');
    cd(srcdir);
    load_system(mdl);
    sum1 = '';
    subs = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'SubSystem');
    euler = '';
    for k = 1:numel(subs)
        nm = get_param(subs{k}, 'Name');
        if strncmp(nm, 'Euler', 5), euler = subs{k}; end
    end
    ss = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Sum');
    for k = 1:numel(ss)
        if strcmp(get_param(ss{k}, 'Name'), 'Sum1'), sum1 = ss{k}; end
    end
    assert(~isempty(euler), 'Euler block not found');
    assert(~isempty(sum1), 'Sum1 not found');
    if strcmp(get_param(sum1, 'Inputs'), '+---+') && ...
       ~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'OC_Fx'))
        fprintf('  %s already injected, skip\n', newname);
        bdclose('all'); return;
    end
    assert(strcmp(get_param(sum1, 'Inputs'), '+---'), 'Sum1 Inputs unexpected: %s', ...
           get_param(sum1, 'Inputs'));
    C  = sprintf('cos(u(6)-%.6f)', BETA);
    S  = sprintf('sin(%.6f-u(6))', BETA);
    if strcmp(sc, 'const')
        U  = '0.3';
        FX = sprintf('%.4f*%s*%s + %.4f*%s*%s*%s*abs(%s)', DL1, U, C, DQ1, U, U, C, C);
        FY = sprintf('%.4f*%s*%s + %.4f*%s*%s*%s*abs(%s)', DL2, U, S, DQ2, U, U, S, S);
    else
        U  = '(0.2+0.1*sin(0.05*u(7)))';
        FX = sprintf('%.4f*(%s)*%s + %.4f*(%s)*(%s)*%s*abs(%s)', DL1, U, C, DQ1, U, U, C, C);
        FY = sprintf('%.4f*(%s)*%s + %.4f*(%s)*(%s)*%s*abs(%s)', DL2, U, S, DQ2, U, U, S, S);
    end
    add_block('simulink/User-Defined Functions/Fcn', [mdl '/OC_Fx'], ...
              'Expr', FX, 'Position', [420 -260 620 -220]);
    add_block('simulink/User-Defined Functions/Fcn', [mdl '/OC_Fy'], ...
              'Expr', FY, 'Position', [420 -160 620 -120]);
    add_block('simulink/Sources/Constant', [mdl '/OC_Zero'], ...
              'Value', '0', 'Position', [420 -60 460 -30]);
    add_block('simulink/Signal Routing/Mux', [mdl '/OC_Mux6'], ...
              'Inputs', '6', 'Position', [660 -200 670 -40]);
    if strcmp(sc, 'slow')
        add_block('simulink/Signal Routing/Mux', [mdl '/OC_MuxT'], ...
                  'Inputs', '2', 'Position', [200 -220 210 -180]);
        clock1 = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Clock');
        assert(~isempty(clock1), 'Clock not found');
        hC = get_param(clock1{1}, 'PortHandles');
        hM = get_param([mdl '/OC_MuxT'], 'PortHandles');
        add_line(mdl, hC.Outport(1), hM.Inport(2), 'autorouting', 'smart');
    end
    hFx = get_param([mdl '/OC_Fx'], 'PortHandles');
    hFy = get_param([mdl '/OC_Fy'], 'PortHandles');
    hZ  = get_param([mdl '/OC_Zero'], 'PortHandles');
    hM6 = get_param([mdl '/OC_Mux6'], 'PortHandles');
    hE  = get_param(euler, 'PortHandles');
    if strcmp(sc, 'const')
        add_line(mdl, hE.Outport(1), hFx.Inport(1), 'autorouting', 'smart');
        add_line(mdl, hE.Outport(1), hFy.Inport(1), 'autorouting', 'smart');
    else
        hMT = get_param([mdl '/OC_MuxT'], 'PortHandles');
        add_line(mdl, hE.Outport(1), hMT.Inport(1), 'autorouting', 'smart');
        add_line(mdl, hMT.Outport(1), hFx.Inport(1), 'autorouting', 'smart');
        add_line(mdl, hMT.Outport(1), hFy.Inport(1), 'autorouting', 'smart');
    end
    add_line(mdl, hFx.Outport(1), hM6.Inport(1), 'autorouting', 'smart');
    add_line(mdl, hFy.Outport(1), hM6.Inport(2), 'autorouting', 'smart');
    for p = 3:6
        add_line(mdl, hZ.Outport(1), hM6.Inport(p), 'autorouting', 'smart');
    end
    set_param(sum1, 'Inputs', '+---+');
    hS = get_param(sum1, 'PortHandles');
    add_line(mdl, hM6.Outport(1), hS.Inport(5), 'autorouting', 'smart');
    newp = fullfile('C:/Users/yhm/wb_matrun/smc_variants', newname);
    if exist([newp '.slx'], 'file'), delete([newp '.slx']); end
    save_system(mdl, newp);
    fprintf('  built %s (%s)\n', newname, sc);
    bdclose('all');
end
