% master_repro.m - full local reproduction of all R2023b-runnable conditions
% Nominal / mismatch / current / saturation / measurement-noise
% PID + BS + ADP (SMC excluded: models saved in R2024b format, local MATLAB is R2023b)
% Output: C:/Users/yhm/wb_matrun/repro_out/*.mat
clear; clc;
ROOT = 'D:/yanjiusheng/yhm论文/自适应反步';
FIN  = fullfile(ROOT, '04_程序_最终版');
FCUR = fullfile(ROOT, '04_程序_最终版_海流');
OUT  = 'C:/Users/yhm/wb_matrun/repro_out';
VAR  = 'C:/Users/yhm/wb_matrun/variants';
if ~exist(OUT,'dir'), mkdir(OUT); end
if ~exist(VAR,'dir'), mkdir(VAR); end
copyfile(fullfile(FIN,'PID','rovmodel75kg.mat'), fullfile(VAR,'rovmodel75kg.mat'), 'f');
diary(fullfile(OUT,'repro.log'));
fprintf('==== master_repro start %s ====\n', datestr(now));

% ---------------- A. nominal (6) ----------------
NOM = {
 fullfile(FIN,'PID','chapter4_cascaded_nonlinear_closed_pid_circle'),        'PID_circle';
 fullfile(FIN,'PID','chapter4_cascaded_nonlinear_closed_pid'),               'PID_step';
 fullfile(FIN,'纯反步_Backstepping','nonlinear_closed_backstepping_circle'), 'BS_circle';
 fullfile(FIN,'纯反步_Backstepping','nonlinear_closed_backstepping_0'),      'BS_step';
 fullfile(FIN,'自适应反步_Adaptive','nonlinear_closed_backstepping_adaptive_circle'), 'ADP_circle';
 fullfile(FIN,'自适应反步_Adaptive','nonlinear_closed_backstepping_adaptive_0'),      'ADP_step'};
fprintf('\n---- A. nominal ----\n');
for i = 1:size(NOM,1)
    [dp, mn] = fileparts(NOM{i,1});
    do_run(dp, mn, NOM{i,2}, 1, 'nominal');
end

% ---------------- B/C. mismatch step + circle (36) ----------------
CASES = {'under20',-1,0.2; 'under40',-1,0.4; 'under60',-1,0.6; ...
         'over20',+1,0.2; 'over40',+1,0.4; 'over60',+1,0.6};
MIS = {
 fullfile(FIN,'纯反步_Backstepping'), 'nonlinear_closed_backstepping_0',      'BS';
 fullfile(FIN,'纯反步_Backstepping'), 'nonlinear_closed_backstepping_circle', 'BS';
 fullfile(FIN,'自适应反步_Adaptive'), 'nonlinear_closed_backstepping_adaptive_0',      'ADP';
 fullfile(FIN,'自适应反步_Adaptive'), 'nonlinear_closed_backstepping_adaptive_circle', 'ADP';
 fullfile(FIN,'PID'), 'chapter4_cascaded_nonlinear_closed_pid',        'PID';
 fullfile(FIN,'PID'), 'chapter4_cascaded_nonlinear_closed_pid_circle', 'PID'};
SCEN = {'step';'circle';'step';'circle';'step';'circle'};
fprintf('\n---- B/C. mismatch ----\n');
for r = 1:size(MIS,1)
    for ci = 1:size(CASES,1)
        tag = sprintf('%s_%s_%s', MIS{r,3}, SCEN{r}, CASES{ci,1});
        f = 1 + CASES{ci,2}*CASES{ci,3};
        do_run(MIS{r,1}, MIS{r,2}, tag, f, 'mismatch');
    end
end

% ---------------- D. ocean current (6) ----------------
CUR = {
 fullfile(FCUR,'恒定流','PID','chapter4_cascaded_nonlinear_closed_pid_circle'),                 'PID_circle_const';
 fullfile(FCUR,'恒定流','纯反步_Backstepping','nonlinear_closed_backstepping_circle'),          'BS_circle_const';
 fullfile(FCUR,'恒定流','自适应反步_Adaptive','nonlinear_closed_backstepping_adaptive_circle'), 'ADP_circle_const';
 fullfile(FCUR,'慢变流','PID','chapter4_cascaded_nonlinear_closed_pid_circle'),                 'PID_circle_slow';
 fullfile(FCUR,'慢变流','纯反步_Backstepping','nonlinear_closed_backstepping_circle'),          'BS_circle_slow';
 fullfile(FCUR,'慢变流','自适应反步_Adaptive','nonlinear_closed_backstepping_adaptive_circle'), 'ADP_circle_slow'};
fprintf('\n---- D. current ----\n');
for i = 1:size(CUR,1)
    [dp, mn] = fileparts(CUR{i,1});
    do_run(dp, mn, CUR{i,2}, 1, 'current');
end

% ---------------- E. saturation (3) ----------------
fprintf('\n---- E. saturation ----\n');
SAT = {
 fullfile(FIN,'PID','chapter4_cascaded_nonlinear_closed_pid_circle'),        'PID_circle_sat';
 fullfile(FIN,'纯反步_Backstepping','nonlinear_closed_backstepping_circle'), 'BS_circle_sat';
 fullfile(FIN,'自适应反步_Adaptive','nonlinear_closed_backstepping_adaptive_circle'), 'ADP_circle_sat'};
for i = 1:size(SAT,1)
    [dp, mn] = fileparts(SAT{i,1});
    try
        build_variant(dp, mn, [mn '_sat'], 'sat');
        do_run(VAR, [mn '_sat'], SAT{i,2}, 1, 'saturation');
    catch e
        fprintf('[FAIL] build/run %s : %s\n', SAT{i,2}, e.message);
    end
end

% ---------------- F. measurement noise (3) ----------------
fprintf('\n---- F. noise ----\n');
NOI = {
 fullfile(FIN,'PID','chapter4_cascaded_nonlinear_closed_pid_circle'),        'PID_circle_noise';
 fullfile(FIN,'纯反步_Backstepping','nonlinear_closed_backstepping_circle'), 'BS_circle_noise';
 fullfile(FIN,'自适应反步_Adaptive','nonlinear_closed_backstepping_adaptive_circle'), 'ADP_circle_noise'};
for i = 1:size(NOI,1)
    [dp, mn] = fileparts(NOI{i,1});
    try
        build_variant(dp, mn, [mn '_noise'], 'noise');
        do_run(VAR, [mn '_noise'], NOI{i,2}, 1, 'noise');
    catch e
        fprintf('[FAIL] build/run %s : %s\n', NOI{i,2}, e.message);
    end
end

bdclose('all');
fprintf('\n==== master_repro done %s ====\n', datestr(now));
diary off;

% ================================================================
% local functions (must be at end of script)
% ================================================================
function do_run(dirp, mdl, tag, f, variant)
    % run one sim and save result; sim ToWorkspace vars land in base workspace
    bdclose('all');
    cd(dirp);
    clearvars -except tag f variant dirp mdl;
    evalin('base', 'load(''rovmodel75kg.mat'');');   % model vars into BASE ws (no clear: keep script vars)
    load_system(mdl);
    set_param(mdl, 'MaxStep', '1e-3');
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
        SAVEV = {'t','eout','bout','c1_hat_data','c2_hat_data'};
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
                        'matlab','R2023b','MaxStep','1e-3','dur',el);
        save(fullfile('C:/Users/yhm/wb_matrun/repro_out', [tag '.mat']), '-struct', 'V', '-v7.3');
        if isfield(V,'t')
            fprintf('[OK] %-22s %6.1fs n=%d\n', tag, el, numel(V.t));
        else
            fprintf('[OK] %-22s %6.1fs (no t saved!)\n', tag, el);
        end
    end
    bdclose('all');
end

function build_variant(srcdir, mdl, newname, mode)
    % mode: 'sat' (thruster saturation +-100 -> +-50 V)
    %       'noise' (measurement noise injected at controller error/measurement input)
    bdclose('all');
    cd(srcdir);
    load('rovmodel75kg.mat');
    load_system(mdl);
    newp = fullfile('C:/Users/yhm/wb_matrun/variants', newname);
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
            nm = get_param(subs{kk}, 'Name');
            if ~isempty(regexpi(nm, 'controller'))
                csub = subs{kk};
            end
        end
        if isempty(csub)
            % cascade PID: outer controller subsystem is named 'PID' (may have spaces)
            for kk = 1:numel(subs)
                nm = get_param(subs{kk}, 'Name');
                if strcmpi(strtrim(nm), 'PID')
                    csub = subs{kk}; break;
                end
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
                tgtport = pp; break;   % error line (Sum2 output)
            end
        end
        if tgtport == 0
            tgtport = 1;
        end
        lh0 = get_param(ph.Inport(tgtport), 'Line');
        assert(lh0 ~= -1, 'injection line not found');
        sph = get_param(lh0, 'SrcPortHandle');
        % noise vector: std 0.01 m position, 0.5 deg attitude, 1 ms sample time
        vp = 1e-4; va = (0.5*pi/180)^2;
        add_block('simulink/Signal Routing/Mux', [mdl '/MN_Mux6'], 'Inputs', '6', ...
                  'Position', [40 -420 45 -300]);
        for jj = 1:6
            if jj <= 3, pw = vp; else, pw = va; end
            add_block('simulink/Sources/Band-Limited White Noise', ...
                sprintf('%s/MN_WN%d', mdl, jj), 'Cov', sprintf('%.10g', pw), ...
                'Ts', '0.001', 'seed', '23341', ...
                'Position', [30 -430+jj*30 70 -410+jj*30]);
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
