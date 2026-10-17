% rerun_noise.m - rebuild noise variants with corrected error-line injection and rerun 3 cases
NOI = {
 fullfile('D:/yanjiusheng/yhm论文/自适应反步/04_程序_最终版','PID','chapter4_cascaded_nonlinear_closed_pid_circle'),        'PID_circle_noise';
 fullfile('D:/yanjiusheng/yhm论文/自适应反步/04_程序_最终版','纯反步_Backstepping','nonlinear_closed_backstepping_circle'), 'BS_circle_noise';
 fullfile('D:/yanjiusheng/yhm论文/自适应反步/04_程序_最终版','自适应反步_Adaptive','nonlinear_closed_backstepping_adaptive_circle'), 'ADP_circle_noise'};
if ~exist('C:/Users/yhm/wb_matrun/variants','dir'), mkdir('C:/Users/yhm/wb_matrun/variants'); end
copyfile(fullfile('D:/yanjiusheng/yhm论文/自适应反步/04_程序_最终版','PID','rovmodel75kg.mat'), ...
         fullfile('C:/Users/yhm/wb_matrun/variants','rovmodel75kg.mat'), 'f');
diary('C:/Users/yhm/wb_matrun/repro_out/repro_noise.log');
for i = 1:size(NOI,1)
    [dp, mn] = fileparts(NOI{i,1});
    try
        build_variant(dp, mn, [mn '_noise'], 'noise');
        do_run('C:/Users/yhm/wb_matrun/variants', [mn '_noise'], NOI{i,2}, 1, 'noise');
    catch e
        fprintf('[FAIL] build/run %s : %s\n', NOI{i,2}, e.message);
    end
end
bdclose('all');
fprintf('RERUN NOISE DONE\n');
diary off;

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
