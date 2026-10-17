% diag_codegen_err.m -- 拿英文原文报错 + fileGenControl 状态
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
warning('off','all');
DST = 'rov_ctrl_deploy';

S = load(fullfile(ROOT,'rovmodel75kg.mat'));
fn = fieldnames(S);
for k = 1:numel(fn), assignin('base', fn{k}, S.(fn{k})); end
assignin('base','M_nom', S.M); assignin('base','Ma_nom', S.Ma); assignin('base','DL_nom', S.DL);

cf = Simulink.fileGenControl('get', 'CodeGenFolder');
cachef = Simulink.fileGenControl('get', 'CacheFolder');
fid = fopen('E:\yanjiusheng\wb_hil\out\fgc_state.txt','w','n','UTF-8');
fprintf(fid, 'CacheFolder   = %s\n', cachef);
fprintf(fid, 'CodeGenFolder = %s\n', cf);
fclose(fid);

if bdIsLoaded(DST), close_system(DST, 0); end
load_system(DST);
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
for k = 1:size(opts,1)
    try, set_param(cfg, opts{k,1}, opts{k,2}); catch, end
end
Simulink.fileGenControl('set', 'CodeGenFolder', 'E:\yanjiusheng\codegen_out_v2', 'createDir', true);

try
    rtwbuild(DST);
    fprintf('BUILD_OK\n');
catch e
    fid = fopen('E:\yanjiusheng\wb_hil\out\codegen_err.txt','w','n','UTF-8');
    fprintf(fid, 'identifier: %s\n\n', e.identifier);
    fprintf(fid, 'message: %s\n', e.message);
    if isprop(e,'cause')
        for kk = 1:numel(e.cause)
            fprintf(fid, 'cause%d: %s\n', kk, e.cause(kk).message);
        end
    end
    fclose(fid);
    fprintf('BUILD_FAIL (see out/codegen_err.txt)\n');
end
close_system(DST, 0);
fprintf('DIAG_DONE\n');
