% inspect_host.m -- 查看 rov_hil_host.slx 关键配置
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
addpath(genpath(fullfile(ROOT, 'pc')));
mdl = 'rov_hil_host';
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(mdl);
fprintf('StopTime   = %s\n', get_param(mdl, 'StopTime'));
fprintf('Solver     = %s / %s\n', get_param(mdl, 'Solver'), get_param(mdl, 'SolverType'));
fprintf('FixedStep  = %s\n', get_param(mdl, 'FixedStep'));
pre = get_param(mdl, 'PreLoadFcn');
fprintf('PreLoadFcn = %s\n', strtrim(pre));
fprintf('--- To Workspace / Scope blocks ---\n');
bl = find_system(mdl, 'LookUnderMasks', 'all', 'BlockType', {'ToWorkspace', 'Scope'});
for i = 1:numel(bl)
    fprintf('%s : %s\n', get_param(bl{i}, 'BlockType'), bl{i});
    if strcmp(get_param(bl{i}, 'BlockType'), 'ToWorkspace')
        fprintf('    VariableName = %s\n', get_param(bl{i}, 'VariableName'));
    end
end
fprintf('--- MATLAB Function / S-Function blocks ---\n');
bl2 = find_system(mdl, 'LookUnderMasks', 'all', 'Type', 'Block');
for i = 1:numel(bl2)
    nm = get_param(bl2{i}, 'Name');
    if ~isempty(regexpi(nm, 'hil|bridge|serial'))
        fprintf('%s  [%s]\n', bl2{i}, get_param(bl2{i}, 'BlockType'));
    end
end
close_system(mdl, 0);
disp('INSPECT_DONE');
