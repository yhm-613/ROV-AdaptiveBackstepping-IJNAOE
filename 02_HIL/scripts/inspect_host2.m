% inspect_host2.m -- 回调与统计出口
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
mdl = 'rov_hil_host';
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(mdl);
for cb = {'StartFcn', 'StopFcn', 'PostLoadFcn', 'InitFcn'}
    v = get_param(mdl, cb{1});
    fprintf('=== %s ===\n%s\n', cb{1}, strtrim(v));
end
% HIL Bridge 输出连到哪
ph = get_param([mdl '/HIL Bridge'], 'PortConnectivity');
for i = 1:numel(ph)
    fprintf('Bridge port %d (%s) -> ', ph(i).Type, ph(i).Name);
    if ~isempty(ph(i).DstBlock)
        for j = 1:numel(ph(i).DstBlock)
            fprintf('%s  ', get_param(ph(i).DstBlock(j), 'Name'));
        end
    end
    fprintf('\n');
end
% 求解器细节
fprintf('MaxStep = %s\n', get_param(mdl, 'MaxStep'));
fprintf('SaveOutput=%s OutputSaveName=%s\n', get_param(mdl,'SaveOutput'), get_param(mdl,'OutputSaveName'));
close_system(mdl, 0);
disp('DONE2');
