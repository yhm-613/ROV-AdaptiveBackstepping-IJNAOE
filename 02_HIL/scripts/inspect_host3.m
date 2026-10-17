% inspect_host3.m -- MATLABFcn 块到底调什么 + outport 列表
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
mdl = 'rov_hil_host';
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(mdl);
b = [mdl '/HIL Bridge'];
fprintf('BlockType = %s\n', get_param(b, 'BlockType'));
try
    fprintf('MATLABFcn = %s\n', get_param(b, 'MATLABFcn'));
catch
end
% 根 Outport
ports = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Outport');
fprintf('根 Outport 数 = %d\n', numel(ports));
for i = 1:numel(ports)
    fprintf('  Outport %s : Port=%s\n', get_param(ports{i}, 'Name'), ...
        get_param(ports{i}, 'Port'));
end
fprintf('SaveOutput=%s OutputSaveName=%s SaveTime=%s\n', ...
    get_param(mdl, 'SaveOutput'), get_param(mdl, 'OutputSaveName'), ...
    get_param(mdl, 'SaveTime'));
close_system(mdl, 0);
disp('DONE3');
