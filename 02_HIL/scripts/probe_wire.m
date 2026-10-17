% probe_wire.m -- 定位 wire_current 的 set_param 报错
clear;
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
Sv = load(fullfile(ROOT,'rovmodel75kg.mat'));
mdl = 'rov_hil_host';
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(mdl);

hs = find_system(mdl, 'SearchDepth', 1, 'Type', 'block');
fprintf('num=%d, class(hs)=%s, class(hs{2})=%s\n', numel(hs), class(hs), class(hs{2}));
for k = 1:numel(hs)
    sid = get_param(hs(k), 'SID');
    if strcmp(char(string(sid)), '1265')
        fprintf('found Sum1: path=%s, class=%s\n', hs(k), class(hs(k)));
        h = hs(k);
        fprintf('Inputs param = %s\n', get_param(h, 'Inputs'));
        set_param(h, 'Inputs', '+---+');
        fprintf('set_param OK, Inputs now %s\n', get_param(h, 'Inputs'));
    end
end
close_system(mdl, 0);
fprintf('PROBE_DONE\n');
