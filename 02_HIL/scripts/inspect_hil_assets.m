% inspect_hil_assets.m -- 全面盘点 rov_hil_host.slx 的结构与可配置性 (v2, ASCII output)
clear;
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
addpath(genpath(fullfile(ROOT,'pc')));
try, warning('off','all'); catch, end

Sv = load(fullfile(ROOT,'rovmodel75kg.mat'));
fn = fieldnames(Sv);
for ii = 1:numel(fn), assignin('base', fn{ii}, Sv.(fn{ii})); end
assignin('base','M_nom', Sv.M); assignin('base','Ma_nom', Sv.Ma);
assignin('base','DL_nom', Sv.DL);
assignin('base','CTRL_DISCRETE', 1);
assignin('base','TS_CTRL', 0.01);

mdl = 'rov_hil_host';
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(mdl);

fprintf('=== MODEL CONFIG ===\n');
fprintf('StopTime=%s  Solver=%s  FixedStep=%s\n', ...
    get_param(mdl,'StopTime'), get_param(mdl,'Solver'), get_param(mdl,'FixedStep'));

fprintf('\n=== TOP-LEVEL BLOCKS ===\n');
hs = find_system(mdl, 'SearchDepth', 1, 'Type','block');
for k = 2:numel(hs)
    fprintf('  %s [%s]\n', blkname(hs(k)), get_param(hs(k),'BlockType'));
end

fprintf('\n=== ALL BLOCKS (deep, incl masks) ===\n');
hs2 = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Type','block');
for k = 2:numel(hs2)
    ref = get_param(hs2(k),'ReferenceBlock');
    tag = '';
    if ~isempty(ref), tag = sprintf(' <- %s', blkname(ref)); end
    fprintf('  %s [%s]%s\n', blkname(hs2(k)), get_param(hs2(k),'BlockType'), tag);
end

fprintf('\n=== PreLoadFcn ===\n');
disp(get_param(mdl,'PreLoadFcn'));

fprintf('\n=== rovmodel75kg.mat VARS ===\n');
for ii = 1:numel(fn)
    v = Sv.(fn{ii});
    if isnumeric(v) && isscalar(v)
        fprintf('  %-12s = %g\n', fn{ii}, v);
    else
        fprintf('  %-12s : %s %s\n', fn{ii}, class(v), mat2str(size(v)));
    end
end

fprintf('\nINSPECT_DONE\n');

function s = blkname(h)
s = get_param(h,'Name');
if iscell(s), s = strjoin(s, ' | '); end
s = strrep(s, sprintf('\n'), ' / ');
end
