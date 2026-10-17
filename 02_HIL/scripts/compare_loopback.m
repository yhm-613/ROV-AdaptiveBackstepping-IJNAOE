% compare_loopback.m -- 对比 HIL 真板 vs PC 回环 D 组 (同 20s 窗口)
ROOT = fileparts(fileparts(mfilename('fullpath')));
D = load(fullfile(ROOT, 'out', 'discretization_check.mat'));
disp(fieldnames(D));
% 找 D 组轨迹变量名
fn = fieldnames(D);
for k = 1:numel(fn)
    v = D.(fn{k});
    if isstruct(v) || (isnumeric(v) && numel(v) > 100)
        fprintf('%s : %s %s\n', fn{k}, class(v), mat2str(size(v)));
    end
end
disp('CMP_DONE');
