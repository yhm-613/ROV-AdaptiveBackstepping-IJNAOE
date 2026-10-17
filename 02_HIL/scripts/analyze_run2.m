% analyze_run2.m -- 先摸清 44 列布局
ROOT = fileparts(fileparts(mfilename('fullpath')));
S = load(fullfile(ROOT, 'out', 'hil_board_run.mat'));
M = S.M;  BR = S.BR;
n = size(M, 1);
t = (0:n-1) * 0.01;
fprintf('col :      min        max       mean      std\n');
for k = 1:size(M, 2)
    fprintf('M(%2d): %9.4f %9.4f %9.4f %9.4f\n', k, min(M(:,k)), max(M(:,k)), mean(M(:,k)), std(M(:,k)));
end
fprintf('\nBR:\n');
for k = 1:size(BR, 2)
    fprintf('BR(%d): min=%9.4f max=%9.4f mean=%9.4f\n', k, min(BR(:,k)), max(BR(:,k)), mean(BR(:,k)));
end
% link 分布
link = BR(:, 7);
fprintf('\nlink 分布: ok=%d timeout=%d badcrc=%d fail=%d\n', ...
    sum(link==0), sum(link==1), sum(link==2), sum(link>=3));
disp('ANALYZE2_DONE');
