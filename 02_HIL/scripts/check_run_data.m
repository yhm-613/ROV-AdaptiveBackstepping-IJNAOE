% check_run_data.m -- 检查 hil_board_run.mat 实际内容
ROOT = fileparts(fileparts(mfilename('fullpath')));
S = load(fullfile(ROOT, 'out', 'hil_board_run.mat'));
disp(fieldnames(S));
fprintf('T : %dx%d, range [%.3f, %.3f]\n', size(S.T,1), size(S.T,2), S.T(1), S.T(end));
fprintf('M : %dx%d\n', size(S.M,1), size(S.M,2));
fprintf('BR: %dx%d\n', size(S.BR,1), size(S.BR,2));
disp('CHECK_DONE');
