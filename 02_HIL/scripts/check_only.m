%% check_only.m -- 只跑 PC 模型 + 诊断（build_deploy / build_codegen 已完成时用）
%  ⚠ 被调用脚本各自有 clear，run() 在调用者工作区执行 -> 不能依赖变量

ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
fprintf('\n\n########## 1/4  build_pc_models ##########\n');
try
    run(fullfile(ROOT, 'scripts', 'build_pc_models.m'));
catch e
    fprintf(2, '[build_pc_models] 未捕获错误: %s\n', e.message);
end

fprintf('\n\n########## 2/4  diag_hil ##########\n');
try
    run(fullfile(ROOT, 'scripts', 'diag_hil.m'));
catch e
    fprintf(2, '[diag_hil] 未捕获错误: %s\n', e.message);
end

fprintf('\n\n########## 3/4  run_discretization_check ##########\n');
try
    run(fullfile(ROOT, 'scripts', 'run_discretization_check.m'));
catch e
    fprintf(2, '[run_discretization_check] 未捕获错误: %s\n', e.message);
end

fprintf('\n\n########## 4/4  run_pil_check ##########\n');
try
    run(fullfile(ROOT, 'scripts', 'run_pil_check.m'));
catch e
    fprintf(2, '[run_pil_check] 未捕获错误: %s\n', e.message);
end

fprintf('\n\n########## ALL DONE ##########\n');
