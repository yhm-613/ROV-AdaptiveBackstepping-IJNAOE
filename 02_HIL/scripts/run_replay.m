%% run_replay.m -- 只跑「回放数据导出 + PiL」两步
%  前置条件：rov_ctrl_deploy.slx 与 生成代码 已是最新（build_deploy + build_codegen 跑过）
%  （这两个脚本开头都有 clear，所以调用方不能依赖自己工作区的变量）
try
    run('E:\yanjiusheng\wb_hil\scripts\run_discretization_check.m');
catch e
    fprintf(2,'[FATAL] run_discretization_check: %s\n', e.message);
end
try
    run('E:\yanjiusheng\wb_hil\scripts\run_pil_check.m');
catch e
    fprintf(2,'[FATAL] run_pil_check: %s\n', e.message);
end
fprintf('\n===== run_replay DONE =====\n');
