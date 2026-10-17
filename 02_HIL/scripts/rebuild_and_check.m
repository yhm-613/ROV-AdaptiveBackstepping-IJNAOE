%% rebuild_and_check.m -- 全链路重建 + 校验（单次 MATLAB 启动）
%
%  顺序：
%    1 build_deploy              重建 rov_ctrl_deploy.slx（含 e_calc 符号修复）
%    2 build_codegen             重新生成 Embedded Coder 代码
%    3 build_pc_models           重建 rov_hil_loopback / rov_hil_host（含入口 ZOH）
%    4 diag_hil                  控制周期 Ts 扫描
%    5 run_discretization_check  A/B/C/D/E 五场景对照 + 导出 PiL 数据
%    6 run_pil_check             部署模型 PiL 回放
%
%  ⚠ 被调用脚本各自开头有 clear，而 run() 是在调用者工作区执行的 ——
%    所以本驱动脚本不能用任何变量保存循环状态，必须逐条字面量写死。

fprintf('\n\n########## 1/6  build_deploy ##########\n');
try
    run('E:\yanjiusheng\wb_hil\scripts\build_deploy.m');
catch e
    fprintf(2, '[build_deploy] 未捕获错误: %s\n', e.message);
    if isprop(e,'cause') && ~isempty(e.cause)
        for kk = 1:numel(e.cause)
            if ~isempty(e.cause{kk}), fprintf(2, '    cause%d: %s\n', kk, e.cause{kk}.message); end
        end
    end
end

fprintf('\n\n########## 2/6  build_codegen ##########\n');
try
    run('E:\yanjiusheng\wb_hil\scripts\build_codegen.m');
catch e
    fprintf(2, '[build_codegen] 未捕获错误: %s\n', e.message);
    if isprop(e,'cause') && ~isempty(e.cause)
        for kk = 1:numel(e.cause)
            if ~isempty(e.cause{kk}), fprintf(2, '    cause%d: %s\n', kk, e.cause{kk}.message); end
        end
    end
end

fprintf('\n\n########## 3/6  build_pc_models ##########\n');
try
    run('E:\yanjiusheng\wb_hil\scripts\build_pc_models.m');
catch e
    fprintf(2, '[build_pc_models] 未捕获错误: %s\n', e.message);
    if isprop(e,'cause') && ~isempty(e.cause)
        for kk = 1:numel(e.cause)
            if ~isempty(e.cause{kk}), fprintf(2, '    cause%d: %s\n', kk, e.cause{kk}.message); end
        end
    end
end

fprintf('\n\n########## 4/6  diag_hil ##########\n');
try
    run('E:\yanjiusheng\wb_hil\scripts\diag_hil.m');
catch e
    fprintf(2, '[diag_hil] 未捕获错误: %s\n', e.message);
end

fprintf('\n\n########## 5/6  run_discretization_check ##########\n');
try
    run('E:\yanjiusheng\wb_hil\scripts\run_discretization_check.m');
catch e
    fprintf(2, '[run_discretization_check] 未捕获错误: %s\n', e.message);
end

fprintf('\n\n########## 6/6  run_pil_check ##########\n');
try
    run('E:\yanjiusheng\wb_hil\scripts\run_pil_check.m');
catch e
    fprintf(2, '[run_pil_check] 未捕获错误: %s\n', e.message);
end

fprintf('\n\n########## ALL DONE ##########\n');
