% probe_params.m -- 查 To Workspace 块的真实参数名
new_system('tmpmdl');
add_block('simulink/Sinks/To Workspace', 'tmpmdl/TW', 'VariableName', 'tmpv');
dp = get_param('tmpmdl/TW', 'DialogParameters');
disp(fieldnames(dp));
fn = fieldnames(dp);
for k = 1:numel(fn)
    if ~isempty(regexpi(fn{k}, 'data|limit|points'))
        try
            fprintf('%s = %s\n', fn{k}, num2str(get_param('tmpmdl/TW', fn{k})));
        catch
            fprintf('%s = (不可读)\n', fn{k});
        end
    end
end
delete_block('tmpmdl/TW');
close_system('tmpmdl', 0);
disp('PROBE_PARAMS_DONE');
