% run_photo_run.m -- 带日志的全景实拍运行包装器
logf = 'E:\yanjiusheng\wb_hil\out\photo_run_log.txt';
try
    diary(logf);
    fprintf('=== photo run start %s ===\n', datestr(now));
    run_hil_board_live;
    fprintf('=== photo run OK %s ===\n', datestr(now));
    diary off
catch e
    disp(getReport(e, 'extended', 'hyperlinks', 'off'));
    fid = fopen(logf, 'a');
    fprintf(fid, 'ERROR:\n%s\n', getReport(e, 'extended', 'hyperlinks', 'off'));
    fclose(fid);
    diary off
end
