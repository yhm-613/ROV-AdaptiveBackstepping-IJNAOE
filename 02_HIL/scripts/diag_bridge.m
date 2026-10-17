% diag_bridge.m -- 裸调 hil_bridge_host 定位「维度不一致」异常的确切位置
ROOT = fileparts(fileparts(mfilename('fullpath')));  % scripts/ 的上级目录 = 工程根
cd(ROOT);
addpath(genpath(fullfile(ROOT, 'pc')));

eta   = [1; 2; 0; 0; 0; 0.5];
nu    = zeros(6, 1);
eta_d = [2; 2; 2; 0; 0; 0];
nu_d  = zeros(6, 1);

% 先看 read() 返回的形状
p = serialport('COM14', 460800, 'Timeout', 0.02);
flush(p);
n = p.NumBytesAvailable;
fprintf('可用字节: %d\n', n);
if n > 0
    v = read(p, n, 'uint8');
    fprintf('read 返回: class=%s  size=[%d %d]\n', class(v), size(v, 1), size(v, 2));
end
% 手动做一次拼接, 验证维度
try
    buf = uint8([]);
    buf = [buf(:); read(p, p.NumBytesAvailable, 'uint8')];
    fprintf('拼接 OK, buf size=[%d %d]\n', size(buf, 1), size(buf, 2));
catch e
    fprintf('拼接失败: %s\n', e.message);
end
clear p;  % 关闭端口, 让桥函数重新打开

% 裸调桥函数 25 次
for i = 1:25
    try
        [u, l] = hil_bridge_host(eta, nu, eta_d, nu_d);
        fprintf('step %2d: link=%d  u(1)=%.4f\n', i, l, u(1));
    catch e
        fprintf('step %2d: 异常!\n%s\n', i, e.getReport('extended', 'hyperlinks', 'off'));
        break
    end
end
disp('DIAG_DONE');
