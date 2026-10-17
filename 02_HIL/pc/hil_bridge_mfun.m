function y = hil_bridge_mfun(u)
%HIL_BRIDGE_MFUN  Interpreted MATLAB Function block body for rov_hil_host.slx
%
%   输入 u : 24x1 = [eta(6); nu(6); eta_d(6); nu_d(6)]
%   输出 y :  7x1 = [u_cmd(6); link(1)]
%
%   link : 0 = 正常，1 = 超时（沿用上一拍指令），2 = 过程中丢弃过坏帧，
%          3 = 串口异常（沿用上一拍指令）
%
%   真正的收发在 hil_bridge_host.m 里完成；这个包装只负责拆/拼向量。
%   用 Interpreted MATLAB Function 块而不是 MATLAB Function 块，
%   是因为它不需要注入 Stateflow 代码、也不需要编译，脚本化最稳。

persistent u_hold
if isempty(u_hold), u_hold = zeros(6,1); end

u = u(:);
eta   = u(1:6);
nu    = u(7:12);
eta_d = u(13:18);
nu_d  = u(19:24);

uc   = u_hold;
link = 0;
try
    [uc, link] = hil_bridge_host(eta, nu, eta_d, nu_d);
    u_hold = uc;
catch err
    % 串口打不开 / 拔线：不要让仿真直接崩，保持上一拍指令并打标记
    warning('hil_bridge_mfun:fail', 'HIL 收发失败: %s', err.message);
    uc   = u_hold;
    link = 3;
end

y = [uc(:); double(link)];
end
