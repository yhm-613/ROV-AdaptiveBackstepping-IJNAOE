function [u, link] = hil_bridge_host(eta, nu, eta_d, nu_d)
%HIL_BRIDGE_HOST  Blocking, frame-synchronised HIL exchange with the MCU.
%
%   [u, link] = hil_bridge_host(eta, nu, eta_d, nu_d)
%
%   Called once per Simulink step (see hil_bridge.m).  Each call performs
%   exactly ONE round trip:
%
%       1. pack a 104-byte SETPOINT frame and write it to the serial port
%       2. block until a CRC-valid 34-byte COMMAND frame comes back
%          (or HIL_TIMEOUT_S elapses)
%       3. return the 6 thruster voltages
%
%   Because the call blocks until the answer arrives, "one Simulink step"
%   is exactly "one MCU control period".  The simulation therefore keeps a
%   1:1 mapping between sim time and the controller's own 10 ms ticks even
%   if the PC runs slower than real time.
%
%   link : 0 = ok, 1 = timeout (previous u held), 2 = at least one bad CRC
%          was skipped during this exchange.
%
%   The serial port is opened on the first call and kept in a persistent
%   variable.  Change HIL_PORT below to match your USB-TTL adapter.

% ------------------------------------------------------------------ %
HIL_PORT      = 'COM14';    % 板载 CH340G (YS-F4Pro), 见 serialportlist(); 2026-10-13 重插后由 COM11 变为 COM14
HIL_BAUD      = 460800;     % must match hil_port_stm32f4.c
HIL_TIMEOUT_S = 0.050;      % 5 control periods
HIL_PERIOD_S  = 0.010;      % 实时节流: 每步墙钟 >= 控制周期, 防突发淹没板上 256B RX 环
                            % 拍照模式曾临时改为 0.100 (20s 闭环约 4 分钟), 拍完已恢复
% ------------------------------------------------------------------ %

persistent port rxbuf seq_tx n_timeout n_bad u_last t_step
if isempty(port)
    port = serialport(HIL_PORT, HIL_BAUD, 'Timeout', 0.02);
    flush(port);
    rxbuf     = uint8([]);
    seq_tx    = uint16(0);
    n_timeout = 0;
    n_bad     = 0;
    u_last    = zeros(6,1);
    t_step    = tic;
    fprintf('[HIL] %s opened @ %d baud\n', HIL_PORT, HIL_BAUD);
end

% --- 1. send the setpoint -----------------------------------------
seq_tx = seq_tx + uint16(1);
% 关键: 发送前丢弃 rxbuf 里积压的自发帧. 板子每 tick 自发一帧 command
% (可能基于好几拍之前的 setpoint), 不清空的话 PC 读到的"下一帧"是陈旧的,
% 环路等效延迟高达 4-5 拍, 足以让 10ms 自适应反步闭环翻滚失稳.
rxbuf = uint8([]);
write(port, hil_pack_setpoint(eta, nu, eta_d, nu_d, seq_tx), 'uint8');

% --- 2. block until a valid command frame is decoded --------------
% DATA_FRESH=0 的帧表示板子该 tick 没吃到新 setpoint (u 基于旧数据),
% 最多跳过 2 帧等 fresh 帧; 超时兜底逻辑不变
t0   = tic;
cmd  = [];
link = 0;
n_stale = 0;
while isempty(cmd)
    n = port.NumBytesAvailable;
    if n > 0
        % R2024b 的 read 返回 1xN double 行向量 (R2023b 是 Nx1 uint8 列),
        % 必须显式归一成 uint8 列向量, 否则 vertcat 在第二步起必然报维度不一致
        rxbuf = [rxbuf(:); reshape(uint8(read(port, n, 'uint8')), [], 1)]; %#ok<AGROW>
    end

    while numel(rxbuf) >= 34
        if rxbuf(1) == 85 && rxbuf(2) == 170 && rxbuf(3) == 2 && rxbuf(4) == 28
            cand = rxbuf(1:34);
            if double(hil_crc16(cand(1:32))) == ...
               double(cand(33)) + 256 * double(cand(34))
                cand_cmd = hil_unpack_command(cand);
                rxbuf = rxbuf(35:end);
                fresh = bitand(double(cand_cmd.status), 2) ~= 0;  % HIL_ST_DATA_FRESH
                if fresh || n_stale >= 2
                    cmd = cand_cmd;        % fresh 帧, 或已等过 2 帧就用最新的
                    break
                end
                n_stale = n_stale + 1;     % 陈旧帧: 丢弃, 继续等
            else
                n_bad = n_bad + 1;         % bad CRC -> skip header, resync
                link  = 2;
                rxbuf = rxbuf(3:end);
            end
        else
            rxbuf = rxbuf(2:end);          % slide the window
        end
    end

    if isempty(cmd)
        if toc(t0) > HIL_TIMEOUT_S
            n_timeout = n_timeout + 1;
            link = 1;
            break
        end
        pause(0.0005);
    end
end

% --- 3. hand the result (or the previous u) to the plant ----------
if isempty(cmd)
    u = u_last;                                % hold last command
else
    u = cmd.u;
    u_last = u;
end

% --- 3.5 实时节流: 本步墙钟耗时不足一个控制周期则补足 ---------------
% PC 仿真不等实时, 一旦回复已缓冲就会突发连发 setpoint,
% 板上 256B RX 环只装 2.5 帧, 突发导致 ~25% 丢帧并使自适应律发散
el = toc(t_step);
if el < HIL_PERIOD_S
    pause(HIL_PERIOD_S - el);
end
t_step = tic;

% --- 4. occasional diagnostics ------------------------------------
if mod(double(seq_tx), 200) == 0
    fprintf('[HIL] frames=%d  timeouts=%d  badcrc=%d  status=%s\n', ...
        double(seq_tx), n_timeout, n_bad, hil_decode_status(cmd_status(cmd)));
end
end

% ------------------------------------------------------------------
function s = cmd_status(cmd)
if isempty(cmd)
    s = uint16(0);
else
    s = cmd.status;
end
end
