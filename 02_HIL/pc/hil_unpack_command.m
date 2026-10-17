function cmd = hil_unpack_command(frame)
%HIL_UNPACK_COMMAND  Decode a 34-byte COMMAND frame (MCU -> PC).
%
%   cmd = hil_unpack_command(frame)   % frame: 34x1 uint8 (or 1x34)
%
%   cmd.u      : 6x1 double, thruster voltages [V]
%   cmd.seq    : uint16, echoes the setpoint sequence that was consumed
%   cmd.status : uint16, HIL_ST_xxx bitfield
%
%   Throws if the sync bytes, the type, the length or the CRC are wrong.

frame = uint8(frame(:));
assert(numel(frame) == 34, 'frame must be 34 bytes, got %d', numel(frame));
assert(frame(1) == hex2dec('55') && frame(2) == hex2dec('AA'), 'bad sync');
assert(frame(3) == 2,   'not a COMMAND frame (type=%d)', frame(3));
assert(frame(4) == 28,  'bad payload length (%d)', frame(4));

rx  = double(frame(33)) + 256 * double(frame(34));
cal = double(hil_crc16(frame(1:32)));
assert(rx == cal, 'CRC mismatch: rx=0x%04X calc=0x%04X', rx, cal);

% column-major float32 decode
u = double(typecast(frame(5:28), 'single'));
cmd.u      = u(:);
cmd.seq    = uint16(double(frame(29)) + 256 * double(frame(30)));
cmd.status = uint16(double(frame(31)) + 256 * double(frame(32)));
end
