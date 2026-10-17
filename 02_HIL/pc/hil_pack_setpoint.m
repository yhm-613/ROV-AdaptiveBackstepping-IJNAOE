function frame = hil_pack_setpoint(eta, nu, eta_d, nu_d, seq)
%HIL_PACK_SETPOINT  Build a 104-byte SETPOINT frame (PC -> MCU).
%
%   frame = hil_pack_setpoint(eta6, nu6, eta_d6, nu_d6, seq)
%
%   eta, nu, eta_d, nu_d : 6x1 double / single, little-endian float32
%   seq                  : scalar, uint16 rolling counter
%   frame                : 104x1 uint8
%
%   Layout:  55 AA 01 62 | eta(24) nu(24) eta_d(24) nu_d(24) seq(2) | crc(2)
%
%   NOTE: typecast() keeps the row count of the input, so every block is
%         explicitly re-shaped into a row vector in column-major order.

eta   = double(eta(:));   assert(numel(eta)   == 6, 'eta must be 6');
nu    = double(nu(:));    assert(numel(nu)    == 6, 'nu must be 6');
eta_d = double(eta_d(:)); assert(numel(eta_d) == 6, 'eta_d must be 6');
nu_d  = double(nu_d(:));  assert(numel(nu_d)  == 6, 'nu_d must be 6');

b_eta   = reshape(typecast(single(eta),   'uint8'), 1, []);
b_nu    = reshape(typecast(single(nu),    'uint8'), 1, []);
b_eta_d = reshape(typecast(single(eta_d), 'uint8'), 1, []);
b_nu_d  = reshape(typecast(single(nu_d),  'uint8'), 1, []);
b_seq   = reshape(typecast(uint16(seq),   'uint8'), 1, []);

payload = [b_eta, b_nu, b_eta_d, b_nu_d, b_seq];
assert(numel(payload) == 98, 'payload must be 98 bytes, got %d', numel(payload));

head  = uint8([hex2dec('55'), hex2dec('AA'), 1, 98]);
body  = [head, payload];
crc   = hil_crc16(body);
frame = [body, reshape(typecast(crc, 'uint8'), 1, [])]';
end
