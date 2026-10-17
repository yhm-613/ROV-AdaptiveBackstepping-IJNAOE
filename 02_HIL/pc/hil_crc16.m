function crc = hil_crc16(bytes)
%HIL_CRC16  CRC16-Modbus (poly 0x8005 reflected -> 0xA001, init 0xFFFF).
%
%   Must match hil_crc16() in firmware/hil_protocol.h byte for byte.
%
%   crc = hil_crc16(uint8([0x31 0x32 ...]))

crc = uint16(65535);
for i = 1:numel(bytes)
    crc = bitxor(crc, uint16(bytes(i)));
    for b = 1:8
        if bitand(crc, uint16(1)) ~= 0
            crc = bitxor(bitshift(crc, -1), uint16(hex2dec('A001')));
        else
            crc = bitshift(crc, -1);
        end
    end
end
end
