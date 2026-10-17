function u = hil_decode_status(status)
%HIL_DECODE_STATUS  Turn the HIL_ST_xxx bitfield into a readable string.
%
%   u = hil_decode_status(0x03)  ->  "INIT_DONE DATA_FRESH"

names = { 'INIT_DONE', 'DATA_FRESH', 'TIMEOUT', ...
          'RX_CRC_ERR', 'RX_OVERFLOW', 'SATURATED', 'TX_FAIL' };

status = double(status);
toks = {};
for k = 1:numel(names)
    if bitand(status, bitshift(1, k-1)) ~= 0
        toks{end+1} = names{k}; %#ok<AGROW>
    end
end
if isempty(toks)
    u = '(none)';
else
    u = strjoin(toks, ' ');
end
end
