function tau = hil_current_force(u)
%HIL_CURRENT_FORCE  Equivalent ocean-current load of paper Eq. (19).
%   u = [eta(6); t]  (vehicle pose, simulation time) -> psi = u(6)
%   Profile and direction come from base workspace:
%     HIL_CURR_PROF : 0 = none, 1 = constant Uc, 2 = slowly varying
%     HIL_CURR_BETA : current direction (rad, earth-fixed)
%   tau_c = DL*nu_c + DQ.*|nu_c|*nu_c,  nu_c = [Uc cos(beta-psi); Uc sin(beta-psi); 0...]
psi = u(6);
t   = u(7);
prof = evalin('base', 'HIL_CURR_PROF');
beta = evalin('base', 'HIL_CURR_BETA');
tau = zeros(6, 1);
if prof < 1
    return
end
switch prof
    case 1
        Uc = 0.3;                       % constant 0.3 m/s (paper 4.3)
    case 2
        Uc = 0.2 + 0.1*sin(0.05*t);     % slowly varying (paper 4.3)
    otherwise
        error('bad HIL_CURR_PROF');
end
uc = Uc * cos(beta - psi);
vc = Uc * sin(beta - psi);
DL = evalin('base', 'DL');
DQ = evalin('base', 'DQ');
tau(1) = DL(1)*uc + DQ(1)*uc*abs(uc);
tau(2) = DL(2)*vc + DQ(2)*vc*abs(vc);
end
