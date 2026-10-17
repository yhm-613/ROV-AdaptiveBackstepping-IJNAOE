function tau_sw = smc_aug(x)
%SMC_AUG  边界层滑模切换项: tau_sw = -RHO .* sat(z2/PHI)
%  x = [V(6); alpha1(6)] (控制器内 Mux 拼接), z2 = V - alpha1
%  RHO_SMC (6x1), PHI_SMC (标量) 每步从 base 工作区读取 (支持扫参)
%  维度防御: 全部强制列向量, 防止行/列隐式扩展产生 6x6
rho = evalin('base', 'RHO_SMC'); rho = rho(:);
phi = evalin('base', 'PHI_SMC');
s = x(1:6) - x(7:12);          % z2 = nu - alpha
s = s(:);                      % 强制 6x1 列
v = max(-1, min(1, s ./ phi)); % sat(v/phi), phi > 0 边界层厚度
tau_sw = -rho .* v;            % 切换增益, 元素独立, 输出 6x1
end
