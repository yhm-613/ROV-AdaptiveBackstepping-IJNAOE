t_vec = t(:); 
L = min([length(x_desired), length(x_actual), length(y_actual), length(z_actual)]);
t_plot = t_vec(1:L);
% 跟踪误差x_act
e_x = x_actual - x_desired;
e_y = y_actual - y_desired;
e_z = z_actual - z_desired;
%% ===== 图2: 位置跟踪误差 =====
figure('Name', '跟踪误差收敛性', 'Position', [500, 500, 900, 500]);

% --- (a) 位置误差 ---
subplot(1,1,1);
plot(t_plot, e_x, 'k-', 'LineWidth', 1); hold on;
plot(t_plot, e_y, 'y-', 'LineWidth', 1);
plot(t_plot, e_z, 'g-', 'LineWidth', 1);
xlabel('time (s)');  ylabel('position error (m)');
title('(a) Position tracking error ', 'Interpreter', 'latex');
legend('$e_x$', '$e_y$', '$e_z$', 'Interpreter', 'latex');
grid on;
