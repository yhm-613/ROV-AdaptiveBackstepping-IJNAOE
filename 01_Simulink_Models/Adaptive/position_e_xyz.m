% 跟踪误差
e_x = x_act - x_d;
e_y = y_act - y_d;
e_z = z_act - z_d;
%% ===== 图2: 位置跟踪误差 =====
figure('Name', '跟踪误差收敛性', 'Position', [500, 500, 900, 500]);

% --- (a) 位置误差 ---
subplot(1,1,1);
plot(t_plot, e_x, 'k-', 'LineWidth', 1); hold on;
plot(t_plot, e_y, 'y-', 'LineWidth', 1);
plot(t_plot, e_z, 'g-', 'LineWidth', 1);
xlabel('time (s)');  ylabel('Position error (m)');
title('(a) Position tracking error', 'Interpreter', 'latex');
legend('$e_x$', '$e_y$', '$e_z$', 'Interpreter', 'latex');
grid on;
