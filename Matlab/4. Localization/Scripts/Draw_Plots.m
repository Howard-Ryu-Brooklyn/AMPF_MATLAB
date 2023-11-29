% xy position and distance
figure,
subplot(5,5,1:20)
plot(f1_pos(1,1), f1_pos(2,1), 'ro', 'LineWidth', 2); hold on;
plot(buf_f2_pos(1,:), buf_f2_pos(2,:), 'b--', 'LineWidth', 2);
xlabel('[m]'); ylabel('[m]'); grid on; grid minor; legend('target', 'robot');
subplot(5,5,21:25)
plot(t.time, buf_L.D, 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('D'); grid on; grid minor;

% % persistent excitation input
% figure,
% subplot(2,1,1)
% plot(t.time, buf_ps(1,:), 'b', 'LineWidth', 2); hold on;
% xlabel('[sec]'); ylabel('[m/s]'); legend('$ps_x$', 'interpreter', 'latex'); grid on; grid minor;
% title('persistent excitation input');
% subplot(2,1,2)
% plot(t.time, buf_ps(2,:), 'b', 'LineWidth', 2); hold on;
% xlabel('[sec]'); ylabel('[m/s]'); legend('$ps_y$', 'interpreter', 'latex');grid on; grid minor;

% localization variables
figure,
subplot(3,1,1)
plot(t.time, buf_L.z_filtered(1,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf_L.z_hat(1,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$filtered\ z_x$', '$estimated\ \hat{z}_x$',  'interpreter', 'latex');
grid on; grid minor; title('z');
subplot(3,1,2)
plot(t.time, buf_L.phi_filtered(1,:), 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); legend('$ filtered\ \phi_x$', 'interpreter', 'latex');
grid on; grid minor; title('phi x');
subplot(3,1,3)
plot(t.time, buf_L.phi_filtered(2,:), 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); legend('$ filtered\ \phi_y$', 'interpreter', 'latex');
grid on; grid minor; title('phi y');

% estimation and real value
figure,
subplot(2,1,1)
plot(t.time, f1_pos(1)*ones(1,t.length), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf_L.x_hat(1,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real\ x_x$', '$estimated\ \hat{x}_x$', 'interpreter', 'latex');
grid on; grid minor; title('x_x');
subplot(2,1,2)
plot(t.time, f1_pos(2)*ones(1,t.length), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf_L.x_hat(2,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real x_y$', '$estimated\ \hat{x}_y$', 'interpreter', 'latex');
grid on; grid minor; title('x_y');

figure,
axes_xhatdot(1) = subplot(3,1,1);
plot(t.time, buf_L.x_hat_dot(1,:), 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); ylim([-0.02 0.05]); legend('$estimated\ \dot{\hat{x}}_x$', 'interpreter', 'latex');
grid on; grid minor; title('$\dot{\hat{x}}_x$', 'interpreter', 'latex');
axes_xhatdot(2) = subplot(3,1,2);
plot(t.time, buf_L.x_hat_dot(2,:), 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); ylim([-0.02 0.05]); legend('$estimated\ \dot{\hat{x}}_y$', 'interpreter', 'latex');
grid on; grid minor; title('$\dot{\hat{x}}_y$', 'interpreter', 'latex');
axes_xhatdot(3) = subplot(3,1,3);
plot(t.time, vecnorm(buf_L.x_hat_dot), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); ylim([-0.02 0.05]); legend('$estimated\ ||\dot{\hat{x}}|| $', 'interpreter', 'latex');
grid on; grid minor; title('$||\dot{\hat{x}}||$', 'interpreter', 'latex');
linkaxes(axes_xhatdot, 'x', 'y');
