% xy position and distance
figure,
subplot(5,5,1:20)
plot(x_pos(1,1), x_pos(2,1), 'ro', 'LineWidth', 2); hold on;
plot(buf.y_pos(1,:), buf.y_pos(2,:), 'b--', 'LineWidth', 2);
xlabel('[m]'); ylabel('[m]'); grid on; grid minor; legend('target', 'robot');
subplot(5,5,21:25)
plot(t.time, buf.D, 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('D'); grid on; grid minor;

% persistent excitation input
figure,
subplot(2,1,1)
plot(t.time, buf.ps(1,:), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$ps_x$', 'interpreter', 'latex'); grid on; grid minor;
title('persistent excitation input');
subplot(2,1,2)
plot(t.time, buf.ps(2,:), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$ps_y$', 'interpreter', 'latex');grid on; grid minor;

% localization variables
figure,
subplot(4,1,1)
plot(t.time, buf.z_real(1,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf.z_filtered(1,:), 'k--', 'LineWidth', 2);
plot(t.time, buf.z_hat(1,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real\ z_x$', '$filtered\ z_x$', '$estimated\ \hat{z}_x$',  'interpreter', 'latex');
grid on; grid minor; title('z');
subplot(4,1,2)
plot(t.time, buf.z_real(2,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf.z_filtered(2,:), 'k--', 'LineWidth', 2);
plot(t.time, buf.z_hat(2,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real\ z_y$', '$filtered\ z_y$', '$estimated\ \hat{z}_y$', 'interpreter', 'latex');
grid on; grid minor;
subplot(4,1,3)
plot(t.time, buf.phi_real(1,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf.phi_filtered(1,:), 'k--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); legend('$real\ \phi_x$', '$ filtered\ \phi_x$', 'interpreter', 'latex');
grid on; grid minor; title('phi');
subplot(4,1,4)
plot(t.time, buf.phi_real(2,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf.phi_filtered(2,:), 'k--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); legend('$real\ \phi_y$', '$ filtered\ \phi_y$', 'interpreter', 'latex');
grid on; grid minor;

% estimation and real value
figure,
subplot(2,1,1)
plot(t.time, x_pos(1)*ones(1,t.length), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf.x_hat(1,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real\ x_x$', '$estimated\ \hat{x}_x$', 'interpreter', 'latex');
grid on; grid minor; title('x_x');
subplot(2,1,2)
plot(t.time, x_pos(2)*ones(1,t.length), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf.x_hat(2,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real x_y$', '$estimated\ \hat{x}_y$', 'interpreter', 'latex');
grid on; grid minor; title('x_y');
