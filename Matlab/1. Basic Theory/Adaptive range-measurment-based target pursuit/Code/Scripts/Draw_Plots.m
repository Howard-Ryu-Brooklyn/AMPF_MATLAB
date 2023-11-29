% xy position, distance, and tracking
figure,
subplot(7,5,1:20)
plot(buf.y_pos(1,:), buf.y_pos(2,:), 'b--', 'LineWidth', 2); hold on;
plot(x_pos(1,1), x_pos(2,1), 'ro', 'LineWidth', 2); hold on;
plot(buf.y_pos(1,1), buf.y_pos(2,1), 'bo', 'LineWidth', 2); hold on;
plot(buf.y_pos(1,end), buf.y_pos(2,end), 'ko', 'LineWidth', 2);
xlabel('[m]'); ylabel('[m]'); grid on; grid minor; legend('target', 'robot init', 'robot end', 'traj');
title("xy position, distance, and tracking");
subplot(7,5,21:25)
plot(t.time, buf.D, 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('D'); grid on; grid minor;
subplot(7,5,26:30)
plot(t.time, buf.y_pos(1,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, x_pos(1)*ones(1,t.length), 'r', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('y pos x', 'x pos x'); grid on; grid minor;
subplot(7,5,31:35)
plot(t.time, buf.y_pos(2,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, x_pos(1)*ones(1,t.length), 'r', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('y pos y', 'x pos x'); grid on; grid minor;

% localization and tracking command
figure,
subplot(3,1,1);
plot(t.time, buf.cmd(1,:), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$cmd_x$', 'interpreter', 'latex'); grid on; grid minor;
title('persistent excitation tracking command');
subplot(3,1,2);
plot(t.time, buf.cmd(2,:), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$cmd_y$', 'interpreter', 'latex');grid on; grid minor;
subplot(3,1,3);
plot(t.time, vecnorm(buf.cmd), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$||cmd||$', 'interpreter', 'latex');grid on; grid minor;

% persistent excitation part
figure,
subplot(4,1,1);
plot(t.time, buf.sigma(1,:), 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[-]'); legend('$\sigma_x$', 'interpreter', 'latex');grid on; grid minor;
title('persistent excitation part: sigma input');
subplot(4,1,2);
plot(t.time, buf.sigma(2,:), 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[-]'); legend('$\sigma_y$', 'interpreter', 'latex');grid on; grid minor;
subplot(4,1,3);
plot(t.time, buf.sigma_dot(1,:), 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[-]'); legend('$\dot{\sigma}_x$', 'interpreter', 'latex');grid on; grid minor;
subplot(4,1,4);
plot(t.time, buf.sigma_dot(2,:), 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[-]'); legend('$\dot{\sigma}_y$', 'interpreter', 'latex');grid on; grid minor;

lim = 1.5;
figure,
ax7 = subplot(4,1,1);
plot(t.time, buf.z_real(1,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf.z_filtered(1,:), 'k--', 'LineWidth', 2);
plot(t.time, buf.z_hat(1,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real\ z_x$', '$filtered\ z_x$', '$estimated\ \hat{z}_x$',  'interpreter', 'latex');
grid on; grid minor; title("localization varable: z"); ylim([-lim, lim]);
ax8 = subplot(4,1,2);
plot(t.time, buf.z_real(2,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf.z_filtered(2,:), 'k--', 'LineWidth', 2);
plot(t.time, buf.z_hat(2,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real\ z_y$', '$filtered\ z_y$', '$estimated\ \hat{z}_y$', 'interpreter', 'latex');
grid on; grid minor; ylim([-lim, lim]);
ax9 = subplot(4,1,3);
plot(t.time, buf.phi_real(1,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf.phi_filtered(1,:), 'k--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); legend('$real\ \phi_x$', '$ filtered\ \phi_x$', 'interpreter', 'latex');
grid on; grid minor; title('localization varables: phi'); ylim([-lim, lim]);
ax10 = subplot(4,1,4);
plot(t.time, buf.phi_real(2,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf.phi_filtered(2,:), 'k--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); legend('$real\ \phi_y$', '$ filtered\ \phi_y$', 'interpreter', 'latex');
grid on; grid minor;  ylim([-lim, lim]);
linkaxes([ax7, ax8, ax9, ax10], 'xy');




figure,
ax11 = subplot(5,1,1);
plot(t.time, x_pos(1)*ones(1,t.length), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf.x_hat(1,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real\ x_x$', '$estimated\ \hat{x}_x$', 'interpreter', 'latex');
grid on; grid minor; title('x_x');
ax12 = subplot(5,1,2);
plot(t.time, x_pos(2)*ones(1,t.length), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf.x_hat(2,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real x_y$', '$estimated\ \hat{x}_y$', 'interpreter', 'latex');
grid on; grid minor; title('x_y');

subplot(5,1,3);
plot(t.time, norm(x_pos) - vecnorm(buf.x_hat), 'b', 'LineWidth', 2); hold on;
plot(t.time, zeros(1,t.length), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$estimated\ err\ \hat{x} - x$', 'interpreter', 'latex');
grid on; grid minor; title('x err');

axhat(1) = subplot(5,1,4);
plot(t.time, buf.x_hat_dot(1,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, zeros(1,t.length), 'r --', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); legend('$\dot{\hat{x}}_x$', 'interpreter', 'latex');
grid on; grid minor; ylim([-0.2 0.2]); title('x dot x');
axhat(2) = subplot(5,1,5);
plot(t.time, buf.x_hat_dot(2,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, zeros(1,t.length), 'r --', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); legend('$\dot{\hat{x}}_y$', 'interpreter', 'latex');
grid on; grid minor; ylim([-0.2 0.2]); title('x dot y');

linkaxes([ax11 ax12],'xy');
linkaxes(axhat, 'xy');
