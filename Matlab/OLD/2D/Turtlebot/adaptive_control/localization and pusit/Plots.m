figure,
subplot(7,5,1:20)
plot(buf_y_pos(1,:), buf_y_pos(2,:), 'b--', 'LineWidth', 2); hold on;
plot(x_pos(1,1), x_pos(2,1), 'ro', 'LineWidth', 2); hold on;
plot(buf_y_pos(1,1), buf_y_pos(2,1), 'bo', 'LineWidth', 2); hold on;
plot(buf_y_pos(1,end), buf_y_pos(2,end), 'ko', 'LineWidth', 2);
xlabel('[m]'); ylabel('[m]'); grid on; grid minor; legend('target', 'robot init', 'robot end', 'traj');
subplot(7,5,21:25)
plot(t.time, buf_D, 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('D'); grid on; grid minor;
subplot(7,5,26:30)
plot(t.time, buf_y_pos(1,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, x_pos(1)*ones(1,t.length), 'r', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('y pos x', 'x pos x'); grid on; grid minor;
subplot(7,5,31:35)
plot(t.time, buf_y_pos(2,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, x_pos(1)*ones(1,t.length), 'r', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('y pos y', 'x pos x'); grid on; grid minor;

figure,
ax_cmd(7) = subplot(3,3,1);
plot(t.time, buf_cmd(1,:), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$cmd_x$', 'interpreter', 'latex'); grid on; grid minor;
title('persistent excitation input');
ax_cmd(8) = subplot(3,3,4);
plot(t.time, buf_cmd(2,:), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$cmd_y$', 'interpreter', 'latex');grid on; grid minor;
ax_cmd(9) = subplot(3,3,7);
plot(t.time, vecnorm(buf_cmd), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$||cmd||$', 'interpreter', 'latex');grid on; grid minor;
ax_cmd(1) = subplot(3,3,2);
plot(t.time, buf_cmd_part1(1,:), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$cmd_x$', 'interpreter', 'latex');grid on; grid minor;
title('cmd for pursuit');
ax_cmd(2) = subplot(3,3,5);
plot(t.time, buf_cmd_part1(2,:), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$cmd_y$', 'interpreter', 'latex');grid on; grid minor;
ax_cmd(3) = subplot(3,3,8);
plot(t.time, vecnorm(buf_cmd_part1), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$||cmd||$', 'interpreter', 'latex');grid on; grid minor;
ax_cmd(4) = subplot(3,3,3);
plot(t.time, buf_cmd_part2(1,:), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$cmd_x$', 'interpreter', 'latex');grid on; grid minor;
title('cmd for localization');
ax_cmd(5) = subplot(3,3,6);
plot(t.time, buf_cmd_part2(2,:), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$cmd_y$', 'interpreter', 'latex');grid on; grid minor;
ax_cmd(6) = subplot(3,3,9);
plot(t.time, vecnorm(buf_cmd_part2), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$||cmd||$', 'interpreter', 'latex');grid on; grid minor;
linkaxes(ax_cmd, 'xy');


% figure,
% axs(1) = subplot(4,1,1);
% plot(t.time, buf_sigma(1,:), 'b', 'LineWidth', 2); hold on;
% xlabel('[sec]'); ylabel('[-]'); legend('$\sigma_x$', 'interpreter', 'latex');grid on; grid minor;
% axs(2) = subplot(4,1,2);
% plot(t.time, buf_sigma(2,:), 'b', 'LineWidth', 2); hold on;
% xlabel('[sec]'); ylabel('[-]'); legend('$\sigma_y$', 'interpreter', 'latex');grid on; grid minor;
% axs(3) = subplot(4,1,3);
% plot(t.time, buf_sigma_dot(1,:), 'b', 'LineWidth', 2); hold on;
% xlabel('[sec]'); ylabel('[-]'); legend('$\dot{\sigma}_x$', 'interpreter', 'latex');grid on; grid minor;
% axs(4) = subplot(4,1,4);
% plot(t.time, buf_sigma_dot(2,:), 'b', 'LineWidth', 2); hold on;
% xlabel('[sec]'); ylabel('[-]'); legend('$\dot{\sigma}_y$', 'interpreter', 'latex');grid on; grid minor;
% linkaxes(axs, 'x');

if (strcmp(model,"NH"))
    figure,
    axvw(1) = subplot(3,1,1);
    plot(t.time, buf_cmdvw(1,:), 'LineWidth', 2);
    xlabel('[sec]'); ylabel('[m/s]'); legend('$v$', 'interpreter', 'latex'); grid on; grid minor;
    axvw(2) = subplot(3,1,2);
    plot(t.time, buf_cmdvw(2,:), 'LineWidth', 2);
    xlabel('[sec]'); ylabel('[rad/s]'); legend('$v$', 'interpreter', 'latex'); grid on; grid minor;
    
    subplot(3,1,3)
    plot(t.time, rem(buf_y_att(1,:)*R2D,360)-180, 'LineWidth', 2);
    xlabel('[sec]'); ylabel('[deg]'); legend('$attitude$', 'interpreter', 'latex');
    grid on; grid minor; title('attitude');
    linkaxes(axvw, 'x');
end

% lim = 1.5;
% figure,
% ax7 = subplot(4,1,1);
% plot(t.time, buf_z_real(1,:), 'b', 'LineWidth', 2); hold on;
% plot(t.time, buf_z_filtered(1,:), 'k--', 'LineWidth', 2);
% plot(t.time, buf_z_hat(1,:), 'r--', 'LineWidth', 2);
% xlabel('[sec]'); ylabel('[m]'); legend('$real\ z_x$', '$filtered\ z_x$', '$estimated\ \hat{z}_x$',  'interpreter', 'latex');
% grid on; grid minor; title('z'); ylim([-lim, lim]);
% ax8 = subplot(4,1,2);
% plot(t.time, buf_z_real(2,:), 'b', 'LineWidth', 2); hold on;
% plot(t.time, buf_z_filtered(2,:), 'k--', 'LineWidth', 2);
% plot(t.time, buf_z_hat(2,:), 'r--', 'LineWidth', 2);
% xlabel('[sec]'); ylabel('[m]'); legend('$real\ z_y$', '$filtered\ z_y$', '$estimated\ \hat{z}_y$', 'interpreter', 'latex');
% grid on; grid minor; ylim([-lim, lim]);
% ax9 = subplot(4,1,3);
% plot(t.time, buf_phi_real(1,:), 'b', 'LineWidth', 2); hold on;
% plot(t.time, buf_phi_filtered(1,:), 'k--', 'LineWidth', 2);
% xlabel('[sec]'); ylabel('[m/s]'); legend('$real\ \phi_x$', '$ filtered\ \phi_x$', 'interpreter', 'latex');
% grid on; grid minor; title('phi'); ylim([-lim, lim]);
% ax10 = subplot(4,1,4);
% plot(t.time, buf_phi_real(2,:), 'b', 'LineWidth', 2); hold on;
% plot(t.time, buf_phi_filtered(2,:), 'k--', 'LineWidth', 2);
% xlabel('[sec]'); ylabel('[m/s]'); legend('$real\ \phi_y$', '$ filtered\ \phi_y$', 'interpreter', 'latex');
% grid on; grid minor;  ylim([-lim, lim]);
% linkaxes([ax7, ax8, ax9, ax10], 'xy');




figure,
ax11 = subplot(5,1,1);
plot(t.time, x_pos(1)*ones(1,t.length), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf_x_hat(1,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real\ x_x$', '$estimated\ \hat{x}_x$', 'interpreter', 'latex');
grid on; grid minor; title('x_x');
ax12 = subplot(5,1,2);
plot(t.time, x_pos(2)*ones(1,t.length), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf_x_hat(2,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real x_y$', '$estimated\ \hat{x}_y$', 'interpreter', 'latex');
grid on; grid minor; title('x_y');

subplot(5,1,3);
plot(t.time, norm(x_pos) - vecnorm(buf_x_hat), 'b', 'LineWidth', 2); hold on;
plot(t.time, zeros(1,t.length), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$estimated\ err\ \hat{x} - x$', 'interpreter', 'latex');
grid on; grid minor; title('x err');

axhat(1) = subplot(5,1,4);
plot(t.time, buf_x_hat_dot(1,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, zeros(1,t.length), 'r --', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); legend('$\dot{\hat{x}}_x$', 'interpreter', 'latex');
grid on; grid minor; ylim([-0.2 0.2]);
axhat(2) = subplot(5,1,5);
plot(t.time, buf_x_hat_dot(2,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, zeros(1,t.length), 'r --', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m/s]'); legend('$\dot{\hat{x}}_y$', 'interpreter', 'latex');
grid on; grid minor; ylim([-0.2 0.2]);

linkaxes([ax11 ax12],'xy');
linkaxes(axhat, 'xy');


