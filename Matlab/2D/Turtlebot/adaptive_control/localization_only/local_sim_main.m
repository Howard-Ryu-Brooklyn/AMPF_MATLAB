clc; close all; clear;

% simulation time
t = struct( ...
    'start', 0, ...
    'final', 50, ...
    'ts', 0.01, ...
    'time', 0, ...
    'length', 0);

t.time = t.start:t.ts:t.final-t.ts;
t.length = length(t.time);

% utilities
D2R=pi/180;
R2D=1/D2R;

% control variable
alpha = 1; % state filter constant
gamma = 1; % adaptive law constant
beta = 3; % control law constant
b = 1; c = 1; %
rho = 0.1; %
sigma_init = [0.6, 0.6]; %

% inital value
dim = 2; % demension
x_init_pos = [2, 3]'; %[m, m] target initial position
y_init_pos = [5, 5]'; %[m, m] robot initial position
zeta1_init = 0; % could be arbitary scalar
zeta2_init = zeros(dim, 1);

% sim variable and buffer
z_real = zeros(dim, 1); % real z
phi_real = zeros(dim, 1); % real phi

z_filtered = zeros(dim, 1); % filtered z
phi_filtered = zeros(dim, 1); % filtered phi

z_hat = zeros(dim, 1);
x_hat = zeros(dim, 1);
x_hat_dot = zeros(dim, 1);

D = norm(x_init_pos - y_init_pos); % distance bewteen target and robot

% state variable for filter
zeta1 = zeta1_init;
zeta2 = zeta2_init;
zeta1_dot = zeros(dim, 1);
zeta2_dot = zeros(dim, 1);

x_pos = x_init_pos;
y_pos = y_init_pos;
y_pre = y_pos;
y_cur = zeros(dim , 1);

pre_zv = norm(y_pos)^2 - D^2;
cur_zv = 0;

ps = zeros(dim, 1); % persistent exciting signal

% buffer
buf_D = zeros(1, t.length);

buf_ps = zeros(dim, t.length);
buf_y_pos = zeros(dim, t.length);

buf_phi_filtered = zeros(dim, t.length);
buf_phi_real = zeros(dim, t.length);
buf_z_filtered = zeros(dim, t.length);
buf_z_real = zeros(dim, t.length);
buf_z_hat = zeros(dim, t.length);
buf_x_hat = zeros(dim, t.length);



for i=1:t.length
    % get distance measurement
    D = norm(y_pos - x_pos);

    % calcuate real value of z and phi
    y_cur = y_pos;
    y_dot = (y_cur - y_pre)/t.ts;
    cur_zv = norm(y_pos)^2 - D^2;
    z_real = 0.5*(cur_zv - pre_zv)/t.ts;
    phi_real = y_dot;

    % calculate filter
    zeta1_dot = -alpha*zeta1 + 0.5*(norm(y_pos)^2 - D^2);
    zeta2_dot = -alpha*zeta2 + y_pos;
    zeta1 = zeta1 + zeta1_dot*t.ts;
    zeta2 = zeta2 + zeta2_dot*t.ts;

    z_filtered = zeta1_dot;
    phi_filtered = zeta2_dot;



    % persistently excited
    ps = [2*sin(i*t.ts), 2*cos(2*i*t.ts)]';
    if(i*t.ts > 30)
        ps = -1.5*(y_pos-x_hat);
        y_pos = y_pos + ps*t.ts;
        % while pursuiting, esitmation could get damaged for some reason.
        % which causes pursuit control law broken
    else
        z_hat = x_hat'*phi_filtered;
        % estimation law
        x_hat_dot = gamma*(z_filtered - z_hat)'*phi_filtered;
        x_hat = x_hat + x_hat_dot*t.ts;

        y_pos = y_pos + ps*t.ts;
    end


    % update variable
    y_pre = y_cur;
    pre_zv = cur_zv;

    % save buffer
    buf_D(1,i) = D;

    buf_z_real(:,i) = z_real;
    buf_z_filtered(:,i) = z_filtered;

    buf_phi_real(:,i) = phi_real;
    buf_phi_filtered(:,i) = phi_filtered;

    buf_y_pos(:,i) = y_pos;
    buf_ps(:,i) = ps;

    buf_z_hat(:,i) = z_hat;
    buf_x_hat(:,i) = x_hat;
end



figure,
subplot(5,5,1:20)
plot(x_pos(1,1), x_pos(2,1), 'ro', 'LineWidth', 2); hold on;
plot(buf_y_pos(1,:), buf_y_pos(2,:), 'b--', 'LineWidth', 2);
xlabel('[m]'); ylabel('[m]'); grid on; grid minor; legend('target', 'robot');
subplot(5,5,21:25)
plot(t.time, buf_D, 'b', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('D'); grid on; grid minor;
% 
figure,
subplot(2,1,1)
plot(t.time, buf_ps(1,:), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$ps_x$', 'interpreter', 'latex'); grid on; grid minor;
title('persistent excitation input');
subplot(2,1,2)
plot(t.time, buf_ps(2,:), 'b', 'LineWidth', 2); hold on;
xlabel('[sec]'); ylabel('[m/s]'); legend('$ps_y$', 'interpreter', 'latex');grid on; grid minor;
% 
% 
% figure,
% subplot(4,1,1)
% plot(t.time, buf_z_real(1,:), 'b', 'LineWidth', 2); hold on;
% plot(t.time, buf_z_filtered(1,:), 'k--', 'LineWidth', 2);
% plot(t.time, buf_z_hat(1,:), 'r--', 'LineWidth', 2);
% xlabel('[sec]'); ylabel('[m]'); legend('$real\ z_x$', '$filtered\ z_x$', '$estimated\ \hat{z}_x$',  'interpreter', 'latex');
% grid on; grid minor; title('z');
% subplot(4,1,2)
% plot(t.time, buf_z_real(2,:), 'b', 'LineWidth', 2); hold on;
% plot(t.time, buf_z_filtered(2,:), 'k--', 'LineWidth', 2);
% plot(t.time, buf_z_hat(2,:), 'r--', 'LineWidth', 2);
% xlabel('[sec]'); ylabel('[m]'); legend('$real\ z_y$', '$filtered\ z_y$', '$estimated\ \hat{z}_y$', 'interpreter', 'latex');
% grid on; grid minor;
% subplot(4,1,3)
% plot(t.time, buf_phi_real(1,:), 'b', 'LineWidth', 2); hold on;
% plot(t.time, buf_phi_filtered(1,:), 'k--', 'LineWidth', 2);
% xlabel('[sec]'); ylabel('[m/s]'); legend('$real\ \phi_x$', '$ filtered\ \phi_x$', 'interpreter', 'latex');
% grid on; grid minor; title('phi');
% subplot(4,1,4)
% plot(t.time, buf_phi_real(2,:), 'b', 'LineWidth', 2); hold on;
% plot(t.time, buf_phi_filtered(2,:), 'k--', 'LineWidth', 2);
% xlabel('[sec]'); ylabel('[m/s]'); legend('$real\ \phi_y$', '$ filtered\ \phi_y$', 'interpreter', 'latex');
% grid on; grid minor;

figure,
subplot(2,1,1)
plot(t.time, x_pos(1)*ones(1,t.length), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf_x_hat(1,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real\ x_x$', '$estimated\ \hat{x}_x$', 'interpreter', 'latex');
grid on; grid minor; title('x_x');
subplot(2,1,2)
plot(t.time, x_pos(2)*ones(1,t.length), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf_x_hat(2,:), 'r--', 'LineWidth', 2);
xlabel('[sec]'); ylabel('[m]'); legend('$real x_y$', '$estimated\ \hat{x}_y$', 'interpreter', 'latex');
grid on; grid minor; title('x_y');

