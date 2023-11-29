% simulation time
t = struct( ...
    'start', 0, ...
    'final', 50, ...
    'ts', 0.01, ...
    'time', 0, ...
    'length', 0);

t.time = t.start:t.ts:t.final-t.ts;
t.length = length(t.time);

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
buf=struct('D', zeros(1, t.length), ...
    'ps', zeros(dim, t.length), ...
    'y_pos', zeros(dim, t.length), ...
    'phi_filtered', zeros(dim, t.length), ...
    'phi_real', zeros(dim, t.length), ...
    'z_filtered', zeros(dim, t.length), ...
    'z_real', zeros(dim, t.length), ...
    'z_hat', zeros(dim, t.length), ...
    'x_hat', zeros(dim, t.length));