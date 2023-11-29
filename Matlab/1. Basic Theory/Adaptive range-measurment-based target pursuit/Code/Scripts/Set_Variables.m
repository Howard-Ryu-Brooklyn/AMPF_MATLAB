
% simulation time
t = struct( ...
    'start', 0, ...
    'final', 1000, ...
    'ts', 0.01, ...
    'time', 0, ...
    'length', 0);

t.time = t.start:t.ts:t.final-t.ts;
t.length = length(t.time);
fprintf("t.final: %f.\nt.ts: %f\n",t.final, t.ts);

% utilities
D2R=pi/180;
R2D=1/D2R;

% control variable
alpha = 1; % state filter constant
gamma = 1; % adaptive law constant
beta = 2; % control law constant
a = 1; % freqency of sigma
sigma_init = [0.7, 0.7]'; 
beta0 = atan2(sigma_init(2), sigma_init(1)); % argument of complex number of sigma
fprintf("beta: %1.2f should be greater than %f \n", beta, a*norm(sigma_init));
fprintf("period of sigma is %f [sec], freqency is %f [hz] \n", 2*pi/abs(a), 1/(2*pi/abs(a)));

% inital value
dim = 2; % demension
x_init_pos = [2, 2]'; %[m, m] target initial position
y_init_pos = [3, 3]'; %[m, m] robot initial position
y_init_att = 0*D2R;
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
y_att = y_init_att;
y_pre = y_pos; 
y_cur = zeros(dim , 1);

pre_zv = norm(y_pos)^2 - D^2; 
cur_zv = 0;

cmd = zeros(dim, 1); % persistent exciting signal
sigma_dot = zeros(dim, 1);
sigma = sigma_init;
sigma_matrix = a*[
    0, 1;
    -1 0];

% buffer
buf=struct('D', zeros(1, t.length), ...
    'ps', zeros(dim, t.length), ...
    'y_pos', zeros(dim, t.length), ...
    'phi_filtered', zeros(dim, t.length), ...
    'phi_real', zeros(dim, t.length), ...
    'z_filtered', zeros(dim, t.length), ...
    'z_real', zeros(dim, t.length), ...
    'z_hat', zeros(dim, t.length), ...
    'x_hat_dot', zeros(dim, t.length), ...
    'x_hat', zeros(dim, t.length), ...
    'cmd', zeros(dim, t.length), ...
    'sigma', zeros(dim, t.length), ...
    'sigma_dot', zeros(dim, t.length));