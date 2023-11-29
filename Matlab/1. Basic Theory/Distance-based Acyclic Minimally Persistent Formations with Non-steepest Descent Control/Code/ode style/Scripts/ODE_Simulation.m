% ------------------------------------------------------------------------------
% Simulation
% ------------------------------------------------------------------------------

tspan= [0 25]; %[sec sec]
R2D = pi/180;
nonsteepest_theta = 30*R2D; %[rad]
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);

DE = @(t,x) NonSteepest_Gradient_Based_ODE(x, G, p_desired, dim, nonsteepest_theta);
[t,x] = ode45(DE, tspan, p0(:), opts); % p0(:) -> column vector
% ------------------------------------------------------------------------------