clc; close all; clear;

% Declare ayclic minimally acyclic persistent graph
% network topology is AMP
% For now, triangle amp
D2R=pi/180;
R2D=1/D2R;

n = 3; % number of agent
G = digraph([2 3 3],[1 1 2]);
% G.Nodes.names = {'Leader', 'Follower 1', 'Follower 2'}';
G.Nodes.names = {'1', '2', '3'}';
% figure(1),
% plot(G,'NodeLabel', G.Nodes.names)
% title('network topology of AMPF');

d = [1, 2]; % demension of xy, theta
th = 1;
xy = 2;

r = 5; % formation size
s = 0.1; % quiver arrow size

% generate p* randomly satisfied with triangle constraints
% p_desired = r*(rand(d(xy), n));
% orient_desired = 2*pi*(rand(d(th), n)-0.5);
p_desired = [3.7563,    2.5298,    4.4545;
             1.2755,    3.4954,    4.7965];
orient_desired = [0.2967   -2.2706   -2.2036];
% check triangle constraints

% calculate relative displacement distance
distance = zeros(1, G.numedges);
for k=1:G.numedges
    distance(k) = norm(p_desired(:,G.Edges{k,1}(1)) - p_desired(:,G.Edges{k,1}(2)));
end

% plot desired framework
% figure(2),
% for k=1:n
%     % draw edges
%     edge=[p_desired(:, G.Edges{k,1}(1)), p_desired(:, G.Edges{k,1}(2))];
%     plot(edge(1,:), edge(2,:), 'k'); hold on;
%     % draw agent orientaion
%     quiver(p_desired(1,k), p_desired(2,k), s*cos(orient_desired(1,k)), s*sin(orient_desired(1,k)), 'k');
%     quiver(p_desired(1,k), p_desired(2,k), -0.5*s*sin(orient_desired(1,k)), 0.5*s*cos(orient_desired(1,k)), 'k*');
%     % put name of node
%     text(p_desired(1,k)+0.05, p_desired(2,k)+0.05, sprintf(G.Nodes.names{k,1},k));
% end
% % draw position of agent
% scatter(p_desired(1,:), p_desired(2,:), 'filled', 'r'); grid on;
% xlabel('[m]'); ylabel('[m]'); axis equal;
% title('desired realization for formation control');

%%
% set intial value of position and attitude randomly
% p0 = r*(rand(d(xy), n) - 0.5);
% a0 = 2*pi*(rand(d(th), n) - 0.5);
p0 = [-0.7417,    0.4263,    2.0860;
      1.6541,    0.2486,   -1.0708];
a0 = [1.6160    2.3825    1.6344];

%%
% follower detect leader when the leader is in its fov angle
% fov = 60 [deg]
fov = 100*D2R;
los_vec21=p0(:,G.Edges{1,1}(2)) - p0(:,G.Edges{1,1}(1));
los_vec31=p0(:,G.Edges{2,1}(2)) - p0(:,G.Edges{2,1}(1));
los21=atan2(los_vec21(2), los_vec21(1));
los31=atan2(los_vec31(2), los_vec31(1));
a0(2) = los21 + fov*(rand(1,1)-0.5);
a0(3) = los31 + fov*(rand(1,1)-0.5);


clc; close all
% x = [att; pos]
DE = @(t,x) NonSteepest_Gradient_Based_ODE(x, G, p_desired, d, 0, 'NH', false);
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[t,x] = ode45(DE, [0 0.3], [a0(:); p0(:)], opts);

att_end = reshape(x(end, 1:d(th)*n), d(th), n);
p_end = reshape(x(end, d(th)*n+1:end), d(xy), n);

att_traj = cell(n,1);
for k = 1:n
    att_traj{k} = x(:, k)';
end

p_traj = cell(n,1);
for k = 1:n
    p_traj{k} = x(:, d(xy)*(k-1)+1+d(th)*n : d(xy)*k+ d(th)*n)';
end

figure(),
for k=1:n
    subplot(3,1,k)
    plot(t, att_traj{k}(1,:)*R2D); hold on;
    xlabel('[sec]'); ylabel('/yaw [deg]'); grid on; grid minor; 
end

figure(3),
for k=1:n
    % draw edges
    edge=[p_end(:, G.Edges{k,1}(1)), p_end(:, G.Edges{k,1}(2))];
    plot(edge(1,:), edge(2,:), 'k'); hold on;
    % draw agent orientaion
    quiver(p0(1,k), p0(2,k), s*cos(a0(k)), s*sin(a0(k)), 'k');
    quiver(p0(1,k), p0(2,k), -0.5*s*sin(a0(k)), 0.5*s*cos(a0(k)), 'k*');
    if k~=1
        quiver(p0(1,k), p0(2,k), s*cos(a0(k) - fov/2), s*sin(a0(k) - fov/2), 'r*');
        quiver(p0(1,k), p0(2,k), s*cos(a0(k) + fov/2), s*sin(a0(k) + fov/2), 'r*');
    end
    quiver(p_end(1,k), p_end(2,k), s*cos(att_end(k)), s*sin(att_end(k)), 'k');
    quiver(p_end(1,k), p_end(2,k), -0.5*s*sin(att_end(k)), 0.5*s*cos(att_end(k)), 'k*');
    if k~=1
        quiver(p_end(1,k), p_end(2,k), s*cos(a0(k) - fov/2), s*sin(a0(k) - fov/2), 'r*');
        quiver(p_end(1,k), p_end(2,k), s*cos(a0(k) + fov/2), s*sin(a0(k) + fov/2), 'r*');
    end

    % trajectories
    plot(p_traj{k}(1,:), p_traj{k}(2,:), 'b:', 'linewidth', 2);
    % put name of node
    text(p0(1,k)+0.05, p0(2,k)+0.05, sprintf(G.Nodes.names{k,1},k));
    text(p_end(1,k)+0.05, p_end(2,k)+0.05, sprintf(G.Nodes.names{k,1},k));

end
% draw position of agent
scatter(p0(1,:),p0(2,:), 'filled', 'b');
scatter(p_end(1,:), p_end(2,:), 'filled', 'r');
xlabel('[m]'); ylabel('[m]'); grid on; axis equal


% for i=1:500:length(p_traj{1})
%     for k=1:n
%             p_snap = reshape(x(i, d(2)*n+1:end), d(1), n);
%             edge=[p_snap(:, G.Edges{k,1}(1)), p_snap(:, G.Edges{k,1}(2))];
%             plot(edge(1,:), edge(2,:), 'g'); hold on;
%         plot(p_traj{k}(1,i), p_traj{k}(2,i), 'g*', 'linewidth', 2); hold on;
%         drawnow
%     end
% end

% figure, 
% plot(t, )




