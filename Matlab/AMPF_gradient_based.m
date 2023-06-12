clc; close all; clear;

% Declare ayclic minimally acyclic persistent graph
% network topology is AMP
% For now, triangle amp
nagent = 3;
G = digraph([2 3 3],[1 1 2]);
% G.Nodes.names = {'Leader', 'Follower 1', 'Follower 2'}';
G.Nodes.names = {'1', '2', '3'}';
figure(1),
plot(G,'NodeLabel', G.Nodes.names)
title('network topology of AMPF');
%%

ndim = 2; % demension
r = 5; % formation size
s = 0.1; % quiver arrow size

% generate p* randomly satisfied with triangle constraints
p_desired = r*(rand(ndim, nagent));

if ndim==2; nori=1; elseif ndim==3; nori=3; end
% when 3D -> RPY order
orient_desired = 2*pi*(rand(nori, nagent)-0.5);
% check triangle constraints

% calculate relative displacement
z = zeros(1,G.numedges);
for k=1:G.numedges
    z(k) = norm(p_desired(:,G.Edges{k,1}(1)) - p_desired(:,G.Edges{k,1}(2)));
end

% plot desired framework
figure(2),
if ndim == 2
    for k=1:nagent
        % draw edges
        edge=[p_desired(:, G.Edges{k,1}(1)), p_desired(:, G.Edges{k,1}(2))];
        plot(edge(1,:), edge(2,:), 'k'); hold on;
        % draw agent orientaion
        quiver(p_desired(1,k), p_desired(2,k), s*cos(orient_desired(1,k)), s*sin(orient_desired(1,k)), 'k');
        quiver(p_desired(1,k), p_desired(2,k), -s*sin(orient_desired(1,k)), s*cos(orient_desired(1,k)), 'k*');
        % put name of node
        text(p_desired(1,k)+0.05, p_desired(2,k)+0.05, sprintf(G.Nodes.names{k,1},k));
    end
    % draw position of agent
    scatter(p_desired(1,:), p_desired(2,:), 'filled', 'r'); grid on;
    xlabel('[m]'); ylabel('[m]'); axis equal;
    title('desired realization for formation control');

elseif ndim ==3
    for k=1:nagent
        % draw edges
        edge=[p_desired(:, G.Edges{k,1}(1)), p_desired(:, G.Edges{k,1}(2))];
        plot3(edge(1,:), edge(2,:), edge(3,:), 'k'); hold on;
        sRn_a = s*R(orient_desired(1,k),'x')*R(orient_desired(2,k),'y')*R(orient_desired(3,k),'z');
        % draw agent orientaion
        quiver3(p_desired(1,k), p_desired(2,k), p_desired(3,k), sRn_a(1,1), sRn_a(1,2), sRn_a(1,3), 'k');
        quiver3(p_desired(1,k), p_desired(2,k), p_desired(3,k), sRn_a(2,1), sRn_a(2,2), sRn_a(2,3), 'k');
        quiver3(p_desired(1,k), p_desired(2,k), p_desired(3,k), sRn_a(3,1), sRn_a(3,2), sRn_a(3,3), 'k');
        % put name of node
        text(p_desired(1,k)+0.05, p_desired(2,k)+0.05, p_desired(3,k)+0.05, sprintf(G.Nodes.names{k,1},k));
    end
    % draw position of agent
    scatter3(p_desired(1,:), p_desired(2,:), p_desired(3,:), 'filled', 'r'); grid on;
    xlabel('[m]'); ylabel('[m]'); axis equal;
    title('desired realization for formation control');

end

%%
% set intial value of position and orientation
p0 = r*(rand(ndim, nagent) - 0.5);
orient0 = 2*pi*(rand(nori, nagent) - 0.5);



%%
close all

% x = [ori; pos]
DE = @(t,x) NonSteepest_Gradient_Based_ODE(x, G, p_desired, ndim, 0, 'nonholonomic');
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[t,x] = ode45(DE, [0 25], [orient0(:); p0(:)], opts);

ori_end = reshape(x(end, 1:nori*nagent), nori, nagent);
p_end = reshape(x(end, nori*nagent+1:end), ndim, nagent);

p_traj = cell(nagent,1);
for k = 1:nagent
    p_traj{k} = x(:, ndim*(k-1)+1+nori*nagent : ndim*k+ nori*nagent)';
end

figure(3),
for k=1:nagent
    % draw edges
    edge=[p_end(:, G.Edges{k,1}(1)), p_end(:, G.Edges{k,1}(2))];
    plot(edge(1,:), edge(2,:), 'k'); hold on;
    % draw agent orientaion
    quiver(p0(1,k), p0(2,k), s*cos(orient0(k)), s*sin(orient0(k)), 'k');
    quiver(p0(1,k), p0(2,k), -s*sin(orient0(k)), s*cos(orient0(k)), 'k*');
    quiver(p_end(1,k), p_end(2,k), s*cos(ori_end(k)), s*sin(ori_end(k)), 'k');
    quiver(p_end(1,k), p_end(2,k), -s*sin(ori_end(k)), s*cos(ori_end(k)), 'k*');
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

for i=1:500:length(p_traj{1})
    for k=1:nagent
            p_snap = reshape(x(i, nori*nagent+1:end), ndim, nagent);
            edge=[p_snap(:, G.Edges{k,1}(1)), p_snap(:, G.Edges{k,1}(2))];
            plot(edge(1,:), edge(2,:), 'g'); hold on;
        plot(p_traj{k}(1,i), p_traj{k}(2,i), 'g*', 'linewidth', 2); hold on;
        drawnow
    end
end




