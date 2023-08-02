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