% Formation Topology
n_agent = 3;
G = digraph([2 3 3],[1 1 2]);
G.Nodes.names = {'1', '2', '3'}';

dim_xy=2;
dim_th=1;

r = 5; % formation size
s = 0.3; % quiver arrow size

F = struct('formation_mode', "Fixed", ... % Random or fixed
    'p_desired', zeros(dim_xy, n_agent), ...
    'att_desired', zeros(dim_th, n_agent), ...
    'distance_desired', zeros(1, G.numedges), ...
    'plot', 'false' ... % true or false
    );

if (strcmp(F.formation_mode, "Random"))
    F.p_desired = r*(rand(dim_xy, n_agent));
    F.att_desired = 2*pi*(rand(dim_th, n_agent)-0.5);
else
    F.p_desired = [3.7563,    2.5298,    4.4545;
        1.2755,    3.4954,    4.7965];
    F.att_desired = [0.2967   -2.2706   -2.2036];
end

% check triangle constraints (TODO)


% calculate relative displacement distance
for k=1:G.numedges
    F.distance_desired(k) = norm(F.p_desired(:,G.Edges{k,1}(1)) - F.p_desired(:,G.Edges{k,1}(2)));
end

% plot desired framework
if(strcmp(F.plot, 'true'))
    figure,
    for k=1:n_agent
        % draw edges
        edge=[F.p_desired(:, G.Edges{k,1}(1)), F.p_desired(:, G.Edges{k,1}(2))];
        plot(edge(1,:), edge(2,:), 'k'); hold on;
        % draw agent orientaion
        quiver(F.p_desired(1,k), F.p_desired(2,k), s*cos(F.att_desired(1,k)), s*sin(F.att_desired(1,k)), 'k');
        quiver(F.p_desired(1,k), F.p_desired(2,k), -0.5*s*sin(F.att_desired(1,k)), 0.5*s*cos(F.att_desired(1,k)), 'k*');
        % put name of node
        text(F.p_desired(1,k)+0.05, F.p_desired(2,k)+0.05, sprintf(G.Nodes.names{k,1},k));
    end
    % draw position of agent
    scatter(F.p_desired(1,:), F.p_desired(2,:), 'filled', 'r'); grid on;
    xlabel('[m]'); ylabel('[m]'); axis equal;
    title('desired realization for formation control');
end