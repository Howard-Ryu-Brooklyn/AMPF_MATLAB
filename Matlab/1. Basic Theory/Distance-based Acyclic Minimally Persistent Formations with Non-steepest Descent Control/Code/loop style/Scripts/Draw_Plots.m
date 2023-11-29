% ------------------------------------------------------------------------------
% Draw Plots
% ------------------------------------------------------------------------------
p_end = zeros(dim,n);
for i=1:3
    p_end(:,i) = BUF(i).pos(:,end);
end

p_traj = cell(n,1);
for k = 1:n
    p_traj{k} =BUF(k).pos;
end

figure(3),
for k=1:n
    % draw edges
    edge=[p_end(:, G.Edges{k,1}(1)), p_end(:, G.Edges{k,1}(2))];
    plot(edge(1,:), edge(2,:), 'k'); hold on;

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

% ------------------------------------------------------------------------------