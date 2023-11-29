% ------------------------------------------------------------------------------
% Draw Plots
% ------------------------------------------------------------------------------
p_end = zeros(d_xy,n);
a_end = zeros(d_ang,n);

for i=1:3
    p_end(:,i) = BUF(i).pos(:,end);
    a_end(:,i) = BUF(i).att(:,end);
end

p_traj = cell(n,1);
for k = 1:n
    p_traj{k} =BUF(k).pos;
end

% % x y position and angle
% figure(3),
% for k=1:n
%     % draw edges
%     edge=[p_end(:, G.Edges{k,1}(1)), p_end(:, G.Edges{k,1}(2))];
%     plot(edge(1,:), edge(2,:), 'k'); hold on;
%     
%     % draw agent orientaion
%     quiver(p0(1,k), p0(2,k), s*cos(a0(k)), s*sin(a0(k)), 'k');
%     quiver(p0(1,k), p0(2,k), -0.5*s*sin(a0(k)), 0.5*s*cos(a0(k)), 'k*');
%     quiver(p_end(1,k), p_end(2,k), s*cos(a_end(k)), s*sin(a_end(k)), 'k');
%     quiver(p_end(1,k), p_end(2,k), -0.5*s*sin(a_end(k)), 0.5*s*cos(a_end(k)), 'k*');
% 
%     % trajectories
%     plot(p_traj{k}(1,:), p_traj{k}(2,:), 'b:', 'linewidth', 2);
% 
%     % put name of node
%     text(p0(1,k)+0.05, p0(2,k)+0.05, sprintf(G.Nodes.names{k,1},k));
%     text(p_end(1,k)+0.05, p_end(2,k)+0.05, sprintf(G.Nodes.names{k,1},k));
% end
% 
% % draw position of agent
% scatter(p0(1,:),p0(2,:), 'filled', 'b');
% scatter(p_end(1,:), p_end(2,:), 'filled', 'r');
% xlabel('[m]'); ylabel('[m]'); grid on; axis equal


% zs and z
zs = zeros(1,n);
zs(1) = norm(p_desired(:,1) - p_desired(:,2)); % z*12
zs(2) = norm(p_desired(:,1) - p_desired(:,3)); % z*13
zs(3) = norm(p_desired(:,2) - p_desired(:,3)); % z*23

z = cell(n,1);
z{1} = vecnorm(p_traj{1} - p_traj{2}); % z12
z{2} = vecnorm(p_traj{1} - p_traj{3}); % z13
z{3} = vecnorm(p_traj{2} - p_traj{3}); % z23

% ztitle = [{'z_{12}'}, {'z_{13}'}, {'z_{23}'}];
% figure(4), 
% for k=1:n
%     subplot(3,1,k)
%     plot(t.time, zs(k)*ones(1,t.length), 'r', 'linewidth', 2); hold on;
%     plot(t.time, z{k}(1,:), 'b--', 'LineWidth', 2); hold on;
%     xlabel('[sec]'); ylabel('[m]'); grid on; grid minor; 
%     title(ztitle(k));
% end 

err_title = [{'z*_{12} - z_{12}'}, {'z*_{13} - z_{13}'}, {'z*_{23} - z_{23}'}];
figure(5)
for k=1:n
    subplot(3,1,k)
    plot(t.time, zs(k)*ones(1,t.length) - z{k}(1,:), 'linewidth', 2); hold on;
    plot(t.time, zeros(1,t.length), 'r--', 'LineWidth', 2); hold on;
    xlabel('[sec]'); ylabel('[m]'); grid on; grid minor; 
    title(err_title(k));
end 
% ------------------------------------------------------------------------------