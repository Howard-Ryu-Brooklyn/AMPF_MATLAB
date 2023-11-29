% ------------------------------------------------------------------------------
% Draw Plots
% ------------------------------------------------------------------------------
p_end = zeros(dim_xy,n);
a_end = zeros(dim_th,n);
equi_circle = 0:0.1:2*pi+0.1;

for i=1:3
    p_end(:,i) = BUF(i).pos(:,end);
    a_end(:,i) = BUF(i).att(:,end);
end

p_traj = cell(n,1);
for k = 1:n
    p_traj{k} =BUF(k).pos;
end

% x y position and angle
figure(3),
for k=1:n
    % draw edges
    edge=[p_end(:, G.Edges{k,1}(1)), p_end(:, G.Edges{k,1}(2))];
    plot(edge(1,:), edge(2,:), 'k'); hold on;
    
    % draw agent orientaion
    quiver(p0(1,k), p0(2,k), s*cos(a0(k)), s*sin(a0(k)), 'k');
    quiver(p0(1,k), p0(2,k), -0.5*s*sin(a0(k)), 0.5*s*cos(a0(k)), 'k*');
    quiver(p_end(1,k), p_end(2,k), s*cos(a_end(k)), s*sin(a_end(k)), 'k');
    quiver(p_end(1,k), p_end(2,k), -0.5*s*sin(a_end(k)), 0.5*s*cos(a_end(k)), 'k*');

    % trajectories
    plot(p_traj{k}(1,:), p_traj{k}(2,:), 'b:', 'linewidth', 2);

    % put name of node
    text(p0(1,k)+0.05, p0(2,k)+0.05, sprintf(G.Nodes.names{k,1},k));
    text(p_end(1,k)+0.05, p_end(2,k)+0.05, sprintf(G.Nodes.names{k,1},k));
end

plot(p_end(1,1) + F.distance_desired(1)*sin(equi_circle), p_end(2,1) + F.distance_desired(1)*cos(equi_circle), 'y--', 'LineWidth', 2); hold on; % equilibrium circle
plot(p_end(1,1) + F.distance_desired(2)*sin(equi_circle), p_end(2,1) + F.distance_desired(2)*cos(equi_circle), 'g--', 'LineWidth', 2); hold on; % equilibrium circle
plot(p_end(1,2) + F.distance_desired(3)*sin(equi_circle), p_end(2,2) + F.distance_desired(3)*cos(equi_circle), 'm--', 'LineWidth', 2); hold on; % equilibrium circle

% draw position of agent
scatter(p0(1,:),p0(2,:), 'filled', 'b');
scatter(p_end(1,:), p_end(2,:), 'filled', 'r');
xlabel('[m]'); ylabel('[m]'); grid on; axis equal


% zs and z
zs = zeros(1,n);
zs(1) = norm(F.p_desired(:,1) - F.p_desired(:,2)); % z*12
zs(2) = norm(F.p_desired(:,1) - F.p_desired(:,3)); % z*13
zs(3) = norm(F.p_desired(:,2) - F.p_desired(:,3)); % z*23

z = cell(n,1);
z{1} = vecnorm(p_traj{1} - p_traj{2}); % z12
z{2} = vecnorm(p_traj{1} - p_traj{3}); % z13
z{3} = vecnorm(p_traj{2} - p_traj{3}); % z23

err_title = [{'z*_{12} - z_{12}'}, {'z*_{13} - z_{13}'}, {'z*_{23} - z_{23}'}];
figure(5)
for k=1:n
    subplot(3,1,k)
    plot(t.time, zs(k)*ones(1,t.length) - z{k}(1,:), 'linewidth', 2); hold on;
    plot(t.time, zeros(1,t.length), 'r--', 'LineWidth', 2); hold on;
    xlabel('[sec]'); ylabel('[m]'); grid on; grid minor; 
    title(err_title(k));
end 

% mode and submode
figure(6)
subplot(3,1,1)
plot(t.time, BUF(2).mode, 'linewidth', 2); hold on;
plot(t.time, BUF(2).submode, '--', 'linewidth', 2);
xlabel('[sec]'); ylabel('[-]'); grid on; grid minor; title('follower 1 mode, FORMATION:1, SEARCH\_LEADER:2, STOP:3');
subplot(3,1,2)
plot(t.time, BUF(3).mode, 'linewidth', 2);
xlabel('[sec]'); ylabel('[-]'); grid on; grid minor; title('follower 2 mode, FORMATION:1, SEARCH\_LEADER:2, STOP:3');
subplot(3,1,3)
plot(t.time, BUF(3).submode, 'linewidth', 2);
xlabel('[sec]'); ylabel('[-]'); grid on; grid minor; title('follower 1 submode, FOLLOW\_LEADER:1, FOLLOW\_FOLLOWER1:2, FOLLOW\_CENTER');

% converge
% figure(7)
% subplot(4,1,1)
% plot(t.time, BUF(2).is_formation_converge, 'linewidth', 2);
% xlabel('[sec]'); ylabel('[-]'); grid on; grid minor; title('follower 1 formation converge');
% subplot(4,1,2)
% plot(t.time, BUF(3).is_leader_converge, 'linewidth', 2);
% xlabel('[sec]'); ylabel('[-]'); grid on; grid minor; title('follower 2  leader converge');
% subplot(4,1,3)
% plot(t.time, BUF(3).is_follower1_converge, 'linewidth', 2);
% xlabel('[sec]'); ylabel('[-]'); grid on; grid minor; title('follower 2  follower 1 converge');
% subplot(4,1,4)
% plot(t.time, BUF(3).is_formation_converge, 'linewidth', 2);
% xlabel('[sec]'); ylabel('[-]'); grid on; grid minor; title('follower 2 formation converge');

% cmd
% figure(8)
% subplot(2,1,1)
% plot(t.time, BUF(2).v, 'linewidth', 2);
% xlabel('[sec]'); ylabel('[m/s]'); grid on; grid minor; title('follower 1 velocity cmd');
% subplot(2,1,2)
% plot(t.time, BUF(2).w, 'linewidth', 2);
% xlabel('[sec]'); ylabel('[rad/s]'); grid on; grid minor; title('follower 1 angular velocity cmd');
% ------------------------------------------------------------------------------