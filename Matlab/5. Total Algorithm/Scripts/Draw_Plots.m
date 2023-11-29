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

% zs,z and zs_esitmated, z_esitmated
zs = zeros(1,n);
zs(1) = norm(F.p_desired(:,1) - F.p_desired(:,2)); % z*12
zs(2) = norm(F.p_desired(:,1) - F.p_desired(:,3)); % z*13
zs(3) = norm(F.p_desired(:,2) - F.p_desired(:,3)); % z*23

z = cell(n,1);
z{1} = vecnorm(p_traj{1} - p_traj{2}); % z12
z{2} = vecnorm(p_traj{1} - p_traj{3}); % z13
z{3} = vecnorm(p_traj{2} - p_traj{3}); % z23
zs_estimated = vecnorm(L.x_hat - A(i).pos); %zs_estimated
z_estimated = vecnorm(buf_L.x_hat - BUF(i).pos); % z_estimated



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

plot(p_end(1,1) + F.distance_desired(1)*sin(equi_circle), p_end(2,1) + F.distance_desired(1)*cos(equi_circle), 'r--', 'LineWidth', 2); hold on; % equilibrium circle
plot(p_end(1,1) + F.distance_desired(2)*sin(equi_circle), p_end(2,1) + F.distance_desired(2)*cos(equi_circle), 'g--', 'LineWidth', 2); hold on; % equilibrium circle
plot(p_end(1,2) + F.distance_desired(3)*sin(equi_circle), p_end(2,2) + F.distance_desired(3)*cos(equi_circle), '--', 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2); hold on; % equilibrium circle
plot(L.x_hat(1) + F.distance_desired(3)*sin(equi_circle), L.x_hat(2) + zs_estimated*cos(equi_circle), 'm--', 'LineWidth', 2); hold on; % equilibrium circle
plot(L.x_hat(1), L.x_hat(2), 'mo', 'LineWidth', 2); hold on; % estimated follower1 pos

% draw position of agent
scatter(p0(1,:),p0(2,:), 'filled', 'b');
scatter(p_end(1,:), p_end(2,:), 'filled', 'r');
xlabel('[m]'); ylabel('[m]'); grid on; axis equal
% xlim([-1.5 2]); ylim([-2 1]);
xlim([-0.4 0.1]); ylim([-0.6 -0.2]);

err_title = [{'z*_{12} - z_{12}'}, {'z*_{13} - z_{13}'}, {'z*_{23} - z_{23}'}];
figure(5)
for k=2:n
    subplot(2,1,k-1)
    plot(t.time, zs(k)*ones(1,t.length) - z{k}(1,:), 'linewidth', 2); hold on;
    if(k==3)
        plot(t.time, zs_estimated*ones(1,t.length) - z_estimated,  'g--', 'LineWidth', 2); hold on;
    end
    plot(t.time, zeros(1,t.length), 'r--', 'LineWidth', 2); hold on;
    xlabel('[sec]'); ylabel('[m]'); grid on; grid minor; 
    title(err_title(k)); xlim([180 300]); ylim([-0.5 0.5]);
end 


% mode and submode
% figure(6)
% subplot(3,1,1)
% plot(t.time, BUF(2).mode, 'linewidth', 2); hold on;
% plot(t.time, BUF(2).submode, '--', 'linewidth', 2); legend('mode', 'submode');
% xlabel('[sec]'); ylabel('[-]'); grid on; grid minor; title('follower 1 mode, FORMATION:1, SEARCH\_LEADER:2, LOCALIZATION:3, STOP:4');
% subplot(3,1,2)
% plot(t.time, BUF(3).mode, 'linewidth', 2);
% xlabel('[sec]'); ylabel('[-]'); grid on; grid minor; title('follower 2 mode, FORMATION:1, SEARCH\_LEADER:2, LOCALIZATION:3, STOP:4');
% subplot(3,1,3)
% plot(t.time, BUF(3).submode, 'linewidth', 2);
% xlabel('[sec]'); ylabel('[-]'); grid on; grid minor; title('follower 2 submode, FOLLOW\_LEADER:1, FOLLOW\_FOLLOWER1:2, FOLLOW\_CENTER');

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

% localization estimation value
figure,
subplot(2,1,1)
plot(t.time, BUF(2).pos(1,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf_L.x_hat(1,:), 'r--', 'LineWidth', 2); xlim([0 L.TOL_TIME]);
legend('참값', '추정된 값');
grid on; xlabel('[sec]'); ylabel('[m]'); title('추정된 팔로워 1의 위치, x축'); 
subplot(2,1,2)
plot(t.time, BUF(2).pos(2,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf_L.x_hat(2,:), 'r--', 'LineWidth', 2); xlim([0 L.TOL_TIME]);
legend('참값', '추정된 값');
grid on; xlabel('[sec]'); ylabel('[m]'); title('추정된 팔로워 1의 위치, y축'); 
% subplot(3,1,3)
% plot(t.time, vecnorm(BUF(2).pos), 'b', 'LineWidth', 2); hold on;
% plot(t.time, vecnorm(buf_L.x_hat), 'r--', 'LineWidth', 2); xlim([0 L.TOL_TIME]);
% grid on; xlabel('[sec]'); ylabel('[m]'); title('추정된 팔로워 1의 위치오차'); 

figure,
subplot(2,1,1)
plot(t.time, buf_L.x_hat_dot(1,:), 'r', 'LineWidth', 2); xlim([0 L.TOL_TIME]);
grid on; xlabel('[sec]'); ylabel('[m/s]');  title('estimatied x dot'); 
subplot(2,1,2)
plot(t.time, buf_L.x_hat_dot(2,:), 'r', 'LineWidth', 2); xlim([0 L.TOL_TIME]);
grid on; xlabel('[sec]'); ylabel('[m/s]');


% ------------------------------------------------------------------------------