end_pos = [A(1).pos, A(2).pos, A(3).pos];
itv = 300;
s_traj = 0.2;
% start end positions and attitudes, edges and trajectories
figure,
for i=1:n_agent
    plot(pos0(1,i), pos0(2,i), 'b*', 'LineWidth', 2); hold on; % start pos
    quiver(pos0(1,i), pos0(2,i), s*cos(att0(i)), s*sin(att0(i)), 'k'); hold on;% start att
    quiver(pos0(1,i), pos0(2,i), -0.5*s*sin(att0(i)), 0.5*s*cos(att0(i)), 'k*'); hold on;

    plot(A(i).pos(1), A(i).pos(2), 'ro', 'LineWidth', 2); hold on; % end pos
    quiver(A(i).pos(1), A(i).pos(2), s*cos(A(i).att), s*sin(A(i).att), 'k'); hold on;% end att
    quiver(A(i).pos(1), A(i).pos(2), -0.5*s*sin(A(i).att), 0.5*s*cos(A(i).att), 'k*'); hold on;

    plot(BUF(i).pos(1,:), BUF(i).pos(2,:), 'k--', 'LineWidth', 2); hold on; % traj pos
    quiver(BUF(i).pos(1,1:itv:end), BUF(i).pos(2,1:itv:end), s_traj*cos(BUF(i).att(1:itv:end)), s_traj*sin(BUF(i).att(1:itv:end)), 'g'); hold on; % traj att
    quiver(BUF(i).pos(1,1:itv:end), BUF(i).pos(2,1:itv:end), -0.5*s_traj*sin(BUF(i).att(1:itv:end)), 0.5*s_traj*cos(BUF(i).att(1:itv:end)), 'g*'); hold on;

    edge = [A(G.Edges{i,1}(1)).pos, A(G.Edges{i,1}(2)).pos]; hold on; % edge
    plot(edge(1,:), edge(2,:), 'r--'); hold on;

    text(A(i).pos(1)+0.05, A(i).pos(2)+0.05, sprintf(G.Nodes.names{i,1},i)); % name of node
end
grid on; grid minor; axis equal;
xlabel('[m]'); ylabel('[m]'); %title('');


% zs, z
zs = zeros(G.numedges);
z = zeros(n_agent, t.length);

for i=1:n_agent
    for j=1:n_agent
        if any(j==successors(G,i))
            zs(i) = vecnorm(F.p_desired(:,j) - F.p_desired(:,i));
            z(i,:) = vecnorm(BUF(i).pos - BUF(j).pos);
            if i==3 && j==2
                zs_estimated = vecnorm(L.x_hat - A(i).pos);
                z_estimated = vecnorm(buf_L.x_hat - BUF(i).pos);
            end
        end
    end
end
legend_array = {["z*12", "z12"], ["z*13", "z13"], ["z*23", "z23", "z*23_{est}", "z23_{est}"]};

figure,
for i=1:n_agent
    subplot(n_agent,1,i)
    plot(t.time, zs(i)*ones(1,t.length), 'b', 'LineWidth', 2); hold on;
    plot(t.time, z(i,:), 'r', 'LineWidth', 2); hold on;
    if i==3
        plot(t.time, zs_estimated*ones(1,t.length), 'k--', 'LineWidth', 2); hold on;
        plot(t.time, z_estimated, 'g--', 'LineWidth', 2);
    end
    grid on; xlabel('[sec]'); ylabel('[m]'); legend((legend_array{i}));
end


% control input (global, local, feedback linearized)
% formation global input
% figure,
% subplot(n_agent,1,1)
% for i=1:n_agent
%     plot(t.time, BUF(i).formation_global(1,:), 'LineWidth', 2); hold on;
%     grid on; xlabel('[sec]'); ylabel('[m/s]'); title('formation global input x');
% end 
% legend('1', '2', '3');
% 
% subplot(n_agent,1,2)
% for i=1:n_agent
%     plot(t.time, BUF(i).formation_global(2,:), 'LineWidth', 2); hold on;
%     grid on; xlabel('[sec]'); ylabel('[m/s]'); title('y');
% end
% legend('1', '2', '3');
% 
% subplot(n_agent,1,3)
% for i=1:n_agent
%     plot(t.time, vecnorm(BUF(i).formation_global), 'LineWidth', 2); hold on;
%     grid on; xlabel('[sec]'); ylabel('[m/s]'); title('magnitude');
% end
% legend('1', '2', '3');


% % Feedback linearized
% figure,
% subplot(2,1,1)
% 
% for i=1:n_agent
%     plot(t.time, BUF(i).feedback_linearized(1,:), 'LineWidth', 2); hold on;
%     grid on; xlabel('[sec]'); ylabel('[m/s]'); 
%     title('Feedback linearization input v'); 
% end
% legend('1', '2', '3');
% 
% subplot(2,1,2)
% for i=1:n_agent
%     plot(t.time, BUF(i).feedback_linearized(2,:)*R2D, 'LineWidth', 2); hold on;
%     grid on; xlabel('[sec]'); ylabel('[deg/s]');
%     title('w'); legend(int2str(i));
% end
% legend('1', '2', '3');

% % localization input
% figure,
% subplot(3,1,1)
% plot(t.time, BUF(3).localization_input(1,:), 'b', 'LineWidth', 2); hold on;
% plot(t.time, BUF(3).pos_dot(1,:), 'r--', 'LineWidth', 2);
% grid on; xlabel('[sec]'); ylabel('[m/s]'); title('persistence excitation input');
% subplot(3,1,2)
% plot(t.time, BUF(3).localization_input(2,:), 'b', 'LineWidth', 2); hold on;
% plot(t.time, BUF(3).pos_dot(2,:), 'r--', 'LineWidth', 2);
% grid on; xlabel('[sec]'); ylabel('[m/s]');
% subplot(3,1,3)
% plot(t.time, vecnorm(BUF(3).localization_input), 'b', 'LineWidth', 2); hold on;
% plot(t.time, vecnorm(BUF(3).pos_dot), 'r--', 'LineWidth', 2);
% grid on; xlabel('[sec]'); ylabel('[m/s]');



% localization estimation value
figure,
subplot(6,1,1)
plot(t.time, BUF(2).pos(1,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf_L.x_hat(1,:), 'r--', 'LineWidth', 2); 
grid on; xlabel('[sec]'); ylabel('[m]'); title('estimatied x');
subplot(6,1,2)
plot(t.time, BUF(2).pos(2,:), 'b', 'LineWidth', 2); hold on;
plot(t.time, buf_L.x_hat(2,:), 'r--', 'LineWidth', 2);
grid on; xlabel('[sec]'); ylabel('[m]');
subplot(6,1,3)
plot(t.time, vecnorm(BUF(2).pos), 'b', 'LineWidth', 2); hold on;
plot(t.time, vecnorm(buf_L.x_hat), 'r--', 'LineWidth', 2);
grid on; xlabel('[sec]'); ylabel('[m]'); legend('err'); %ylim([-L.LOCALIZATION_TOLERANCE 2*L.LOCALIZATION_TOLERANCE]);

subplot(6,1,4)
plot(t.time, buf_L.x_hat_dot(1,:), 'r', 'LineWidth', 2);
grid on; xlabel('[sec]'); ylabel('[m/s]');  title('estimatied x dot');
subplot(6,1,5)
plot(t.time, buf_L.x_hat_dot(2,:), 'r', 'LineWidth', 2);
grid on; xlabel('[sec]'); ylabel('[m/s]');
subplot(6,1,6)
plot(t.time, vecnorm(buf_L.x_hat_dot), 'r', 'LineWidth', 2); hold on;
plot(t.time, L.LOCALIZATION_TOLERANCE*ones(1,t.length), 'b', 'LineWidth', 2);
grid on; xlabel('[sec]'); ylabel('[m/s]'); ylim([-L.LOCALIZATION_TOLERANCE 2*L.LOCALIZATION_TOLERANCE]);

% % leader capture?
% figure,
% for i=1:n_agent
%     subplot(3,1,i)
%     plot(t.time, BUF(i).lost_leader(1,:), 'b', 'LineWidth', 2);
%     grid on; xlabel('[sec]'); ylabel('[-]'); title('lost leader 1, capture 0');
% end

% mode
figure,
for i=1:n_agent
    subplot(3,1,i)
    plot(t.time, BUF(i).mode(1,:), 'b.', 'LineWidth', 2);
    grid on; xlabel('[sec]'); ylabel('[-]'); title('1: formation, 2: localization, 3: search leader');
    ylim([1 3]);
end

% submode
figure,
for i=1:n_agent
    subplot(3,1,i)
    plot(t.time, BUF(i).submode(1,:), 'b.', 'LineWidth', 2);
    grid on; xlabel('[sec]'); ylabel('[-]'); title('1: leader, 2: follower1, 3: follower2');
    ylim([1 3]);
end




% % los angle between follower and leader
% figure,
% for i=1:n_agent
%     subplot(3,1,i)
%     plot(t.time, BUF(i).lf_losang(1,:)*R2D, 'b', 'LineWidth', 2);
%     grid on; xlabel('[sec]'); ylabel('[deg]');
% end

% % attitude
% figure,
% for i=1:n_agent
%     subplot(n_agent,1,i)
%     plot(t.time, BUF(i).att(:)*R2D, 'LineWidth', 2);
%     grid on; xlabel('[sec]'); ylabel('[deg]'); 
%     legend(int2str(i));
% end

% pos_dot att_dot
figure,
subplot(2,1,1)
plot(t.time, vecnorm(BUF(3).pos_dot), 'b', 'LineWidth',2);
grid on; xlabel('[sec]'); ylabel('[m/s]');
subplot(2,1,2)
plot(t.time, (BUF(3).att_dot), 'b', 'LineWidth',2);
grid on; xlabel('[sec]'); ylabel('[rad/s]');
