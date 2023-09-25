att_end = buf_att(1,:,t.length);
p_end = buf_pos(:,:,t.length);

att_traj = cell(n,1);
for k = 1:n
    att_traj{k} = buf_att(1,k,:);
end

p_traj = cell(n,1);
for k = 1:n
    p_traj{k} = buf_pos(:,k,:);
end

z = cell(n,1);
z{1} = vecnorm(p_traj{1} - p_traj{2}); % z12
z{2} = vecnorm(p_traj{1} - p_traj{3}); % z13
z{3} = vecnorm(p_traj{2} - p_traj{3}); % z23

z_s(1) = norm(p_desired(:,1) - p_desired(:,2)); % z*12
z_s(2) = norm(p_desired(:,1) - p_desired(:,3)); % z*13
z_s(3) = norm(p_desired(:,2) - p_desired(:,3)); % z*23

cmd_vw = cell(n,1);
for k=1:n
    cmd_vw{k} = buf_vw(:,k,:);
end

attitle = [{'leader'}, {'follower1'}, {'follower2'}];
figure(),
for k=1:n
    subplot(3,1,k)
    plot(t.time, att_traj{k}(1,:)*R2D); hold on;
    xlabel('[sec]'); ylabel('yaw [deg]'); grid on; grid minor; 
    title(attitle(k));
end


ztitle = [{'z_{12}'}, {'z_{13}'}, {'z_{23}'}];
figure(), 
for k=1:n
    subplot(3,1,k)
    plot(t.time, z_s(k)*ones(1,t.length), 'r', 'linewidth', 2); hold on;
    plot(t.time, z{k}(1,:), 'b--', 'LineWidth', 2); hold on;
    xlabel('[sec]'); ylabel('[m]'); grid on; grid minor; 
    title(ztitle(k));
end 


vwtitle = [{'leader'}, {'follower1'}, {'follower2'}];
figure(),
for k=1:n
    subplot(2,3,k)
    plot(t.time, cmd_vw{k}(1,:))
    xlabel('[sec]'); ylabel('v [m/s]'); grid on; grid minor; 
    title(vwtitle(k));
    subplot(2,3,3+k)
    plot(t.time, cmd_vw{k}(2,:))
    xlabel('[sec]'); ylabel('\omega [rad/s]'); grid on; grid minor; 
end



figure(),
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