% ------------------------------------------------------------------------------
% Desired formation, Randomly generated
% ------------------------------------------------------------------------------
dim_xy = 2; % x, y
dim_th = 1; % heading angle

r = 5; % formation size
s = 0.1; % quiver arrow size

% ----------------------------------------------------
% ----------------     PANEL     ------------------
% ----------------------------------------------------
F = struct('formation_mode', "Random", ... % Random or fixed
    'p_desired', zeros(dim_xy, n), ...
    'att_desired', zeros(dim_th, n), ...
    'distance_desired', zeros(1, G.numedges), ...
    'plot', 'false' ... % true or false
    );
% ----------------------------------------------------


% generate p* randomly satisfied with triangle constraints
% (not examined because almost all satisfied)
if (strcmp(F.formation_mode, "Random"))
    F.p_desired = r*(rand(dim_xy, n));
    F.att_desired = 2*pi*(rand(dim_th, n)-0.5);
    fprintf("Desired Formation has been generated Randomly \n\n");
else
    %     % CASE 1
    %     F.p_desired = [1.6110, 0.3658, 1.7730;
    %         1.1534, 0.4799, 0.0573];
    %     F.att_desired = [-0.0635,-2.0865, 3.0076];
    % CASE 2`
    F.p_desired = [0.2772, 1.3982, 1.9186;
        0.2986, 1.7818, 1.0944 ];
    F.att_desired = [-1.5236    2.1408   -1.5439];
    % CASE 3
    %     F.p_desired =[0.1623    1.5514    0.8717;
    %         1.8588    0.9736    0.8936];
    %     F.att_desired = [1.5942   -0.7512    0.4261];
    fprintf("Desired Formation has been generated as USER DEFINED \n\n");
end

% calculate relative displacement distance
for k=1:G.numedges
    F.distance_desired(k) = norm(F.p_desired(:,G.Edges{k,1}(1)) - F.p_desired(:,G.Edges{k,1}(2)));
end
fprintf("desired distance of formation is shown as below\n");
fprintf("zs12: %1.2f,  zs13: %1.2f,  zs23: %1.2f \n\n", F.distance_desired(1), F.distance_desired(2), F.distance_desired(3));


% plot desired framework
if(strcmp(F.plot, 'true'))
    figure,
    for k=1:n
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
% ------------------------------------------------------------------------------