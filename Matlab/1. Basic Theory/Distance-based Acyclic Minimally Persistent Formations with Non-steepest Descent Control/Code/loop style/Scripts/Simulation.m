% ------------------------------------------------------------------------------
% Simulation
% ------------------------------------------------------------------------------

% time
t = struct('start', 0, 'final', 10, 'ts', 0.01, 'time', 0, 'length', 0);
t.time = t.start:t.ts:t.final-t.ts;
t.length = length(t.time);
theta = 30*D2R;

% variables
bar_error_ji = zeros(n, n);
Q = [cos(theta), sin(theta);
    -sin(theta), cos(theta)];

pos = p0;
pos_dot = zeros(size(pos));

% buffer
buf = struct('pos', zeros(dim, t.length), 'pos_dot', zeros(dim,t.length));
BUF = [buf, buf, buf];

for k=1:t.length

    % caluate non-steepest gradient descent control law
    for i=1:n
        u_global = zeros(dim,1);

        for j=1:n
            if any(j == successors(G,i))
                zji = pos(:,j) - pos(:,i);
                z_s_ji = p_desired(:,j) - p_desired(:,i);
                bar_error_ji(i,j) = norm(zji)^2 - norm(z_s_ji)^2; % 1x1
                u_global = u_global  +  Q*bar_error_ji(i,j)*zji;
            end
        end % for j

        % Single Integrator pdot=u
        pos_dot(:,i) = u_global;

    end % for i

    % euler integration
    pos = pos + pos_dot*t.ts;

    % save buffer
    for i=1:3
        BUF(i).pos(:,k) = pos(:,i);
        BUF(i).pos_dot(:,k) = pos_dot(:,i);
    end
end % for k
% ------------------------------------------------------------------------------