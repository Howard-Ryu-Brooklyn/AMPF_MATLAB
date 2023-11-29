% ------------------------------------------------------------------------------
% Simulation
% ------------------------------------------------------------------------------

% time
t = struct('start', 0, 'final', 60, 'ts', 0.01, 'time', 0, 'length', 0);
t.time = t.start:t.ts:t.final-t.ts;
t.length = length(t.time);
theta = 0*D2R;

% constants
V_SAT = 0.22;
W_SAT = 2.84;

% variables
bar_error_ji = zeros(n, n);
Q = [cos(theta), sin(theta);
    -sin(theta), cos(theta)];

pos = p0;
att = a0;
pos_dot = zeros(size(pos));
att_dot = zeros(size(att));

v=zeros(1,n); w(1,n)=0; % non holonomic input

% buffer
buf = struct('pos', zeros(d_xy, t.length), ...
    'pos_dot', zeros(d_xy,t.length), ...
    'att', zeros(d_ang, t.length), ...
    'att_dot', zeros(d_ang, t.length), ...
    'v', zeros(n,t.length), ...
    'w',zeros(n,t.length));

BUF = [buf, buf, buf];

for k=1:t.length

    % caluate non-steepest gradient descent control law
    for i=1:n
        u_global = zeros(d_xy,1);

        for j=1:n
            if any(j == successors(G,i))
                zji = pos(:,j) - pos(:,i);
                z_s_ji = p_desired(:,j) - p_desired(:,i);
                bar_error_ji(i,j) = norm(zji)^2 - norm(z_s_ji)^2; % 1x1
                u_global = u_global  +  Q*bar_error_ji(i,j)*zji;
            end
        end % for j

        % Nonholonomic agent model
        h = [cos(att(i)), sin(att(i))];
        h_perp = [-sin(att(i)), cos(att(i))];

        v(i) = h*u_global;
        w(i) = h_perp*u_global;

        % Saturation
        v(i) = min(max(-V_SAT, v(i)), V_SAT);
        w(i) = min(max(-W_SAT, w(i)), W_SAT);

        pos_dot(1,i) = v(i)*cos(att(i));
        pos_dot(2,i) = v(i)*sin(att(i));
        att_dot(i) = w(i);
    end % for i

    % euler integration
    pos = pos + pos_dot*t.ts;
    att = att + att_dot*t.ts;

    % save buffer
    for b=1:3
        BUF(b).pos(:,k) = pos(:,b);
        BUF(b).pos_dot(:,k) = pos_dot(:,b);
        BUF(b).att(:,k) = att(:,b);
        BUF(b).att_dot(:,k) = att_dot(:,b);
        BUF(b).v(:,k) = v(b);
        BUF(b).w(:,k) = w(b);
    end % for b

end % for k
% ------------------------------------------------------------------------------