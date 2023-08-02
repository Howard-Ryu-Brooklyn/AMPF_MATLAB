clc; clear; close all;

% simulation time
t = struct( ...
    'start', 0, ...
    'final', 10, ...
    'ts', 0.01, ...
    'time', 0, ...
    'length', 0);

t.time = t.start:t.ts:t.final-t.ts;
t.length = length(t.time);

% utilities
D2R=pi/180;
R2D=1/D2R;

Declare_Topology();

Set_intial_value();


% Sim_variables();
bar_error_ji = zeros(n, n);
pos = p0;
att = a0;
att_dot = zeros(size(att));
pos_dot = zeros(size(pos));
v = 0; w = 0;
q_th = 0*D2R;
Q = [cos(q_th), sin(q_th);
    -sin(q_th), cos(q_th)];

model = 'NH';
movingleader=false;

buf_pos_dot = zeros(d(xy),n,t.length);
buf_att_dot = zeros(d(th),n,t.length);
buf_pos = zeros(d(xy),n,t.length);
buf_att = zeros(d(th),n,t.length);
buf_u_global = zeros(2,n,t.length);
buf_vw = zeros(2,n,t.length);

% simulation loop
for k=1:t.length

    % calculate control input and get dots
    for i=1:n
        u_global = zeros(d(xy),1);

        for j=1:n
            if any(j == successors(G,i))
                zji = pos(:,j) - pos(:,i);
                z_s_ji = p_desired(:,j) - p_desired(:,i);
                bar_error_ji(i,j) = norm(zji)^2 - norm(z_s_ji)^2; % 1x1
                u_global = u_global  +  Q*bar_error_ji(i,j)*zji;
            end
        end % for j

        % agent model
        if strcmp(model, 'SI') % Single Integrator pdot=u
            pos_dot(:,i) = u_global;
            att_dot(i) = 0;
        elseif strcmp(model, 'NH') % Nonholonomic
            h = [cos(att(i)), sin(att(i))];
            h_perp = [-sin(att(i)), cos(att(i))];

            v = h*u_global;
            w = h_perp*u_global;

            buf_vw(1,i,k) = v;
            buf_vw(2,i,k) = w;
            % vw cmd -> staturation -> w1, w2 -> motor -> torque -> turtlebot3 dynamics -> pos' att'
            pos_dot(1,i) = v*cos(att(i));
            pos_dot(2,i) = v*sin(att(i));
            att_dot(i) = w;
        end % if agent model

    end % for k

    if movingleader==true
        % circle
        pos_dot(:,1) = 0.2*[-pos(2,1); pos(1,1)];
    end
    
    for i = 1:n
        % Euler integration
        pos(:,i) = pos(:,i) + pos_dot(:,i)*t.ts;
        att(:,i) = att(:,i) + att_dot(:,i)*t.ts;

        % Save Buffer
        buf_pos_dot(:,i,k) = pos_dot(:,i);
        buf_att_dot(:,i,k) = att_dot(:,i);
        buf_pos(:,i,k) = pos(:,i);
        buf_att(:,i,k) = att(:,i);
        buf_u_global(:,i,k)=u_global;

    end
end

Plot_results();