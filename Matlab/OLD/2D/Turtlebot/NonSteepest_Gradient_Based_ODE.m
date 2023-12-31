function x_dot = NonSteepest_Gradient_Based_ODE(x, G, p_desired, d, theta, model, movingleader)
%GRADIENT_BASED_ODE 이 함수의 요약 설명 위치
%theta: non-steepest gradient descent angle

n = G.numnodes;
bar_error_ji = zeros(n, n);
l = 0.15; %[m]

Q = [cos(theta), sin(theta);
    -sin(theta), cos(theta)];
%     ode45에서 열벡터 형태로 계산하기 때문에 바꿔줘야함
th = 1;
xy = 2;


% x = [ori; pos]
att = x(1:d(th)*n);
pos = reshape(x(d(th)*n+1:end), [d(xy), n]);

att_dot = zeros(size(att));
pos_dot = zeros(size(pos));

for k=1:n
    u_global = zeros(d(xy),1);

    for j=1:n
        if any(j == successors(G,k))
            zji = pos(:,j) - pos(:,k);
            z_s_ji = p_desired(:,j) - p_desired(:,k);
            bar_error_ji(k,j) = norm(zji)^2 - norm(z_s_ji)^2; % 1x1
            u_global = u_global  +  Q*bar_error_ji(k,j)*zji;
        end
    end % for j

    % agent model
    if strcmp(model, 'SI') % Single Integrator pdot=u
        pos_dot(:,k) = u_global;
        att_dot(k) = 0;
    elseif strcmp(model, 'FL') % non-holonomic with Feedback Linearization pdot=u
        input = [cos(att(k)), sin(att(k));
            -1/l*sin(att(k)), 1/l*cos(att(k))]*u_global;

        dots=[cos(att(k)), 0; ...
            sin(att(k)), 0; ...
            0, 1]*input;

        pos_dot(:,k) = dots(1:2);
        att_dot(k)= dots(3);
    elseif strcmp(model, 'NH') % Nonholonomic
        h = [cos(att(k)), sin(att(k))];
        h_perp = [-sin(att(k)), cos(att(k))];

        v = h*u_global;
        w = h_perp*u_global;
        
        pos_dot(1,k) = v*cos(att(k));
        pos_dot(2,k) = v*sin(att(k));
        att_dot(k) = w;
    end % if agent model

end % for k

if movingleader==true
    % circle
    pos_dot(:,1) = 0.2*[-pos(2,1); pos(1,1)];
end

x_dot = [att_dot(:); pos_dot(:)];
end % function

