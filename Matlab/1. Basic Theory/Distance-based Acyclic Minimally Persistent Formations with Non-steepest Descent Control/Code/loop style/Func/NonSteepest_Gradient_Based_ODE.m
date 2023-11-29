function x_dot = NonSteepest_Gradient_Based_ODE(x, G, p_desired, d, theta)

n = G.numnodes; % # of agents
bar_error_ji = zeros(n, n);
Q = [cos(theta), sin(theta);
    -sin(theta), cos(theta)];

%     ode45에서 열벡터 형태로 계산하기 때문에 바꿔줘야함
pos = reshape(x, [d, n]);
pos_dot = zeros(size(pos));

% caluate non-steepest gradient descent control law
for k=1:n
    u_global = zeros(d,1);

    for j=1:n
        if any(j == successors(G,k))
            zji = pos(:,j) - pos(:,k);
            z_s_ji = p_desired(:,j) - p_desired(:,k);
            bar_error_ji(k,j) = norm(zji)^2 - norm(z_s_ji)^2; % 1x1
            u_global = u_global  +  Q*bar_error_ji(k,j)*zji;
        end
    end % for j

    % Single Integrator pdot=u
    pos_dot(:,k) = u_global;

end % for k

% return
x_dot = pos_dot(:);
end

