x = 0:0.02:5;
ntheta = 100;
theta = linspace(0,2*pi, ntheta);
A = 1;
d = 0.15;

% cos(theta)*sin(x) + sin(theta)*cos(2*x);
% sin(x) + cos(2*x);

lgnd_ary = [];
max_v_ary = [];
max_w_ary = [];

figure,
for i = 1:ntheta
    plot(x, A*cos(theta(i))*sin(x) + A*sin(theta(i))*cos(2*x)); hold on;
    grid on;
    lgnd_ary = [lgnd_ary, {num2str(theta(i))}];
    fprintf("at theta %s, maximum v value is %s\n",theta(i), max(A*cos(theta(i))*sin(x) + A*sin(theta(i))*cos(2*x)));
    fprintf("at theta %s, maximum w value is %s\n",theta(i), max(-1/d*A*sin(theta(i))*cos(x) + 1/d*A*cos(theta(i))*sin(2*x)) );
    max_w_ary = [max_w_ary max(-1/d*A*sin(theta(i))*cos(x) + 1/d*A*cos(theta(i))*sin(2*x) ) ];
    max_v_ary = [max_v_ary max(A*cos(theta(i))*sin(x) + A*sin(theta(i))*cos(2*x))];
end
% legend(lgnd_ary);

[max_v_value, max_v_index] = max(max_v_ary);
fprintf("total maximum is %s at theta %s\n", max_v_value, mod(theta(max_v_index),pi));

[max_w_value, max_w_index] = max(max_w_ary);
fprintf("total maximum is %s at theta %s\n", max_w_value, mod(theta(max_w_index),pi));



% max value -> sqrt(2)