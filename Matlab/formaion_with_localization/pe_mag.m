at = pi/16;
theta = pi/4;

x = linspace(0,20,200);
ntheta = 100;
theta = linspace(0,2*pi, ntheta);
A = 0.22/sqrt(2);

% cos(theta)*sin(x) + sin(theta)*cos(2*x);
% sin(x) + cos(2*x);

lgnd_ary = [];
max_ary = [];

figure,
for i = 1:ntheta
    plot(x, A*cos(theta(i))*sin(x) + A*sin(theta(i))*cos(2*x)); hold on;
    grid on;
    lgnd_ary = [lgnd_ary, {num2str(theta(i))}];
    fprintf("at theta %s, maximum value is %s\n",theta(i), max(A*cos(theta(i))*sin(x) + A*sin(theta(i))*cos(2*x)));
    max_ary = [max_ary max(A*cos(theta(i))*sin(x) + A*sin(theta(i))*cos(2*x))];
end
legend(lgnd_ary);

[max_value, max_index] = max(max_ary);
fprintf("total maximum is %s at theta %s\n", max_value, theta(max_index));

% max value -> sqrt(2)