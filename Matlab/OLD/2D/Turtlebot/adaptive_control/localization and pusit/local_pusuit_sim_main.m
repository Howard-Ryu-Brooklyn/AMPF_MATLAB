clc; close all; clear;

Declare_variables_buffers();

model = "SI"; % "SI" or "NH"
mode = "SAT"; % "" or "SAT"

fprintf("model: %s is used. \n", model);

if (strcmp(mode,"SAT"))
    disp("Saturation is on");
end

r = [-0.3 0.3]';


% simulation loop
for i=1:t.length
    % get distance measurement
    D = norm(y_pos - x_pos);
    
    % calcuate real value of z and phi
    y_cur = y_pos;
    y_dot = (y_cur - y_pre)/t.ts;
    cur_zv = norm(y_pos)^2 - D^2;
    z_real = 0.5*(cur_zv - pre_zv)/t.ts;
    phi_real = y_dot;

    % calculate filter
    zeta1_dot = -alpha*zeta1 + 0.5*(norm(y_pos)^2 - D^2);
    zeta2_dot = -alpha*zeta2 + y_pos;
    zeta1 = zeta1 + zeta1_dot*t.ts;
    zeta2 = zeta2 + zeta2_dot*t.ts;

    z_filtered = zeta1_dot;
    phi_filtered = zeta2_dot;

    % estimation law
    z_hat = x_hat'*phi_filtered;
    x_hat_dot = gamma*(z_filtered - z_hat)'*phi_filtered;
    x_hat = x_hat + x_hat_dot*t.ts;
    
    % pursuit alogrithm
    cmd = x_hat_dot - beta*(y_pos - (x_hat) - r) + (1-exp(-(D-norm(r))))*sigma_dot;
    % cmd = 0.1*sigma_dot;
    sigma_dot = sigma_matrix*sigma;
    sigma = norm(sigma_init)*[cos(a*(i*t.ts - t.start)+beta0); sin(a*(i*t.ts - t.start)+beta0)]; % analytical integration of sigma dot
    
    % model and calculate position of y
    if(strcmp(model, "NH"))
        h = [cos(y_att), sin(y_att)];
        h_perp = [-sin(y_att), cos(y_att)];

        v = h*cmd;
        w = h_perp*cmd;
        if (strcmp(mode,"SAT"))
            [v, w] = sat([v,w]', "NH");
        end

        y_pos(1) = y_pos(1) + v*cos(y_att)*t.ts;
        y_pos(2) = y_pos(2) + v*sin(y_att)*t.ts;
        y_att = y_att + w*t.ts;
    elseif (strcmp(model, "SI"))
        if (strcmp(mode,"SAT"))
            [cmd(1), cmd(2)]= sat(cmd, "SI");
        end
        y_pos = y_pos + cmd*t.ts;
    end

    % update variable
    y_pre = y_cur;
    pre_zv = cur_zv;
    
    % save buffer
    buf_D(1,i) = D;

    buf_z_real(:,i) = z_real;
    buf_z_filtered(:,i) = z_filtered;

    buf_phi_real(:,i) = phi_real;
    buf_phi_filtered(:,i) = phi_filtered;

    buf_y_pos(:,i) = y_pos;
    buf_y_att(:,i) = y_att;
    buf_cmd(:,i) = cmd;
    buf_cmdvw(:,i) = [v w]';

    buf_z_hat(:,i) = z_hat;
    buf_x_hat(:,i) = x_hat;
    buf_x_hat_dot(:,i) = x_hat_dot;
    buf_sigma(:,i) = sigma;
    buf_sigma_dot(:,i) = sigma_dot;
    buf_cmd_part1(:,i) = x_hat_dot - beta*(y_pos - (x_hat));
    buf_cmd_part2(:,i) = (1-exp(-D))*sigma_dot;
end

Plots();