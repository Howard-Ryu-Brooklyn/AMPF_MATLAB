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
    cmd = x_hat_dot - beta*(y_pos - x_hat) + (1-exp(-D))*sigma_dot;
    % cmd = 0.1*sigma_dot;
    sigma_dot = sigma_matrix*sigma;
    sigma = norm(sigma_init)*[cos(a*(i*t.ts - t.start)+beta0); sin(a*(i*t.ts - t.start)+beta0)]; % analytical integration of sigma dot
    
    % calculate position of y
    y_pos = y_pos + cmd*t.ts;

    % update variable
    y_pre = y_cur;
    pre_zv = cur_zv;
    
    % save buffer
    buf.D(1,i) = D;
    buf.z_real(:,i) = z_real;
    buf.z_filtered(:,i) = z_filtered;
    buf.phi_real(:,i) = phi_real;
    buf.phi_filtered(:,i) = phi_filtered;
    buf.y_pos(:,i) = y_pos;
    buf.cmd(:,i) = cmd;
    buf.z_hat(:,i) = z_hat;
    buf.x_hat(:,i) = x_hat;
    buf.x_hat_dot(:,i) = x_hat_dot;
    buf.sigma(:,i) = sigma;
    buf.sigma_dot(:,i) = sigma_dot;
end