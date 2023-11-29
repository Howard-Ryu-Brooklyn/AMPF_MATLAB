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

    % persistently excited
    ps = [2+2*sin(i*t.ts), 2*cos(2*i*t.ts)]';

    z_hat = x_hat'*phi_filtered;
    % estimation law
    x_hat_dot = gamma*(z_filtered - z_hat)'*phi_filtered;
    x_hat = x_hat + x_hat_dot*t.ts;

    y_pos = y_pos + ps*t.ts;

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
    buf.ps(:,i) = ps;
    buf.z_hat(:,i) = z_hat;
    buf.x_hat(:,i) = x_hat;
end