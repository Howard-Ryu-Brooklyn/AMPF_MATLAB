for k=1:t.length

    noisy_f2_pos = f2_pos + 0.2*rand(1,1); % UWB noise N(0, 0.02)
    
    L.D = norm(f1_pos - noisy_f2_pos);
    L.zeta1_dot = -L.alpha*L.zeta1 + 0.5*(norm(noisy_f2_pos)^2 - L.D^2);
    L.zeta2_dot = -L.alpha*L.zeta2 + noisy_f2_pos;
    L.zeta1 = L.zeta1 + L.zeta1_dot*t.ts;
    L.zeta2 = L.zeta2 + L.zeta2_dot*t.ts;
    L.z_filtered = L.zeta1_dot;
    L.phi_filtered = L.zeta2_dot;
    L.z_hat = L.x_hat'*L.phi_filtered;
    
    % estimation law
    L.x_hat_dot = L.gamma*(L.z_filtered - L.z_hat)*L.phi_filtered;
    L.x_hat = L.x_hat + L.x_hat_dot*t.ts;
    
%     if(norm(L.x_hat_dot) < L.TOLERANCE)
%         break;
%     end
    
    % persistently excited
    ps = [L.pe_amp*sin(L.pe_freq*(k*t.ts)), L.pe_amp*cos(2*L.pe_freq*(k*t.ts))]';
    


    % --------------------------------------
    % ------------  AGENT  ------------
    % --------------------------------------
    % feedback linearization
    feedback_linearized = [
        cos(f2_att), sin(f2_att);
        -1/d*sin(f2_att), 1/d*cos(f2_att)] * ps;
    
    % Saturation
    feedback_linearized = min([V_SAT, W_SAT]' , max([-V_SAT, -W_SAT]', feedback_linearized));
    
    % nonholonomic agent mdel
    f2_pos_dot = [cos(f2_att), -d*sin(f2_att); sin(f2_att), d*cos(f2_att)] * feedback_linearized;
    f2_att_dot = feedback_linearized(2);

    % Euler Integration
    f2_pos = f2_pos + f2_pos_dot*t.ts;
    f2_att = f2_att + f2_att_dot*t.ts;
    % ---------------------------------------

    % save buffer
    buf_L.D(:,k) = L.D;
    buf_L.zeta1(:,k) = L.zeta1;
    buf_L.zeta2(:,k) = L.zeta2;
    buf_L.zeta1_dot(:,k) = L.zeta1_dot;
    buf_L.zeta2_dot(:,k) = L.zeta2_dot;
    buf_L.z_filtered(:,k) = L.z_filtered;
    buf_L.phi_filtered(:,k) = L.phi_filtered;
    buf_L.z_hat(:,k) = L.z_hat;
    buf_L.x_hat_dot(:,k) = L.x_hat_dot;
    buf_L.x_hat(:,k) = L.x_hat;

    buf_f2_pos(:,k) = f2_pos;
    buf_ps(:,k) = ps;
end