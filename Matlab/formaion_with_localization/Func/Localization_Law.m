for i=1:n_agent
    if (A(i).localization_flag == 1)
        L.D = norm(A(i).pos - A(2).pos);
        L.zeta1_dot = -L.alpha*L.zeta1 + 0.5*(norm(A(i).pos)^2 - L.D^2);
        L.zeta2_dot = -L.alpha*L.zeta2 + A(i).pos;
        L.zeta1 = L.zeta1 + L.zeta1_dot*t.ts;
        L.zeta2 = L.zeta2 + L.zeta2_dot*t.ts;
        L.z_fil = L.zeta1_dot;
        L.phi_fil = L.zeta2_dot;
        L.z_hat = L.x_hat'*L.phi_fil;

        % estimation law
        L.x_hat_dot = L.gamma*(L.z_fil - L.z_hat)*L.phi_fil;
        L.x_hat = L.x_hat + L.x_hat_dot*t.ts;

        L.tol_cnt = L.tol_cnt + 1;

        % localization is finished after TOL_TIME
        if L.tol_cnt*t.ts > L.TOL_TIME
            L.tol_tf = k*t.ts;
            fprintf("localization has compeleted during %2.1f ~ %2.1f \n", L.tol_ts, L.tol_tf);
            A(i).localization_flag = 0;
        end
    end
end