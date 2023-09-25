

% save buffer
for i=1:n_agent
    BUF(i).mode(:,k) = A(i).mode;
    BUF(i).submode(:,k) = A(i).submode;
    BUF(i).lost_leader(k) = A(i).lost_leader;
    BUF(i).lf_losvec(:,k) = A(i).lf_losvec;
    BUF(i).lf_losang(k) = A(i).lf_losang;
    BUF(i).localization_flag(k) = A(i).localization_flag;
    BUF(i).pos(:,k) = A(i).pos;
    BUF(i).pos_dot(:,k) = A(i).pos_dot;
    BUF(i).att(:,k) = A(i).att;
    BUF(i).att_dot(:,k) = A(i).att_dot;
    BUF(i).formation_global(:,k) = A(i).formation_global;
    BUF(i).formation_local(:,k) = A(i).formation_local;
    BUF(i).feedback_linearized(:,k) = A(i).feedback_linearized;
    BUF(i).localization_input(:,k) = A(i).localization_input;
end
buf_L.zeta1(:,k) = L.zeta1;
buf_L.zeta2(:,k) = L.zeta2;
buf_L.zeta1_dot(:,k) = L.zeta1_dot;
buf_L.zeta2_dot(:,k) = L.zeta2_dot;
buf_L.z_fil(:,k) = L.z_fil;
buf_L.phi_fil(:,k) = L.phi_fil;
buf_L.z_hat(:,k) = L.z_hat;
buf_L.x_hat_dot(:,k) = L.x_hat_dot;
buf_L.x_hat(:,k) = L.x_hat;