
% ------------------------------------------------------------------------------
% Simulation
% ------------------------------------------------------------------------------

for k=1:t.length % time k

    for i=1:n
        % Calculate lost leader
        A(i).losvec = A(1).pos - A(i).pos;
        A(i).losang = atan2(A(i).losvec(2), A(i).losvec(1));

        if A(i).losang < 0 % [0~4*pi]
            A(i).losang = A(i).losang + 2*pi;
        end

        if A(i).att - A(i).losang > FOV || A(i).att - A(i).losang < -FOV
            A(i).lost_leader = true;
        elseif A(i).att - A(i).losang > 1*D2R || A(i).att - A(i).losang < -1*D2R
            A(i).center_leader = false;
        else
            A(i).center_leader = true;
            A(i).lost_leader = false;
        end

        % Calculate follower center
        if A(3).w_f < 0.05
            A(3).center_follower1 = true;
        else
            A(3).center_follower1 = false;
        end

        % Follower 1 mode management
        if(A(2).lost_leader == true || A(2).center_leader == false)
            A(2).mode = SEARCH_LEADER;
        else
            if(A(2).is_formation_converge)
                A(2).mode = STOP;
                A(2).submode = NOT_APPLICABLE;
            else
                if (A(2).center_leader == false)
                    A(2).mode = SEARCH_LEADER;
                end
                A(2).mode = FORMATION;
                A(2).mode = FOLLOW_LEADER;
            end
        end

        % Follower 2 mode management
        if(k*t.ts < L.TOL_TIME) % Localization Complete
            A(3).mode = LOCALIZATION;
        elseif(A(3).lost_leader == true && A(3).submode == FOLLOW_LEADER)
            A(3).mode = SEARCH_LEADER;
        else
            if(A(3).is_formation_converge)
                A(3).mode = STOP;
                A(3).submode = NOT_APPLICABLE;
            else
                A(3).mode = FORMATION;
                if(A(3).is_follower1_converge)
                    A(3).submode = FOLLOW_LEADER;
                    if (A(3).center_leader == false)
                        A(3).mode = SEARCH_LEADER;
                    end
                elseif(A(3).is_leader_converge)
                    if (A(3).center_follower1 == false)
                        A(3).submode = FOLLOWER_CENTER;
                    else
                        A(3).submode = FOLLOW_FOLLOWER1;
                    end% follower center if
                end% agent convergence if
            end% total convergence if
        end% search leader if


        % caluate non-steepest gradient descent control law
        A(i).u_global = zeros(dim_xy,1);

        for j=1:n
            if any(j == successors(G,i))
                if i==3 && j==2
                    zji = L.x_hat - A(i).pos;
                else
                    zji = A(j).pos - A(i).pos;
                end
                zs_ji = F.p_desired(:,j) - F.p_desired(:,i);
                bar_error_ji(i,j) = norm(zji)^2 - norm(zs_ji)^2; % 1x1
                A(i).input_source(:,j) = bar_error_ji(i,j)*zji;
                A(i).u_global = A(i).u_global  +  Q*bar_error_ji(i,j)*zji;
            end
        end % for j

        % Nonholonomic agent model
        h = [cos(A(i).att), sin(A(i).att)];
        h_perp = [-sin(A(i).att), cos(A(i).att)];

        A(i).v_l = h*A(i).input_source(:, FOLLOW_LEADER);
        A(i).w_l = h_perp*A(i).input_source(:, FOLLOW_LEADER);

        A(i).v_f = h*A(i).input_source(:, FOLLOW_FOLLOWER1);
        A(i).w_f = h_perp*A(i).input_source(:, FOLLOW_FOLLOWER1);

        A(i).is_leader_converge = sqrt(A(i).v_l*A(i).v_l + A(i).w_l*A(i).w_l) < CONVERGE_THRESHOLD;
        A(i).is_follower1_converge = sqrt(A(i).v_f*A(i).v_f + A(i).w_f*A(i).w_f) < CONVERGE_THRESHOLD;
        A(i).is_formation_converge = A(i).is_leader_converge & A(i).is_follower1_converge;

        % CMD
        if(A(i).mode == LOCALIZATION)
            nosy_pos = A(3).pos + 0*rand(1,1);
            L.D = norm(A(2).pos - nosy_pos);
            L.zeta1_dot = -L.alpha*L.zeta1 + 0.5*(norm(nosy_pos)^2 - L.D^2);
            L.zeta2_dot = -L.alpha*L.zeta2 + nosy_pos;
            L.zeta1 = L.zeta1 + L.zeta1_dot*t.ts;
            L.zeta2 = L.zeta2 + L.zeta2_dot*t.ts;
            L.z_filtered = L.zeta1_dot;
            L.phi_filtered = L.zeta2_dot;
            L.z_hat = L.x_hat'*L.phi_filtered;

            % estimation law
            L.x_hat_dot = L.gamma*(L.z_filtered - L.z_hat)*L.phi_filtered;
            L.x_hat = L.x_hat + L.x_hat_dot*t.ts;

            % p.e. input and feedback linearization
            feedback_linearized = [ cos(A(3).att), sin(A(3).att);
                -1/d*sin(A(3).att), 1/d*cos(A(3).att)] * [L.pe_amp*sin(L.pe_freq*(k*t.ts)), L.pe_amp*cos(2*L.pe_freq*(k*t.ts))]';
            A(i).v = feedback_linearized(1);
            A(i).w = feedback_linearized(2);
        elseif(A(i).mode == SEARCH_LEADER)
            A(i).v = 0;
            A(i).w = W_SEARCH;
        elseif(A(i).mode == FORMATION)
            if(A(i).submode==FOLLOW_LEADER)
                A(i).v = A(i).v_l;
                A(i).w = A(i).w_l;
            elseif(A(i).submode == FOLLOW_FOLLOWER1)
                A(i).v = A(i).v_f;
                A(i).w = A(i).w_f;
            elseif(A(i).submode == FOLLOWER_CENTER)
                A(i).v = 0;
                A(i).w = A(i).w_f;
            end
        elseif(A(i).mode == STOP)
            A(i).v = 0;
            A(i).w = 0;
        end

        % Saturation
        A(i).v = min(max(-V_SAT, A(i).v), V_SAT);
        A(i).w = min(max(-W_SAT, A(i).w), W_SAT);

        A(i).pos_dot = h'*A(i).v;
        A(i).att_dot = A(i).w;

        % euler integration
        A(i).pos = A(i).pos + A(i).pos_dot*t.ts;
        A(i).att = mod(A(i).att + A(i).att_dot*t.ts, 2*pi); % [-2pi, 2pi]

        % save buffer
        BUF(i).mode(:,k) = A(i).mode;
        BUF(i).submode(:,k) = A(i).submode;
        BUF(i).lost_leader(:,k) = A(i).lost_leader;
        BUF(i).losvec(:,k) = A(i).losvec;
        BUF(i).losang(:,k) = A(i).losang;
        BUF(i).pos(:,k) = A(i).pos;
        BUF(i).pos_dot(:,k) = A(i).pos_dot;
        BUF(i).att(:,k) = A(i).att;
        BUF(i).att_dot(:,k) = A(i).att_dot;
        BUF(i).u_global(:,k) = A(i).u_global;
        BUF(i).v(:,k) = A(i).v;
        BUF(i).w(:,k) = A(i).w;
        BUF(i).v_l(:,k) = A(i).v_l;
        BUF(i).w_l(:,k) = A(i).w_l;
        BUF(i).v_f(:,k) = A(i).v_f;
        BUF(i).w_f(:,k) = A(i).w_f;
        BUF(i).is_leader_converge(:,k) = A(i).is_leader_converge;
        BUF(i).is_follower1_converge(:,k) = A(i).is_follower1_converge;
        BUF(i).is_formation_converge(:,k) = A(i).is_formation_converge;
    end % for i

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

end % for k
% ------------------------------------------------------------------------------
% ------------------------------------------------------------------------------