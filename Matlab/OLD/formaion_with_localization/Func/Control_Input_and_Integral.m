% Feedback linearization of nonholonomic at fixed point off (d) the center
% of the wheel axis
for i=1:n_agent
    if (A(i).mode == 1) % formation mode
        % projection law
        h = [cos(A(i).att), sin(A(i).att) ];
        h_perp = [-sin(A(i).att), cos(A(i).att)];

        v = h*A(i).input_source(:,A(i).submode);
        w = h_perp*A(i).input_source(:,A(i).submode);

        v = min(V_SAT, max(-V_SAT, v));
        w = min(W_SAT, max(-W_SAT, w));

        A(i).pos_dot = h'*v;
        A(i).att_dot = w;

    elseif (A(i).mode == 2) % localization mode
        A(i).localization_input = [L.pe_amp*sin(L.pe_freq*(k*t.ts + 15)), L.pe_amp*cos(2*L.pe_freq*(k*t.ts + 15))]';
        A(i).feedback_linearized = [
            cos(A(i).att), sin(A(i).att);
            -1/d*sin(A(i).att), 1/d*cos(A(i).att)] * A(i).localization_input;

        % Saturation
        A(i).feedback_linearized(1) = min(V_SAT, max(-V_SAT, A(i).feedback_linearized(1)));
        A(i).feedback_linearized(2) = min(W_SAT, max(-W_SAT, A(i).feedback_linearized(2)));

        A(i).pos_dot = [cos(A(i).att), -d*sin(A(i).att); sin(A(i).att), d*cos(A(i).att)] * A(i).feedback_linearized;
        A(i).att_dot = A(i).feedback_linearized(2);

    elseif (A(i).mode == 3) % search leader mode
        A(i).pos_dot = 0;
        if (A(i).lf_losang > 0)
            A(i).att_dot = SEARCH_W;
        else
            A(i).att_dot = -SEARCH_W;
        end

    end

    % Euler integration
    A(i).pos = A(i).pos + t.ts * A(i).pos_dot;
    A(i).att = A(i).att + t.ts * A(i).att_dot;
    A(i).att = mod(A(i).att,2*pi);
end



