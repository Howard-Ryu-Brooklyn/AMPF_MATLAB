% mode mangagement
for i=1:n_agent
    if (A(i).localization_flag == true)
        A(i).mode = 2; % localization mode
    elseif (A(i).lost_leader == true && A(i).submode == 1)
        A(i).mode = 3; % search leader mode
    else
        A(i).mode = 1; %formation mode
    end
end

% calculate lost leader
for i=1:n_agent
    A(i).lf_losvec = A(1).pos - A(i).pos;
    A(i).lf_losang = atan2(A(i).lf_losvec(2), A(i).lf_losvec(1));

    if A(i).lf_losang < 0
        A(i).lf_losang = A(i).lf_losang + 2*pi;
    end

    if A(i).att - A(i).lf_losang > FOV || A(i).att - A(i).lf_losang < -FOV
        A(i).lost_leader = true;
    else
        A(i).lost_leader = false;
    end

end

% submode mangagement
if (A(3).localization_flag == false)
    if (vecnorm(A(3).input_source(:,A(3).submode)) < SUBMODE_TRESHOLD)
        if (A(3).submode == 1)
            A(3).submode = 2;
        else
            A(3).submode = 1;
        end
    end
end

% % Follower 2
% for i=1:n_agent
%     if (vecnorm(A(i).input_source(:,A(i).submode)) < SUBMODE_TRESHOLD)
%         A(i).submode = mod(A(i).submode + 1, 3);
%         if (A(i).submode == 0);       A(i).submode = A(i).submode + 1; end
%         if (A(i).submode == i);       A(i).submode = 1; end
%     end
% end