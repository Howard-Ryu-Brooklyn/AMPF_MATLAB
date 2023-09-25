function [v, w] = sat(in, type)
% saturation value
W_SAT = 2.84; %[rad/s]
V_SAT = 0.22; %[m/s]

SI_SAT = 0.22;

if (strcmp(type, "SI"))
    if (in(1,1) > SI_SAT)
        v = SI_SAT;
    elseif(in(1,1) < SI_SAT && in(1,1) > -SI_SAT)
        v = in(1,1);
    elseif(in(1,1) < -SI_SAT)
        v = -SI_SAT;
    end
    
    if (in(2,1) > SI_SAT)
        w = SI_SAT;
    elseif(in(2,1) < SI_SAT && in(2,1) > -SI_SAT)
        w = in(2,1);
    elseif(in(2,1) < -SI_SAT)
        w = -SI_SAT;
    end

elseif (strcmp(type, "NH"))
    if (in(1,1) > V_SAT)
        v = V_SAT;
    elseif(in(1,1) < V_SAT && in(1,1) > -V_SAT)
        v = in(1,1);
    elseif(in(1,1) < -V_SAT)
        v = -V_SAT;
    end

    if (in(2,1) > W_SAT)
        w = W_SAT;
    elseif(in(2,1) < W_SAT && in(2,1) > -W_SAT)
        w = in(2,1);
    elseif(in(2,1) < -W_SAT)
        w = -W_SAT;
    end
       
end


end