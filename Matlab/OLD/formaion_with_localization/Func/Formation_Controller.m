% Formation Controller

for i=1:n_agent % ith agent
    A(i).formation_global = zeros(dim_xy,1);
    A(i).input_source = zeros(dim_xy, n_agent);
    
    for j=1:n_agent % jth agent
        if any(j == successors(G,i))
            if i==3 && j==2
                zji = L.x_hat - A(i).pos;
            else
                zji = A(j).pos - A(i).pos;
            end
            % zji = A(j).pos - A(i).pos;
            
            z_s_ji = F.p_desired(:,j) - F.p_desired(:,i);
            bar_error_ji(i,j) = norm(zji)^2 - norm(z_s_ji)^2; % 1x1
            A(i).input_source(:,j) = bar_error_ji(i,j)*zji;
            A(i).formation_global = A(i).formation_global  +  bar_error_ji(i,j)*zji;
        end
    end % for j
end % for i
