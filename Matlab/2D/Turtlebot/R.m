function rotation_matrix = R(theta, dir)


if strcmp(dir,'x') % x
    rotation_matrix = ...
        [1 0 0;
        0 cos(theta) -sin(theta);
        0 sin(theta) cos(theta)];

elseif strcmp(dir,'y') % y

    rotation_matrix = ...
        [cos(theta) 0 sin(theta);
        0 1 0;
        -sin(theta) 0 cos(theta)];

elseif strcmp(dir,'z') % z
    rotation_matrix = ...
        [cos(theta) -sin(theta) 0;
        sin(theta) cos(theta) 0;
        0 0 1];
else
    disp("wrong direction. 1~3");
    rotation_matrix = 0;
end

end