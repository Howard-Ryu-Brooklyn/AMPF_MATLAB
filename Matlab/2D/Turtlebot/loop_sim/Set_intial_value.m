% set intial value of position and attitude randomly

% p0 = r*(rand(d(xy), n) - 0.5);
% a0 = 2*pi*(rand(d(th), n) - 0.5);

p0 = [-0.7417,    0.4263,    2.0860;
      1.6541,    0.2486,   -1.0708];
a0 = [1.6160    2.3825    1.6344];

% follower detect leader when the leader is in its fov angle
% fov = 60 [deg]
fov = 100*D2R;
los_vec21=p0(:,G.Edges{1,1}(2)) - p0(:,G.Edges{1,1}(1));
los_vec31=p0(:,G.Edges{2,1}(2)) - p0(:,G.Edges{2,1}(1));
los21=atan2(los_vec21(2), los_vec21(1));
los31=atan2(los_vec31(2), los_vec31(1));
a0(2) = los21 + fov*(rand(1,1)-0.5);
a0(3) = los31 + fov*(rand(1,1)-0.5);