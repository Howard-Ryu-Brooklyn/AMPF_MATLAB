bar_error_ji = zeros(n, n);
pos = p0;
att = a0;
att_dot = zeros(size(att));
pos_dot = zeros(size(pos));
v = 0; w = 0;
q_th = 0*D2R;
Q = [cos(q_th), sin(q_th);
    -sin(q_th), cos(q_th)];

model = 'NH';
movingleader=false;

buf_pos_dot = zeros(d(xy),n,t.length);
buf_att_dot = zeros(d(th),n,t.length);
buf_pos = zeros(d(xy),n,t.length);
buf_att = zeros(d(th),n,t.length);
buf_u_global = zeros(2,n,t.length);
buf_vw = zeros(2,n,t.length);