V =  rand([3 1]);
V = V / norm(V);

R = cross(rand([3 1]), V);
R = R / norm(R);

conv = sew_conv([1;0;0])
test_sew_class(conv)

stereo = sew_stereo(R,V)
test_sew_class(stereo)

function test_sew_class(SEW)
ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];

S = rand([3 1]);
E = rand([3 1]);
W = rand([3 1]);

psi = SEW.fwd_kin(S, E, W);

k = SEW.inv_kin(S,W,psi);

w_hat = vec_normalize(W-S);
p = (eye(3)-w_hat*w_hat')*(W-S);
norm(cross(k,p))

[J_e, J_w] = SEW.jacobian(S, E, W);
delta = 1e-8;

J_e_num = ...
[(SEW.fwd_kin(S, E+delta*ex, W) - SEW.fwd_kin(S, E, W))/delta
 (SEW.fwd_kin(S, E+delta*ey, W) - SEW.fwd_kin(S, E, W))/delta
 (SEW.fwd_kin(S, E+delta*ez, W) - SEW.fwd_kin(S, E, W))/delta]';

J_w_num = ...
[(SEW.fwd_kin(S, E, W+delta*ex) - SEW.fwd_kin(S, E, W))/delta
 (SEW.fwd_kin(S, E, W+delta*ey) - SEW.fwd_kin(S, E, W))/delta
 (SEW.fwd_kin(S, E, W+delta*ez) - SEW.fwd_kin(S, E, W))/delta]';

norm(J_e - J_e_num)
norm(J_w - J_w_num)
end

function n = vec_normalize(vec)
    n =  vec / norm(vec);
end
