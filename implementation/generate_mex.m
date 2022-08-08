% TODO change path
% [theta1, theta2, theta3] = subproblem5_linear(p0,p1,p2,p3,k1,k2,k3)
P = sp5_setup();
[theta1, theta2, theta3] = subproblem5_linear(P.p0, P.p1, P.p2, P.p3, P.k1, P.k2, P.k3)
codegen -report subproblem5_linear.m -args {P.p0, P.p1, P.p2, P.p3, P.k1, P.k2, P.k3}
%% 

function P = sp5_setup()
    P.p1 = rand_vec;
    P.p2 = rand_vec;
    P.p3 = rand_vec;
    
    P.k1 = rand_normal_vec;
    P.k2 = rand_normal_vec;
    P.k3 = rand_normal_vec;
    
    P.theta1 = rand_angle;
    P.theta2 = rand_angle;
    P.theta3 = rand_angle;
    
    P.p0 = -(rot(P.k1,P.theta1)*P.p1 -rot(P.k2,P.theta2)*(P.p2+rot(P.k3,P.theta3)*P.p3));
end

function v = rand_normal_vec()
    v = rand([3 1]);
    v = v/norm(v);
end

function v = rand_vec()
    v = rand([3 1]);
end

function theta = rand_angle()
    theta = rand*2*pi-pi;
end