function theta = subproblem1_linear(p1, p2, k)
%  p2 = rot(k, theta)*p1

KxP = cross(k, p1);

A = [KxP -cross(k,KxP)];
p = p2-k*k'*p1;

x = A'*p;

theta = atan2(x(1),x(2));

end