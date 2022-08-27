function theta = subproblem1_linear(p1, p2, k)
%  p2 = rot(k, theta)*p1

KxP = cross(k, p1);
A = [KxP -cross(k,KxP)];

x = A'*p2;

theta = atan2(x(1),x(2));

end