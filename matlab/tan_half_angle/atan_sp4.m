% Solve Subproblem 4 according to
% "General frame for arbitrary 3R subproblems based on the POE model"
%
% Since they use x = tan(theta/2), there is a singularity around theta = pi
% We see that the solution breaks down sometimes in this case
% (about 8.2 % chance)



p = rand_vec;
k = rand_normal_vec;
h = rand_normal_vec;
%theta = rand_angle;
theta = pi;

d = h'*rot(k,theta)*p;


A_11 = cross(k,p);
A_1 = [A_11 -cross(k,A_11)];
A = h'*A_1;

b = d - h'*k*(k'*p);

x1 = A(1);
y1 = A(2);
z1 = b;

num_p = x1 + sqrt(x1^2 + y1^2 - z1^2);
num_n = x1 - sqrt(x1^2 + y1^2 - z1^2);
den = y1 + z1;
theta_test_p = 2*atan(num_p/den);
theta_test_n = 2*atan(num_n/den);

wrapToPi(theta_test_p - theta)
wrapToPi(theta_test_n - theta)

%%

subproblem.sp_4(h,p,k,d)