addpath("implementation\")

% Also make sure to add path to general-robotics-toolbox

%% Subproblem 1

p1 = rand_vec;
k = rand_normal_vec;
theta = rand_angle

p2 = rot(k,theta)*p1;

t = subproblem1_linear(p1,p2,k)

norm(p2 - rot(k,t)*p1)

%% Subproblem 2

p1 = rand_vec;

k1 = rand_normal_vec;
k2 = rand_normal_vec;

theta1 = rand_angle;
theta2 = rand_angle;

p2 = rot(k2, -theta2)*rot(k1,theta1)*p1;

[t1,t2] = subproblem2_linear(p1,p2,k1,k2)

for i = 1:length(t1)
    norm(rot(k2, t2(i))*p2 - rot(k1,t1(i))*p1)
end

%% Subproblem 2 Extension
p0 = rand_vec;
p1 = rand_vec;

k1 = rand_normal_vec;
k2 = rand_normal_vec;

theta1 = rand_angle
theta2 = rand_angle

p2 = rot(k2, -theta2)*(p0 + rot(k1,theta1)*p1);

[t1,t2] = subproblem2_linear_extended(p0,p1,p2,k1,k2)


norm(p0 + rot(k1,t1)*p1 - rot(k2, t2)*p2)

%% Subproblem 3

p1 = rand([3 1])*10;
p2 = rand([3 1])*10;
k = rand([3 1]);
k = k/norm(k);
theta = rand*2*pi-pi

d = norm(p2-rot(k,theta)*p1);

t_b = subproblem3_linear(p1,p2,k,d)

norm(p2-rot(k,t_b(1))*p1) - d
norm(p2-rot(k,t_b(2))*p1) - d

%% Subproblem 4

p = rand([3 1])*10;
k = rand([3 1]);
k = k/norm(k);
h = rand([3 1]);
h = h/norm(h);
theta = rand*2*pi-pi

d = h'*rot(k,theta)*p;

t = subproblem4_linear(h,p,k,d)

%% Subproblem 5

p1 = rand([3 1])*10;
p2 = rand([3 1])*10;
p3 = rand([3 1])*10;

k1 = rand([3 1]);
k1 = k1/norm(k1);

k2 = rand([3 1]);
k2 = k2/norm(k2);

k3 = rand([3 1]);
k3 = k3/norm(k3);

theta1 = rand()*2*pi-pi
theta2 = rand()*2*pi-pi
theta3 = rand()*2*pi-pi

p0 = -(rot(k1,theta1)*p1 -rot(k2,theta2)*(p2+rot(k3,theta3)*p3));
p0 + rot(k1,theta1)*p1 - rot(k2,theta2)*(p2+rot(k3,theta3)*p3)

% global H_ap
% global N_ap
% global x1_ap
% H_ap = k2'*(p0 + rot(k1,theta1)*p1)
% k2'*(p2+rot(k3,theta3)*p3)
% 
% N_ap = norm(p0 + rot(k1,theta1)*p1 - H_ap*k2)
% norm(p2+rot(k3,theta3)*p3 - H_ap*k2)
% 
% x1_ap = [sin(theta1); cos(theta1)]

[t1, t2, t3] = subproblem5_linear(p0,p1,p2,p3,k1,k2,k3)

for i = 1:length(t1)
    norm(p0 + rot(k1,t1(i))*p1 - rot(k2,t2(i))*(p2+rot(k3,t3(i))*p3))
end

%% Subproblem 6

p1 = rand_vec;
p2 = rand_vec;
p3 = rand_vec;
p4 = rand_vec;
k1 = rand_normal_vec;
k2 = rand_normal_vec;
h = rand_normal_vec;

theta1 = rand_angle
theta2 = rand_angle

d1 =  h'* rot(k1, theta1) * p1 + h'* rot(k2, theta2) * p2;
d2 =  h'* rot(k1, theta1) * p3 + h'* rot(k2, theta2) * p4;

[t1, t2] = subproblem6_linear( ...
    h, k1, k2, p1, p2, p3, p4, d1, d2)

for i = 1:length(t1)
    [abs(h'* rot(k1, t1(i)) * p1 + h'* rot(k2, t2(i)) * p2 - d1)
    abs(h'* rot(k1, t1(i)) * p3 + h'* rot(k2, t2(i)) * p4 - d2)]
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
