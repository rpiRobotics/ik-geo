% Test how often the code shown in atan_sp4.m provides the wrong solution
% Running the code results in about 8.2 % chance of one solution being NaN


N = 1e5;
positives = 0;

for i=1:N
    positives = positives + soln_has_nan;
end

positives / N *100

function is_nan = soln_has_nan
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
    
    is_nan =  isnan(theta_test_p) || isnan(theta_test_n);
end