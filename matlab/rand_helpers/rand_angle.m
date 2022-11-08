function theta = rand_angle(size)
    if nargin < 1
        size = 1;
    end
    theta = rand(size)*2*pi-pi;
end
