function [is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical] = detect_intersecting_parallel_axes(kin)

THRESH = 1e-3;

N = length(kin.joint_type);

% Make sure kin.P and kin.H are the right sizes
assert(width(kin.P) == N + 1);
assert(width(kin.H) == N);

is_intersecting   = false([1, N-1]); % joints i, i+1 intersect
is_intersecting_nonconsecutive = false([1, N-2]); % joints i, i+2 intersect
is_parallel       = false([1, N-1]); % joints i, i+1 parallel
is_spherical      = false([1, N-2]); % joints i, i+1, i+2 spherical
% (Three parallel joints can be implied from is_parallel)

% Test for 2R and 3R joints
prev_intersection = NaN(3,1);
for i = 1:N-1
    j = i+1;

    p_ij = kin.P(:,j);
    h_i = kin.H(:,i);
    h_j = kin.H(:,j);
    
    ab = pinv([h_i h_j]) * p_ij;
    
    dist_ij = norm(p_ij - [h_i h_j] * ab);
    if dist_ij < THRESH
        is_intersecting(i) = true;
        intersection = sum(kin.P(:,1:j),2) + h_j*ab(2);
        if norm(intersection - prev_intersection) < THRESH
            is_spherical(i-1) = true;
        end
        prev_intersection = intersection;
    else
        prev_intersection = NaN(3,1);
    end
end


% Test for nonconsecutive intersecting axes i, i+2
% There may be a more elegant way to do this
% Set joint i+1 to three angles and see if it's always intersecting or parallel
for i = 1:N-2
    is_intersecting_nonconsecutive(i) = true;
    j = i+1;
    k = i+2;

    for theta = [0, 0.1, 0.2]
        p_ik = kin.P(:,j) + rot(kin.H(:,j), theta)*kin.P(:,k);
        h_i = kin.H(:,i);
        h_k = rot(kin.H(:,j), theta)*kin.H(:,k);

        ab = pinv([h_i h_k]) * p_ik;
        dist_ik = norm(p_ik - [h_i h_k] * ab);

        if not(dist_ik < THRESH || abs(dot(h_i, h_k)) > 1-THRESH)
            is_intersecting_nonconsecutive(i) = false;
            break
        end
    end
end


% Test for 2R|| (and implicitly 3R||) joints
for i = 1:N-1
    j = i+1;
    h_i = kin.H(:,i);
    h_j = kin.H(:,j);

    if abs(dot(h_i, h_j)) > 1-THRESH
        is_parallel(i) = true;
    end
end

end