% RAGHAVAN-ROTH MANIPULATOR (RRM)
% Intersecting Joints: 
% Intersecting Nonconsecutive Joints: 
% Parallel Joints: 
% Spherical Joints: 
% 
% MANOCHA-CANNY MANIPULATOR (MCM)
% Intersecting Joints: (3, 4) (5, 6) 
% Intersecting Nonconsecutive Joints: 
% Parallel Joints: (2, 3) (4, 5) 
% Spherical Joints: 
% 
% IACE LABORATORY MANIPULATOR (ILM)
% Intersecting Joints: (1, 2) (2, 3) (5, 6) 
% Intersecting Nonconsecutive Joints: (1, 3) 
% Parallel Joints: 
% Spherical Joints: 
% 
% PAINT MANIPULATOR (PTM)
% Intersecting Joints: (4, 5) (5, 6) 
% Intersecting Nonconsecutive Joints: 
% Parallel Joints: (1, 2) (3, 4) 
% Spherical Joints: 

clc
fprintf('\nRAGHAVAN-ROTH MANIPULATOR (RRM)\n')
DH = ... % # d a alpha
[1 0.9 0.8 20
2 3.7 1.2 31
3 1 0.33 45
4 0.5 1.8 81
5 2.1 0.6 12
6 0.63 2.2 100];
kin = dh_to_kin(deg2rad(DH(:,4)), DH(:,3), DH(:,2));
[is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical] = detect_intersecting_parallel_axes(kin);
print_intersecting_parallel_axes(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)

%%
fprintf('\nMANOCHA-CANNY MANIPULATOR (MCM)\n')
kin = dh_to_kin(deg2rad([90 1 90 1 90 1]), [0.3 1 0 1.5 0 0], [0 0 0.2 0 0 0]);
[is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical] = detect_intersecting_parallel_axes(kin);
print_intersecting_parallel_axes(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)
%%
fprintf('\nIACE LABORATORY MANIPULATOR (ILM)\n')
DH = ... % # d a alpha
[1 0.40518 0 30
2 0.00584 0 60
3 0.26004 0.06419 90
4 -0.20103 0.04036 90
5 0 0 90
6 0.0951 0 0];
kin = dh_to_kin(deg2rad(DH(:,4)), DH(:,3), DH(:,2));
[is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical] = detect_intersecting_parallel_axes(kin);
print_intersecting_parallel_axes(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)

%%
fprintf('\nPAINT MANIPULATOR (PTM)\n')
DH = ... % # d a alpha
[1 0 0.16417 0
2 0 0.9993 90
3 0 0.29574 0
4 1.56192 0 90
5 0.07701 0 50
6 0.0865 0 -50];
kin = dh_to_kin(deg2rad(DH(:,4)), DH(:,3), DH(:,2));
[is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical] = detect_intersecting_parallel_axes(kin);
print_intersecting_parallel_axes(is_intersecting, is_intersecting_nonconsecutive, is_parallel, is_spherical)