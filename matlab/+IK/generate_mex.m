% TODO change path

setups = {
    IK_setups.IK_spherical_2_parallel
    IK_setups.IK_3_parallel
    IK_setups.IK_3_parallel_2_intersecting
    IK_setups.IK_spherical
    IK_setups.IK_spherical_2_intersecting
    IK_setups.IK_2_intersecting
    IK_setups.IK_gen_6_dof
};
% IK_setups.IK_2R_2R_3R

for i = 1:length(setups)
    setups{i}.generate_mex()
end