#pragma once

extern "C" {


// End effector position 3D vector
struct ikgen_EEPos
{
	double x, y, z;
};

// End effector orientation quaternion
struct ikgen_EEOri
{
	double x, y, z, w;
};

// Joint position angles in rad, 6D vector
struct ikgen_JointPos
{
	double x1, x2, x3, x4, x5, x6;
};

// Compute joint positions from end effector position, orientation
ikgen_JointPos* computeInverseKinematics(
		const ikgen_EEPos& eePos,
		const ikgen_EEOri& eeOri,
		int& num_solutions);

// Return the robot name
char* ikgen_RobotName();


} // extern "C"
