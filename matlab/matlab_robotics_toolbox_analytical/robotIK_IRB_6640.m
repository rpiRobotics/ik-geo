function qOpts = robotIK_IRB_6640(eeTform, enforceJointLimits, sortByDistance, referenceConfig)
%robotIK_IRB_6640 Function for generating closed-form inverse kinematics solutions to the DH robot given by the parameters specified below
%   $Revision: $ $Date: $
%
%   Generated on 05-Jun-2023 17:34:12


dhParams = [0.32 -1.5707963267949 0.78 0;1.075 0 0 0;0.2 -1.5707963267949 0 0;0 1.5707963267949 1.1425 0;0 -1.5707963267949 0 0;0 -1.5707963267949 0.2 0];
thetaOffsets = [0 -1.5707963267949 0 0 0 -1.5707963267949];
lastThreeAxesSign = [1 -1 1];
jointLimits = [-3.14159265358979 3.14159265358979;-3.14159265358979 3.14159265358979;-3.14159265358979 3.14159265358979;-3.14159265358979 3.14159265358979;-3.14159265358979 3.14159265358979;-3.14159265358979 3.14159265358979];
isJointRevolute = ([true true true true true true]);

% Compute the shifted joint limits, which are the limits during solution, where theta offsets are not yet in play
shiftedJointLimits = jointLimits + repmat(thetaOffsets(:),1,2);

% Convert the end effector pose in the global frame to the end effector described by the DH parameters relative to the DH-described origin
baseToWorldTform = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
actEEToDhEETform = [6.12323399573677e-17 -1 0 0;1 6.12323399573677e-17 0 0;0 0 1 0;0 0 0 1];
eePose = baseToWorldTform*eeTform*actEEToDhEETform;

% Parse optional inputs
narginchk(1,4);
if nargin < 4
    referenceConfig = zeros(1,6);
end
if nargin < 3
    sortByDistance = false;
end
if nargin < 2
    enforceJointLimits = true;
end

% If joint limits are not enforced, set the shifted joint limits to have infinite range
if ~enforceJointLimits
    shiftedJointLimits = repmat([-inf inf], size(jointLimits, 1), 1);
end
% Map the desired end effector pose to the pose of the central intersecting joint.
eeToJt5 = [1 0 0 0;0 -1 -1.22464679914735e-16 0.2;0 1.22464679914735e-16 -1 -1.22464679914735e-17;0 0 0 1];
jt5Pose = eePose*eeToJt5;

% Solve for the position of the first three joints from the pose of joint 5
q123Opts = solveFirstThreeDHJoints(jt5Pose(1:3,4), dhParams);

% Solve for the positions of the intersecting axes
% For each position solution, this configuration of the last three axes produces at least two possible orientation solutions
numRotationSolns = 2;
q456Opts = zeros(numRotationSolns*size(q123Opts,1), size(q123Opts,2));

% The next step seeks to find the orientation, which is entirely governed by the last three joints. This means that rotation from the fourth joint to the end effector can be mapped to three rotations in-place about the fifth joint. Since the rotations are in-place, they can be defined relative to the fourth joint axes, assuming a fixed pose rotation at the end to align with the end effector. The fixed rotation is found using the DH parameters, and corresponds to the rotation of the end effector relative to the fourth joint when the last three joints are all zero.
eeFixedAlpha = dhParams(4,2) + dhParams(5,2) + dhParams(6,2);
eeFixedRotation = [1 0 0; 0 cos(eeFixedAlpha) -sin(eeFixedAlpha); 0 sin(eeFixedAlpha) cos(eeFixedAlpha)];
for jtIdx = 1:size(q123Opts,1)
    % Get the position of the fourth joint at its zero position when the first three joints are positioned for IK
    jt4ZeroPose = getJoint4PoseFromDH(q123Opts(jtIdx,:));
    
    % Compute the rotation matrix needed to get to the end
    % The orientation of the end effector in the world frame can be written:
    %    eeRot = jt4ZeroRot*(Rotation about axes 4-6)*eeFixedRotation
    % Then the goal is to solve for the rotation about the axes and relate them to he known form from the DH parameters, if a valid solution exists:
    %    (Rotation about axes 4-6) = jt4ZeroRot'*eeRot*eeFixedRotation'
    jt4ToEERot = jt4ZeroPose(1:3,1:3)'*eePose(1:3,1:3)*eeFixedRotation';
    
    % This orientation produces at least two configurations for every solution, when joint limits allow
    orientationSolns = convertRotationToZYZAxesAngles(jt4ToEERot, lastThreeAxesSign, shiftedJointLimits(4:6,:));
    q456Opts(jtIdx,:) = orientationSolns(1,:);
    q456Opts(jtIdx + size(q123Opts,1),:) = orientationSolns(2,:);
    
    % Offset theta to reflect the source robot configuration
    q123Opts(jtIdx,:) = q123Opts(jtIdx,:) - thetaOffsets(1:3);
    q456Opts(jtIdx,:) = q456Opts(jtIdx,:) - thetaOffsets(4:6);
    q456Opts(jtIdx + size(q123Opts,1),:) = q456Opts(jtIdx + size(q123Opts,1),:) - thetaOffsets(4:6);
    
    % Remove solutions that violate joint limits
    if enforceJointLimits
        q123Opts(jtIdx,:) = applyJointLimits(q123Opts(jtIdx,:), jointLimits(1:3,:), isJointRevolute(1:3));
        q456Opts(jtIdx,:) = applyJointLimits(q456Opts(jtIdx,:), jointLimits(4:6,:), isJointRevolute(1:3));
        q456Opts(jtIdx + size(q123Opts,1),:) = applyJointLimits(q456Opts(jtIdx + size(q123Opts,1),:), jointLimits(4:6,:), isJointRevolute(1:3));
    end
    
end

% Filter out any remaining rows with NaNs in them by getting the index of the valid rows and only assembling those in the final output
allSolnOpts = [repmat(q123Opts, numRotationSolns, 1) q456Opts];
isValidRowIdx = all(~isnan(allSolnOpts),2);
qOptsAllSolns = allSolnOpts(isValidRowIdx,:);

% Create a copy of the solutions that wraps all revolute joints to pi, then round within solution tolerance.
qOptsWrappedAndRounded = round(robotics.internal.wrapToPi(qOptsAllSolns)*1.000000e+06)/1.000000e+06;

% Find the indices of all unique values after wrapping to pi
[~, isUniqueValidRowIdx] = unique(qOptsWrappedAndRounded, 'rows');

% Select only unique solutions from the original set of solutions
qOpts = qOptsAllSolns(sort(isUniqueValidRowIdx),:);
% Sort results using a distance metric
if sortByDistance
    qOpts = sortByEuclideanDistance(qOpts, referenceConfig(:)', isJointRevolute);
end

% Helper functions
    function sortedSolutions = sortByEuclideanDistance(solutions, referenceConfig, isJointRevolute)
        %sortByEuclideanDistance Sort a matrix of solution configurations relative to a reference configuration by Euclidean norm
        %   This function sorts a matrix of configurations using a pre-defined
        %   distance metric. The computed distance between any state and the
        %   reference state, referenceConfig, is a Euclidean norm of difference
        %   between a revolute joint's values which is then wrapped to [-pi, pi],
        %   and a displacement between a prismatic joint's values.
        
        %   Copyright 2020 The MathWorks, Inc.
        
        % Compute the distances between each configuration and the reference
        dist = robotics.manip.internal.RigidBodyTreeUtils.distance(...,
            referenceConfig, solutions, isJointRevolute);
        
        % Sort the outputs
        [~, sortedIdx] = sort(dist);
        sortedSolutions = solutions(sortedIdx,:);
        
    end

    function validConfig = applyJointLimits(inputConfig, jointLimits, isJointRevolute)
        %applyJointLimits Convert solutions with invalid joint limits to NaNs
        %   Given an N-element configuration, an Nx2 set of lower and upper joint
        %   limits, and an N-element vector indicating the joint type (revolute or
        %   prismatic), this function checks whether the configuration is within
        %   the joint limits. If not, the configuration is converted to NaNs.
        
        %   Copyright 2020-2021 The MathWorks, Inc.
        
        % Initialize output
        validConfig = inputConfig;
        
        for i = 1:numel(inputConfig)
            if jointLimits(i,1) > inputConfig(i) || jointLimits(i,2) < inputConfig(i)
                
                % Compute the offset from the lower joint limit and compare that to
                % the total range
                wrappedJointValueOffset = robotics.internal.wrapTo2Pi(inputConfig(i) - jointLimits(i,1));
                
                % If the wrapped value is 2*pi, make sure it is instead registered
                % as zero to ensure this doesn't fall outside the range
                if isEqualWithinTolerance(wrappedJointValueOffset, 2*pi)
                    wrappedJointValueOffset = 0;
                end
                
                jointRange = jointLimits(i,2) - jointLimits(i,1);
                
                if isJointRevolute(i) && ((wrappedJointValueOffset < jointRange) || isEqualWithinTolerance(wrappedJointValueOffset, jointRange))
                    
                    % Make sure the final value is definitively inside the joint
                    % limits if it was on the bound
                    wrappedJointValueOffset = min(wrappedJointValueOffset, jointRange);
                    
                    % Update the configuration
                    validConfig(i) = jointLimits(i,1) + wrappedJointValueOffset;
                else
                    % If any element is NaN, the whole array will be thrown out so
                    % there is no need to continue
                    validConfig = nan(size(validConfig));
                    return;
                end
            end
        end
        
    end

    function [actAngles, jointsInGimbalLock] = convertRotationToZYZAxesAngles(tgtRotation, axesSign, jointLim)
        %convertRotationToZYZAxesAngles Convert desired orientation to rotation about Z-Y-Z
        %   This function is used to three angles of rotation corresponding to
        %   consecutive joint angles whose joint axes intersect at a single, common
        %   point. This function addresses the case where the first joint rotation
        %   is about Z, and the subsequent angles, in order and defined relative to
        %   the first joint frame, are about Y and then Z. The function accepts the
        %   rotation of the last joint relative to the origin of the first one, as
        %   well as the sign of each axes. The second output indicates joints that
        %   are in gimbal lock, where 1 indicates that they are, and zero indicates
        %   that they are not. When joints are in gimbal lock, the affected joint
        %   axes align and an infinite combination of joint angles are possible (as
        %   long as the total rotation still matches the target). The default
        %   assumption is that rotation is divided over the joint along the
        %   affected axis.
        %   Copyright 2020 The MathWorks, Inc.
        eulAngles = rotm2eul(tgtRotation, 'ZYZ');
        
        % The jointsInGimalLock variable indicates redundant joints, i.e. joints
        % that complement each other and can have an infinite pair of values in the
        % directJointAngleMaps output. Initialize this value to zeros (no joints in
        % gimbal lock). This is a flag that is consistent across rotation functions
        % and may be used by the caller function.
        jointsInGimbalLock = [0 0 0];
        % When the middle angle is zero, the first and last joints are co-axial,
        % meaning there are an infinite set of solutions. Use a helper function to
        % distribute the values consistently given knowledge of the joint limits.
        if isEqualWithinTolerance(eulAngles(2), 0)
            
            newTgtRotation = tgtRotation;
            newTgtRotation(1:2,3) = 0;
            newTgtRotation(3,1:2) = 0;
            newTgtRotation(3,3) = 1;
            eulAngles = rotm2eul(newTgtRotation, 'ZYZ');
            variableJtIdx = [1 3];
            jointsInGimbalLock(variableJtIdx) = [1 1];
            totalRotation = sum(eulAngles(variableJtIdx));
            eulAngles(variableJtIdx) = distributeRotationOverJoints(totalRotation, axesSign(variableJtIdx), jointLim(variableJtIdx,:));
            
            % In this case the alternate Euler angles aren't required, as they will
            % also result in a set of co-axial joints. However, to ensure codegen
            % compatibility, the size must stay the same Therefore, return a set of
            % NaN angles (so the second solution may be thrown out). Note that the
            % axes sign is handled inside the distributeRotationOverJoints helper
            % function.
            actAngles = [eulAngles; nan(1,3)];
        else
            % For finite solutions when the middle angle is non-zero, there are two possible solutions to this problem
            % that can be derived from the first solution set
            eulAltUnwrapped = eulAngles;
            eulAltUnwrapped(:,2) = -eulAltUnwrapped(:,2);
            eulAltUnwrapped = eulAltUnwrapped + pi;
            eulAltUnwrapped(:,2) = eulAltUnwrapped(:,2) - pi;
            eulAnglesAlt = robotics.internal.wrapToPi(eulAltUnwrapped);
            
            % Output the angles given the axes signs
            actAngles = [eulAngles; eulAnglesAlt]*diag(axesSign);
        end
    end

    function jointAngles = distributeRotationOverJoints(totalRotation, axesSigns, jointLim)
        %distributeRotationOverJoints Distribute a rotation over several in-line revolute joints
        %   When revolute joints are co-axial, the total rotation can be distributed
        %   over the joints in a number of ways. This function assigns a default
        %   behavior that respects the joint limits. For the case where no joint
        %   limits are required, they should be provided as infinite, i.e [-inf
        %   inf] for each joint. The default behaviors are as follows:
        %
        %      - If any joint limits have a range of at minimum 2*pi, all total
        %        rotation amounts are achievable and the rotation is distributed
        %        evenly over the joints with infinite range, assuming the other
        %        joints are centered within their range.
        %
        %      - If all joints have finite ranges with total range less than 2*pi,
        %        some total rotation amounts may not be feasible and the rotation
        %        is distributed as much as possible on the distal joints (unused
        %        more proximal joints are centered). If the solution is infeasible,
        %        a NaN-vector is returned.
        %
        %   The goal of these distributions is to favor solutions that are
        %   efficient. This function accepts the total rotation (in radians) to be
        %   divided over N joints, the signs of those N joints (whether rotation is
        %   positive or negative about the axes), and the joint limits, given as an
        %   Nx2 matrix.
        %
        %   If joint limits are ignored, they can be provided as infinite; the
        %   behavior is equivalent. This function returns an N-element row vector.
        
        %   Copyright 2020 The MathWorks, Inc.
        
        % Get the total number of joints from the joint limit input
        N = size(jointLim, 1);
        
        % Initialize the output
        jointAngles = zeros(1,N);
        
        % Remap the joint limits to fit the assumption that all axes are positive.
        % Since the joint limits can contain infinite elements, it is necessary to
        % use element-wise multiplication, as matrix multiplication can result in
        % NaNs when it causes sums of infinities.
        jointLim = repmat(axesSigns(:),1,2).*jointLim;
        
        % Re-order the joint limits to ensure the lower limit always comes first
        % (in case the of a sign flip in the previous line)
        jointLim = sort(jointLim,2);
        
        % Determine the total ranges of each joint. Since all joints are revolute,
        % a range of 2*pi or greater is equivalent to an infinite range as the IK
        % problem does not distinguish between periodic equivalents. Note that a
        % downstream helper in the IK solution, applyJointLimits, includes a check
        % that wraps periodic equivalents back to their values given the joint
        % limits.
        jointRange = jointLim(:,2) - jointLim(:,1);
        isRevJointFullRange = (jointRange > 2*pi);
        for limIdx = 1:size(jointRange,1)
            % Use a tolerance check on the equality. Since isEqualWithinTolerance
            % returns a scalar, it is necessary to do this inside a for-loop
            isRevJointFullRange(limIdx) = isRevJointFullRange(limIdx) || isEqualWithinTolerance(jointRange(limIdx), 2*pi);
        end
        
        % There are two primary cases: when some joints have full range, any
        % solution is feasible and the variable values are distributed over these
        % joints. When all of the joints have range of less than 2*pi, the problem
        % is more complex, as some solutions may not be feasible.
        if any(isRevJointFullRange)
            % If any of the joint have infinite ranges, use that to distribute the
            % total rotation. First, place the joints with finite ranges in the
            % middle of their respective ranges, then distribute the remaining
            % joint rotation evenly over the joints with at least 2*pi range.
            jointIdxVector = 1:N;
            jointsWithIncompleteRange = jointIdxVector(~isRevJointFullRange);
            for i = 1:numel(jointsWithIncompleteRange)
                jointIdx = jointsWithIncompleteRange(i);
                jointAngles(jointIdx) = sum(jointLim(jointIdx,:))/2;
            end
            
            % Compute the remaining rotation and wrap it to the interval [-pi, pi],
            % then distribute over the joints with complete range
            wrappedRemainder = robotics.internal.wrapToPi(totalRotation - sum(jointAngles));
            jointsWithCompleteRange = jointIdxVector(isRevJointFullRange);
            for j = 1:numel(jointsWithCompleteRange)
                jointIdx = jointsWithCompleteRange(j);
                jointAngles(jointIdx) = wrappedRemainder/numel(jointsWithCompleteRange);
            end
            
        else
            % Use an algorithm that favors loading distal joints, which are
            % typically easier to change: first set all the joints to their
            % mid-range values. Then iterate over the joints from first to last,
            % moving each joint up or down based on the difference in the current
            % total from the desired total, until the desired total is reached.
            % This is essentially a cascaded bang-bang controller.
            
            % Initialize the joint angles to their mid-range values
            jointAngles(:) = (sum(jointLim,2)/2)';
            
            % Iterate over the joints, using a feedback law to move them closer
            % to the desired total
            jointIdxVector = N:-1:1;
            wrappedTotalRotation = robotics.internal.wrapToPi(totalRotation);
            for jointIdx = 1:numel(jointIdxVector)
                diffRotation = robotics.internal.wrapToPi(wrappedTotalRotation - sum(jointAngles));
                jointAngles(jointIdx) = jointAngles(jointIdx) + sign(diffRotation)*min(abs(diffRotation), jointRange(jointIdx)/2);
            end
            
            % Check if the sum of the joint angles reaches the desired total. If
            % not, the solution is infeasible and a vector of NaNs is returned.
            if ~isEqualWithinTolerance(robotics.internal.wrapToPi(sum(jointAngles)), wrappedTotalRotation)
                jointAngles = nan(size(jointAngles));
                return;
            end
        end
        
        % Factor back in the axes signs. Since all valid joint angles are finite,
        % matrix multiplication is the most efficient approach.
        jointAngles = jointAngles*diag(axesSigns);
        
    end

end

function outputThetas = solveFirstThreeDHJoints(jt5Pos, dhParams)
%solveFirstThreeDHJoints Solve for the first three joint angles of a DH-parameterized robot
%   This function computes the first three joint angles of a robot
%   parameterized using Denavit-Hartenberg parameters. The function accepts
%   a matrix of the fixed DH parameters, as well as the position of the
%   fifth joint. The matrix of DH parameters is of size 6x4 for the 6
%   non-fixed joints, where each row has the order [a alpha d 0], where a
%   is a translation along x, alpha is a rotation about x, and d is a
%   translation along z. The last value, which typically refers to theta
%   (the rotation about z) for that joint, is not yet known; this function
%   will solve for theta for the first three joints. When a robot has the
%   last three axes intersecting, the position and orientation of the end
%   effector can be split up: the position is entirely determined by the
%   first three joints, while the orientation is governed by the last three
%   joints (provided the first three are known). Furthermore, the position
%   of any end effector can be related to the position of the fifth joint
%   frame, which corresponds to the joint frame at the midpoint of the
%   three intersecting joints. This frame will have the same position in
%   any valid solution to the particular IK problem (though its orientation
%   may differ), and its translation relative to the base frame is entirely
%   defined by the first three joints. This function solves for those first
%   three joints given the position of the joint 5 frame relative to the
%   base. This solution method and notation follows Chp.3 of Pieper's 1968
%   thesis, but adds two corrections, as well as minor notation changes and
%   the addition of constraints to ensure only feasible solutions are
%   output:
%
%   Pieper, D. The Kinematics Of Manipulators Under Computer Control.
%   Stanford University (1968).

% Extract DH parameters from matrix
[a1, a2, a3] = deal(dhParams(1,1), dhParams(2,1), dhParams(3,1));
[alpha1, alpha2, alpha3] = deal(dhParams(1,2), dhParams(2,2), dhParams(3,2));

% Note that Pieper uses "s" instead of "d" in his solutions
[d1, d2, d3, d4] = deal(dhParams(1,3), dhParams(2,3), dhParams(3,3), dhParams(4,3));

% Three variables derived from jt5Pos
z3 = jt5Pos(3);
R3 = jt5Pos(1)^2 + jt5Pos(2)^2 + (jt5Pos(3) - d1)^2;
z = z3 - d1;

% The first step is to solve for theta3. This is achieved by eliminating
% theta1 and theta2 through a number of substitutions and sum-of-squares
% operations. The resultant equation for theta3, is a function of
% sin(theta3) and cos(theta3), but this can be further mapped to a
% polynomial in h, where h = tan(theta3/2). This substitutions is made
% possible by the Weierstrass transformation, which maps sin(theta3) to
% 2*h/(1 + h^2) and cos(theta3) to (1-h^2)/(1+h^2). The goal is then to
% solve the polynomial for h and map the solutions back to theta3.

% As a1 and sin(alpha1) are both nonzero, use sum of squares to eliminate theta2, which produces a quartic in h
[hSolns, ~, hasPiSoln] = solveForHGeneralCase(R3, z, a1, a2, a3, alpha1, alpha2, alpha3, d2, d3, d4);
% Initialize the matrix of possible solutions
possThetas = zeros(16,3);
% After all solutions are processed, rows with NaNs will be removed
% Initialize theta3 to NaN and replace based on actual solutions
possThetas(:,3) = NaN;
for hIdx = 1:numel(hSolns)
    % Ensure only real solutions to h are converted
    h3 = replaceImagWithNaN(hSolns(hIdx));
    if isnan(h3)
        % When h3 is imaginary, theta3 = NaN
        possThetas(hIdx,3) = NaN;
        possThetas(4+hIdx,3) = NaN;
    else
        % When h3 is real, there are two possible equivalent values of theta3
        possThetas(hIdx,3) = 2*atan2(h3,1);
        possThetas(4+hIdx,3) = 2*atan2(-h3,-1);
    end
end
if hasPiSoln
    possThetas(numel(hSolns)+1,3) = pi;
end
for theta3Idx = 1:8
    % If theta3 is NaN or imaginary, replace whole row with NaNs and skip to next row
    theta3 = replaceImagWithNaN(possThetas(theta3Idx,3));
    if isnan(possThetas(theta3Idx,3))
        possThetas(theta3Idx,:) = [NaN NaN NaN];
        continue
    end
    
    % Compute key subexpressions f1 to f3 and F1 to F4, which are functions of theta3
    f = computef13SupportingEquations(a3, alpha3, d3, d4, theta3);
    F = computeF14SupportingEquations(a1, a2, alpha1, alpha2, f(1), f(2), f(3), d2);
    % Compute theta2. The exact approach depends on the DH
    % parameters, but the gist is the same: since the equations
    % output multiple solutions, but some are actually just results
    % of the sum of squares, i.e., they solve the local problem,
    % but do not actually solve the overlying problem. Rather than
    % compute all solutions and filter at the end, we filter here
    % by always solving using two different equations. Then we
    % choose only the solution that satisfies both equations.
    
    % Since a1 and sin(alpha1) are both nonzero, solve for theta2 using equation 3.25 and 3.26
    theta2Opts = solveTrigEquations(F(1)*2*a1, F(2)*2*a1, R3 - F(3));
    theta2Constraint = solveTrigEquations(-F(2)*sin(alpha1), F(1)*sin(alpha1), z - F(4));
    
    % Choose the solution(s) that solve both equations
    theta2 = chooseCorrectSolution(theta2Opts, theta2Constraint, 1.000000e-06);
    
    % Theta2 is a 2-element vector with up to two valid solutions (invalid
    % solutions are represented by NaNs). Iterate over the possible values
    % and add the second solution set in the latter half of the matrix (so
    % they aren't overwritten by subsequent loops).
    for theta2Idx = 1:2
        % Update the local index so it's reflective of the indexed value of theta2
        solIdx = theta3Idx + 8*(theta2Idx-1);
        
        % Update the value of theta3 in case it was previously set to NaN,
        % and replace any invalid values of theta2 with NaN
        possThetas(solIdx,3) = theta3;
        possThetas(solIdx,2) = replaceImagWithNaN(theta2(theta2Idx));
        
        % If any of the joint variables in NaN, replace it and all the
        % remaining joints to solve with NaNs and move on to the next loop
        if isnan(possThetas(solIdx, 2))
            possThetas(solIdx, 1:2) = [NaN NaN];
            continue;
        end
        
        % Compute theta1 from the first two elements of eq 3.20
        g = computeg12SupportingEquations(a1, a2, alpha1, alpha2, f(1), f(2), f(3), d2, theta2(theta2Idx));
        theta1Opts = solveTrigEquations(g(1), g(2), jt5Pos(1));
        theta1Constraint = solveTrigEquations(-g(2), g(1), jt5Pos(2));
        theta1Opts = chooseCorrectSolution(theta1Opts, theta1Constraint, 1.000000e-06);
        
        % Since theta1 is the last value that is solved for, only one
        % of the solutions will be valid, and chooseCorrectSolution
        % sorts the results so that if there is only one solution, it
        % is always the first element (and the other element is nan)
        theta1 = theta1Opts(1);
        
        % Update the array of possible theta values
        possThetas(solIdx,1) = replaceImagWithNaN(theta1);
        
    end
    
end

% Now we are left with an 8x3 matrix where some values are NaN. The
% function will only output the rows where all elements are non-NaN.
outputThetas = possThetas(all(~isnan(possThetas),2),:);
% Helper functions
    function f = computef13SupportingEquations(a3,alpha3,s3,s4,theta3)
        %computef13SupportingEquations Compute f1 to f3 supporting equations
        %   This function computes f1 to f3, which are functions of theta3. For a
        %   given robot with three consecutive revolute axes, the position of a
        %   joint a distance s4 along the joint 3 axis can be described as:
        %      P = T1*T2*T3*[0; 0; s4; 1],
        %   where Ti represent transformation matrices associated with links. Then
        %   this equation may be rewritten as P = T1*T2*f. This function computes
        %   the values of f that satisfy the rewritten equation.
        
        %   Copyright 2020 The MathWorks, Inc.
        
        % Initialize output
        f = zeros(3,1);
        
        % Compute component terms
        t2 = sin(alpha3);
        t3 = cos(theta3);
        t4 = sin(theta3);
        
        % Assemble outputs. Note that there is technically a fourth output, f(4) =
        % 1, but its value is unused, so it is not computed or returned.
        f(1) = a3.*t3+s4.*t2.*t4;
        f(2) = a3.*t4-s4.*t2.*t3;
        f(3) = s3 + s4.*cos(alpha3);
        
    end


    function F = computeF14SupportingEquations(a1,a2,alpha1,alpha2,f1,f2,f3,s2)
        %computeF14SupportingEquations Compute intermediate variables F1 to F4
        %   This function computes F1 to F4, which are intermediate variables in
        %   Pieper's derivation that are functions of the theta3 joint position
        %   (and constant parameters). The function accepts several DH parameters,
        %   as well as the intermediate variables f1 to f3, which are functions of
        %   theta3, and outputs the four F1 to F4 intermediate variables.
        
        %   Copyright 2020 The MathWorks, Inc.
        
        % Initialize output
        F = zeros(1,4);
        
        F(1) = a2+f1;
        
        t2 = cos(alpha2);
        t3 = sin(alpha2);
        F(2) = -f2.*t2+f3.*t3;
        
        t4 = f3.*t2;
        t5 = f2.*t3;
        F(3) = s2.*(t4+t5).*2.0+a2.*f1.*2.0+a1.^2+a2.^2+f1.^2+f2.^2+f3.^2+s2.^2;
        
        F(4) = cos(alpha1).*(s2+t4+t5);
        
    end

    function g = computeg12SupportingEquations(a1,a2,alpha1,alpha2,f1,f2,f3,s2,theta2)
        %computeg12SupportingEquations Compute g1 and g2 supporting equations
        %   This function computes g1 and g2, which are functions of theta2 and
        %   theta3.
        
        %   Copyright 2020 The MathWorks, Inc.
        % Initialize output
        g = zeros(1,2);
        
        % Compute component terms
        t2 = cos(alpha1);
        t3 = cos(alpha2);
        t4 = sin(alpha2);
        t5 = cos(theta2);
        t6 = sin(theta2);
        t7 = a2+f1;
        t8 = f2.*t3;
        t9 = f3.*t4;
        t10 = -t9;
        t11 = t8+t10;
        
        % Assemble outputs
        g(1) = a1+t5.*t7-t6.*t11;
        g(2) = sin(alpha1).*(s2+f2.*t4+f3.*t3)-t2.*t6.*t7-t2.*t5.*t11;
        
    end


    function theta = solveTrigEquations(a,b,c)
        %solveTrigEquations Solve equations of the form a*cos(theta) + b*sin(theta) = c for theta
        %   This function solves the common trigonometric equality by equating the
        %   solution to cos(phi)sin(theta) + sin(phi)cos(theta) = sin(phi + theta).
        %   The function returns two possible solutions for theta.
        
        %   Copyright 2020 The MathWorks, Inc.
        
        theta = nan(1,2);
        % Handle the trivial case
        if isEqualWithinTolerance(a,0) && isEqualWithinTolerance(b,0) && isEqualWithinTolerance(c,0)
            theta(1) = 0;
            return;
        elseif isEqualWithinTolerance(a,0) && isEqualWithinTolerance(b,0) && ~isEqualWithinTolerance(c,0)
            return;
        end
        % As long as a or b are nonzero, a set of general solutions may be found
        d = sqrt(a^2 + b^2);
        cPrime = c/d;
        if cPrime < 1 || isEqualWithinTolerance(cPrime,1)
            % Throw out the imaginary solutions, which occur when cPrime > 1
            phi1 = atan2(a,b);
            phi2 = atan2(-a,-b);
            theta(1) = real(asin(complex(cPrime))) - phi1;
            theta(2) = -real(asin(complex(cPrime))) - phi2;
        end
        
    end

    function correctSolution = chooseCorrectSolution(solutionPair1, solutionPair2, solTolerance)
        %chooseCorrectSolution Choose the solution that appears in both solutionPair1 and solutionPair2
        %   This helper function is used to choose a correct solution when two
        %   solutions are provided, e.g. as the result of a sum of squares. The
        %   function accepts two 2-element vectors, solutionPair1 and
        %   solutionPair2, which represent the solution options from the source and
        %   constraint equations, respectively. The correct solution will be the
        %   solution both of the source equation, as well as a constraint equation
        %   for the same problem. This helper simply chooses the value that occurs
        %   in both the original and constraint solutions, within a tolerance.
        
        %   Copyright 2020 The MathWorks, Inc.
        
        % Filter any imaginary values out of the solution pairs by replacing them
        % with NaNs
        realSolutionPair1 = zeros(1,2);
        realSolutionPair2 = zeros(1,2);
        for i = 1:2
            % Have to wrap to pi so that the two values are comparable
            realSolutionPair1(i) = robotics.internal.wrapToPi(replaceImagWithNaN(solutionPair1(i)));
            realSolutionPair2(i) = robotics.internal.wrapToPi(replaceImagWithNaN(solutionPair2(i)));
        end
        
        % To check equivalence, it's insufficient to just check whether the values
        % are equal, because they are periodic. For example, -pi and pi are both
        % valid outcomes of wrapToPi that fail a basic equality test but are
        % equivalent in this context. Therefore, it's necessary to check that the
        % difference of the two values, when wrapped to pi, is inside the expected tolerance.
        correctSolution = nan(1,2);
        for i = 1:2
            for j = 1:2
                isCorrect = abs(robotics.internal.wrapToPi(realSolutionPair1(i) - realSolutionPair2(j))) < solTolerance;
                if isCorrect
                    correctSolution(i) = realSolutionPair1(i);
                end
            end
        end
        
        % Sort the output so that if there is one correct solution it is always in
        % the first element slot
        correctSolution = sort(correctSolution);
        
    end

    function q = replaceImagWithNaN(qToCheck)
        %replaceImagWithNaN Replace imaginary and empty elements with NaNs
        %   This function replaces imaginary values with NaNs. This is useful when
        %   the element is part of a matrix, and rendering one element of the
        %   matrix imaginary will make the entire matrix imaginary. Furthermore, it
        %   may be used to filter invalid solutions.
        
        %   Copyright 2020 The MathWorks, Inc.
        
        if isempty(qToCheck)
            q = NaN;
        elseif ~isEqualWithinTolerance(imag(qToCheck), 0)
            q = NaN;
        else
            q = real(qToCheck);
        end
        
    end

    function [hSolns, hasFiniteNumSol, hasPiSoln] = solveForHGeneralCase(R3, z3, a1, a2, a3, alpha1, alpha2, alpha3, d2, d3, d4)
        %solveForHGeneralCase Solve for h when a1 and alpha1 are nonzero
        %   To solve for theta3, it is necessary to reparameterize a trigonometric
        %   equation in terms of a new parameter h = tan(that3/2) using the
        %   Weierstrass equation. This function solves equation 3.39 in the Pieper
        %   inverse kinematics solution for h in the case where DH parameters a1
        %   and alpha1 are both nonzero. The equation arises from the sum of
        %   squares of the following two equations:
        %      3.25: R3 = F1cos(theta2) + F2sin(theta2)2a1 + F3
        %      3.26: z3 = F1sin(theta2) - F2cos(theta2)sin(alpha1) + F4
        %   Here F1 to F4 are functions of theta3 (and the constant DH parameters),
        %   and R3 and z3 are functions of P, a known position input from the IK
        %   problem, and DH parameter d1:
        %      R3 = P(1)^2 + P(2)^2 + (P(3) – d1)^2
        %      z3 = P(3) - d1
        %   The sum of squares produces a single equation that may be
        %   reparameterized in h, producing a quartic polynomial in h. This
        %   function solves that polynomial for the values of h given R3, z3, and
        %   the DH parameters of the associated serial manipulator.
        
        %   Copyright 2020 The MathWorks, Inc.
        
        % Compute the polynomial coefficients
        [A,B,C,D,E] = getQuarticPolynomialCoeffs(R3, z3, ...
            a1, a2, a3, ...
            alpha1, alpha2, alpha3, ...
            d2, d3, d4   ...
            );
        % Add exception to handle the trivial case
        if isEqualWithinTolerance([A B C D E], [0 0 0 0 0])
            % The trivial case happens when the rotation of theta3 has no impact on
            % the end effector position (only on the orientation) because the next
            % joint lies on the axis of rotation. Since this equation is derived
            % from the position solution, any real-valued orientation solution
            % would work. Default to zero.
            hSolns = 0;
            hasFiniteNumSol = false;
            hasPiSoln = true;
        else
            % Solve polynomial. While there are four solutions to the quartic, there
            % can be at most two solutions for this variable -- the others are false
            % solutions that arise from the sum of squares. These will be eliminated
            % below by using constraint equations.
            hSolns = solveQuarticPolynomial([A B C D E]);
            hasFiniteNumSol = true;
            
            fTerms = computef13SupportingEquations(a3, alpha3, d3, d4, pi);
            FTerms = computeF14SupportingEquations(a1, a2, alpha1, alpha2, fTerms(1), fTerms(2), fTerms(3), d2);
            
            % Check if there is a solution at theta3 = pi, for which h is undefined, by
            % checking if R3 = F3 (eq 3.25) is satisfied for that solution.
            eq339LHS = (FTerms(4) - z3)^2/sin(alpha1)^2 + (FTerms(3) - R3)^2/(4*a1^2);
            eq339RHS = FTerms(1)^2 + FTerms(2)^2;
            hasPiSoln = isEqualWithinTolerance(eq339LHS, eq339RHS);
        end
        
        % Helper functions
        function [h4Coef, h3Coef, h2Coef, h1Coef, h0Coef] = getQuarticPolynomialCoeffs(R3s,z3s,a1,a2,a3,alpha1,alpha2,alpha3,d2,d3,d4)
            %getQuarticPolynomialCoeffs Compute the coefficients of the quartic polynomial
            %   The first part of the solution is a fourth-order polynomial in h =
            %   tan(theta3/2). This function computes the coefficients of that
            %   polynomial, A*h^4 + B*h^3 + C*h^2 + D*h + E = 0.
            
            %Polynomial coefficients for DH robot
            t2 = cos(alpha1);
            t3 = cos(alpha2);
            t4 = cos(alpha3);
            t5 = sin(alpha1);
            t6 = sin(alpha2);
            t7 = sin(alpha3);
            t8 = R3s.^2;
            t9 = a1.^2;
            t10 = a2.^2;
            t12 = a2.^3;
            t13 = a3.^2;
            t15 = a3.^3;
            t17 = d2.^2;
            t18 = d2.^3;
            t19 = d3.^2;
            t21 = d3.^3;
            t22 = d4.^2;
            t24 = d4.^3;
            t26 = z3s.^2;
            t11 = t9.^2;
            t14 = t10.^2;
            t16 = t13.^2;
            t20 = t17.^2;
            t23 = t19.^2;
            t25 = t22.^2;
            t27 = t2.^2;
            t28 = t3.^2;
            t29 = t4.^2;
            t30 = t4.^3;
            t32 = t5.^2;
            t33 = t6.^2;
            t34 = t7.^2;
            t35 = t7.^3;
            t43 = t9.*t26.*4.0;
            t52 = d2.*t2.*t9.*z3s.*8.0;
            t80 = a3.*t2.*t6.*t9.*z3s.*1.6e+1;
            t81 = d3.*t2.*t3.*t9.*z3s.*8.0;
            t110 = d4.*t2.*t3.*t4.*t9.*z3s.*8.0;
            t112 = d4.*t2.*t6.*t7.*t9.*z3s.*8.0;
            t31 = t29.^2;
            t36 = t34.^2;
            t37 = t8.*t32;
            t38 = t11.*t32;
            t39 = t14.*t32;
            t40 = t16.*t32;
            t41 = t20.*t32;
            t42 = t23.*t32;
            t44 = R3s.*a2.*a3.*t32.*4.0;
            t45 = R3s.*t9.*t32.*2.0;
            t46 = R3s.*t10.*t32.*2.0;
            t47 = R3s.*t13.*t32.*2.0;
            t48 = a2.*t15.*t32.*4.0;
            t49 = a3.*t12.*t32.*4.0;
            t50 = R3s.*t17.*t32.*2.0;
            t51 = R3s.*t19.*t32.*2.0;
            t53 = a2.*a3.*t9.*t32.*4.0;
            t54 = a2.*a3.*t17.*t32.*4.0;
            t55 = a2.*a3.*t19.*t32.*4.0;
            t56 = R3s.*d2.*d3.*t3.*t32.*4.0;
            t57 = R3s.*d3.*d4.*t4.*t32.*4.0;
            t63 = -t52;
            t64 = t9.*t10.*t32.*2.0;
            t65 = t9.*t13.*t32.*2.0;
            t66 = t10.*t13.*t32.*6.0;
            t67 = t9.*t17.*t27.*4.0;
            t68 = t9.*t17.*t32.*2.0;
            t69 = t9.*t19.*t32.*2.0;
            t70 = t10.*t17.*t32.*2.0;
            t71 = t10.*t19.*t32.*2.0;
            t72 = t13.*t17.*t32.*2.0;
            t73 = t13.*t19.*t32.*2.0;
            t74 = t17.*t19.*t32.*2.0;
            t76 = R3s.*a3.*d2.*t6.*t32.*8.0;
            t77 = R3s.*a2.*d4.*t7.*t32.*8.0;
            t83 = d2.*t3.*t21.*t32.*4.0;
            t84 = d3.*t3.*t18.*t32.*4.0;
            t85 = d4.*t4.*t21.*t32.*4.0;
            t86 = a2.*a3.*d2.*d3.*t3.*t32.*8.0;
            t87 = a2.*a3.*d3.*d4.*t4.*t32.*8.0;
            t88 = d3.*d4.*t4.*t17.*t32.*4.0;
            t91 = R3s.*d2.*d4.*t3.*t4.*t32.*4.0;
            t92 = R3s.*d2.*d4.*t6.*t7.*t32.*4.0;
            t95 = -t80;
            t96 = -t81;
            t97 = a3.*t6.*t18.*t32.*8.0;
            t98 = d2.*t6.*t15.*t32.*8.0;
            t99 = d4.*t7.*t12.*t32.*8.0;
            t100 = d2.*d3.*t3.*t9.*t32.*4.0;
            t101 = d2.*d3.*t3.*t10.*t32.*4.0;
            t102 = d2.*d3.*t3.*t13.*t32.*4.0;
            t103 = d3.*d4.*t4.*t9.*t32.*4.0;
            t104 = d3.*d4.*t4.*t10.*t32.*4.0;
            t105 = d3.*d4.*t4.*t13.*t32.*4.0;
            t106 = a3.*d2.*t6.*t19.*t32.*8.0;
            t107 = a2.*d4.*t7.*t17.*t32.*8.0;
            t108 = a2.*d4.*t7.*t19.*t32.*8.0;
            t111 = d4.*t3.*t4.*t18.*t32.*4.0;
            t113 = d4.*t6.*t7.*t18.*t32.*4.0;
            t114 = R3s.*t22.*t29.*t32.*2.0;
            t115 = R3s.*t22.*t32.*t34.*2.0;
            t116 = a3.*d2.*t6.*t9.*t27.*1.6e+1;
            t117 = d2.*d3.*t3.*t9.*t27.*8.0;
            t118 = a3.*d2.*t6.*t9.*t32.*8.0;
            t119 = a3.*d2.*t6.*t10.*t32.*8.0;
            t120 = a2.*d4.*t7.*t9.*t32.*8.0;
            t121 = a2.*d4.*t7.*t13.*t32.*8.0;
            t122 = a2.*d2.*t6.*t13.*t32.*1.6e+1;
            t123 = a3.*d4.*t7.*t9.*t32.*1.6e+1;
            t124 = a3.*d4.*t7.*t10.*t32.*1.6e+1;
            t125 = d3.*t24.*t30.*t32.*4.0;
            t126 = a2.*a3.*t22.*t29.*t32.*4.0;
            t127 = -t110;
            t128 = a2.*a3.*t22.*t32.*t34.*4.0;
            t129 = d2.*d4.*t3.*t4.*t9.*t32.*4.0;
            t130 = d2.*d4.*t3.*t4.*t10.*t32.*4.0;
            t131 = d2.*d4.*t3.*t4.*t13.*t32.*4.0;
            t132 = a2.*a3.*d2.*d4.*t3.*t4.*t32.*8.0;
            t134 = d2.*d4.*t6.*t7.*t9.*t32.*4.0;
            t135 = d2.*d4.*t6.*t7.*t10.*t32.*4.0;
            t136 = d2.*d4.*t6.*t7.*t13.*t32.*4.0;
            t137 = a2.*a3.*d2.*d4.*t6.*t7.*t32.*8.0;
            t138 = a2.*d2.*d3.*d4.*t3.*t7.*t32.*1.6e+1;
            t139 = a3.*d2.*d3.*d4.*t4.*t6.*t32.*1.6e+1;
            t141 = d2.*d4.*t6.*t7.*t19.*t32.*4.0;
            t142 = a2.*t24.*t32.*t35.*8.0;
            t144 = t9.*t19.*t27.*t28.*4.0;
            t145 = t9.*t22.*t29.*t32.*2.0;
            t146 = t10.*t22.*t29.*t32.*2.0;
            t147 = t13.*t22.*t29.*t32.*2.0;
            t148 = t9.*t19.*t32.*t33.*4.0;
            t149 = t9.*t22.*t32.*t34.*2.0;
            t150 = t10.*t22.*t32.*t34.*2.0;
            t151 = t13.*t22.*t32.*t34.*2.0;
            t152 = t17.*t19.*t28.*t32.*4.0;
            t153 = t17.*t22.*t29.*t32.*2.0;
            t154 = t19.*t22.*t29.*t32.*6.0;
            t155 = t17.*t22.*t32.*t34.*2.0;
            t156 = t19.*t22.*t32.*t34.*2.0;
            t157 = a3.*d3.*t3.*t6.*t9.*t27.*1.6e+1;
            t158 = d2.*d4.*t3.*t4.*t9.*t27.*8.0;
            t159 = a3.*d3.*t3.*t6.*t9.*t32.*1.6e+1;
            t160 = d2.*t3.*t24.*t30.*t32.*4.0;
            t161 = d3.*t4.*t24.*t32.*t34.*4.0;
            t162 = d2.*d4.*t6.*t7.*t9.*t27.*8.0;
            t163 = a3.*d3.*t3.*t6.*t17.*t32.*1.6e+1;
            t164 = a2.*d3.*t4.*t7.*t22.*t32.*1.6e+1;
            t165 = d2.*d4.*t3.*t4.*t19.*t32.*1.2e+1;
            t166 = d2.*t6.*t24.*t32.*t35.*4.0;
            t168 = d2.*d3.*t3.*t22.*t32.*t34.*4.0;
            t170 = t25.*t29.*t32.*t34.*2.0;
            t171 = a2.*t7.*t24.*t29.*t32.*8.0;
            t172 = d3.*d4.*t4.*t9.*t27.*t28.*8.0;
            t173 = a3.*d4.*t7.*t9.*t27.*t33.*1.6e+1;
            t175 = d3.*d4.*t4.*t9.*t32.*t33.*8.0;
            t176 = a3.*d2.*t6.*t22.*t29.*t32.*8.0;
            t177 = d3.*d4.*t4.*t17.*t28.*t32.*8.0;
            t178 = d2.*d3.*t3.*t22.*t29.*t32.*1.2e+1;
            t179 = a3.*d2.*t6.*t22.*t32.*t34.*8.0;
            t180 = a2.*d2.*t6.*t22.*t32.*t34.*1.6e+1;
            t181 = a3.*d4.*t7.*t17.*t32.*t33.*1.6e+1;
            t182 = a3.*d4.*t3.*t4.*t6.*t9.*t27.*1.6e+1;
            t183 = a3.*d4.*t3.*t4.*t6.*t9.*t32.*1.6e+1;
            t184 = d2.*t3.*t4.*t24.*t32.*t34.*4.0;
            t185 = d3.*d4.*t3.*t6.*t7.*t9.*t27.*8.0;
            t186 = a2.*d2.*t3.*t4.*t7.*t22.*t32.*1.6e+1;
            t187 = a3.*d4.*t3.*t4.*t6.*t17.*t32.*1.6e+1;
            t188 = d2.*t6.*t7.*t24.*t29.*t32.*4.0;
            t189 = d3.*d4.*t3.*t6.*t7.*t9.*t32.*8.0;
            t190 = d3.*d4.*t3.*t6.*t7.*t17.*t32.*8.0;
            t191 = d2.*d3.*t4.*t6.*t7.*t22.*t32.*8.0;
            t193 = t9.*t22.*t27.*t28.*t29.*4.0;
            t194 = t9.*t22.*t27.*t33.*t34.*4.0;
            t195 = t9.*t22.*t28.*t32.*t34.*4.0;
            t196 = t9.*t22.*t29.*t32.*t33.*4.0;
            t197 = t17.*t22.*t28.*t29.*t32.*4.0;
            t198 = t17.*t22.*t32.*t33.*t34.*4.0;
            t199 = t3.*t4.*t6.*t7.*t9.*t22.*t27.*8.0;
            t200 = t3.*t4.*t6.*t7.*t9.*t22.*t32.*8.0;
            t201 = t3.*t4.*t6.*t7.*t17.*t22.*t32.*8.0;
            t58 = -t45;
            t59 = -t46;
            t60 = -t47;
            t61 = -t50;
            t62 = -t51;
            t75 = t25.*t31.*t32;
            t78 = -t56;
            t79 = -t57;
            t82 = t25.*t32.*t36;
            t89 = -t64;
            t90 = -t65;
            t93 = -t76;
            t94 = -t77;
            t109 = -t91;
            t133 = -t114;
            t140 = -t115;
            t143 = -t120;
            t167 = -t137;
            t169 = -t148;
            t174 = t28.*t123;
            t192 = -t175;
            t202 = -t195;
            t203 = -t196;
            
            h4Coef = t37+t38+t39+t40+t41+t42+t43+t44-t48-t49+t53-t54-t55+t58+t59+t60+t61+t62+t63+t66+t67+t68+t69+t70+t71+t72+t73+t74+t75+t78+t79+t82+t83+t84+t85-t86-t87+t88+t89+t90-t92+t96+t100+t101+t102+t103+t104+t105+t109+t111-t112+t113+t117+t125-t126+t127-t128+t129+t130+t131-t132+t133+t134+t135+t136+t140+t141+t144+t145+t146+t147+t149+t150+t151+t152+t153+t154+t155+t156+t158+t160+t161+t162+t165+t166+t167+t168+t169+t170+t172+t177+t178+t184+t185+t188+t189+t190+t191+t192+t193+t194+t197+t198+t199+t200+t201+t202+t203;
            if nargout > 1
                h3Coef = t93+t94+t95+t97+t98+t99+t106+t107+t108+t116+t118+t119+t121-t122+t123-t124+t138+t139+t142+t143+t157+t159+t163+t164+t171+t173+t176+t179+t180+t181+t182+t183+t186+t187-a3.*d4.*t7.*t9.*t28.*t32.*1.6e+1;
            end
            if nargout > 2
                h2Coef = t37.*2.0+t38.*2.0+t39.*2.0+t40.*2.0+t41.*2.0+t42.*2.0+t75.*2.0+t82.*2.0+t9.*t26.*8.0-R3s.*t9.*t32.*4.0-R3s.*t10.*t32.*4.0-R3s.*t13.*t32.*4.0-R3s.*t17.*t32.*4.0-R3s.*t19.*t32.*4.0-t9.*t10.*t32.*4.0+t9.*t17.*t27.*8.0+t9.*t13.*t32.*1.2e+1-t10.*t13.*t32.*4.0+t9.*t17.*t32.*4.0+t10.*t17.*t32.*4.0+t9.*t19.*t32.*4.0+t10.*t19.*t32.*4.0+t13.*t17.*t32.*4.0+t13.*t19.*t32.*4.0+t17.*t19.*t32.*4.0-R3s.*t22.*t29.*t32.*4.0-R3s.*t22.*t32.*t34.*4.0+d3.*t3.*t18.*t32.*8.0+d2.*t3.*t21.*t32.*8.0+d4.*t4.*t21.*t32.*8.0+d3.*t24.*t30.*t32.*8.0+t9.*t13.*t27.*t33.*1.6e+1-t9.*t13.*t28.*t32.*1.6e+1+t9.*t19.*t27.*t28.*8.0+t9.*t22.*t29.*t32.*4.0-t9.*t19.*t32.*t33.*8.0+t10.*t22.*t29.*t32.*4.0+t13.*t17.*t32.*t33.*1.6e+1+t13.*t22.*t29.*t32.*4.0+t17.*t19.*t28.*t32.*8.0-t9.*t22.*t32.*t34.*1.2e+1+t10.*t22.*t32.*t34.*2.0e+1+t17.*t22.*t29.*t32.*4.0+t13.*t22.*t32.*t34.*4.0+t19.*t22.*t29.*t32.*1.2e+1+t17.*t22.*t32.*t34.*4.0+t19.*t22.*t32.*t34.*4.0+t25.*t29.*t32.*t34.*4.0-d2.*t2.*t9.*z3s.*1.6e+1-R3s.*d2.*d3.*t3.*t32.*8.0-R3s.*d3.*d4.*t4.*t32.*8.0+d2.*d3.*t3.*t9.*t27.*1.6e+1+d2.*d3.*t3.*t9.*t32.*8.0+d2.*d3.*t3.*t10.*t32.*8.0+d3.*d4.*t4.*t9.*t32.*8.0+d2.*d3.*t3.*t13.*t32.*8.0+d3.*d4.*t4.*t10.*t32.*8.0+d3.*d4.*t4.*t13.*t32.*8.0+d3.*d4.*t4.*t17.*t32.*8.0+d4.*t3.*t4.*t18.*t32.*8.0+d2.*t3.*t24.*t30.*t32.*8.0+d3.*t4.*t24.*t32.*t34.*8.0+t9.*t22.*t27.*t28.*t29.*8.0-t9.*t22.*t27.*t33.*t34.*8.0+t9.*t22.*t28.*t32.*t34.*8.0-t9.*t22.*t29.*t32.*t33.*8.0+t17.*t22.*t28.*t29.*t32.*8.0-t17.*t22.*t32.*t33.*t34.*8.0-d3.*t2.*t3.*t9.*z3s.*1.6e+1+d2.*d4.*t3.*t4.*t9.*t27.*1.6e+1+d2.*d4.*t3.*t4.*t9.*t32.*8.0+d2.*d4.*t3.*t4.*t10.*t32.*8.0+d2.*d4.*t3.*t4.*t13.*t32.*8.0+d2.*d4.*t3.*t4.*t19.*t32.*2.4e+1+d3.*d4.*t4.*t9.*t27.*t28.*1.6e+1-d3.*d4.*t4.*t9.*t32.*t33.*1.6e+1+d3.*d4.*t4.*t17.*t28.*t32.*1.6e+1+d2.*d3.*t3.*t22.*t29.*t32.*2.4e+1+d2.*d3.*t3.*t22.*t32.*t34.*8.0+d2.*t3.*t4.*t24.*t32.*t34.*8.0-d4.*t2.*t3.*t4.*t9.*z3s.*1.6e+1-R3s.*d2.*d4.*t3.*t4.*t32.*8.0+a2.*a3.*d2.*d4.*t6.*t7.*t32.*4.8e+1;
            end
            if nargout > 3
                h1Coef = t93+t94+t95+t97+t98+t99+t106+t107+t108+t116+t118+t119+t121+t122-t123+t124+t138+t139+t142+t143+t157+t159+t163+t164+t171-t173+t174+t176+t179-t180-t181+t182+t183+t186+t187;
            end
            if nargout > 4
                h0Coef = t37+t38+t39+t40+t41+t42+t43-t44+t48+t49-t53+t54+t55+t58+t59+t60+t61+t62+t63+t66+t67+t68+t69+t70+t71+t72+t73+t74+t75+t78+t79+t82+t83+t84+t85+t86+t87+t88+t89+t90+t92+t96+t100+t101+t102+t103+t104+t105+t109+t111+t112-t113+t117+t125+t126+t127+t128+t129+t130+t131+t132+t133-t134-t135-t136+t140-t141+t144+t145+t146+t147+t149+t150+t151+t152+t153+t154+t155+t156+t158+t160+t161-t162+t165-t166+t167+t168+t169+t170+t172+t177+t178+t184-t185-t188-t189-t190-t191+t192+t193+t194+t197+t198-t199-t200-t201+t202+t203;
            end
        end
    end

    function polySolns = solveQuarticPolynomial(polyCoeffs)
        %solveQuarticPolynomial Solve 4th order polynomial
        %   This function accepts a vector [A B C D E] of coefficients and solves
        %   the equation of the form Ax^4 + Bx^3 + Cx^2 + Dx + E = 0 using an
        %   analytical formulation.
        %
        %   Reference:
        %       Weisstein, Eric W. "Quartic Equation." From MathWorld--A
        %       Wolfram Web Resource. https://mathworld.wolfram.com/QuarticEquation.html
        
        %   Copyright 2020 The MathWorks, Inc.
        
        % Analytical methods are not robust to division by zero, so filter out
        % cases that are actually linear, quadratic, or cubic
        isCoeffZero = [...
            isEqualWithinTolerance(polyCoeffs(1),0) ...
            isEqualWithinTolerance(polyCoeffs(2),0) ...
            isEqualWithinTolerance(polyCoeffs(3),0) ...
            isEqualWithinTolerance(polyCoeffs(4),0) ...
            isEqualWithinTolerance(polyCoeffs(5),0) ...
            ];
        
        if all(isCoeffZero)
            % All coefficients are zero. This is a trivial solution; output zero
            polySolns = 0;
        elseif all(isCoeffZero([1 2 3]))
            % The first three coefficients are zero; this problem is linear
            polySolns = -polyCoeffs(5)/polyCoeffs(4);
        elseif all(isCoeffZero([1 2]))
            % The first two coefficients are zero; this problem is quadratic
            polySolns = [...
                (-polyCoeffs(4) + sqrt(complex(polyCoeffs(4)^2 - 4*polyCoeffs(3)*polyCoeffs(5))))/(2*polyCoeffs(3)); ...
                (-polyCoeffs(4) - sqrt(complex(polyCoeffs(4)^2 - 4*polyCoeffs(3)*polyCoeffs(5))))/(2*polyCoeffs(3))];
        elseif isCoeffZero(1)
            % The first coefficient are zero; this problem is cubic. Solve this
            % using the cubic solver subroutine, which accepts inputs in standard
            % polynomial form.
            polySolns = solveCubicPolynomial(polyCoeffs(3)/polyCoeffs(2), polyCoeffs(4)/polyCoeffs(2), polyCoeffs(5)/polyCoeffs(2));
        else
            % This problem is quartic
            
            % Rewrite in standard polynomial form. Be sure to use different
            % variables than are used elsewhere in code, as this may otherwise
            % create global variables that corrupt data in other parts of the
            % generated solution.
            polyA0 = polyCoeffs(5)/polyCoeffs(1);
            polyA1 = polyCoeffs(4)/polyCoeffs(1);
            polyA2 = polyCoeffs(3)/polyCoeffs(1);
            polyA3 = polyCoeffs(2)/polyCoeffs(1);
            
            % Compute a real solution to the resolvent cubic polynomial
            cubicRoots = solveCubicPolynomial(-polyA2, polyA1*polyA3 - 4*polyA0, 4*polyA2*polyA0- polyA1^2 - polyA3^2*polyA0);
            
            % Select a real-valued root
            resCubicRealRoot = cubicRoots(1);
            for i = 1:3
                if imag(cubicRoots(i)) == 0
                    resCubicRealRoot = cubicRoots(i);
                    continue;
                end
            end
            
            % To minimize code generation issues, declare contents of the square
            % roots to be complex to avoid unexpected complex terms
            
            % Compute supporting elements
            R = sqrt(complex(1/4*polyA3^2 - polyA2 + resCubicRealRoot));
            
            if isEqualWithinTolerance(R, 0)
                D = sqrt(complex(3/4*polyA3^2 - 2*polyA2 + 2*sqrt(resCubicRealRoot^2 - 4*polyA0)));
                E = sqrt(complex(3/4*polyA3^2 - 2*polyA2 - 2*sqrt(resCubicRealRoot^2 - 4*polyA0)));
            else
                D = sqrt(complex(3/4*polyA3^2 - R^2 - 2*polyA2 + 1/4*(4*polyA3*polyA2 - 8*polyA1 - polyA3^3)/R));
                E = sqrt(complex(3/4*polyA3^2 - R^2 - 2*polyA2 - 1/4*(4*polyA3*polyA2 - 8*polyA1 - polyA3^3)/R));
            end
            
            % Assemble the four solutions
            polySolns = [...
                -1/4*polyA3 + 1/2*R + 1/2*D; ...
                -1/4*polyA3 + 1/2*R - 1/2*D; ...
                -1/4*polyA3 - 1/2*R + 1/2*E; ...
                -1/4*polyA3 - 1/2*R - 1/2*E];
        end
        
        
        function cubicRoots = solveCubicPolynomial(b2, b1, b0)
            %solveCubicPolynomial Solve for a real-valued root to a cubic polynomial
            %   This function solves for a real root of the cubic polynomial in
            %   the form x^3 + b2*x^2 + b1*x + b0 = 0. This type of polynomial
            %   has three roots, two of which may be complex.
            
            % Use Cardano's formula
            cubicQ = (3*b1 - b2^2)/9;
            cubicR = (9*b2*b1 - 27*b0 - 2*b2^3)/54;
            cubicD = cubicQ^3 + cubicR^2;
            
            % Make sure to call square roots with sqrt(complex()) to ensure
            % code generation support when numbers are negative
            if cubicD < 0
                cubicS = (cubicR + sqrt(complex(cubicD)))^(1/3);
                cubicT = (cubicR - sqrt(complex(cubicD)))^(1/3);
            else
                % When D is greater than zero, use nthroot, which ensures the
                % real-valued root is returned
                cubicS = nthroot((cubicR + sqrt(cubicD)),3);
                cubicT = nthroot((cubicR - sqrt(cubicD)),3);
            end
            
            cubicRoots = [...
                -1/3*b2 + (cubicS + cubicT); ...
                -1/3*b2 - 1/2*(cubicS + cubicT) + 1/2*1i*sqrt(3)*(cubicS - cubicT); ...
                -1/3*b2 - 1/2*(cubicS + cubicT) - 1/2*1i*sqrt(3)*(cubicS - cubicT); ...
                ];
        end
    end

end

function jt4Pose = getJoint4PoseFromDH(q123)
%getJoint4PoseFromDH Get the pose of the fourth joint when the first three joints set to q123 and joint4 angle is zero

dhParams = [0.32 -1.5707963267949 0.78 0;1.075 0 0 0;0.2 -1.5707963267949 0 0;0 1.5707963267949 1.1425 0;0 -1.5707963267949 0 0;0 -1.5707963267949 0.2 0];

% Initialize output
jt4Pose = eye(4);
for i = 1:3
    a = dhParams(i,1);
    alpha = dhParams(i,2);
    d = dhParams(i,3);
    theta = q123(i);
    
    Ttheta = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1];
    TFixed = [1 0 0 a; 0 cos(alpha) -sin(alpha) 0; 0 sin(alpha) cos(alpha) d; 0 0 0 1];
    
    jt4Pose = jt4Pose*Ttheta*TFixed;
end

end

function isEquiv = isEqualWithinTolerance(mat1, mat2)
%isEqualWithinTolerance Check if two matrices are equal within a set tolerance
%   This is a convenience function designed for inputs with up to two
%   dimensions. If the input has 3+ dimensions, a non-scalar output will be
%   returned.

tol = 1.000000e-06;
diffMat = abs(mat1 - mat2);
isEquiv = all(all(diffMat < tol));

end
