function generate_openrave_xml(kin, robot_name)
% http://openrave.programmingvision.com/wiki/index.php/Format:XML

N = length(kin.joint_type);

KinBody = struct;

% Generate all the bodies: Base, N links, End Effector
KinBody.Body(1) = Body("base", [], [0;0;0]);
prev_name = "base";
for i = 1:N
    body_idx = i + 1; % offset 1 for base body

    name = "link_" + i;
    KinBody.Body(body_idx) = Body(name, prev_name, kin.P(:,i));
    prev_name = name;
end
KinBody.Body(N+2) = Body("EE", prev_name, kin.P(:,N+1));

% Generate N joints
for i = 1:N
    if i == 1
        body1_name = "base";
    else
        body1_name = "link_"+(i-1);
    end
    body2_name = "link_"+i;
    KinBody.Joint(i) = Joint("joint_"+i, body1_name, body2_name, kin.H(:,i));
end

% Constant joint to connect last link to EE
KinBody.Joint(N+1) = lockedJoint("joint_EE", "link_"+N, "EE");

% Generate the manipulator
KinBody.Manipulator = Manipulator("base", "EE", ["joint_"+(1:N) "joint_EE"]);

% Generate and write out the robot
robot = struct;
robot.nameAttribute = robot_name;
robot.KinBody = KinBody;
writestruct(robot, robot_name+".xml", StructNodeName="Robot");
end

function b = Body(name, prev_name, p_ij)
    b = struct;
    b.nameAttribute = name;
    b.typeAttribute = "dynamic";
    b.offsetfrom = prev_name;
    b.Translation = mat2str(p_ij);
end

function j = Joint(name, body1_name, body2_name, h_i)
    j = struct;
    j.circularAttribute = true;
    j.nameAttribute = name;
    j.typeAttribute = "hinge";

    j.enableAttribute = [];

    j.body(1) = body1_name;
    j.body(2) = body2_name;
    j.offsetfrom = body2_name;
    j.axis = mat2str(h_i);
    j.limits = [];
end

function j = lockedJoint(name, body1_name, body2_name)
    j = struct;
    j.circularAttribute = [];
    j.nameAttribute = name;
    j.typeAttribute = [];

    j.enableAttribute = false;

    j.body(1) = body1_name;
    j.body(2) = body2_name;
    j.offsetfrom = [];
    j.axis = [];
    j.limits = mat2str([0 0]);
    
end

function m = Manipulator(base_name, effector_name, joint_names)
    m = struct;
    m.effector = effector_name;
    m.base = base_name;
    m.joints = mat2str(joint_names);
end


function s = mat2str(mat)
    if isempty(mat)
        s = [];
    end
    s = strjoin(string(mat));
end