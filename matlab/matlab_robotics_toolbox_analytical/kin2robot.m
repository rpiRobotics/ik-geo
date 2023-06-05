function robot = kin2robot(kin)

robot = rigidBodyTree("DataFormat","column");

prev = 'base';

for i = 1:width(kin.H)
    body_name = "body_"+i;
    jnt_i = rigidBodyJoint("joint_"+i, 'revolute');
    jnt_i.HomePosition = 0;
    jnt_i.JointAxis = kin.H(:,i)';

    tform = trvec2tform(kin.P(:,i)');
    setFixedTransform(jnt_i,tform);
    body_i = rigidBody(body_name);
    body_i.Joint = jnt_i;

    addBody(robot,body_i,prev)
    prev = body_name;
end

bodyEndEffector = rigidBody('end_effector');
tform = trvec2tform(kin.P(:,end)');
if isfield(kin, 'RT')
    tform = tform * rotm2tform(kin.RT);
end
setFixedTransform(bodyEndEffector.Joint,tform);
addBody(robot,bodyEndEffector,prev);

end
