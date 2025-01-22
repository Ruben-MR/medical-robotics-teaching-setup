%#codegen
function [currentRobotState] = fun_ForwardKinematicsMacroRobot(jointPositions, jointVelocities, actualTorques, cmdedTorques, externalTorquesKuka, robotParameters, robotStateStruct, previousRPY, calibratedGravVector)

currentRobotState = robotStateStruct;

%joint positions
currentRobotState.jointPosition = jointPositions;

%joint velocitites
currentRobotState.jointVelocity = jointVelocities;

%kukaJointPositions = jointPositions(2:8,1);
[endoscopeTipTransform, endoscopeTipJacobian, posJacobianPerJoint, singularityDistanceVec] = getMacroRobotForwardKinematicsData(jointPositions);

%taskspace Position
currentRobotState.taskspacePose = endoscopeTipTransform;

%RPY orientation
currentRobotState.currentRPY = rotmatToRollPitchYaw(endoscopeTipTransform(1:3,1:3), previousRPY);

%Current pose in rpy (6D)
currentRobotState.currentPoseRPY = [endoscopeTipTransform(1:3,4); currentRobotState.currentRPY];

%jacobian
currentRobotState.jacobian = endoscopeTipJacobian; 
currentRobotState.posJacobianPerJoint = posJacobianPerJoint;

currentRobotState.jacobianDeterminant = sqrt(abs(det(endoscopeTipJacobian*endoscopeTipJacobian')));
currentRobotState.singularityDistanceVec = singularityDistanceVec;

%taskspace Velocity
currentRobotState.taskspaceVelocity = currentRobotState.jacobian*currentRobotState.jointVelocity;

currentRobotState.actualTorques = actualTorques;
currentRobotState.cmdedTorques = cmdedTorques;
currentRobotState.externalTorquesKuka = externalTorquesKuka;

for i = 1:length(jointPositions)
    currentRobotState.velNormPerJoint(i)  = norm(posJacobianPerJoint(:,:,i)*jointVelocities);
end

end
