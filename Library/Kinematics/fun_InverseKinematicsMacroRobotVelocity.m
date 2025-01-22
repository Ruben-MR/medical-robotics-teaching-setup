 %#codegen
function [desiredJointPositions, ikDebugData]  =  fun_InverseKinematicsMacroRobotVelocity(desiredVelocity, currentJacobian, currentJointPositions, isJointSaturated, singularityDistanceVec, posJacPerJoint, kP, kO, kNS, desiredNullSpaceVelocity, robotParameters, Ts, ikDebugDataStruct)

%macroRobotInverseKinematicsFunction - Performs the inverse kinematics for
% the macroRobot 
%
% Syntax:  [jointPositionCandidates] = fun_InverseKinematicsMacroRobotVelocity(desiredVelocity, currentJacobian, currentJointPositions, currentPose, robotLinkParameters, Ts,  kP, kO, kNS)
%
% Inputs:
%    desiredVelocity - the desired pose (6x1) vector in the taskspace
%                           as [posX, posY, posZ, yaw, pitch, roll]
%    currentJacobian - the DH parameters of the GG1 as defined in Lynch and
%                           Park, 2016
%     currentJointPositions
%     currentPose
%     robotLinkParameters
%     Ts
%     kP
%     kO
%     kNS
%
% Outputs:
%    jointPositionCandidates - mathematical solutions to the inverse
%    kinematics, as a structure (iksolutions bus) having 2 solutions (solution1, solution2) for each joint variable.
%    Also has an ikError variable with the definition:
%
% Example:
%    jointPositionCandidates = fun_InverseKinematicsKuka(desiredCartesianPose, robotLinkParamaters);
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
% Used in: Simulink block embedded function lib_InverseKinematicsKuka
%
% Author: Murali Karnam
% email: murali.karnam@unibas.ch
% October 2019; Last revision: 22-October-2019
%------------- BEGIN CODE --------------

LAMBDA = 0.001; %For pseudo inverse
JACOBIAN_THRESHOLD = 0.0010; % Threshold to check if the robot is close to workspace limits
JACOBIAN_THRESHOLD_SMOOTH = 0.004;
SINGULARITY_NORM_MAX = 0.82-0.02; %Stop at 1cm away
SINGULARITY_NORM_L1 = 0.82-0.1; % Slow from 5cm of buffer

%invWEIGHT = eye(length(currentJointPositions));
 
ikDebugData = ikDebugDataStruct; 

useJacobian = currentJacobian;
desVel = [kP*desiredVelocity(1:3); kO*desiredVelocity(4:6)];

%% Pure kuka control

% For desired motion
% rmr: modified for only kuka
useJacobianKuka = useJacobian;
desQDot = zeros(length(currentJointPositions),1);
desQDotPosOriCtrl = zeros(7,1);

jacPosOriInverse = pseudoInverseMat(useJacobianKuka, LAMBDA);

desQDotPosOriCtrl = jacPosOriInverse*desVel;

% For null space motion
jacobianP1NullSpace = eye(7) - jacPosOriInverse*useJacobianKuka;

selectedNSJointMotion = zeros(7,1);
[eVectors, eValues] = eig(jacobianP1NullSpace);

if(eValues(1,1) > 0.1)
    selectedNSJointMotion = real(eVectors(:,1));
end

selectedNSJointMotion = sign(selectedNSJointMotion(1))*selectedNSJointMotion;

% Normalize NS motion
selectedNSJointMotionNormed = 0.*selectedNSJointMotion;
if(max(abs(selectedNSJointMotion)) > 0.01)
    selectedNSJointMotionNormed = selectedNSJointMotion./max(abs(selectedNSJointMotion));
end

% rmr: Modified for only kuka

desQDotP2_ns = kNS*(selectedNSJointMotionNormed'*desiredNullSpaceVelocity)*selectedNSJointMotionNormed;  

desQDot(:,1) = desQDotPosOriCtrl + desQDotP2_ns;

%% Singularity avoidance specifically for 5th Joint
% rmr: changed index for only kuka
singularityLimX = 1;
j7Vel = posJacPerJoint(:,:,5)*desQDot;
j3Vel = posJacPerJoint(:,:,1)*desQDot;

j73Vel = j7Vel-j3Vel;

velWSDir = j73Vel'*singularityDistanceVec;
singularityDistanceNorm = norm(singularityDistanceVec);

if(singularityDistanceNorm >= SINGULARITY_NORM_MAX && velWSDir > 0)
    singularityLimX = 0;
elseif(singularityDistanceNorm >= SINGULARITY_NORM_L1 && singularityDistanceNorm < SINGULARITY_NORM_MAX)% && velWSDir > 0)
    singularityLimX = (1-1/(SINGULARITY_NORM_MAX-SINGULARITY_NORM_L1)*(singularityDistanceNorm-SINGULARITY_NORM_L1));
end


%% Packing the complete velocity

desDelQ = singularityLimX*desQDot*robotParameters.Ts;

% No motion if a joint has hit it's limit

allowMotion = 1;
%Moved this part to low level controller
isDesiredMotionJointSaturated = isJointSaturated.*sign(desDelQ);
for i = 1:length(currentJointPositions)
    currAllowMotion = (1-abs(isDesiredMotionJointSaturated(i)));
%     if(isDesiredMotionJointSaturated(i) > 0 && currAllowMotion < allowMotion)
    if(currAllowMotion < allowMotion)
        allowMotion = currAllowMotion;
    end
end
desiredJointPositions = currentJointPositions + desDelQ;


%% Debug data
% rmr: removed concatenation for only KUKA
ikDebugData.delQ = desDelQ;
ikDebugData.desQDotPosOriCtrl = desQDotPosOriCtrl*Ts;
ikDebugData.desQDotP2_ns = desQDotP2_ns;
ikDebugData.nsDelQ =  desQDotP2_ns*Ts;
ikDebugData.allowMotion = allowMotion;
ikDebugData.singularityLimX = singularityLimX;

end

