%#codegen
function [cmdJointPositions]  = ...
    fun_lowLevelController(desiredPositions, actualPositions, lastCommandedPosition, jointPositionsForControl, posJacPerJoint,  enableDownscaleMotors, resetPositionTick, largeJointStep, highLevelControlSelection, isJointSaturated, minJointStep, kRATE, kEPS, robotParameters)

%Default output: command the last commanded positions. 
cmdJointPositions = lastCommandedPosition;

%Calculating the desired change in jointPositions
desiredDelQ = (desiredPositions - jointPositionsForControl);
% desiredDelQWithJSat = desiredDelQ;

jointSatScaleDownMaxWithTS = 0;
for i = 1:length(desiredPositions)
%     if(isJointSaturated(i)*desiredDelQ(i) > 0)
        if(abs(isJointSaturated(i)) > jointSatScaleDownMaxWithTS)
            jointSatScaleDownMaxWithTS = abs(isJointSaturated(i));
        end
%         desiredDelQWithJSat(i) = (1-abs(isJointSaturated(i)))*desiredDelQ(i);
%     end
end
%jointSatScaleDownMax = max(jointSatScaleDown);
desiredDelQWithJSat = (1.0 - jointSatScaleDownMaxWithTS)*desiredDelQ;

desiredDelQAbs = abs(desiredDelQWithJSat);
% maxDelQAbs = max(desiredDelQAbs);

%Variable to store the state of the controller
state = zeros(length(desiredPositions),1);


% Calculating the maximum permissible jointStep such that the velocity
% limit at this pose in task space is not violated only of the end-effector
% tip for linear velocity
[jointStepAllowedAtCurrentPose, monitorScale, maxXDotDesNorm] = findMaxPossibleVelocityAtThisPoseForThisDemand(posJacPerJoint, desiredDelQWithJSat, robotParameters.X_DOT_MAX, robotParameters.Ts);

maxJointStepAllowed = enableDownscaleMotors*jointStepAllowedAtCurrentPose./robotParameters.jointVelocitySafetyFactor;

inverseScalingFactors = ones(length(desiredPositions),1);
inverseScalingFactorsCart = ones(length(desiredPositions),1);
inverseScalingFactorsJoint = ones(length(desiredPositions),1);

for i = 1:length(desiredPositions)
    if(desiredDelQAbs(i) > maxJointStepAllowed(i) && maxJointStepAllowed(i)>0) %Checking task space limit
%         inverseScalingFactors(i) = desiredDelQAbs(i)/maxJointStepAllowed(i);
        inverseScalingFactorsCart(i) = desiredDelQAbs(i)/maxJointStepAllowed(i);
    end
    
    if(desiredDelQAbs(i) > robotParameters.Q_DOT_MAX(i)*robotParameters.Ts)    %Cheking joint space limit
%         inverseScalingFactors(i) = desiredDelQAbs(i)/robotParameters.Q_DOT_MAX(i);
        inverseScalingFactorsJoint(i) = desiredDelQAbs(i)/(robotParameters.Q_DOT_MAX(i)*robotParameters.Ts);
    end
end
    
%Finding the limiting joint and the max reduction scale
inverseScalingFactors = max(inverseScalingFactorsCart, inverseScalingFactorsJoint);
% [inverseScalingFactorForAllJoints, limitingJoint] = max(inverseScalingFactors);
[inverseScalingFactorForAllJoints, limitingJoint] = max(inverseScalingFactorsCart);

%Inverting the scaling factor
scalingFactorForAllJoints = 1;
if(inverseScalingFactorForAllJoints > 1) % Changed from 0.1
    scalingFactorForAllJoints = 1.0/inverseScalingFactorForAllJoints;
end

delQConstrained = desiredDelQWithJSat*scalingFactorForAllJoints;

%If a non-smooth input is taken, gains for the kuka joints. 
kEPS_ALL = ones(7,1)*kEPS(2); %delQ below which a slower gain is used
kRATE_ALL = ones(7,1)*kRATE(2); %the slower gain
minJointStep_ALL = ones(7,1)*minJointStep(2);

isSmoothInput = robotParameters.isSmoothHighLevelControllers(highLevelControlSelection);

if(resetPositionTick)
    cmdJointPositions = actualPositions;
    state = ones(length(desiredPositions),1);
elseif(enableDownscaleMotors >0)  
    for i = 1:length(desiredPositions)
        %If coming from a TS controller
        %Checking if it is close to the desired position
        
        % if(isSmoothInput)
            if(desiredDelQAbs(i) <= minJointStep_ALL(i)) % Too small a step, then skip
                state(i) = 2;
            elseif(desiredDelQAbs(i) <= maxJointStepAllowed(i))
                cmdJointPositions(i) = desiredPositions(i);
                state(i) = 3;
            else
                cmdJointPositions(i) = jointPositionsForControl(i) + delQConstrained(i);%externalMaximumStep;
                state(i) = 4;
            end
            
        % Assume the last joint of the system to perform the large step
        if(largeJointStep && i == length(desiredPositions))
            cmdJointPositions(i) = actualPositions(i) +  0.01;%0.6*pi/180;
            state(i) = 9;
        end
            
    end
end
    
delQcmd = cmdJointPositions - jointPositionsForControl;
% conditionVal = maxJointStepAllowed;
% monitorScale = scalingFactorForAllJoints;
end