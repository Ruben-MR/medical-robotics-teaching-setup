%#codegen
function [saturatedPositions, isJointSaturated] = fun_SaturateMacroRobotPosition(inputPositions, robotUpperLimits, robotLowerLimits, robotLowerLimitsSlowDown, robotUpperLimitsSlowDown)

saturatedPositions = inputPositions;
isJointSaturated = zeros(length(inputPositions),1);

for i = 1:length(inputPositions)
    if(inputPositions(i) <= robotLowerLimits(i) )
        saturatedPositions(i) = robotLowerLimits(i);
        isJointSaturated(i) = -1;
    elseif(inputPositions(i) <= robotLowerLimitsSlowDown(i))
        isJointSaturated(i) = -1/(robotLowerLimits(i)-robotLowerLimitsSlowDown(i))*(inputPositions(i)-robotLowerLimitsSlowDown(i));
    end
    
    if(inputPositions(i) >= robotUpperLimits(i) )
        saturatedPositions(i) = robotUpperLimits(i);
        isJointSaturated(i) = 1;
    elseif(inputPositions(i) >= robotUpperLimitsSlowDown(i))
        isJointSaturated(i) = 1/(robotUpperLimits(i)-robotUpperLimitsSlowDown(i))*(inputPositions(i)-robotUpperLimitsSlowDown(i));
    end
end 
