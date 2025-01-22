%#codegen
function [delQAllowed, scale, maxXDotDesNorm] = findMaxPossibleVelocityAtThisPoseForThisDemand(jacobian, delQDes, X_DOT_MAX, Ts)
%FINDMAXPOSSIBLEVELOCITYATTHISPOSE Finds the maximum allowed joint step per tick
%without violating the task space speed limits in position for the
%end-effector tip

    
    delQAllowed = abs(delQDes); %Imp, in case the desired ts vel is zero
    xDotDesNorm = zeros(size(jacobian, 2),1);
    for i = 1:size(jacobian, 2) % Why 8?
        xDotDes = jacobian(:,:,i)*delQDes; % m/tick or rad/tick
        xDotDesNorm(i) = sqrt(xDotDes'*xDotDes)/Ts; % m/s or rad/s
    end
    
    scale = 1;
    maxXDotDesNorm = max(xDotDesNorm);
    if(maxXDotDesNorm > X_DOT_MAX)
        scale = maxXDotDesNorm/X_DOT_MAX;
    end
    
%     if(scale > 0) %Will not happen only if the desired demand is all zero. 
    delQAllowed = abs(delQDes)/scale;
%     end

end


 