%#codegen
function [ phi ] = rotMatToRotVec(C)
% Input: a rotation matrix C
% Output: the rotational vector which describes the rotation C

% Compute the rotional vector
phi = zeros(3,1);

cosTheta = (trace(C) - 1)/2.0;
sinTheta = 0;

if(abs(cosTheta) <= 1)
    sinTheta = sqrt(abs(1 - cosTheta^2));
end

theta = atan2(sinTheta,cosTheta);

if(mod(abs(theta),pi) > 0.001)
    
    phi = theta/(2*sinTheta)*[C(3,2) - C(2,3);...
                              C(1,3) - C(3,1);...
                              C(2,1) - C(1,2)];
end


end
