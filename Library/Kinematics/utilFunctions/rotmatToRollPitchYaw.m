%#codegen
function [ angles ] = rotmatToRollPitchYaw(R, previousAngles)

angles = previousAngles; %in order x,y,z

thX = [atan2(R(3,2), sqrt(R(1,2)^2 + R(2,2)^2)),  atan2(R(3,2), -sqrt(R(1,2)^2 + R(2,2)^2))];
thY = [0,0,];
thZ = [0,0];

del = [0,0];

sol = zeros(3,2);

minDelta = 0.001;
minInd = 1;
for i = 1:length(thX)
    costhX = cos(thX(i));
    if( abs(costhX) > minDelta)
        thY(i) = atan2(-R(3,1)/costhX, R(3,3)/costhX);
        thZ(i) = atan2(-R(1,2)/costhX, R(2,2)/costhX);
    else
        thyz = atan2(R(2,1), R(1,1));
        thZ(i) =  thyz/2; % thY+thZ actually
        thY(i) =  thyz/2;
    end
    sol(:,i) = [thX(i); thY(i); thZ(i)]; 
    del(i) = max(abs( previousAngles - sol(:,i) ));
    if(del(i) < del(minInd))
        minInd = i;
    end
end

angles = sol(:,minInd);
end

