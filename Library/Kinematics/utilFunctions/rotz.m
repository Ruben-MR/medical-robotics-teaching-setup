%#codegen
function [ R ] = rotz( theta )
%ROTZ Gives rotation matrix about x by theta
R = eye(3);
c = cos(theta);
s = sin(theta);

R = [c -s 0;...
     s c 0;...
     0 0 1];



end

