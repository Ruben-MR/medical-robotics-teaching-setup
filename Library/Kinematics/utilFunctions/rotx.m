%#codegen
function [ R ] = rotx( theta )
%ROTZ Gives rotation matrix about x by theta
R = eye(3);
c = cos(theta);
s = sin(theta);

R = [1 0 0;...
     0 c -s;...
     0 s c];



end

