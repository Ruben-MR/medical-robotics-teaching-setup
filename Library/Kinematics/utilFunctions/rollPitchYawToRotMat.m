%#codegen
function [ R ] = rollPitchYawToRotMat(angles)
%Using yaw-pitch-roll about z, x, y in that order. Angles are always sent
%in x, y, z order. 

x = angles(1);
y = angles(2);
z = angles(3);


R = rotz(z)*rotx(x)*roty(y); 


