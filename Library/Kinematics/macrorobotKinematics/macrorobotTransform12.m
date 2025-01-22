%#codegen
function T12 = macrorobotTransform12(Q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 2 to frame 1. T_12
  if (length(Q)>1)
  	q = Q(2);
  else
      q = Q;
  end
  
  T12 = eye(4);
  T12(1:3,1:3) = roty(q);
  T12(1:3,4) = [0; 0; 0.2025];
   
%   T12 = [ cos(q), 0,  sin(q),     0;
%                0, 1,      0,     0;
%          -sin(q), 0, cos(q), 0.2025;
%                0, 0,      0,     1];
end

