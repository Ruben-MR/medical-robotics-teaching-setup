%#codegen
function T56 = macrorobotTransform56(Q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 5 to frame 6. T_56
  if (length(Q)>1)
  	q = Q(6);
  else
      q = Q;
  end

  T56 = eye(4);
  T56(1:3,1:3) = roty(q);
  T56(1:3,4) = [0; 0; 0.2155];
       
%   T56 = [ cos(q), 0, sin(q),     0;
%                0, 1,      0,    -0.06;
%          -sin(q), 0, cos(q), 0.21625;
%                0, 0,      0,     1];
end
