%#codegen
function T67 = macrorobotTransform67(Q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 5 to frame 6. T_56
  if (length(Q)>1)
  	q = Q(7);
  else
      q = Q;
  end
  
  T67 = eye(4);
  T67(1:3,1:3) = rotz(q);
  T67(1:3,4) = [0; 0; 0.0809];
  
%   T67 = [cos(q), -sin(q), 0,    0;
%          sin(q),  cos(q), 0,   0.06;
%               0,       0, 1,   0.08;
%               0,       0, 0,     1];
%      
end
