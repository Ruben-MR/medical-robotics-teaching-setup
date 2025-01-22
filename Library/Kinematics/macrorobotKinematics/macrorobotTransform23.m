%#codegen
function T23 = macrorobotTransform23(Q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 2 to frame 3. T_23
  if (length(Q)>1)
  	q = Q(3);
  else
      q = Q;
  end

  T23 = eye(4);
  T23(1:3,1:3) = rotz(q);
  T23(1:3,4) = [0; 0; 0.2045];
  
%   T23 = [cos(q), -sin(q), 0,     0;
%          sin(q),  cos(q), 0,     0;
%               0,       0, 1, 0.20375;
%               0,       0, 0,     1];

end
