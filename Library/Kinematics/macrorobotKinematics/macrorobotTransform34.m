%#codegen
function T34 = macrorobotTransform34(Q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 3 to frame 4. T_34
  if (length(Q)>1)
  	q = Q(4);
  else
      q = Q;
  end

  T34 = eye(4);
  T34(1:3,1:3) = roty(-q);
  T34(1:3,4) = [0; 0; 0.2155];

  
% T34 = [ cos(q), 0,  -sin(q),     0;
%          0, 1,      0,     0;
%         sin(q), 0, cos(q), 0.21625;
%          0, 0,      0,     1];   

end

