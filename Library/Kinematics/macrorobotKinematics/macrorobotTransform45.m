%#codegen
function T45 = macrorobotTransform45(Q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 5 to frame 4. T_45
  if (length(Q)>1)
  	q = Q(5);
  else
      q = Q;
  end
  

  T45 = eye(4);
  T45(1:3,1:3) = rotz(q);
  T45(1:3,4) = [0; 0; 0.1845];
  
%     T45 = [cos(q), -sin(q), 0,     0;
%            sin(q),  cos(q), 0,     0;
%               0,       0, 1, 0.18375;
%               0,       0, 0,     1];

end

