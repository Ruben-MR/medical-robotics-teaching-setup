%#codegen
function TL1 = macrorobotTransform01(Q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 1 to frame 0. T_01
  if (length(Q)>1)
      q = Q(1);
  else
      q = Q;
  end
  
  TL1 = eye(4);
  TL1(1:3,1:3) = rotz(q);
  TL1(1:3,4) = [0;0;0.1575];
    
end