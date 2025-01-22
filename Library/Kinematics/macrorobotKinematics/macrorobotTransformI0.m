%#codegen
function TI0 = macrorobotTransformI0()
  % Input: void
  % Output: homogeneous transformation Matrix from the inertial frame I to frame 0. T_I0
  
  TI0 = eye(4,4);

% %Reverse Robot's TransformI0
%     TI0 = [1 0 0 0
%            0 -1 0 0
%            0 0 -1 1.9
%            0 0 0 1];

end
