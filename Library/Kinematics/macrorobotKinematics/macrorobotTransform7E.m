%#codegen
function T7E = macrorobotTransform7E()
  % Input: void
  % Output: homogeneous transformation Matrix from frame 6 to the end-effector frame E. T_6E
  
    T7E = eye(4);
    T7E(3,4) = 0.0451;
    
    
end

