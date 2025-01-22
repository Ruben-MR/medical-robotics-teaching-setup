%#codegen
function TEE = macrorobotTransformEE_miniendoscope()
  % Input: void
  % Output: homogeneous transformation Matrix from frame origin of frame E_E to the end of the end-effector frame E_E.
  
    TEE = [1    0    0    0
           0    1    0    0
           0    0    1    0.130 % Mini-endoscope length
           0    0    0    1];
       
end