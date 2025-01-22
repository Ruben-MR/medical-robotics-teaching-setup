%#codegen
function [endoscopeTipTransform, endoscopeTipJacobian, J_P_All, singularityDistanceVec] = getMacroRobotForwardKinematicsData(q)
% Input: vector of generalized coordinates (joint angles)
% Output: Jacobian of the end-effector translation which maps joint
% velocities to end-effector linear velocities in I frame.

% Compute the relative homogeneous transformation matrices.
T_I0 = macrorobotTransformI0();
T_01 = macrorobotTransform01(q(1));
T_12 = macrorobotTransform12(q(2));
T_23 = macrorobotTransform23(q(3));
T_34 = macrorobotTransform34(q(4));
T_45 = macrorobotTransform45(q(5));
T_56 = macrorobotTransform56(q(6));
T_67 = macrorobotTransform67(q(7));
T_7E = macrorobotTransform7E();
T_EE = macrorobotTransformEE_miniendoscope();

endoscopeTipTransform = T_I0*T_01*T_12*T_23*T_34*T_45*T_56*T_67*T_7E*T_EE;

T_I_All = zeros(4,4,length(q)+1);
T_I_All(4,4,:) = 1;
T_I_All(:,:,1) = T_I0*T_01;
T_I_All(:,:,2) = T_I_All(:,:,1)*T_12;
T_I_All(:,:,3) = T_I_All(:,:,2)*T_23;
T_I_All(:,:,4) = T_I_All(:,:,3)*T_34;
T_I_All(:,:,5) = T_I_All(:,:,4)*T_45;
T_I_All(:,:,6) = T_I_All(:,:,5)*T_56;
T_I_All(:,:,7) = T_I_All(:,:,6)*T_67;
T_I_All(:,:,8) = T_I_All(:,:,7)*T_7E*T_EE;

singularityDistanceVec = T_I_All(1:3, 4, 6) - T_I_All(1:3, 4, 2);


% Compute the end-effector position vector.
r_I_IE = endoscopeTipTransform(1:3,4);


% Compute the homogeneous transformation matrices from frame k to the
% inertial frame I.
T_I1 = T_I0*T_01;
T_I2 = T_I1*T_12;
T_I3 = T_I2*T_23;
T_I4 = T_I3*T_34;
T_I5 = T_I4*T_45;
T_I6 = T_I5*T_56;
T_I7 = T_I6*T_67;

% Extract the rotation matrices from each homogeneous transformation
% matrix.
R_I1 = T_I1(1:3,1:3);
R_I2 = T_I2(1:3,1:3);
R_I3 = T_I3(1:3,1:3);
R_I4 = T_I4(1:3,1:3);
R_I5 = T_I5(1:3,1:3);
R_I6 = T_I6(1:3,1:3);
R_I7 = T_I7(1:3,1:3);

% Extract the position vectors from each homogeneous transformation
% matrix.
r_I_I1 = T_I1(1:3,4);
r_I_I2 = T_I2(1:3,4);
r_I_I3 = T_I3(1:3,4);
r_I_I4 = T_I4(1:3,4);
r_I_I5 = T_I5(1:3,4);
r_I_I6 = T_I6(1:3,4);
r_I_I7 = T_I7(1:3,4);

% Define the unit vectors around which each link rotates in the precedent
% coordinate frame.
n_1 = [0 0 1]';
n_2 = [0 1 0]';
n_3 = [0 0 1]';
n_4 = [0 -1 0]';
n_5 = [0 0 1]';
n_6 = [0 1 0]';
n_7 = [0 0 1]';

n_All = zeros(3,length(q));
n_All(:,1) = n_1;
n_All(:,2) = n_2;
n_All(:,3) = n_3;
n_All(:,4) = n_4;
n_All(:,5) = n_5;
n_All(:,6) = n_6;
n_All(:,7) = n_7;



% Compute the translational jacobian.
J_P = [ cross(R_I1*n_1, r_I_IE - r_I_I1) ...
        cross(R_I2*n_2, r_I_IE - r_I_I2) ...
        cross(R_I3*n_3, r_I_IE - r_I_I3) ...
        cross(R_I4*n_4, r_I_IE - r_I_I4) ...
        cross(R_I5*n_5, r_I_IE - r_I_I5) ...
        cross(R_I6*n_6, r_I_IE - r_I_I6) ...
        cross(R_I7*n_7, r_I_IE - r_I_I7) ...
        ];

J_R = [ R_I1*n_1 ...
        R_I2*n_2 ...
        R_I3*n_3 ...
        R_I4*n_4 ...
        R_I5*n_5 ...
        R_I6*n_6 ...
        R_I7*n_7 ...
        ];

endoscopeTipJacobian = zeros(6,length(q));
endoscopeTipJacobian(1:3,:) = J_P;
endoscopeTipJacobian(4:6,:) = J_R;

%%
J_P_All = zeros(3,length(q),length(q));

for i = 1:length(q)
    for j = 1:i
        J_P_All(:,j,i) = cross(T_I_All(1:3,1:3,j)*n_All(:,j), T_I_All(1:3,4,i+1) - T_I_All(1:3,4,j));
    end
end

end