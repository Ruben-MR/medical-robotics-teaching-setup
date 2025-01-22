%% Script to define multiple parameters of the robot arm and control of the simulation
Ts = 0.000250;
TsControl = 0.001;
nJoints = 8;
nJointsKuka = 7;

%% Positive Axis definitions
directionAxis = [0 -1 -1 -1 -1 -1 1 1];

%% Velocity
vmax_Endoscope = 0.1; % Maximal speed [rad/sec]
vmax_kuka = 0.01; % Maximal speed [rad/s] for each joint of Kuka allowed. Set to the same value as the safety limit on the sunrise cabinet side.

%%
% 2018-12-12ng + mk  KUKA joint limits
limitPhi = (pi/180)*[[-170 170];
                     [-120 120];
                     [-170 170];
                     [-120 120];
                     [-170 170];
                     [-120 120];
                     [-175 175];]; % rad
limitDefault = [0 0];% rad 

lowerLimit_Phi = limitPhi(:,1);
upperLimit_Phi = limitPhi(:,2);

%All limits in one variable for Kuka
safePositionLimit = + 5*pi/180; % adding a buffer of 1 deg to make sure the kuka does not go into the limits
safePositionLimitSlowDown = + 2.5*pi/180; % adding a buffer of 1 deg to make sure the kuka does not go into the limits

kukaLowerLimits = lowerLimit_Phi + safePositionLimit;
kukaUpperLimits = upperLimit_Phi- safePositionLimit;

kukaLowerLimitsSlow = kukaLowerLimits + safePositionLimitSlowDown;
kukaUpperLimitsSlow = kukaUpperLimits - safePositionLimitSlowDown;


robotLowerLimits = kukaLowerLimits;
robotUpperLimits = kukaUpperLimits;

robotLowerLimitsSlowDown = kukaLowerLimitsSlow;
robotUpperLimitsSlowDown = kukaUpperLimitsSlow;

%% Intialize structures for bus signals
robotLinkParameters = 1;
robotStateStruct = Simulink.Bus.createMATLABStruct('RobotState');
robotParameters = Simulink.Bus.createMATLABStruct('RobotParams');
robotParameters.X_DOT_MAX = 0.8*0.25; %m/s; Must be changed if changed in KUKA side
robotParameters.Q_DOT_MAX = 0.8*[5; 5; 5; 5; 5; 5; 5]; %rad/s; Must be changed if changed in KUKA side
robotParameters.jointVelocitySafetyFactor = 1.0;
robotParameters.Ts = 0.001; %at what rate is the FRI updated
robotParameters.Tsensor = 0.001; % sensory update rate

ikDebugDataStruct = Simulink.Bus.createMATLABStruct('IK_DebugData');