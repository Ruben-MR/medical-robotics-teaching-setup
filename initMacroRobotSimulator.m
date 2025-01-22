%% Main Init file for the macro robot. It should run all files necesary for obtaining parameters and defining buses.
clear all;
close all;
clc;

% Add path for sub-functions
addpath(genpath('Library'))

% Initialize bus signals for the model and the parameters for the robot
initRobotBusSignals();
initRobotParameters;

% Define the initial conditions for the simulation
initState = [0.0; pi/6; 0.0; -pi/2; 0; +pi/4; 0];

initCartStateMat = getMacroRobotForwardKinematicsData(initState);

initCartState = [initCartStateMat(1:3,4); rotmatToRollPitchYaw(initCartStateMat(1:3,1:3), [0;0;0])];

%completeRobotAssembly_DataFile;
KukaIIWA14_DataFile;