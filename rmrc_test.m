% Clear workspace and set up environment
clc;
clf;
clear;
close all;

% Load or create the UR3 robot model
myUR3 = UR3e();  % Replace this with the command to load your UR3 robot

% Create an instance of the RMRC class for the UR3 robot
path = RMRC(myUR3);  % Instantiate RMRC for the UR3 robot

% Initial joint configuration (all joints at zero)
q0 = zeros(1, myUR3.model.n);

% Get the current end-effector position using forward kinematics
startTr_struct = myUR3.model.fkine(q0);  % Get the forward kinematics
if isobject(startTr_struct)
    startTr = startTr_struct.T;  % Extract the .T property if it's an object
else
    startTr = startTr_struct;    % Directly use if it's already a matrix
end

% Define the target transformation matrix
endTr = transl(0.4, 0.4, 0.5);  % Example target transformation

% Set the total time and control frequency
t = 5;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr, endTr, t, deltaT);
