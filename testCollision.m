clc;
clf;
clear;
close all;


% set(0, 'DefaultFigureWindowStyle', 'docked'); % Dock figures in the workspace


%% Define the robot and its ellipsoid centers and radii
robot = UR3e(transl(0, 0, 0)); % Initialize the UR3 robot


centerPoints = [0.0, 0.0, 0.05; % Base 
                0.0, -0.01, 0.01; % Link 1 
                0.125, 0.0, 0.125; % Link 2 
                0.105, 0.0, 0.05; % Link 3 
                0.0, 0.0, 0.01; % Link 4 
                0.0, 0.0, 0.06; % Link 5 
                0.0, 0.0, 0.0;]; % end-effector

radii = [0.08, 0.09, 0.055;  
         0.075, 0.085, 0.075;
         0.175, 0.08, 0.085; 
         0.15, 0.06, 0.085; 
         0.04, 0.055, 0.065;
         0.04, 0.045, 0.125; 
         0.0, 0.0, 0.0;]; 

% robot.model.tool

%% Panda
robot2 = Panda(transl(-0.5, 0.5, 0));

centerPoints_panda = [
        -0.025, 0.0, 0.06;
        0.0, 0.0975, -0.035;
        0.0, 0.03, 0.075;
        -0.075, -0.085, 0.0;
        0.05, 0.0, 0.035;
        0.0, 0.125, 0.025;
        -0.05, 0.025, 0.0;
        ];
radii_Panda = [
        0.135, 0.125, 0.075;
        0.1, 0.175, 0.1;
        0.125, 0.12, 0.125;
        0.075, 0.075, 0.095;
        0.125, 0.125, 0.095;
        0.085, 0.15, 0.105;
        0.110, 0.1, 0.085;
    ];
% 
% %% Create an instance of the CollisionEllipsoid class
collisionHandler = CollisionEllipsoid(robot, centerPoints, radii);

% Draw ellipsoids
collisionHandler.drawEllipsoids();

% % Create a mesh (obstacle) and plot it

workspace = [-2, 2.5, -2, 2.5, 0, 2];
axis(workspace)
center = [1,0,0.5];
rotation = [pi/3,pi/4,2*pi/7];

cubePoints = meshcube(0.8,0.2,rotation,0.02,center);

%% Create Motion Handler
controller = MotionHandler(robot,centerPoints,radii,cubePoints);

controller2 = MotionHandler(robot2,centerPoints_panda,radii_Panda,cubePoints);

%% Animate robot with RMRC
path = RMRC(robot);
q0 = zeros(1, robot.model.n);

% Get the current end-effector position using forward kinematics
startTr_struct = robot.model.fkine(q0);  % Get the forward kinematics
if isobject(startTr_struct)
    startTr = startTr_struct.T;  % Extract the .T property if it's an object
else
    startTr = startTr_struct;    % Directly use if it's already a matrix
end

% Define the target transformation matrix
endTr = transl(0.3, 0.3, 0.5);  % Example target transformation

% Set the total time and control frequency
t = 5;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds
 
% path.ResolvedMotionRateControl(startTr,endTr,t,deltaT);

[s,x,steps] = path.ResolvedMotionRateControlPath(startTr,endTr,t,deltaT);
% 
% 
% isCollision = pathCheck(x,cubePoints);

plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)

% collision = controller.pathCheck(x , cubePoints)

% controller.obstaclePoints(cubePoints);

controller.runRMRC(startTr, endTr, t, deltaT)
controller2.runRMRC(startTr, endTr, t, deltaT)

% controller.runIK(startTr, endTr, t, deltaT)


%% Check for collisions between the robot's ellipsoids and the obstacle
% isColliding = collisionHandler.detectCollision(cubePoints);
% if isColliding
%     disp('Collision detected with the obstacle!');
% else
%     disp('No collision detected.');
% end

% 
% function isCollision = pathCheck(path, obstaclePoints)
%     isCollision = false;
%     for i=1:length(path)
%         for j=1:size(obstaclePoints)
%             dist = sqrt((obstaclePoints(j,1)-path(1,i))^2+(obstaclePoints(j,2)-path(2,i))^2+(obstaclePoints(j,3)-path(3,i))^2);
%             if any(dist < 0.1)
%                 isCollision = true;
%                 break;
%             end
%         end
%     end
% end
