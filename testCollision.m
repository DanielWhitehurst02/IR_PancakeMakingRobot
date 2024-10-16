clc;
clf;
clear;
close all;

set(0, 'DefaultFigureWindowStyle', 'docked'); % Dock figures in the workspace

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

%% Create an instance of the CollisionEllipsoid class
collisionHandler = CollisionEllipsoid(robot, centerPoints, radii);

% Draw ellipsoids
collisionHandler.drawEllipsoids();

% % Create a mesh (obstacle) and plot it

% One side of the cube
[Y,Z] = meshgrid(-0.25:0.02:0.25,-0.25:0.02:0.25);
sizeMat = size(Y);
X = repmat(0.25,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z);

% Combine one surface as a point cloud
cubePoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];         

% Plot the cube's point cloud         
% cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
cubePoints = cubePoints + repmat([0.3,0,0.3],size(cubePoints,1),1);
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
axis equal



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
endTr = transl(0.4, 0.4, 0.5);  % Example target transformation

% Set the total time and control frequency
t = 5;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds
path.ResolvedMotionRateControl(startTr,endTr,t,deltaT);

%% Check for collisions between the robot's ellipsoids and the obstacle
isColliding = collisionHandler.detectCollision(cubePoints);
if isColliding
    disp('Collision detected with the obstacle!');
else
    disp('No collision detected.');
end


