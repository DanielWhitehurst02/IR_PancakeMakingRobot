robot = UR3e(transl(0, 0, 0));
leftFinger = Finger();  % Initialize left finger
rightFinger = Finger2();  % Initialize right finger

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


hold on
%%
center = [0.5, 0, 0.5];  % Center of the cube
rot = [pi/3, pi/4, 2*pi/7];  % Rotation in x, y, z

% Call meshcube which also calls RectangularPrism and returns all the values
[cubePoints, vertex, faces, faceNormals] = CollisionMesh(0.5, 0.5, rot, 0.02, center);


%% Create Motion Handler
controller = MotionHandlerWithGripper(robot,centerPoints,radii,cubePoints,leftFinger,rightFinger);

%%
% Set the total time and control frequency
t = 5;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds
q0 = zeros(1, robot.model.n);

% Get the current end-effector position using forward kinematics
startTr_struct = robot.model.fkine(q0);  % Get the forward kinematics
if isobject(startTr_struct)
    startTr = startTr_struct.T;  % Extract the .T property if it's an object
else
    startTr = startTr_struct;    % Directly use if it's already a matrix
end

startTr = transl(0, 0.3, 0.5);
endTr = transl(0.2, -0.3, 0.5);

controller.runIK(startTr, endTr, 50)


% Define the ground as a large rectangle at Z = -0.1
groundVertex = [2 -2 -0.1; 2 2 -0.1; -2 2 -0.1; -2 -2 -0.1];
groundFaces = [1 2 3; 1 3 4];
groundFaceNormals = repmat([0, 0, 1], 2, 1); % Normal pointing upward (Z-axis)

% Combine ground vertices and normals with obstacle vertices and normals
vertex = [vertex; groundVertex];
faces = [faces; groundFaces + size(vertex,1) - 4];  % Adjust face indices for the ground vertices
faceNormals = [faceNormals; groundFaceNormals];
%%
input('Want to simulate collision avoidance');
% Call the collision avoidance function
startTr = transl(0, 0.3, 0.5);
endTr = transl(0.2, -0.3, 0.5);
q_start = robot.model.fkine(robot.model.getpos());
q_end = robot.model.ikine(endTr);
centerpnt = [0 0 0];
runCollisionAvoidance(centerpnt, robot, q_start, q_end, vertex, faces, faceNormals, 100);
