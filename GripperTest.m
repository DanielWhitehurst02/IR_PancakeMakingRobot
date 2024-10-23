robot = Panda();
leftFinger = Finger();  % Initialize left finger
rightFinger = Finger2();  % Initialize right finger

% Panda
centerPoints = [-0.025, 0.0, 0.06; % Base
                0.0, 0.0975, -0.035; % Link 1 
                0.0, 0.03, 0.075; % Link 2 
                -0.075, -0.085, 0.0; % Link 3 
                0.05, 0.0, 0.035; % Link 4 
                0.0, 0.125, 0.025; % Link 5 
                -0.05,  0.025, 0.0;]; % Link 6
            
radii = [0.135, 0.125, 0.075; 
         0.1, 0.175, 0.1; 
         0.125, 0.12, 0.125; 
         0.075, 0.075, 0.095; 
         0.125, 0.125, 0.095; 
         0.085, 0.15, 0.105; 
         0.110, 0.1, 0.085;]; 

%% Collision Mesh
center = [-0.5, 0, 0.5];  % Center of the cube
rot = [pi/3, pi/4, 2*pi/7];  % Rotation in x, y, z

% Call meshcube which also calls RectangularPrism and returns all the values
[cubePoints, vertex, faces, faceNormals] = CollisionMesh(0.5, 0.5, rot, 0.02, center);

%%
motionHandler = MotionHandlerWIthGripperAndObjects(robot, centerPoints, radii, cubePoints, leftFinger, rightFinger,troty(-pi/2) * trotz(-pi),transl(0,0.15,0));
%%
robot.model.teach();

%GripperAttachedPositionsValues = [troty(-pi/2) * trotz(-pi),transl(0,0.15,0)]