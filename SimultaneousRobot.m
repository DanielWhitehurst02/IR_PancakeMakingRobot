function SimultaneousRobot(app)

% Collision Mesh
center = [-0.5, 0, 0.5];
rot = [pi/3, pi/4, 2*pi/7];
[cubePoints, vertex, face, faceNormals, CollisionObjectV] = CollisionMesh(0.5, 0.5, rot, 0.02, center);
hold on
axis([-3 3 -3 3 0 3])
%% Initialize robots and their respective MotionHandler instances
robot1 = UR3();
leftFinger1 = Finger();  % Initialize left finger
rightFinger1 = Finger2();  % Initialize right finger

centerPoints1 = [0.0, 0.0, 0.05; % Base 
                    0.0, -0.01, 0.01; % Link 1 
                    0.125, 0.0, 0.125; % Link 2 
                    0.105, 0.0, 0.05; % Link 3 
                    0.0, 0.0, 0.01; % Link 4 
                    0.0, 0.0, 0.06; % Link 5 
                    0.0, 0.0, 0.0;]; % end-effector
    
radii1 = [0.08, 0.09, 0.055;  
         0.075, 0.085, 0.075;
         0.175, 0.08, 0.085; 
         0.15, 0.06, 0.085; 
         0.04, 0.055, 0.065;
         0.04, 0.045, 0.125; 
         0.0, 0.0, 0.0;]; 

robot2 = Panda();
leftFinger2 = Finger();  % Initialize left finger
rightFinger2 = Finger2();  % Initialize right finger

centerPoints2 = [-0.025, 0.0, 0.06; % Base
                0.0, 0.0975, -0.035; % Link 1 
                0.0, 0.03, 0.075; % Link 2 
                -0.075, -0.085, 0.0; % Link 3 
                0.05, 0.0, 0.035; % Link 4 
                0.0, 0.125, 0.025; % Link 5 
                -0.05,  0.025, 0.0;]; % Link 6
            
radii2 = [0.135, 0.125, 0.075; 
         0.1, 0.175, 0.1; 
         0.125, 0.12, 0.125; 
         0.075, 0.075, 0.095; 
         0.125, 0.125, 0.095; 
         0.085, 0.15, 0.105; 
         0.110, 0.1, 0.085;]; 

%% Some funtcions that would allow us to control the robot with the app sliders

%% Calling Motion handlers
motionHandler1 = MotionHandlerWithGripperAndObjects(robot1, centerPoints1, radii1, cubePoints, leftFinger1, rightFinger1,app);
motionHandler2 = MotionHandlerWithGripperAndObjects(robot2, centerPoints2, radii2, cubePoints, leftFinger2, rightFinger2,app);

% Define start and end transformations
startTr1 = robot1.model.fkine(robot1.model.getpos());
endTr1 = transl(0.2, 0.2, 0.3);
startTr2 = robot2.model.fkine(robot2.model.getpos());
endTr2 = transl(0.3, -0.2, 0.4);

% Run animations in parallel
f1 = parfeval(@() motionHandler1.runIK(startTr1, endTr1, 50), 0);
f2 = parfeval(@() motionHandler2.runIK(startTr2, endTr2, 50), 0);

end