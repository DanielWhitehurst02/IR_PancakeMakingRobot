% Initialize robots and define initial positions
robot1 = UR3e(transl(-0.5, 0.2, 0.5));
robot2 = Panda(transl(0.5, 0.5, 0));
leftFinger1 = Finger();
rightFinger1 = Finger2();
leftFinger2 = Finger();
rightFinger2 = Finger2();

[leftFinger1A, rightFinger1A] = attachGrippersToRobot(robot1,leftFinger1,rightFinger1);
[leftFinger2A, rightFinger2A] = attachGrippersToRobot(robot2,leftFinger2,rightFinger2,troty(-pi/2) * trotz(-pi),transl(0,0.15,0));
%%

centerPoints_U = [0.0, 0.0, 0.05; % Base 
                0.0, -0.01, 0.01; % Link 1 
                0.125, 0.0, 0.125; % Link 2 
                0.105, 0.0, 0.05; % Link 3 
                0.0, 0.0, 0.01; % Link 4 
                0.0, 0.0, 0.06; % Link 5 
                0.0, 0.0, 0.0;]; % end-effector

radii_U = [0.08, 0.09, 0.055;  
         0.075, 0.085, 0.075;
         0.175, 0.08, 0.085; 
         0.15, 0.06, 0.085; 
         0.04, 0.055, 0.065;
         0.04, 0.045, 0.125; 
         0.0, 0.0, 0.0;];

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
    


%% Defining collision points
center = [0, 0, 0.5];  % Center of the cube
rot = [pi/3, pi/4, 2*pi/7];  % Rotation in x, y, z

% Call meshcube which also calls RectangularPrism and returns all the values
[cubePoints, vertex, faces, faceNormals] = CollisionMesh(0.5, 0.5, rot, 0.02, center);


%% Define collision handlers for each robot
% Initialize collision handlers for each robot
collisionHandler1 = CollisionEllipsoidDynamic(robot1, centerPoints_U, radii_U);
collisionHandler1.setObstaclePoints(cubePoints);
collisionHandler2 = CollisionEllipsoidDynamic(robot2, centerPoints_panda, radii_Panda);
collisionHandler2.setObstaclePoints(cubePoints);
%% Define end transformations
endTr1 = transl(0, 0.4, 0.5) * troty(pi);
endTr2 = transl(0.3, 0.4, 0.7) * trotx(-pi/2);

% Get the initial transformation matrices for both robots
startTr1 = robot1.model.fkine(robot1.model.getpos()).T;
startTr2 = robot2.model.fkine(robot2.model.getpos()).T;
%%
% Run IK for both robots, passing grippers, transformations, and collision detection
runIKForTwoRobots(robot1, robot2, startTr1, endTr1, startTr2, endTr2, 100, leftFinger1A, rightFinger1A, leftFinger2A, rightFinger2A, collisionHandler1, collisionHandler2);

%% Function to Perform IK for Two Robots Simultaneously
function runIKForTwoRobots(robot1, robot2, startTr1, endTr1, startTr2, endTr2, steps, leftFinger1, rightFinger1, leftFinger2, rightFinger2, collisionHandler1, collisionHandler2)
    % Calculate initial and final joint configurations for robot1
    qStart1 = robot1.model.ikcon(startTr1);
    qEnd1 = robot1.model.ikcon(endTr1);
    qMatrix1 = jtraj(qStart1, qEnd1, steps);

    % Calculate initial and final joint configurations for robot2
    qStart2 = robot2.model.ikcon(startTr2);
    qEnd2 = robot2.model.ikcon(endTr2);
    qMatrix2 = jtraj(qStart2, qEnd2, steps);

    % Initialize running state for both robots
    running1 = true;
    running2 = true;

    % Main loop to animate both robots and update grippers step-by-step
    for i = 1:steps
        % Check for collisions for robot1
        running1 = checkForCollisionAndPause(collisionHandler1, running1);
        
        % Animate robot1 if running
        if running1
            q_current1 = qMatrix1(i, :);
            robot1.model.animate(q_current1);
            updateGripperPositionWithEndEffector(robot1, leftFinger1, rightFinger1);
        end
        
        % Check for collisions for robot2
        running2 = checkForCollisionAndPause(collisionHandler2, running2);
        
        % Animate robot2 if running
        if running2
            q_current2 = qMatrix2(i, :);
            robot2.model.animate(q_current2);
            updateGripperPositionWithEndEffector(robot2, leftFinger2, rightFinger2, troty(-pi/2) * trotz(-pi), transl(0, 0.15, 0));
        end
        
        drawnow;  % Force figure update for real-time animation
        pause(0.01);  % Control animation speed
    end
end

%% Standalone Function to Attach Grippers to Robot End-Effector
function [leftFinger, rightFinger] = attachGrippersToRobot(robot, leftFinger, rightFinger, gripperRotation, gripperTranslation)
    % Set default rotation and translation if not provided
    if nargin < 4 || isempty(gripperRotation)
        gripperRotation = trotx(pi/2);  % Default rotation
    end
    if nargin < 5 || isempty(gripperTranslation)
        gripperTranslation = transl(0, 0, 0);  % Default translation (no offset)
    end

    disp('Attaching grippers to the robot...');
    
    % Get the current end-effector transformation
    endEffectorTr = robot.model.fkine(robot.model.getpos);

    % Apply rotation and translation to attach the grippers
    leftFinger.model.base = endEffectorTr.T * gripperRotation * gripperTranslation;
    rightFinger.model.base = endEffectorTr.T * gripperRotation * gripperTranslation;

    % Plot the grippers
    leftFinger.PlotAndColourRobot();
    rightFinger.PlotAndColourRobot();

    disp('Grippers successfully attached.');
end

%% Standalone Function to Update Gripper Position with Robot's End Effector
function updateGripperPositionWithEndEffector(robot, leftFinger, rightFinger, gripperRotation, gripperTranslation)
    % Set default rotation and translation if not provided
    if nargin < 4 || isempty(gripperRotation)
        gripperRotation = trotx(pi/2);  % Default rotation
    end
    if nargin < 5 || isempty(gripperTranslation)
        gripperTranslation = transl(0, 0, 0);  % Default translation (no offset)
    end

    % Get the current end-effector transformation
    endEffectorTr = robot.model.fkine(robot.model.getpos);

    % Update the grippers' positions based on end-effector transformation
    leftFinger.model.base = endEffectorTr.T * gripperRotation * gripperTranslation;
    rightFinger.model.base = endEffectorTr.T * gripperRotation * gripperTranslation;

    % Animate the grippers to reflect updated positions
    leftFinger.model.animate(leftFinger.model.getpos);
    rightFinger.model.animate(rightFinger.model.getpos);
end


%% Collision Check Function
% Function to check for collisions and stop or resume motion based on detection
function running = checkForCollisionAndPause(collisionHandler, running)
    % collisionHandler.drawEllipsoids();  % Draw and update ellipsoid positions
    
    % Detect collision
    collision = collisionHandler.detectCollision();
    if collision
        disp('Collision detected! Pausing robot motion...');
        running = false;
        
        % Pause loop until collision is resolved
        while collision
            pause(0.1);  % Allow time for system to process changes
            % collisionHandler.drawEllipsoids();  % Continuously update positions
            collision = collisionHandler.detectCollision();
            if ~collision
                disp('Collision resolved. Resuming robot motion...');
                running = true;
            end
        end
    end
end