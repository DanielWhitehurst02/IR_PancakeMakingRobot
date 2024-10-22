% Load UR3 robot model
robot = UR3();

hold on; % Ensure we retain all models in the scene

% Initialize figure with robot at home position
%robot.model.plot(zeros(1,6));
axis([-1 1 -1 1 -0.5 1]);

%% Attach grippers to the UR3 robot's end-effector
[leftFinger, rightFinger] = AttachGrippers(robot);


% Time vector for smooth animation
t = 0:0.05:2;  % Adjust the timing to control the speed of motion

% Define a function to move the robot and animate the grippers

% Define the desired end-effector orientation (Z-axis pointing up, aligned with the XY plane)
desired_orientation = troty(-pi/2);  % This makes sure the end-effector is facing upward

spatula_T1 = robot.model.fkine([0 0 0 0 0 0]);
spatula_end_T1 = robot.model.fkine([-1.38230076757951	-0.628318530717959	0.753982236861551	0	0	0]);

spatula_T2 = spatula_end_T1;
spatula_end_T2 = transl(-0.3, 0.3, 0.05) * desired_orientation; % This is the arm moving to the spatula

spatula_T3 = spatula_end_T2;
spatula_end_T3 = robot.model.fkine([0 -1.50796447372310 0.753982236861551 1.25663706143592 1.57079632679490 0]); % From spatula position to the stove

% Define multiple motions as transformation matrices (position and orientation)
start_T = robot.model.fkine([0 -1.50796447372310 0.753982236861551 1.25663706143592 1.57079632679490 0]);  % Start by positioning the robot to the standby position
end_T = transl(-0.4, 0, 0.2) * desired_orientation;  % This will be the pancake position

start_T2 = end_T;  % Now we reached that position 
end_T2 = transl(-0.5, 0, 0.1) * desired_orientation;  % Move forward to slide under the pancake

% After sliding and lifting the pancake
start_T3 = end_T2;  % After sliding
end_T3 = transl(-0.5, 0, 0.4) * desired_orientation;  % Lift the pancake

% Rotate only the base joint for end_T4
start_T4 = end_T3;  % After lifting

% Get the current joint configuration after lifting
current_q = robot.model.ikcon(start_T4);  % This retrieves the current joint angles

% Modify only the base joint (first joint) for rotation
rotated_q = current_q;  % Copy the current joint configuration
rotated_q(1) = -0.376991118430775;  % Rotate the base joint (about -22 degrees)

% Use fkine to calculate the transformation matrix with the new base joint rotation
end_T4 = robot.model.fkine(rotated_q);  % Apply only the base joint rotation

% After rotating the base joint (end_T4)
start_T5 = end_T4;  % After rotating the base

% Modify only the last joint (q6) to perform the flip
rotated_q6 = current_q;  % Copy the current joint configuration
rotated_q6(6) = -pi;  % Set q6 to rotate the end-effector (flip)

% Use fkine to calculate the transformation matrix with the new q6 rotation
end_T5 = robot.model.fkine(rotated_q6);  % Apply only the q6 rotation for flipping

% Visualize the target positions
hold on;

% Control the grippers to open before the first motion
OpenOrCloseGrippers(leftFinger, rightFinger, 'open', length(t));

% Move the robot to the spatula and attach the spatula
move_robot_with_grippers(robot, leftFinger, rightFinger, spatula_T1, spatula_end_T1, t);

% Close the Grippers (picking up the spatula)
OpenOrCloseGrippers(leftFinger, rightFinger, 'close', length(t));

% Move the spatula to the stove
move_robot_with_grippers(robot, leftFinger, rightFinger, spatula_T2, spatula_end_T2, t);

move_robot_with_grippers(robot, leftFinger, rightFinger, spatula_T3, spatula_end_T3, t);

% Execute the other robot motions
move_robot_with_grippers(robot, leftFinger, rightFinger, start_T, end_T, t);
move_robot_with_grippers(robot, leftFinger, rightFinger, start_T2, end_T2, t);
move_robot_with_grippers(robot, leftFinger, rightFinger, start_T3, end_T3, t);
move_robot_with_grippers(robot, leftFinger, rightFinger, start_T4, end_T4, t);
move_robot_with_grippers(robot, leftFinger, rightFinger, start_T5, end_T5, t);

disp('All motions completed!');


%% Attach Grippers to the Robot (returns left and right fingers)
function [leftFinger, rightFinger] = AttachGrippers(robot)
    disp('Attaching grippers to the robot...');
    
    % Get the transformation matrix of the end effector
    endEffectorTr = robot.model.fkine(robot.model.getpos);
    
    % Create left and right gripper instances
    leftFinger = Finger();   % Create left finger instance
    rightFinger = Finger2(); % Create right finger instance (flipped)

    % Attach the left finger
    leftFinger.model.base = endEffectorTr.T * trotx(pi/2);

    % Attach the right finger
    rightFinger.model.base = endEffectorTr.T * trotx(pi/2);
    
    % Plot the grippers
    leftFinger.PlotAndColourRobot();
    rightFinger.PlotAndColourRobot();
    
    disp('Grippers successfully attached to the robot.');
end

%% Open or Close the Grippers
function OpenOrCloseGrippers(leftFinger, rightFinger, action, steps)
    openAnglesLeft = [0, 0, 0];  % Open angles for left finger
    openAnglesRight = [0, 0, 0]; % Open angles for right finger
    closedAnglesLeft = [deg2rad(5), deg2rad(5), deg2rad(5)];  % Close angles for left finger
    closedAnglesRight = [-deg2rad(5), -deg2rad(5), -deg2rad(5)];  % Close angles for right finger

    if strcmp(action, 'open')
        qMatrixLeft = jtraj(closedAnglesLeft, openAnglesLeft, steps);   % Open trajectory for left finger
        qMatrixRight = jtraj(closedAnglesRight, openAnglesRight, steps); % Open trajectory for right finger
    elseif strcmp(action, 'close')
        qMatrixLeft = jtraj(openAnglesLeft, closedAnglesLeft, steps);   % Close trajectory for left finger
        qMatrixRight = jtraj(openAnglesRight, closedAnglesRight, steps); % Close trajectory for right finger
    end

    % Animate gripper opening/closing
    for i = 1:steps
        leftFinger.model.animate(qMatrixLeft(i, :));
        rightFinger.model.animate(qMatrixRight(i, :));
        pause(0.01);  % Small pause for smoother rendering
    end
end

function move_robot_with_grippers(robot, leftFinger, rightFinger, start_T, end_T, t)
    global isSpatulaAttached spatulaHandle spatulaVertices; % Access global variables

    % Use a good initial joint configuration to avoid flipping
    current_q = [0 -1.50796447372310 0.753982236861551 1.25663706143592 1.57079632679490 0];  % Initial guess with a natural starting position
    
    % Inverse kinematics to calculate joint angles for both positions using `ikcon`
    q_start = robot.model.ikcon(start_T, current_q);  % Solve for start position
    q_end = robot.model.ikcon(end_T, q_start);  % Solve for end position

    % Ensure that ikcon successfully found a solution for both positions
    if isempty(q_start) || isempty(q_end)
        disp('Inverse kinematics failed to find a solution for one of the positions.');
        return;  % Exit the function if IK fails
    end
    
    % Generate joint trajectory using jtraj
    q_traj = jtraj(q_start, q_end, length(t));
    
    % Animate the robot's movement and synchronize the grippers
    for i = 1:length(t)
        robot.model.animate(q_traj(i, :));
    
        % Get the transformation of the end-effector
        endEffectorTr = robot.model.fkine(q_traj(i, :));
    
        % Update the grippers' positions to follow the end-effector
        leftFinger.model.base = endEffectorTr.T * trotx(pi/2);
        rightFinger.model.base = endEffectorTr.T * trotx(pi/2);
    
        leftFinger.model.animate(leftFinger.model.getpos);
        rightFinger.model.animate(rightFinger.model.getpos);
   
        pause(0.01);  % Small pause for smoother rendering
    end
end





