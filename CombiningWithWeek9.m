clear all
close all
clc

% Initialize the robot model
robot = UR3(transl(0,0.2,0));
robot.model.plot(zeros(1,6))

% Define the shift value along the x-axis
shift_x = 0.3;
hold on
% Scale factor to convert dimensions to smaller decimal scales
scale_factor = 0.1;

% Cube 1: Shifted by 0.3 along x-axis
cube1 = collisionBox(3 * scale_factor, 4 * scale_factor, 1 * scale_factor); 
translation1 = trvec2tform([shift_x, 0, -0.1 * scale_factor]); % Shift in x
cube1.Pose = translation1; 
show(cube1);

% Create a collision cylinder (Pancake) with adjusted translation
radius = 0.5 * scale_factor; 
height = 2 * scale_factor;

cylinder_Pancake = collisionCylinder(radius, height); 
translation_pancake = trvec2tform([shift_x, 0, height/2]); % Shifted by 0.3 in x-axis
cylinder_Pancake.Pose = translation_pancake; 
show(cylinder_Pancake);

% Cube 2: Shifted by 0.3 along x-axis
cube2 = collisionBox(3 * scale_factor, 2 * scale_factor, 1 * scale_factor);
translation2 = trvec2tform([shift_x, -3 * scale_factor, 0]); 
cube2.Pose = translation2; 
show(cube2);

% Cube 3: Shifted by 0.3 along x-axis
cube3 = collisionBox(2 * scale_factor, 2 * scale_factor, 1 * scale_factor);
translation3 = trvec2tform([-3 * scale_factor + shift_x, -3 * scale_factor, 0]); 
cube3.Pose = translation3; 
show(cube3);

% Spatula (Cylinder) shifted by 0.3 along x-axis
cylinder_Pancake1 = collisionCylinder(radius, 2 * scale_factor); 
translation_pancake1 = trvec2tform([-3.5 * scale_factor + shift_x, -3 * scale_factor, 2 * scale_factor]); 
cylinder_Pancake1.Pose = translation_pancake1; 
show(cylinder_Pancake1);

% Cube 4: Shifted by 0.3 along x-axis
cube4 = collisionBox(1 * scale_factor, 2 * scale_factor, 2 * scale_factor);
translation4 = trvec2tform([-4 * scale_factor + shift_x, -3 * scale_factor, 0.5 * scale_factor]); 
cube4.Pose = translation4; 
show(cube4);

% Cube 5: Shifted by 0.3 along x-axis
cube5 = collisionBox(4 * scale_factor, 2 * scale_factor, 1 * scale_factor);
translation5 = trvec2tform([0.5 * scale_factor + shift_x, 3 * scale_factor, 0]); 
cube5.Pose = translation5; 
show(cube5);

% Set the view and axis properties
axis equal;
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
view(3);

% Define the spatula triangle dimensions
x_length = 0.2; % Length of the base of the triangle (along the x-axis)
y_height = 0.2; % Height of the triangle (along the y-axis)

% Define the triangle vertices relative to the origin (end effector frame)
triangle_vertices = [0, 0, 0; 
                     x_length, 0, 0; 
                     0, y_height, 0]; % Vertices in the local frame of the spatula

%% Animations

% Define initial and spatula location before picking up the spatula
OriginalLocation = transl(0, 0, 0.5);  % Pick location with a 45-degree rotation around z-axis
Spatula_standby_Location = transl(0, -0.1, 0.3) * trotx(pi/2);   % Drop location with a -90-degree rotation around z-axis

q = zeros(1, 6); % Initial joint configuration guess (6-DOF robot)
steps = 50; % Number of steps in the animation

% Use ikcon to find joint configurations for pick and drop locations
qPick = robot.model.ikcon(OriginalLocation, q);   % Solves for joint angles for the pick pose
qDrop = robot.model.ikcon(Spatula_standby_Location, qPick); % Solves for joint angles for the drop pose

% Animate the robot moving from initial pose to the pick pose using RMRC
[qMatrixPick, posErrorPick, angErrorPick] = RMRCAnimation(robot, q, qPick, steps, triangle_vertices);

% Animate the robot moving from pick pose to the drop pose using RMRC
[qMatrixDrop, posErrorDrop, angErrorDrop] = RMRCAnimation(robot, qPick, qDrop, steps, triangle_vertices);

%% Prepare to pick up spatula docking the position
Spatula_docking = transl(0, -0.25, 0.3) * trotx(pi/2);   % Drop location with a -90-degree rotation around z-axis
qDocking = robot.model.ikcon(Spatula_docking, qDrop);

[qMatrixDocking, posErrorDocking, angErrorDocking] = RMRCAnimation(robot, qDrop, qDocking, steps, triangle_vertices);

%% Moving to pancake picking up motion
pancake_Standby = transl(0.3, 0, 0.2) * troty(deg2rad(120)) * trotz(pi/2);
qPStandby = robot.model.ikcon(pancake_Standby, qDocking);

[qMatrixStandby, posErrorStandby, angErrorStandby] = RMRCAnimation(robot, qDocking, qPStandby, steps, triangle_vertices);

%% Sliding the spatula under the pancake
pancake_Slide = transl(0.4, 0, 0.2) * troty(deg2rad(120)) * trotz(pi/2);
qPSlide = robot.model.ikcon(pancake_Slide, qPStandby);

[qMatrixSlide, posErrorSlide, angErrorSlide] = RMRCAnimation(robot, qPStandby, qPSlide, steps, triangle_vertices);

%% Lifting the pancake
pancake_lift = transl(0.3, -0.15, 0.3) * troty(pi/2) * trotz(pi/2);
qPLift = robot.model.ikcon(pancake_lift, qPSlide);

[qMatrixLift, posErrorLift, angErrorLift] = RMRCAnimation(robot, qPSlide, qPLift, steps, triangle_vertices);

%% Flipping the pancake
pancake_flip = transl(0.3, -0.15, 0.3) * (troty(pi/2) * trotz(pi));
qPFlip = robot.model.ikcon(pancake_flip, qPLift);

[qMatrixFlip, posErrorFlip, angErrorFlip] = RMRCAnimation(robot, qPLift, qPFlip, steps, triangle_vertices);

% Plot the results
plotRMRCResults(posErrorPick, angErrorPick, 'Pick Position and Angle Error');
plotRMRCResults(posErrorDrop, angErrorDrop, 'Drop Position and Angle Error');

% End of the main script

%% RMRC Function
% Corrected RMRC Function with .T handling
function [qMatrix, posError, angError] = RMRCAnimation(robot, qStart, qEnd, steps, triangle_vertices)
    % RMRC parameters
    deltaT = 0.05;  % Time step
    W = diag([1,1,1,1,1,1]);  % Weighting matrix for velocity
    epsilon = 0.1;  % Manipulability threshold for DLS

    % Generate joint trajectory using RMRC
    qMatrix = zeros(steps,6);
    posError = zeros(3,steps);
    angError = zeros(3,steps);
    qMatrix(1,:) = qStart;
    
    % Get the final transformation for qEnd (end pose)
    T_end = robot.model.fkine(qEnd).T;  % Forward kinematics for the end configuration, using .T to get the homogeneous transformation
    finalPosition = T_end(1:3,4);       % Extract the translation component
    
    for i = 1:steps-1
        T = robot.model.fkine(qMatrix(i,:)).T;  % Get forward transformation for the current joint state and extract .T for homogeneous matrix
        currentPosition = T(1:3,4);             % Extract the translation component
        
        deltaX = finalPosition - currentPosition;  % Position error

        Rd = T_end(1:3,1:3);  % Desired rotation matrix from the final pose
        Ra = T(1:3,1:3);      % Current rotation matrix
        Rdot = (Rd - Ra) / deltaT;  % Rotation error
        S = (Rdot - Rdot') / 2;     % Skew-symmetric matrix
        angular_velocity = [S(3,2); S(1,3); S(2,1)];  % Angular velocity

        xdot = W * [deltaX/deltaT; angular_velocity];  % End-effector velocity

        % Get the Jacobian
        J = robot.model.jacob0(qMatrix(i,:));

        % Use Damped Least Squares if manipulability is low
        if abs(det(J*J')) < epsilon
            lambda = 0.1;  % Damping coefficient
            invJ = pinv(J'*J + lambda^2*eye(6)) * J';
        else
            invJ = pinv(J);  % Regular pseudoinverse
        end

        qdot = invJ * xdot;  % Joint velocities
        qMatrix(i+1,:) = qMatrix(i,:) + qdot' * deltaT;  % Update joint angles
        
        % Record position and angle errors
        posError(:,i) = deltaX;
        angError(:,i) = tr2rpy(Rd*Ra');
    end

    % Animate the robot arm
    animateWithTriangle(robot, qMatrix, triangle_vertices, steps);
end

%% Function to animate with triangle visualization
function animateWithTriangle(robot, qMatrix, triangle_vertices, steps)
    % This function animates the robot while drawing the spatula triangle
    % and plotting the path of the end-effector.
    
    % Initialize array to store end-effector positions for path plotting
    end_effector_positions = zeros(steps, 3);
    
    for i = 1:steps
        % Get the current joint configuration
        q_current = qMatrix(i, :);
        
        % Animate the robot with the current joint configuration
        robot.model.animate(q_current); % Preserve other objects
        
        % Obtain the transformation matrix of the end effector
        T_end_effector = robot.model.fkine(q_current);

        % Check if T_end_effector is an SE3 object and convert it to a matrix if necessary
        if isa(T_end_effector, 'SE3')
            T_end_effector = T_end_effector.T; % Convert SE3 object to a 4x4 matrix
        end
        
        % Extract the rotation (R) and translation (position) from the end-effector's transformation
        R = T_end_effector(1:3, 1:3); % Extract rotation matrix
        end_effector_pos = T_end_effector(1:3, 4); % Extract translation
        
        % Store the current end-effector position
        end_effector_positions(i, :) = end_effector_pos';
        
        % Define a local shift matrix to shift the triangle vertices before applying the rotation
        local_shift = transl(0, -0.15, 0); % Shift in y direction by -0.15
        
        % Align the triangle so that its hypotenuse is along the end effector's z-axis
        local_rotation = troty(-pi/2); % Rotate around the y-axis by -90 degrees
        
        % Apply the local shift and rotation to the triangle vertices
        transformed_vertices = (local_shift(1:3, 1:3) * local_rotation(1:3, 1:3) * triangle_vertices')';
        transformed_vertices = transformed_vertices + local_shift(1:3, 4)'; % Apply translation shift
        
        % Apply the end-effector's rotation and translation to the transformed vertices
        global_vertices = (R * transformed_vertices')' + end_effector_pos';
        
        % Clear only previous triangle to avoid overlapping artifacts
        delete(findobj(gca, 'Tag', 'Triangle')); % Delete the old triangle while preserving other objects
        
        % Plot the triangle using fill3
        fill3(global_vertices(:, 1), global_vertices(:, 2), global_vertices(:, 3), 'g', 'FaceAlpha', 0.5, 'Tag', 'Triangle');
        
        % Plot the path of the end-effector by connecting the current position to the previous one
        if i > 1
            plot3([end_effector_positions(i-1, 1), end_effector_positions(i, 1)], ...
                  [end_effector_positions(i-1, 2), end_effector_positions(i, 2)], ...
                  [end_effector_positions(i-1, 3), end_effector_positions(i, 3)], 'r-', 'LineWidth', 2);
        end
        
        pause(0.05); % Adjust the pause duration for smooth animation
    end
end

%% Function to plot RMRC results
function plotRMRCResults(posError, angError, titleText)
    figure;
    subplot(2,1,1);
    plot(posError'*1000,'LineWidth',1);
    xlabel('Step');
    ylabel('Position Error (mm)');
    legend('X-Axis','Y-Axis','Z-Axis');
    title([titleText, ' - Position Error']);
    
    subplot(2,1,2);
    plot(angError','LineWidth',1);
    xlabel('Step');
    ylabel('Angle Error (rad)');
    legend('Roll','Pitch','Yaw');
    title([titleText, ' - Angle Error']);
end
