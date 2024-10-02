clear all
close all
clc

robot = UR3(transl(0,0.2,0));
robot.model.plot(zeros(1,6))
% Define the shift value along the x-axis
shift_x = 0.3;
hold on
% Scale factor to convert dimensions to smaller decimal scales
scale_factor = 0.1;

% Cube 1: Shifted by 0.3 along x-axis
cube1 = collisionBox(3 * scale_factor, 4 * scale_factor, 1 * scale_factor); 
rotation1 = axang2tform([0 0 1 0]); % No rotation around z-axis
translation1 = trvec2tform([shift_x, 0, -0.1 * scale_factor]); % Shift in x
cube1.Pose = translation1 * rotation1; 
show(cube1);

% Create a collision cylinder (Pancake) with adjusted translation
radius = 0.5 * scale_factor; 
height = 2 * scale_factor;

cylinder_Pancake = collisionCylinder(radius, height); 
rotation_Pancake = axang2tform([0 0 1 0]); 
translation_pancake = trvec2tform([shift_x, 0, height/2]); % Shifted by 0.3 in x-axis
cylinder_Pancake.Pose = translation_pancake * rotation_Pancake; 
show(cylinder_Pancake);

% Cube 2: Shifted by 0.3 along x-axis
cube2 = collisionBox(3 * scale_factor, 2 * scale_factor, 1 * scale_factor);
rotation2 = axang2tform([0 0 1 0]); 
translation2 = trvec2tform([shift_x, -3 * scale_factor, 0]); 
cube2.Pose = translation2 * rotation2; 
show(cube2);

% Cube 3: Shifted by 0.3 along x-axis
cube3 = collisionBox(2 * scale_factor, 2 * scale_factor, 1 * scale_factor);
rotation3 = axang2tform([0 0 1 0]); 
translation3 = trvec2tform([-3 * scale_factor + shift_x, -3 * scale_factor, 0]); 
cube3.Pose = translation3 * rotation3; 
show(cube3);

% Spatula (Cylinder) shifted by 0.3 along x-axis
cylinder_Pancake1 = collisionCylinder(radius, 2 * scale_factor); 
rotation_Pancake1 = axang2tform([0 1 0 deg2rad(-30)]); 
translation_pancake1 = trvec2tform([-3.5 * scale_factor + shift_x, -3 * scale_factor, 2 * scale_factor]); 
cylinder_Pancake1.Pose = translation_pancake1 * rotation_Pancake1; 
show(cylinder_Pancake1);

% Cube 4: Shifted by 0.3 along x-axis
cube4 = collisionBox(1 * scale_factor, 2 * scale_factor, 2 * scale_factor);
rotation4 = axang2tform([0 0 1 0]); 
translation4 = trvec2tform([-4 * scale_factor + shift_x, -3 * scale_factor, 0.5 * scale_factor]); 
cube4.Pose = translation4 * rotation4; 
show(cube4);

% Cube 5: Shifted by 0.3 along x-axis
cube5 = collisionBox(4 * scale_factor, 2 * scale_factor, 1 * scale_factor);
rotation5 = axang2tform([0 0 1 0]); 
translation5 = trvec2tform([0.5 * scale_factor + shift_x, 3 * scale_factor, 0]); 
cube5.Pose = translation5 * rotation5; 
show(cube5);

% Set the view and axis properties
axis equal;
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
view(3);
%% Animations

% Define initial and spactula location before picking up the spactula
OriginalLocation = transl(0, 0, 0.5);  % Pick location with a 45-degree rotation around z-axis
Spatula_standby_Location = transl(0, -0.1, 0.3) * trotx(pi/2);   % Drop location with a -90-degree rotation around z-axis

q = zeros(1, 6); % Initial joint configuration guess (6-DOF robot)

% Plot the workspace and draw circles at pick and drop locations
hold on;

% Number of steps in the animation
steps = 50;

% Use ikcon to find joint configurations for pick and drop locations
qPick = robot.model.ikcon(OriginalLocation, q);   % Solves for joint angles for the pick pose
qDrop = robot.model.ikcon(Spatula_standby_Location, qPick); % Solves for joint angles for the drop pose

qMatrix = jtraj(q,qPick,steps);

for i = 1:steps
    % Get the current joint configuration
    q_current = qMatrix(i, :);
    
    % Animate the robot with the current joint configuration
    robot.model.animate(q_current);
    pause(0.05);  % Adjust the pause duration as needed for smooth animation
end

% Generate a smooth trajectory from pick to drop
qMatrix = jtraj(qPick, qDrop, steps);

% Animate the robot moving from pick to drop with rotation
for i = 1:steps
    % Get the current joint configuration
    q_current = qMatrix(i, :);
    
    % Animate the robot with the current joint configuration
    robot.model.animate(q_current);
    pause(0.05);  % Adjust the pause duration as needed for smooth animation
end

%% next step

%Prepare to pick up spactula docking the position
Spatula_docking = transl(0, -0.25, 0.3) * trotx(pi/2);   % Drop location with a -90-degree rotation around z-axis

qDocking = robot.model.ikcon(Spatula_docking, qDrop);
qMatrix = jtraj(qDrop,qDocking,steps);

% Animate the robot moving from pick to drop with rotation
for i = 1:steps
    % Get the current joint configuration
    q_current = qMatrix(i, :);
    
    % Animate the robot with the current joint configuration
    robot.model.animate(q_current);
    pause(0.05);  % Adjust the pause duration as needed for smooth animation
end
%% Animating Gripper

% Animate the gripper closing animation

%% next step

%After grapping the spactula we defined a mid point to avoid collision

midPoint1 = transl(0, 0, 0.5)* (trotx(pi/2));

qMidPoint1 = robot.model.ikcon(midPoint1, qDocking);
qMatrix = jtraj(qDocking,qMidPoint1,steps);

% Animate the robot moving from pick to drop with rotation
for i = 1:steps
    % Get the current joint configuration
    q_current = qMatrix(i, :);
    
    % Animate the robot with the current joint configuration
    robot.model.animate(q_current);
    pause(0.05);  % Adjust the pause duration as needed for smooth animation
end

% Rotating the end effector to the desired place
midPoint2 = transl(-0.1, 0, 0.5) * troty(pi/2) * trotz(pi/2);

qMidPoint2 = robot.model.ikcon(midPoint2, qMidPoint1);
qMatrix = jtraj(qMidPoint1,qMidPoint2,steps);

% Animate the robot moving from pick to drop with rotation
for i = 1:steps
    % Get the current joint configuration
    q_current = qMatrix(i, :);
    
    % Animate the robot with the current joint configuration
    robot.model.animate(q_current);
    pause(0.05);  % Adjust the pause duration as needed for smooth animation
end

%% now we do the pancake picking up motion

% We will position the spactula 
pancake_Standby = transl(0.3,0,0.2) * troty(deg2rad(120)) * trotz(pi/2);
qPStandby = robot.model.ikcon(pancake_Standby,qMidPoint2);
qMatrix = jtraj(qMidPoint2,qPStandby,steps);

% Animate the robot moving from pick to drop with rotation
for i = 1:steps
    % Get the current joint configuration
    q_current = qMatrix(i, :);
    
    % Animate the robot with the current joint configuration
    robot.model.animate(q_current);
    pause(0.05);  % Adjust the pause duration as needed for smooth animation
end

%% Sliding the spactula under the pancake

pancake_Slide = transl(0.4,0,0.2) * troty(deg2rad(120)) * trotz(pi/2);
qPSlide = robot.model.ikcon(pancake_Slide, qPStandby);

qMatrix = jtraj(qPStandby,qPSlide,steps);

% Animate the robot moving from pick to drop with rotation
for i = 1:steps
    % Get the current joint configuration
    q_current = qMatrix(i, :);
    
    % Animate the robot with the current joint configuration
    robot.model.animate(q_current);
    pause(0.05);  % Adjust the pause duration as needed for smooth animation
end

%% After sliding the spactula under the pancake lift the pancake
pancake_lift = transl(0.3,-0.15,0.3) * troty(pi/2) * trotz(pi/2);
qPLift = robot.model.ikcon(pancake_lift,qPSlide);

qMatrix = jtraj(qPSlide,qPLift,steps);

for i = 1:steps
    % Get the current joint configuration
    q_current = qMatrix(i, :);
    
    % Animate the robot with the current joint configuration
    robot.model.animate(q_current);
    pause(0.05);  % Adjust the pause duration as needed for smooth animation
end

%% Flipping the pancake

pancake_flip = transl(0.3,-0.15,0.3) * (troty(pi/2) * trotz(pi));
qPFlip = robot.model.ikcon(pancake_flip,qPLift);

qMatrix = jtraj(qPLift,qPFlip,steps);

for i = 1:steps
    % Get the current joint configuration
    q_current = qMatrix(i, :);
    
    % Animate the robot with the current joint configuration
    robot.model.animate(q_current);
    pause(0.05);  % Adjust the pause duration as needed for smooth animation
end
