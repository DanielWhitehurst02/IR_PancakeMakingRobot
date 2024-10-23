% Load a 3DOF Planar Robot
L1 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi pi]);
L2 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi pi]);
L3 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi pi]);
robot = SerialLink([L1, L2, L3], 'name', 'myRobot');
hold on
% Set workspace and initial joint configuration
q = zeros(1, 3);
robot.plot(q, 'workspace', [-2 2 -2 2 -0.05 2]);

% Load the pen .ply object and place it
mesh_h = PlaceObject('BlueSyrupBottle.ply');
axis equal
vertices = get(mesh_h, 'Vertices'); % Get vertices of the pen model

% Animate the robot and move the pen
steps = 50; % Number of steps in the animation
qMatrix = jtraj([0, 0, 0], [pi/4, pi/4, pi/4], steps); % Trajectory from initial to target

for i = 1:steps
    % Animate the robot moving along the trajectory
    robot.animate(qMatrix(i, :));
    
    % Get the current end-effector transformation using forward kinematics
    tr = robot.fkine(qMatrix(i, :));
    
    % Transform the vertices of the pen to follow the end-effector
    transformedVertices = [vertices, ones(size(vertices, 1), 1)] * tr.T';
    set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update pen position
    
    % Update the plot
    drawnow();
    pause(0.01); % Control the speed of the animation
end

