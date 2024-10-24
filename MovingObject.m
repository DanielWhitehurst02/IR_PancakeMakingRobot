% Load the UR3e Robot
robot = UR3e();  % Assuming UR3e is a class with the 'model' property for the robot
hold on
% Load the object (Blue Syrup Bottle)
mesh_h = PlaceObject('BlueSyrupBottle.ply');
vertices = get(mesh_h, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl([0, 0.3, 0.5])';
set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update object position
%%
axis equal

% Define start and end transformations
startTr = transl(0, 0.3, 0.5); % Start pose
endTr = transl(0.2, -0.3, 0.5); % End pose

% Set initial joint configuration
q0 = zeros(1, robot.model.n);  % Initial joint configuration

% Solve IK for start and end configurations
qStart = robot.model.ikcon(startTr, q0);  % Joint configuration for start pose
qEnd = robot.model.ikcon(endTr, qStart);  % Joint configuration for end pose

% Generate a trajectory between start and end
steps = 50;  % Number of steps in the trajectory
qMatrix = jtraj(qStart, qEnd, steps);  % Trajectory generation

% Animate the robot and move the object
for i = 1:steps
    % Animate the robot moving along the trajectory
    robot.model.animate(qMatrix(i, :));
    
    % Get the current end-effector transformation using forward kinematics
    tr = robot.model.fkine(qMatrix(i, :));
    
    % Transform the vertices of the object to follow the end-effector
    transformedVertices = [vertices, ones(size(vertices, 1), 1)] * tr.T';
    set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update object position
    
    % Update the plot
    drawnow();
    pause(0.05);  % Control the speed of the animation
end
