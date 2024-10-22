
%% Initialize Robot
centerpnt = [2, 0, -0.5];
robot = UR3e(transl(centerpnt + [-1, 0, +0.5])); % Initialize robot at a specific position

% Define the initial joint configuration (0 degrees for the base)
q_start = robot.model.getpos();  % All joint angles start at 0
q_end = [pi + pi/2, 0, 0, 0, 0, 0]  % Rotate the base 180 degrees (pi radians)

%% Define rectangular prism for visualization
side = 1.5;

plotOptions.plotFaces = true;
[vertex, faces, faceNormals] = RectangularPrism(centerpnt - side/2, centerpnt + side/2, plotOptions);

% Define the ground as a large rectangle at Z = -0.1
groundVertex = [2 -2 -0.1; 2 2 -0.1; -2 2 -0.1; -2 -2 -0.1];
groundFaces = [1 2 3; 1 3 4];
groundFaceNormals = repmat([0, 0, 1], 2, 1); % Normal pointing upward (Z-axis)

% Combine ground vertices and normals with obstacle vertices and normals
vertex = [vertex; groundVertex];
faces = [faces; groundFaces + size(vertex,1) - 4];  % Adjust face indices for the ground vertices
faceNormals = [faceNormals; groundFaceNormals];

% Call the collision avoidance function
runCollisionAvoidance(centerpnt, robot, q_start, q_end, vertex, faces, faceNormals, 100);
