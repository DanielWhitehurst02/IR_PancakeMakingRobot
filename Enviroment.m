%% Clear workspace
clf
clc
clear all

%% Setting up workspace
workspace = [-2,3,-3,2.5,0,2];
hold on
axis equal
axis(workspace)
surf([-2.5,-2.5;3.5,3.5] ...
,[-2.5,2.5;-2.5,2.5] ...
,[0.01,0.01;0.01,0.01] ...
,'CData',imread('florring.jpg') ...
,'FaceColor','texturemap');

%defining pickable objects variables
BlueBottle=[0.8,1.25,0.88];
PinkBottle=[1.05,1.25,0.88];
BrownBottle=[1.3,1.25,0.88];
DoughBottle=[0.6,1.25,0.88];
Spatula=[-0.42,0.5,0.95];

%Place other objects

  % Place the object
mesh_hBlue = PlaceObject('BlueSyrupBottle.ply');
vertices = get(mesh_hBlue, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(BlueBottle)';
set(mesh_hBlue, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_hPink = PlaceObject('PinkSyrupBottle.ply');
vertices = get(mesh_hPink, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(PinkBottle)';
set(mesh_hPink, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_hBrown = PlaceObject('BrownSyrupBottle.ply');
vertices = get(mesh_hBrown, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(BrownBottle)';
set(mesh_hBrown, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_hDough = PlaceObject('DoughBottle.ply');
vertices = get(mesh_hDough, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(DoughBottle)';
set(mesh_hDough, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_hSpatula = PlaceObject('spatula.ply');
vertices = get(mesh_hSpatula, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(Spatula)';
set(mesh_hSpatula, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_hPancake = PlaceObject('pancake.ply');
vertices = get(mesh_hPancake, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl([-0.3, -0.33, 0.57])';
set(mesh_hPancake, 'Vertices', transformedVertices(:, 1:3)); % Update object position

PlaceObject('Stove.ply',[-0.2,0.5,0.5]);
PlaceObject('ConveyorBelt.ply',[0.2,-0.5,0]);
PlaceObject('Table.ply',[-1.7,0.5,-0.15]);
PlaceObject('Table.ply',[1.6,0.5,-0.15]);
PlaceObject('Shelf.ply',[1,1.3,0.88]);
PlaceObject('tableBrown2.1x1.4x0.5m.ply',[-0.45,0.78,0]);
PlaceObject('tableBrown2.1x1.4x0.5m.ply',[0.7,0.78,0]);
PlaceObject('SpatulaHolder.ply',[-0.4,0.7,0.52]);
PlaceObject('FireExtinguisherElevated.ply',[-1.5,-0.95,0.5]);
PlaceObject('Plate.ply', [-0.3, -0.33, 0.565]);
PlaceObject('emergencyStopButton.ply',[0,-1.7,0]);
PlaceObject('glass1.ply', [2.12,1.85,1.8]);
PlaceObject('glass2.ply', [-1.9,1.85,1.8]);
PlaceObject('glass3.ply', [2.1,1.85,1.8]);
PlaceObject('barrier1.5x0.2x1m.ply',[0.8,-1.3,0]);
PlaceObject('barrier1.5x0.2x1m.ply',[-0.5,-1.3,0]);

for position = [1.6,1.55,0; 1.6,-1.2,0;]'
    h = PlaceObject('barrier1.5x0.2x1m.ply', position');
    verts = [get(h,'Vertices'), ones(size(get(h,'Vertices'),1),1)] * trotz(pi/2);
    set(h,'Vertices',verts(:,1:3))
end

for position = [-1.67,0.15,-0.15]'
    h = PlaceObject('Table.ply', position');
    verts = [get(h,'Vertices'), ones(size(get(h,'Vertices'),1),1)] * trotz(pi/2);
    set(h,'Vertices',verts(:,1:3))
end

robot = UR3(transl(-0.5,0.2,0.5));


path = RMRC(robot);  % Instantiate RMRC for the UR3 robot

% Initial joint configuration (all joints at zero)
q0 = zeros(1, robot.model.n);

% Get the current end-effector position using forward kinematics
startTr_struct = robot.model.fkine(q0);  % Get the forward kinematics
if isobject(startTr_struct)
    startTr = startTr_struct.T;  % Extract the .T property if it's an object
else
    startTr = startTr_struct;    % Directly use if it's already a matrix
end

% Define the target transformation matrix

endTr = transl(Spatula)*trotx(-pi/2) % Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr, endTr, t, deltaT);
%% Second animation
a=robot.model.fkine(robot.model.getpos()).T;

endTr1 = transl(-0.2,0.2,0.9)*troty(pi/2) % Example target transformation 
% Control step time (in seconds)
% Set the total time and control frequency

path.ResolvedMotionRateControl(a, endTr1, t, deltaT);
%% Third animation
b=robot.model.fkine(robot.model.getpos()).T;
endTr2 = transl(-0.2,0.2,0.8)*troty(pi/2); % Example target transformation

% Set the total time and control frequency

path.ResolvedMotionRateControl(b, endTr2, t, deltaT);

%% Fourth Animation
c=robot.model.fkine(robot.model.getpos()).T;
endTr3 = transl(-0.1,0.2,0.8)*troty(pi/2); % Example target transformation

% Set the total time and control frequency
path.ResolvedMotionRateControl(c, endTr3, t, deltaT);
%% Fifth Animation
d=robot.model.fkine(robot.model.getpos()).T;
endTr4 = transl(-0.1,0.2,1)*troty(pi/2); % Example target transformation

% Set the total time and control frequency

path.ResolvedMotionRateControl(d, endTr4, t, deltaT);
%% Sixth Animation
E=robot.model.fkine(robot.model.getpos()).T;
endTr5 = transl(-0.1,0.2,1)*troty(pi/2)*trotz(-pi); % Example target transformation
% Set the total time and control frequency
deltaT = 0.2;
path.ResolvedMotionRateControl(E, endTr5, t, deltaT);
%% Seventh animation
F=robot.model.fkine(robot.model.getpos()).T;
endTr6 = transl(-0.1,0.2,0.8)*troty(pi/2)*trotz(-pi); % Example target transformation
% Set the total time and control frequency

path.ResolvedMotionRateControl(F, endTr6, t, deltaT);

%% Eight animation
G=robot.model.fkine(robot.model.getpos()).T;
endTr7 = transl(-0.2,0.2,0.9)*troty(pi/2); % Example target transformation
% Set the total time and control frequency
deltaT = 0.05;
path.ResolvedMotionRateControl(G, endTr7, t, deltaT);

%% Ninth Animation
H=robot.model.fkine(robot.model.getpos()).T;
endTr8 = transl(-0.2,0.2,0.8)*troty(pi/2); % Example target transformation
% Set the total time and control frequency
deltaT = 0.05;
path.ResolvedMotionRateControl(H, endTr8, t, deltaT);
