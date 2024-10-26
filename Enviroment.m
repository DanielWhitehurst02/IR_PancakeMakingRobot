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
mesh_h = PlaceObject('BlueSyrupBottle.ply');
vertices = get(mesh_h, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(BlueBottle)';
set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_h = PlaceObject('PinkSyrupBottle.ply');
vertices = get(mesh_h, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(PinkBottle)';
set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_h = PlaceObject('BrownSyrupBottle.ply');
vertices = get(mesh_h, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(BrownBottle)';
set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_h = PlaceObject('DoughBottle.ply');
vertices = get(mesh_h, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(DoughBottle)';
set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_h = PlaceObject('spatula.ply');
vertices = get(mesh_h, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(Spatula)';
set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_h = PlaceObject('pancake.ply');
vertices = get(mesh_h, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl([-0.3, -0.33, 0.57])';
set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update object position

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

