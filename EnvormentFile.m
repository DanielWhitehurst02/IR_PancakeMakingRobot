%% Clear workspace
clf
clc
clear all
%% Setting up workspace
workspace = [-2,2.5,-2,2.5,0,2];
hold on
axis equal
axis(workspace)
surf([-2.5,-2.5;3.5,3.5] ...
,[-2.5,2.5;-2.5,2.5] ...
,[0.01,0.01;0.01,0.01] ...
,'CData',imread('florring.jpg') ...
,'FaceColor','texturemap');


%PlaceObject('tableRound0.3x0.3x0.3m.ply',[0.03,-0.45,0]);
PlaceObject('BlueSyrupBottle.ply',[1.5,0,0.55]);
PlaceObject('PinkSyrupBottle.ply',[1.5,0.1,0.55]);
PlaceObject('BrownSyrupBottle.ply',[1.5,-0.1,0.55]);
PlaceObject('DoughBottle.ply',[0.2,-0.1,0.5]);
PlaceObject('Stove.ply',[-0.2,0.5,0.5]);
PlaceObject('ConveyorBelt.ply',[0.7,-0.5,0]);
PlaceObject('Plate.ply',[0.03,-0.5,0.6]);
PlaceObject('Table.ply',[-1.7,0.5,-0.15]);
PlaceObject('Table.ply',[1.6,0.5,-0.15]);
PlaceObject('tableBrown2.1x1.4x0.5m.ply',[-0.45,0.78,0]);
PlaceObject('tableBrownmod.ply',[0.7,0,0]);
for position = [-1.67,0.15,-0.15]'
    h = PlaceObject('Table.ply', position');
    verts = [get(h,'Vertices'), ones(size(get(h,'Vertices'),1),1)] * trotz(pi/2);
    set(h,'Vertices',verts(:,1:3))
end

robot1 = UR3(transl(-0.3,0.2,0.5));
robot2 = Panda(transl(0.5,0.5,0.5))
