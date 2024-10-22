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
PlaceObject('BlueSyrupBottle.ply',[0.9,1.25,0.88]);
PlaceObject('PinkSyrupBottle.ply',[1.15,1.25,0.88]);
PlaceObject('BrownSyrupBottle.ply',[1.4,1.25,0.88]);
PlaceObject('DoughBottle.ply',[0.6,1.25,0.88]);
PlaceObject('Stove.ply',[-0.2,0.5,0.5]);
PlaceObject('ConveyorBelt.ply',[0.2,-0.5,0]);
PlaceObject('Plate.ply',[0.03,-0.5,0.6]);
PlaceObject('Table.ply',[-1.7,0.5,-0.15]);
PlaceObject('Table.ply',[1.6,0.5,-0.15]);
PlaceObject('Shelf.ply',[1,1.3,0.88]);
PlaceObject('tableBrown2.1x1.4x0.5m.ply',[-0.45,0.78,0]);
PlaceObject('tableBrown2.1x1.4x0.5m.ply',[0.7,0.78,0]);
PlaceObject('SpatulaHolder.ply',[-0.4,0.7,0.52]);
PlaceObject('spatula.ply',[-0.42,0.52,0.85]);
PlaceObject('FireExtinguisherElevated.ply',[-1.5,-0.9,0.5]);


for position = [-1.67,0.15,-0.15]'
    h = PlaceObject('Table.ply', position');
    verts = [get(h,'Vertices'), ones(size(get(h,'Vertices'),1),1)] * trotz(pi/2);
    set(h,'Vertices',verts(:,1:3))
end

robot = UR3(transl(-0.3,0.2,0.5));