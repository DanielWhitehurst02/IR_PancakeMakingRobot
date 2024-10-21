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

%% Standard codes goes here
robot1 = UR3(transl(-0.3,0.2,0.5));
%%
robot2 = Panda(transl(0.5,0.5,0.5));

%% defining mesh cube
center = [0.2,0,0.7];

cubePoints = meshcube(0.5,0.5,[pi/3,pi/4,2*pi/7],0.02,center);

%% UR3 motion Handler
% UR3
centerPoints_UR3 = [0.0, 0.0, 0.05; % Base 
                0.0, -0.01, 0.01; % Link 1 
                0.125, 0.0, 0.125; % Link 2 
                0.105, 0.0, 0.05; % Link 3 
                0.0, 0.0, 0.01; % Link 4 
                0.0, 0.0, 0.06; % Link 5 
                0.0, 0.0, 0.0;]; % end-effector
            
radii_UR3 = [0.08, 0.09, 0.055;  
         0.075, 0.085, 0.075;
         0.175, 0.08, 0.085; 
         0.15, 0.06, 0.085; 
         0.04, 0.055, 0.065;
         0.04, 0.045, 0.125; 
         0.0, 0.0, 0.0;]; 

obstaclePoints1 = cubePoints;

motionHandler1 = MotionHandler(robot1,centerPoints_UR3,radii_UR3,obstaclePoints1);

start_trl = robot1.model.fkine(robot1.model.getpos()).T;
end_trl = transl(-0.1,0,0.7)
motionHandler1.runIK(start_trl,transl(-0.4,0.3,0.7),50);

