function DoughPouring(app)
    
    robot2 = Panda(transl(0.9,0.5,0.5));
    hold on
    leftFinger2 = Finger();  % Initialize left finger
    rightFinger2 = Finger2();  % Initialize right finger
    
 centerPoints2 = [
        -0.025, 0.0, 0.06;
        0.0, 0.0975, -0.035;
        0.0, 0.03, 0.075;
        -0.075, -0.085, 0.0;
        0.05, 0.0, 0.035;
        0.0, 0.125, 0.025;
        -0.05, 0.025, 0.0;
    ];
    radii2 = [
        0.135, 0.125, 0.075;
        0.1, 0.175, 0.1;
        0.125, 0.12, 0.125;
        0.075, 0.075, 0.095;
        0.125, 0.125, 0.095;
        0.085, 0.15, 0.105;
        0.110, 0.1, 0.085;
    ];


    cubePoints2 = []
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
BlueBottle=[0.8,1.25,0.82];
PinkBottle=[1.05,1.25,0.82];
BrownBottle=[1.3,1.25,0.82];
DoughBottle=[0.6,1.25,0.82];
Spatula=[-0.42,0.5,0.95];

%Place other objects

  % Place the object
mesh_hBlue = PlaceObject('BlueSyrupBottle.ply');
verticesBlue = get(mesh_hBlue, 'Vertices');
transformedVertices = [verticesBlue, ones(size(verticesBlue, 1), 1)] * transl(BlueBottle)';
set(mesh_hBlue, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_hPink = PlaceObject('PinkSyrupBottle.ply');
vertices = get(mesh_hPink, 'Vertices');
transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl(PinkBottle)';
set(mesh_hPink, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_hBrown = PlaceObject('BrownSyrupBottle.ply');
verticesPink = get(mesh_hBrown, 'Vertices');
transformedVertices = [verticesPink, ones(size(verticesPink, 1), 1)] * transl(BrownBottle)';
set(mesh_hBrown, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_hDough = PlaceObject('DoughBottle.ply');
verticesDough = get(mesh_hDough, 'Vertices');
transformedVertices = [verticesDough, ones(size(verticesDough, 1), 1)] * transl(DoughBottle)';
set(mesh_hDough, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_hSpatula = PlaceObject('spatula.ply');
verticesSpatula = get(mesh_hSpatula, 'Vertices');
transformedVertices = [verticesSpatula, ones(size(verticesSpatula, 1), 1)] * transl(Spatula)';
set(mesh_hSpatula, 'Vertices', transformedVertices(:, 1:3)); % Update object position

mesh_hPancake = PlaceObject('pancake2.ply');
verticesPancake = get(mesh_hPancake, 'Vertices');
transformedVertices = [verticesPancake, ones(size(verticesPancake, 1), 1)] * transl([-0.3, -0.33, 0.57])';
set(mesh_hPancake, 'Vertices', transformedVertices(:, 1:3)); % Update object position

PlaceObject('Stove.ply',[-0.2,0.5,0.5]);
PlaceObject('ConveyorBelt.ply',[0.2,-0.5,0]);
PlaceObject('Table.ply',[-1.7,0.5,-0.15]);
PlaceObject('Table.ply',[1.6,0.5,-0.15]);
PlaceObject('Shelf.ply',[1,1.3,0.52]);
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


time = 4;  % Total time for movement (in seconds)
deltaT = 0.2;  % Control step time (in seconds


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
    time = 5;  % Total time for movement (in seconds)
    deltaT = 0.05;  % Control step time (in seconds
    q02 = zeros(1, robot2.model.n);

    
    motionHandler2 = MotionHandlerWIthGripperAndObjects(robot2, centerPoints2, radii2, cubePoints2, leftFinger2, rightFinger2,app,troty(-pi/2) * trotz(-pi),transl(0,0.15,0));
    %% First
    endTr = transl(0.61,1.1,1.1); % Example target transformation
    t = 10;  % Total time for movement (in seconds)
    deltaT = 0.05;
    motionHandler2.runRMRC(endTr,time,deltaT);
    %% Second
    endTr = transl(0.61,1.1,0.95)*troty(pi/2)*trotx(pi);
    motionHandler2.runRMRC(endTr,time,deltaT);
   
    motionHandler2.OpenOrCloseGrippers('close',50);
    %% Third
    endTr = transl(-0.2,0.8,0.95)*troty(pi/2);
    t = 10;  % Total time for movement (in seconds)
    deltaT = 0.01;
    motionHandler2.runRMRC(endTr,time,deltaT,mesh_hDough,verticesDough);
    %% Fourth
    t = 10;  % Total time for movement (in seconds)
    deltaT = 0.05;
    endTr = transl(-0.2,0.2,0.95)*troty(pi/2);
    motionHandler2.runRMRC(endTr,time,deltaT,mesh_hDough,verticesDough);
    %% Fifth (Dough Pouring)
    t = 10;  % Total time for movement (in seconds)
    deltaT = 0.1;
    endTr = transl(-0.2,0.2,0.95)*troty(-pi/2)*trotx(-pi/2);
    motionHandler2.runRMRC(endTr,time,deltaT,mesh_hDough,verticesDough);
   
     %% Sixth 
    t = 10;  % Total time for movement (in seconds)
    deltaT = 0.05;
    endTr = transl(-0.2,0.2,0.95)*troty(pi/2);
    motionHandler2.runRMRC(endTr,time,deltaT,mesh_hDough,verticesDough);
     %% Seventh
    endTr = transl(0.61,1,0.95)*troty(pi/2);
    t = 10;  % Total time for movement (in seconds)
    deltaT = 0.05;
    motionHandler2.runRMRC(endTr,time,deltaT,mesh_hDough,verticesDough);
     %% Seventh
    endTr = transl(0.61,1,0.95)*troty(pi/2)*trotx(pi);
    t = 10;  % Total time for movement (in seconds)
    deltaT = 0.05;
    motionHandler2.runRMRC(endTr,time,deltaT,mesh_hDough,verticesDough);
   
    motionHandler2.OpenOrCloseGrippers('open',50);
   
end
