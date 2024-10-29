function robot_and_pancake(app)
    
    robot1 = UR3(transl(-0.5,0.2,0.5));
    hold on
    leftFinger1 = Finger();  % Initialize left finger
    rightFinger1 = Finger2();  % Initialize right finger
    
    centerPoints1 = [0.0, 0.0, 0.05; % Base 
                    0.0, -0.01, 0.01; % Link 1 
                    0.125, 0.0, 0.125; % Link 2 
                    0.105, 0.0, 0.05; % Link 3 
                    0.0, 0.0, 0.01; % Link 4 
                    0.0, 0.0, 0.06; % Link 5 
                    0.0, 0.0, 0.0;]; % end-effector
    
    radii1 = [0.08, 0.09, 0.055;  
             0.075, 0.085, 0.075;
             0.175, 0.08, 0.085; 
             0.15, 0.06, 0.085; 
             0.04, 0.055, 0.065;
             0.04, 0.045, 0.125; 
             0.0, 0.0, 0.0;]; 


    cubePoints1 = []
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

%mesh_hPancake = PlaceObject('pancake2.ply');
%verticesPancake = get(mesh_hPancake, 'Vertices');
%transformedVertices = [verticesPancake, ones(size(verticesPancake, 1), 1)] * transl([-0.3, -0.33, 0.57])';
%set(mesh_hPancake, 'Vertices', transformedVertices(:, 1:3)); % Update object position

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
   

    endTr0 = transl(-0.42,0.5,0.95)*trotx(-pi/2) % Example target transformation
    motionHandler1 = MotionHandlerWIthGripperAndObjects(robot1, centerPoints1, radii1, cubePoints1, leftFinger1, rightFinger1,app);
    motionHandler1.runRMRC(endTr0, time, deltaT);
    motionHandler1.OpenOrCloseGrippers('open',50);
    motionHandler1.OpenOrCloseGrippers('close',50);
    endTr1 = transl(-0.2,0.2,0.9)*troty(pi/2) % Example target transformation 
    motionHandler1.runRMRC(endTr1,time,deltaT,mesh_hSpatula,verticesSpatula);
    endTr2 = transl(-0.2,0.2,0.8)*troty(pi/2)*trotz(-pi/2); 
    motionHandler1.runRMRC(endTr2,time,deltaT,mesh_hSpatula,verticesSpatula);
    endTr3 = transl(-0.1,0.2,0.8)*troty(pi/2)*trotz(-pi/2);
    motionHandler1.runRMRC(endTr3,time,deltaT,mesh_hSpatula,verticesSpatula);
    endTr4 = transl(-0.1,0.2,1)*troty(pi/2)*trotz(-pi/2);
    


    % Place the pancake at an initial position after the dough
    mesh_hPancake = PlaceObject('pancake2.ply');
    verticesPancake = get(mesh_hPancake, 'Vertices');
    transformedVertices = [verticesPancake, ones(size(verticesPancake, 1), 1)] * transl([-0.1, 0.2, 0.7])';
    set(mesh_hPancake, 'Vertices', transformedVertices(:, 1:3)); % Update pancake's initial position
    
    
   %% we attach the pancake to the gripper
    motionHandler1.runRMRC(endTr4,time,deltaT,mesh_hSpatula,verticesSpatula,mesh_hPancake,verticesPancake);
   
    
    % Move robot away, leaving pancake at the drop position
    endTr5 = transl(-0.1, 0.2, 0.9) * troty(pi/2) * trotz(pi/2);
    motionHandler1.runRMRC(endTr5, time, deltaT, mesh_hSpatula, verticesSpatula,mesh_hPancake,verticesPancake );
    
    
    %Set new location for the pancake
    delete(mesh_hPancake); %% We delete the previous one
    mesh_hPancake = PlaceObject('pancake2.ply');
    verticesPancake = get(mesh_hPancake, 'Vertices')
    transformedVertices = [verticesPancake, ones(size(verticesPancake, 1), 1)] * transl([-0.3, -0.33, 0.57])';
    set(mesh_hPancake, 'Vertices', transformedVertices(:, 1:3)); % Update pancake's initial position


    endTr6 = transl(-0.1,0.2,0.8)*troty(pi/2)*trotz(pi/2);
    motionHandler1.runRMRC(endTr6,time,deltaT,mesh_hSpatula,verticesSpatula);
    endTr7 = transl(-0.2,0.2,0.9)*troty(pi/2)*trotz(-pi/2);
    motionHandler1.runRMRC(endTr7,time,deltaT,mesh_hSpatula,verticesSpatula);
    endTr8 = transl(-0.2,0.2,0.8)*troty(pi/2)*trotz(-pi/2); 
    motionHandler1.runRMRC(endTr8,time,deltaT,mesh_hSpatula,verticesSpatula);
    endTr9 = transl(-0.1,0.2,0.8)*troty(pi/2)*trotz(-pi/2);
    motionHandler1.runRMRC(endTr9,time,deltaT,mesh_hSpatula,verticesSpatula);
    endTr10 = transl(-0.1,0.2,0.9)*troty(pi/2)*trotz(-pi/2);
    motionHandler1.runRMRC(endTr10,time,deltaT,mesh_hSpatula,verticesSpatula);
    endTr11 = transl(-0.3,-0.2,0.8)*troty(pi/2)*trotz(-pi/2);
    motionHandler1.runRMRC(endTr11,time,deltaT,mesh_hSpatula,verticesSpatula);
    endTr12 = transl(-0.3,-0.2,0.8)*troty(pi/2)*trotz(-pi/2); 
    motionHandler1.runRMRC(endTr12,time,deltaT,mesh_hSpatula,verticesSpatula);
    endTr13 = transl(-0.3,-0.2,0.8)*troty(pi/2)*trotz(pi/2);
    motionHandler1.runRMRC(endTr13,time,deltaT,mesh_hSpatula,verticesSpatula);
    % Define the target transformation matrix
   
end
