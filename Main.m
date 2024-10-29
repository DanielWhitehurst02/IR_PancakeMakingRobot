function Main(app)

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
    %%Initialization Robot
    robot = UR3(transl(-0.5,0.2,0.5));
    % robot2 = UR3e(transl(0.5, 0.5, 0));
    leftFinger = Finger();  % Initialize left finger
    rightFinger = Finger2();  % Initialize right finger
    
    centerPoints = [0.0, 0.0, 0.05; % Base 
                    0.0, -0.01, 0.01; % Link 1 
                    0.125, 0.0, 0.125; % Link 2 
                    0.105, 0.0, 0.05; % Link 3 
                    0.0, 0.0, 0.01; % Link 4 
                    0.0, 0.0, 0.06; % Link 5 
                    0.0, 0.0, 0.0;]; % end-effector
    
    radii = [0.08, 0.09, 0.055;  
             0.075, 0.085, 0.075;
             0.175, 0.08, 0.085; 
             0.15, 0.06, 0.085; 
             0.04, 0.055, 0.065;
             0.04, 0.045, 0.125; 
             0.0, 0.0, 0.0;];

    robot2 = Panda(transl(0.9, 0.5, 0.5));


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
    
    leftFinger2 = Finger();  % Initialize left finger
    rightFinger2 = Finger2();  % Initialize right finger
    
    hold on
    

    %%
    %center = [-1.5, 0, 0.5];  % Center of the cube
    %rot = [pi/3, pi/4, 2*pi/7];  % Rotation in x, y, z
    %
    %% Call meshcube which also calls RectangularPrism and returns all the values
    %[cubePoints, vertex, faces, faceNormals,redHandle] = CollisionMesh(0.5, 0.5, rot, 0.02, center);
    %delete(redHandle);
    cubePoints = [];
    cubePoints2=[];
    %% Create Motion Handler
    % Set the total time and control frequency
    t = 5;  % Total time for movement (in seconds)
    deltaT = 0.05;  % Control step time (in seconds

    %% Joint Animation Loop (outside of isRunning loop)
    
    while ~app.isRunning

        if app.animationState == 0

        % disp('slider changed')
        smoothAnimationLoopUR3(app,robot,robot2);

        elseif app.animationState == 1
         % disp('box changed')
        setEndEffectorPositionUR3(app,robot,robot2)

        elseif app.animationState == 2
        velocityControlLoopUR3(app,robot,robot2)
        end
    end

    
    %% Calling Motion handler
    motionHandler1 = MotionHandlerWIthGripperAndObjects(robot, centerPoints, radii, cubePoints, leftFinger, rightFinger,app);

    motionHandler2 = MotionHandlerWIthGripperAndObjects(robot2, centerPoints2, radii2, cubePoints2, leftFinger2, rightFinger2,app,troty(-pi/2) * trotz(-pi),transl(0,0.15,0));

    
    %% Start the simulation loop
    i = 0;
    while app.isRunning
       i = i+1;
                    %% First
                endTr = transl(0.61,1.1,1.1); % Example target transformation
                time = 10;  % Total time for movement (in seconds)
                deltaT = 0.05;
                motionHandler2.runRMRC(endTr,time,deltaT);
                %% Second
                endTr = transl(0.61,1.1,0.95)*troty(pi/2)*trotx(pi);
                motionHandler2.runRMRC(endTr,time,deltaT);
                delete(mesh_hDough); %% We delete the previous one
                mesh_hDough = PlaceObject('DoughBottle2.ply');
                verticesDough = get(mesh_hDough, 'Vertices')
                transformedVertices = [verticesDough, ones(size(verticesDough, 1), 1)] * transl([0.6,1.25,0.82])';
                set(mesh_hDough, 'Vertices', transformedVertices(:, 1:3)); 

                motionHandler2.OpenOrCloseGrippers('close',50);
                %% Third
                endTr = transl(-0.2,0.8,0.95)*troty(pi/2);
                  % Total time for movement (in seconds)
                deltaT = 0.01;
                motionHandler2.runRMRC(endTr,time,deltaT,mesh_hDough,verticesDough);
                %% Fourth
                % Total time for movement (in seconds)
                deltaT = 0.05;
                endTr = transl(-0.2,0.2,0.95)*troty(pi/2);
                motionHandler2.runRMRC(endTr,time,deltaT,mesh_hDough,verticesDough);
                %% Fifth (Dough Pouring)
                  % Total time for movement (in seconds)
                deltaT = 0.1;
                endTr = transl(-0.2,0.2,0.95)*troty(-pi/2)*trotx(-pi/2);
                motionHandler2.runRMRC(endTr,time,deltaT,mesh_hDough,verticesDough);
                
                % Place the pancake at an initial position after the dough
                mesh_hPancake = PlaceObject('pancake2.ply');
                verticesPancake = get(mesh_hPancake, 'Vertices');
                transformedVertices = [verticesPancake, ones(size(verticesPancake, 1), 1)] * transl([0.1, 0.2, 0.7])';
                set(mesh_hPancake, 'Vertices', transformedVertices(:, 1:3)); % Update pancake's initial position
                
                 %% Sixth 
                  % Total time for movement (in seconds)
                deltaT = 0.05;
                endTr = transl(-0.2,0.2,0.95)*troty(pi/2);
                motionHandler2.runRMRC(endTr,time,deltaT,mesh_hDough,verticesDough);
                 %% Seventh
                endTr = transl(0.61,1,0.95)*troty(pi/2);
                 % Total time for movement (in seconds)
                deltaT = 0.05;
                motionHandler2.runRMRC(endTr,time,deltaT,mesh_hDough,verticesDough);
                 %% Eight
                endTr = transl(0.61,1,0.95)*troty(pi/2)*trotx(pi);
                 % Total time for movement (in seconds)
                deltaT = 0.05;
                motionHandler2.runRMRC(endTr,time,deltaT,mesh_hDough,verticesDough);
                motionHandler2.OpenOrCloseGrippers('open',50);
            
                 %% Ninth
                endTr = transl(0.61,0.9,0.95)*troty(pi/2)*trotx(pi);
                 % Total time for movement (in seconds)
                deltaT = 0.05;
                motionHandler2.runRMRC(endTr,time,deltaT);
                endTr0 = transl(-0.42,0.5,0.95)*trotx(-pi/2) % Example target transformation
                %% Pancake Animation (UR3)
                deltaT=0.1
                motionHandler1 = MotionHandlerWIthGripperAndObjects(robot, centerPoints, radii, cubePoints, leftFinger, rightFinger,app);
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
                
            
            
                
                
               %% we attach the pancake to the gripper
                motionHandler1.runRMRC(endTr4,time,deltaT,mesh_hSpatula,verticesSpatula,mesh_hPancake,verticesPancake);
               
                
                % Move robot away, leaving pancake at the drop position
                endTr5 = transl(-0.1, 0.2, 0.9) * troty(pi/2) * trotz(pi/2);
                motionHandler1.runRMRC(endTr5, time, deltaT, mesh_hSpatula, verticesSpatula,mesh_hPancake,verticesPancake );
                
                
                %Set new location for the pancake
                delete(mesh_hPancake); %% We delete the previous one
                mesh_hPancake = PlaceObject('pancake2.ply');
                verticesPancake = get(mesh_hPancake, 'Vertices')
                transformedVertices = [verticesPancake, ones(size(verticesPancake, 1), 1)] * transl([0.1, 0.2, 0.7])';
                set(mesh_hPancake, 'Vertices', transformedVertices(:, 1:3)); % Update pancake's initial position
            
            
                endTr6 = transl(-0.1,0.2,0.8)*troty(pi/2)*trotz(pi/2);
                motionHandler1.runRMRC(endTr6,time,deltaT,mesh_hSpatula,verticesSpatula,mesh_hPancake,verticesPancake);
                endTr7 = transl(-0.2,0.2,0.9)*troty(pi/2)*trotz(-pi/2);
                motionHandler1.runRMRC(endTr7,time,deltaT,mesh_hSpatula,verticesSpatula,mesh_hPancake,verticesPancake);
                endTr8 = transl(-0.2,0.2,0.8)*troty(pi/2)*trotz(-pi/2); 
                motionHandler1.runRMRC(endTr8,time,deltaT,mesh_hSpatula,verticesSpatula,mesh_hPancake,verticesPancake);
                endTr9 = transl(-0.1,0.2,0.8)*troty(pi/2)*trotz(-pi/2);
                motionHandler1.runRMRC(endTr9,time,deltaT,mesh_hSpatula,verticesSpatula,mesh_hPancake,verticesPancake);
                endTr10 = transl(-0.1,0.2,0.9)*troty(pi/2)*trotz(-pi/2);
                motionHandler1.runRMRC(endTr10,time,deltaT,mesh_hSpatula,verticesSpatula,mesh_hPancake,verticesPancake);
                endTr11 = transl(-0.3,-0.2,0.8)*troty(pi/2)*trotz(-pi/2);
                motionHandler1.runRMRC(endTr11,time,deltaT,mesh_hSpatula,verticesSpatula,mesh_hPancake,verticesPancake);
                endTr12 = transl(-0.3,-0.2,0.8)*troty(pi/2)*trotz(-pi/2); 
                motionHandler1.runRMRC(endTr12,time,deltaT,mesh_hSpatula,verticesSpatula,mesh_hPancake,verticesPancake);
                endTr13 = transl(-0.1,-0.2,0.8)*troty(pi/2)*trotz(pi/2);
                motionHandler1.runRMRC(endTr13,time,deltaT,mesh_hSpatula,verticesSpatula,mesh_hPancake,verticesPancake);
                
                %Set new location for the pancake
                delete(mesh_hPancake); %% We delete the previous one
                mesh_hPancake = PlaceObject('pancake2.ply');
                verticesPancake = get(mesh_hPancake, 'Vertices')
                transformedVertices = [verticesPancake, ones(size(verticesPancake, 1), 1)] * transl([-0.3, -0.33, 0.57])';
                set(mesh_hPancake, 'Vertices', transformedVertices(:, 1:3)); % Update pancake's initial position

      
                % Define the target transformation matrix
                 %% Tenth (Syrup)
                endTr = transl(0.8,1.1,0.95)*troty(pi/2)*trotx(pi);
                t = 10;  % Total time for movement (in seconds)
                deltaT = 0.05;
                motionHandler2.runRMRC(endTr,time,deltaT);
                motionHandler2.OpenOrCloseGrippers('close',50);
               
                %% Eleventh
                 endTr = transl(0.8,0,0.95);
                t = 10;  % Total time for movement (in seconds)
                deltaT = 0.05;
                motionHandler2.runRMRC(endTr,time,deltaT,mesh_hBlue,verticesBlue);
              
            
                %% Twelfth
                endTr = transl(0.8,-0.9,0.95);
                t = 10;  % Total time for movement (in seconds)
                deltaT = 0.05;
                motionHandler2.runRMRC(endTr,time,deltaT,mesh_hBlue,verticesBlue);
                %% Thirteenth
                endTr = transl(0.8,-0.9,0.95)*troty(-pi/2);
                t = 10;  % Total time for movement (in seconds)
                deltaT = 0.05;
                motionHandler2.runRMRC(endTr,time,deltaT,mesh_hBlue,verticesBlue);
            
                %% Fourteenth
                endTr = transl(0.8,0,0.95)*troty(pi/2);
                t = 10;  % Total time for movement (in seconds)
                deltaT = 0.05;
                motionHandler2.runRMRC(endTr,time,deltaT,mesh_hBlue,verticesBlue);
                
                %% Fifteenth
                 endTr = transl(0.8,0.5,0.95)*troty(pi/2)*trotx(pi);
                t = 10;  % Total time for movement (in seconds)
                deltaT = 0.05;
                motionHandler2.runRMRC(endTr,time,deltaT);
                
               
                %% Fifteenth
                 endTr = transl(0.8,1,0.95)*troty(pi/2)*trotx(pi);
                t = 10;  % Total time for movement (in seconds)
                deltaT = 0.05;
                motionHandler2.runRMRC(endTr,time,deltaT);
                motionHandler2.OpenOrCloseGrippers('open',50);
                end
            
                % If the loop has exited
                disp('Simulation Stopped.');

                % Enter an infinite "pause" state to keep the loop active without re-running the animation
                while true
                    pause(0.1);  % Adjust pause duration as needed to reduce CPU load
                end
end



%% Smooth Joint Angle Animation
function smoothAnimationLoopUR3(app, robot,robot2)
 
   % Access the current values of the sliders for joint angles
   targetJointAngles = [app.ur3_joint1, app.ur3_joint2, app.ur3_joint3, ...
                        app.ur3_joint4, app.ur3_joint5, app.ur3_joint6];

   targetJointAngles2 = [app.panda_joint1, app.panda_joint2, app.panda_joint3, ...
                        app.panda_joint4, app.panda_joint5, app.panda_joint6, app.panda_joint7];
   
   % Get the current joint angles of the robot
   currentJointAngles = robot.model.getpos();
   currentJointAngles2 = robot2.model.getpos();
   % Number of interpolation steps for smooth animation
   steps = 5;
  
   % Interpolate joint angles between current and target
   jointTrajectory = zeros(steps, length(targetJointAngles));

   jointTrajectory2 = zeros(steps, length(targetJointAngles2));

   for i = 1:length(targetJointAngles)
       jointTrajectory(:, i) = linspace(currentJointAngles(i), targetJointAngles(i), steps);
   end

   for i = 1:length(targetJointAngles2)
       jointTrajectory2(:, i) = linspace(currentJointAngles2(i), targetJointAngles2(i), steps);
   end

   % Animate the robot smoothly by iterating through the interpolated joint angles
   for i = 1:steps
       % Update the robot's pose based on the interpolated joint angles
       robot.model.animate(jointTrajectory(i, :));
       robot2.model.animate(jointTrajectory2(i, :));
  
       % Pause for a short time to allow for smooth updating
       pause(0.05);  % Adjust the pause duration for smoother animation
   end

    
    endTrs = robot.model.fkine(targetJointAngles);
    endTrs1 = endTrs.T;

    app.ur3_endposX= endTrs1(1,4);
    app.ur3_endposY = endTrs1(2,4);
    app.ur3_endposZ = endTrs1(3,4);

    endTrs2 = robot2.model.fkine(targetJointAngles2);
    endTrs3 = endTrs2.T;

    app.panda_endposX= endTrs3(1,4);
    app.panda_endposY = endTrs3(2,4);
    app.panda_endposZ = endTrs3(3,4);

    %Set Display to values
    app.XEditField.Value = app.ur3_endposX;
    app.YEditField.Value = app.ur3_endposY;
    app.ZEditField.Value = app.ur3_endposZ;

    app.XEditField_2.Value = app.panda_endposX;
    app.YEditField_2.Value = app.panda_endposY;
    app.ZEditField_2.Value = app.panda_endposZ;
  
   % Pause for a short time to allow for the sliders to update values
   pause(0.1);  % Adjust the pause duration as needed
end

%% Velocity Control End effector
function velocityControlLoopUR3(app, robot,robot2)
   % Initialize the velocity gains
   Kv = 0.2;  % Linear velocity gain
   Kw = 0.5;  % Angular velocity gain
   dt = 0.05;  % Time step for updating the simulation
  
   % Get linear and angular velocities from the App Designer inputs
   vx = Kv * app.ur3_velocityX;
   vy = Kv * app.ur3_velocityY;
   vz = Kv * app.ur3_velocityZ;
   wx = Kw * app.ur3_angularX;
   wy = Kw * app.ur3_angularY;
   wz = Kw * app.ur3_angularZ;
   % Create the combined velocity vector (6x1 for linear and angular velocities)
   dx = [vx; vy; vz; wx; wy; wz];
  
   % Get the current joint angles and calculate the Jacobian
   q = robot.model.getpos();
   J = robot.model.jacob0(q);
  
   % Calculate joint velocities using the Jacobian pseudoinverse
   dq = pinv(J) * dx;
  
   % Update the joint angles based on the velocities
   q_new = q + dq' * dt;

   % app.Link1Slider.Value =  rad2deg(q_new(1));
   % app.Link2Slider.Value =  rad2deg(q_new(2));
   % app.Link3Slider.Value =  rad2deg(q_new(3));
   % app.Link4Slider.Value =  rad2deg(q_new(4));
   % app.Link5Slider.Value =  rad2deg(q_new(5));
   % app.Link6Slider.Value =  rad2deg(q_new(6));
  
   % Animate the robot with the updated joint angles
   robot.model.animate(q_new);
  
   % Pause for a short time before updating again
   pause(0.05);  % Adjust the pause duration as needed
  
end


function setEndEffectorPositionUR3(app, robot,robot2)
    % Get the target end-effector position from the App Designer inputs
    posX = app.ur3_endposX;
    posY = app.ur3_endposY;
    posZ = app.ur3_endposZ;

    posX2 = app.panda_endposX;
    posY2 = app.panda_endposY;
    posZ2 = app.panda_endposZ;
    
    % Define the desired transformation matrix using transl (translation)
    targetTr = transl(posX, posY, posZ);
    targetTr2 = transl(posX2, posY2, posZ2);
    
    % Get the current joint angles of the robot
    currentJointAngles = robot.model.getpos();
    currentJointAngles2 = robot2.model.getpos();
    
    % Use inverse kinematics (ikcon) to compute joint angles for the desired position
    newJointAngles = robot.model.ikcon(targetTr, currentJointAngles);
    newJointAngles2 = robot2.model.ikcon(targetTr2, currentJointAngles2);
    
    
    app.Link1Slider.Value =  rad2deg(newJointAngles(1));
    app.Link2Slider.Value =  rad2deg(newJointAngles(2));
    app.Link3Slider.Value =  rad2deg(newJointAngles(3));
    app.Link4Slider.Value =  rad2deg(newJointAngles(4));
    app.Link5Slider.Value =  rad2deg(newJointAngles(5));
    app.Link6Slider.Value =  rad2deg(newJointAngles(6));
    % 
    app.Link1Slider_2.Value =  rad2deg(newJointAngles2(1));
    app.Link2Slider_2.Value =  rad2deg(newJointAngles2(2));
    app.Link3Slider_2.Value =  rad2deg(newJointAngles2(3));
    app.Link4Slider_2.Value =  rad2deg(newJointAngles2(4));
    app.Link5Slider_2.Value =  rad2deg(newJointAngles2(5));
    app.Link6Slider_2.Value =  rad2deg(newJointAngles2(6));
    app.Link7Slider.Value =  rad2deg(newJointAngles2(7));

    % Animate the robot to move to the desired position
    robot.model.animate(newJointAngles);
    robot2.model.animate(newJointAngles2);
    
    % Optional: display the new joint angles (for debugging or display purposes)
    disp('Updated Joint Angles for Desired End-Effector Position:');
    % disp(newJointAngles);
    pause(0.5);
end
