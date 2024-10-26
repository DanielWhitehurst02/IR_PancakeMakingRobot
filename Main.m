function Main(app)
   
%%
%% Clear workspace

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
%%

    
    
    
    %%Initialization Robot
    robot = UR3e(transl(-0.5,0.2,0.5));
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

    robot2 = Panda(transl(0.5, 0.5, 0));

    robot2.CreateModel()
    centerPoints_panda = [
        -0.025, 0.0, 0.06;
        0.0, 0.0975, -0.035;
        0.0, 0.03, 0.075;
        -0.075, -0.085, 0.0;
        0.05, 0.0, 0.035;
        0.0, 0.125, 0.025;
        -0.05, 0.025, 0.0;
    ];
    radii_Panda = [
        0.135, 0.125, 0.075;
        0.1, 0.175, 0.1;
        0.125, 0.12, 0.125;
        0.075, 0.075, 0.095;
        0.125, 0.125, 0.095;
        0.085, 0.15, 0.105;
        0.110, 0.1, 0.085;
    ];
    
    
    
    hold on
    

    %%
    center = [-1.5, 0, 0.5];  % Center of the cube
    rot = [pi/3, pi/4, 2*pi/7];  % Rotation in x, y, z
    
    % Call meshcube which also calls RectangularPrism and returns all the values
    [cubePoints, vertex, faces, faceNormals] = CollisionMesh(0.5, 0.5, rot, 0.02, center);

    %%
    % Place the object
    mesh_h = PlaceObject('BlueSyrupBottle.ply');
    vertices = get(mesh_h, 'Vertices');
    transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transl([0, 0.3, 0.5])';
    set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update object position
    
    %% Create Motion Handler
    % Initialize MotionHandler with grippers
    
    

    % Set the total time and control frequency
    t = 5;  % Total time for movement (in seconds)
    deltaT = 0.05;  % Control step time (in seconds
    q0 = zeros(1, robot.model.n);
    steps = 50;
    % Get the current end-effector position using forward kinematics
    startTr_struct = robot.model.fkine(q0);  % Get the forward kinematics
    if isobject(startTr_struct)
        startTr = startTr_struct.T;  % Extract the .T property if it's an object
    else
        startTr = startTr_struct;    % Directly use if it's already a matrix
    end
    
    startTr = transl(0, 0.3, 0.5);
    endTr = transl(0.2, -0.3, 0.5) * troty(-pi/2);
    endTr2 = transl(-0.3, -0.3, 0.5) * troty(-pi/2);

    endTr3 = transl(0.2, 0.6, 0.5) * troty(-pi/2);
    endTr4 = transl(-0.3, 0.6, 0.5) * troty(-pi/2);

    % goalMat = {(transl(0.2, -0.3, 0.5) * troty(-pi/2)),
               % (transl(0.3, -0.3, 0.5) * troty(0))}
     goalMat = {endTr,
                endTr2}
    
    disp(goalMat{1})
    %disp(size(goalMat,1))



    %% Joint Animation Loop (outside of isRunning loop)
    % This loop runs before `isRunning` and continuously updates joint angles.
    %while ~app.isRunning
    %    % Access the current values of the sliders for joint angles
    %    jointAngles = [app.ur3_joint1, app.ur3_joint2, app.ur3_joint3, ...
    %                   app.ur3_joint4, app.ur3_joint5, app.ur3_joint6];
    %    
    %    % Update the robot's pose based on the current joint angles
    %    robot.model.animate(jointAngles);
    %    
    %    % Pause for a short time to allow for smooth updating
    %    pause(0.1);  % Adjust the pause duration to control how fast the loop updates
    %end
    
    %smoothAnimationLoop(app, robot);
    %velocityControlLoop(app, robot);
    %setEndEffectorPosition(app, robot);
    
    while ~app.isRunning
        smoothAnimationLoopUR3(app,robot);
    end

    
    %% Calling Motion handler
    motionHandler = MotionHandlerWIthGripperAndObjects(robot, centerPoints, radii, cubePoints, leftFinger, rightFinger,app);

    % motionHandler2 = MotionHandlerWIthGripperAndObjects(robot2, centerPoints, radii, cubePoints, leftFinger, rightFinger,app);

    % motion
    
    % s = motionHandler.setGoals(goalMat,50);
   
    
    % disp(s{1})

    %% Start the simulation loop
    i = 0;
    while app.isRunning
       i = i+1;
       % goalreached = motionHandler.iterateRMRC(i,endTr,5,0.05,mesh_h,vertices);
       % motionHandler.runRMRC(startTr,endTr,5,0.05,mes)
        % motionHandler.runRMRC( endTr,5,0.05,mesh_h,vertices);
        motionHandler.runtwoRMRC(robot2,endTr,endTr3,5,0.05,mesh_h,vertices);
        % disp(goalreached)
        pause(0.1)
        motionHandler.runtwoRMRC(robot2,endTr2,endTr4,5,0.05,mesh_h,vertices);
        % Pause for a short time (simulate delay between iterations)
        pause(0.1);
        hold on;

    end

    % If the loop has exited
    disp('Simulation Stopped.');
end



%% Smooth Joint Angle Animation
function smoothAnimationLoopUR3(app, robot)
 
   % Access the current values of the sliders for joint angles
   targetJointAngles = [app.ur3_joint1, app.ur3_joint2, app.ur3_joint3, ...
                        app.ur3_joint4, app.ur3_joint5, app.ur3_joint6];
  
   % Get the current joint angles of the robot
   currentJointAngles = robot.model.getpos();
  
   % Number of interpolation steps for smooth animation
   steps = 5;
  
   % Interpolate joint angles between current and target
   jointTrajectory = zeros(steps, length(targetJointAngles));
   for i = 1:length(targetJointAngles)
       jointTrajectory(:, i) = linspace(currentJointAngles(i), targetJointAngles(i), steps);
   end
  
   % Animate the robot smoothly by iterating through the interpolated joint angles
   for i = 1:steps
       % Update the robot's pose based on the interpolated joint angles
       robot.model.animate(jointTrajectory(i, :));
  
       % Pause for a short time to allow for smooth updating
       pause(0.05);  % Adjust the pause duration for smoother animation
   end
  
   % Pause for a short time to allow for the sliders to update values
   pause(0.1);  % Adjust the pause duration as needed
end

%% Velocity Control End effector
function velocityControlLoopUR3(app, robot)
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
  
   % Animate the robot with the updated joint angles
   robot.model.animate(q_new);
  
   % Pause for a short time before updating again
   pause(0.05);  % Adjust the pause duration as needed
  
end


function setEndEffectorPositionUR3(app, robot)
    % Get the target end-effector position from the App Designer inputs
    posX = app.ur3_endposX;
    posY = app.ur3_endposY;
    posZ = app.ur3_endposZ;
    
    % Define the desired transformation matrix using transl (translation)
    targetTr = transl(posX, posY, posZ);
    
    % Get the current joint angles of the robot
    currentJointAngles = robot.model.getpos();
    
    % Use inverse kinematics (ikcon) to compute joint angles for the desired position
    newJointAngles = robot.model.ikcon(targetTr, currentJointAngles);
    
    % Animate the robot to move to the desired position
    robot.model.animate(newJointAngles);
    
    % Optional: display the new joint angles (for debugging or display purposes)
    disp('Updated Joint Angles for Desired End-Effector Position:');
    disp(newJointAngles);
end
