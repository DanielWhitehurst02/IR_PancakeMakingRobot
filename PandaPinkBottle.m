robot = Panda(transl(0.9,0.5,0.5));
a=robot.model.fkine(robot.model.getpos()).T;
%% Second animation
path = RMRC(robot);  % Instantiate RMRC for the UR3 robot

% Initial joint configuration (all joints at zero)
startTr=robot.model.fkine(robot.model.getpos()).T;
endTr = transl(0.61,1.1,1.1); % Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr, endTr, t, deltaT);
 %% Third animation   

% Define the target transformation matrix
startTr=robot.model.fkine(robot.model.getpos()).T;
endTr = transl(0.61,1.1,0.95)*troty(pi/2)*trotx(pi);% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr, endTr, t, deltaT);
%% Fourth animation    
% Define the target transformation matrix
startTr=robot.model.fkine(robot.model.getpos()).T;
endTr = transl(-0.2,0.2,1)*troty(pi/2);% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.01;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr, endTr, t, deltaT);
%% fifth animation
% Define the target transformation matrix
startTr=robot.model.fkine(robot.model.getpos()).T;
endTr = transl(-0.2,0.2,1)*troty(-pi/2);% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr, endTr, t, deltaT);

%% sixth animation
% Define the target transformation matrix
startTr=robot.model.fkine(robot.model.getpos()).T;
endTr = transl(-0.2,0.2,1)*troty(-pi/2)*trotx(-pi/2);% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.1;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr, endTr, t, deltaT);

%% seventh animation
% Define the target transformation matrix
startTr=robot.model.fkine(robot.model.getpos()).T;
endTr = transl(-0.2,0.2,1)*troty(pi/2);% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.1;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr, endTr, t, deltaT);
%% Eight Animation
path = RMRC(robot);  % Instantiate RMRC for the UR3 robot

% Initial joint configuration (all joints at zero)
startTr=robot.model.fkine(robot.model.getpos()).T;
endTr = transl(0.61,0.9,0.85)*troty(pi/2);
% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr, endTr, t, deltaT);

%%  Ninth Animation

% Initial joint configuration (all joints at zero)
startTr1=robot.model.fkine(robot.model.getpos()).T;
endTr1 = transl(0.61,0.9,1);
% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr1, endTr1, t, deltaT);
%% Tenth animation
% Initial joint configuration (all joints at zero)
startTr1=robot.model.fkine(robot.model.getpos()).T;
endTr1 = transl(0.61,1.1,1);
% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr1, endTr1, t, deltaT);
%% Eleventh animation
% Initial joint configuration (all joints at zero)
startTr1=robot.model.fkine(robot.model.getpos()).T;
endTr1 = transl(0.61,1.2,1);
% Example target transformation

% Set the total time and control frequency
t = 20;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr1, endTr1, t, deltaT);
%% Eleventh animation
% Initial joint configuration (all joints at zero)
startTr1=robot.model.fkine(robot.model.getpos()).T;
endTr1 = transl(0.61,1.2,1);
% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr1, endTr1, t, deltaT);
%% twelefth animation
% Initial joint configuration (all joints at zero)
startTr1=robot.model.fkine(robot.model.getpos()).T;


endTr1 = transl(1.05,0.7,1);
% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.1;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr1, endTr1, t, deltaT);
a= robot.model.getpos;
%% Thirteenth Animation
startTr1=robot.model.fkine(robot.model.getpos()).T;


endTr1 = transl(1.05,1.1,1);
% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.1;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr1, endTr1, t, deltaT);


%% fourteenth Animation
startTr1=robot.model.fkine(robot.model.getpos()).T;


endTr1 = transl(1.05,1.1,1.05)*troty(pi/2);
% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.2;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr1, endTr1, t, deltaT);


%% fifteenth Animation
startTr1=robot.model.fkine(robot.model.getpos()).T;


endTr1 = transl(0.8,-0.45,0.8)*troty(pi/2);
% Example target transformation

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.2;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr1, endTr1, t, deltaT);

%% fifteenth Animation
startTr1=robot.model.fkine(robot.model.getpos()).T;


endTr1 = transl(0.8,-0.9,0.8)*troty(pi/2);
% Example target transformation

% Set the total time and control frequency
t = 20;  % Total time for movement (in seconds)
deltaT = 0.2;  % Control step time (in seconds)

% Call the RMRC method to move the UR3
path.ResolvedMotionRateControl(startTr1, endTr1, t, deltaT);

%% Sixteenth animation

pick=[0.9,1.2,0.95];
q = robot.model.getpos();
steps=50;

    
        
      % Calculate pick-up and drop joint angles based on current robot configuration
        qpick = robot.model.ikine(transl(pick), 'q0', q, 'mask', [1,1,1,0,0,1]);
        
     
        qForwardMatrix = jtraj(robot.model.getpos(), qpick, steps);
        for i = 1:steps
            qcurrent = qForwardMatrix(i,:);
            robot.model.animate(qcurrent);
            T_robot = robot.model.fkine(qcurrent).T; % Get the robot's end-effector transformation matrix
            pause(0.05)
        end

        %% Sixteenth animation

pick=[1.05,1.2,0.95];
q = robot.model.getpos();
steps=50;

    
        
      % Calculate pick-up and drop joint angles based on current robot configuration
        qpick = robot.model.ikine(transl(pick), 'q0', q, 'mask', [1,1,1,0,0,1]);
        
     
        qForwardMatrix = jtraj(robot.model.getpos(), qpick, steps);
        for i = 1:steps
            qcurrent = qForwardMatrix(i,:);
            robot.model.animate(qcurrent);
            T_robot = robot.model.fkine(qcurrent).T; % Get the robot's end-effector transformation matrix
            pause(0.05)
        end
 %% Seventeenth animation
% Initial joint configuration (all joints at zero)
startTr = robot.model.fkine(robot.model.getpos()).T;

% Define the target transformation matrix: 90-degree rotation around y-axis
endTr = startTr * troty(pi/2); % This rotates the end-effector by 90 degrees around the y-axis

% Set the total time and control frequency
t = 10;  % Total time for movement (in seconds)
deltaT = 0.05;  % Control step time (in seconds)

% Call the RMRC method to rotate the end-effector
path.ResolvedMotionRateControl(startTr, endTr, t, deltaT);