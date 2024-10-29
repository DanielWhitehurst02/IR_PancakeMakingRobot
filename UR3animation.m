%% First animation
robot = UR3e(transl(-0.5,0.2,0.5));


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

endTr = transl([-0.42,0.5,0.95])*trotx(-pi/2) % Example target transformation

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

%% Tenth Animation

I=robot.model.fkine(robot.model.getpos()).T;
endTr9 = transl(-0.1,0.2,0.8)*troty(pi/2); % Example target transformation

% Set the total time and control frequency
path.ResolvedMotionRateControl(I, endTr9, t, deltaT);
%% Eleveth Animation
J=robot.model.fkine(robot.model.getpos()).T;
endTr10 = transl(-0.1,0.2,0.9)*troty(pi/2); % Example target transformation

% Set the total time and control frequency

path.ResolvedMotionRateControl(J, endTr10, t, deltaT);

%% Twelve Animation
k=robot.model.fkine(robot.model.getpos()).T;
endTr11 = transl(-0.3,-0.2,0.8)*troty(pi/2)*trotx(pi/2); % Example target transformation

% Set the total time and control frequency

path.ResolvedMotionRateControl(k, endTr11, t, deltaT);
%% Thirteenth Animation
L=robot.model.fkine(robot.model.getpos()).T;
endTr12 = transl(-0.3,-0.2,0.8)*troty(pi/2)*trotx(pi/2)*trotz(-pi); % Example target transformation

% Set the total time and control frequency

path.ResolvedMotionRateControl(L, endTr12, t, deltaT);
%% Fourteenth animation
M=robot.model.fkine(robot.model.getpos()).T;
endTr13 = transl(-0.3,-0.2,0.6)*troty(pi/2)*trotx(pi/2)*trotz(-pi); % Example target transformation

% Set the total time and control frequency
deltaT = 0.2;
path.ResolvedMotionRateControl(M, endTr13, t, deltaT);
%% Fifteenth animation
N=robot.model.fkine(robot.model.getpos()).T;
endTr14 = transl(-0.4,-0.1,0.8)*troty(pi/2)*trotx(pi/2)*trotz(-pi); % Example target transformation

% Set the total time and control frequency
deltaT = 0.05;
path.ResolvedMotionRateControl(N, endTr14, t, deltaT);