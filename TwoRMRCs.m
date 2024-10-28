% Main Script: Initialize Robots and Run RMRC for Two Robots

% Initialize robots and define initial positions
robot1 = UR3e(transl(-0.5, 0.2, 0.5));
robot2 = Panda(transl(0.5, 0.5, 0));

% Define end transformations
endTr1 = transl(0, 0.4, 0.5) * troty(pi);
endTr2 = transl(0, 0.4, 0.7) * trotx(-pi/2);

% Get the initial transformation matrices for both robots
startTr1 = robot1.model.fkine(robot1.model.getpos()).T;
startTr2 = robot2.model.fkine(robot2.model.getpos()).T;

% Parameters for RMRC
time = 5;        % Duration of motion
deltaT = 0.05;   % Time step

% Call the function to run RMRC for both robots
disp('Running RMRC for both robots...');
runRMRCWithTwoRobots(robot1, robot2, startTr1, endTr1, startTr2, endTr2, time, deltaT);

%% Function to Perform RMRC for Two Robots
function runRMRCWithTwoRobots(robot1, robot2, startTr1, endTr1, startTr2, endTr2, time, deltaT)
    disp('Starting RMRC motion control for two robots...');
    steps = round(time / deltaT);
    s = lspb(0, 1, steps);  % Scalar for trajectory
    epsilon = 0.1;  % Manipulability threshold
    W = diag([1 1 1 0.1 0.1 0.1]);  % Weighting matrix for control

    % Initialize joint trajectories
    qMatrix1 = zeros(steps, robot1.model.n);  % Joint trajectory for Robot 1
    qMatrix2 = zeros(steps, robot2.model.n);  % Joint trajectory for Robot 2
    qMatrix1(1,:) = robot1.model.getpos();
    qMatrix2(1,:) = robot2.model.getpos();

    % SLERP setup for smooth rotation interpolation
    R_start1 = startTr1(1:3,1:3); R_end1 = endTr1(1:3,1:3);
    R_start2 = startTr2(1:3,1:3); R_end2 = endTr2(1:3,1:3);
    q_start1 = UnitQuaternion(R_start1); q_end1 = UnitQuaternion(R_end1);
    q_start2 = UnitQuaternion(R_start2); q_end2 = UnitQuaternion(R_end2);

    % Main RMRC loop for both robots
    for i = 1:steps-1
        % Debugging statement to check loop progress
        disp(['Running step: ', num2str(i), '/', num2str(steps-1)]);

        % Position and orientation interpolation for Robot 1
        pos_interp1 = transl((1 - s(i)) * startTr1(1:3,4)' + s(i) * endTr1(1:3,4)');
        q_interp1 = q_start1.interp(q_end1, s(i));
        T_desired1 = pos_interp1 * [q_interp1.R, [0; 0; 0]; 0 0 0 1];

        % Position and orientation interpolation for Robot 2
        pos_interp2 = transl((1 - s(i)) * startTr2(1:3,4)' + s(i) * endTr2(1:3,4)');
        q_interp2 = q_start2.interp(q_end2, s(i));
        T_desired2 = pos_interp2 * [q_interp2.R, [0; 0; 0]; 0 0 0 1];

        % Calculate current transformations and errors for Robot 1
        T_current1 = robot1.model.fkine(qMatrix1(i,:)).T;
        deltaX1 = T_desired1(1:3,4) - T_current1(1:3,4);
        Rdot1 = (1 / deltaT) * (T_desired1(1:3,1:3) - T_current1(1:3,1:3));
        S1 = Rdot1 * T_current1(1:3,1:3)';
        xdot1 = W * [(1 / deltaT) * deltaX1; S1(3,2); S1(1,3); S1(2,1)];

        % Calculate current transformations and errors for Robot 2
        T_current2 = robot2.model.fkine(qMatrix2(i,:)).T;
        deltaX2 = T_desired2(1:3,4) - T_current2(1:3,4);
        Rdot2 = (1 / deltaT) * (T_desired2(1:3,1:3) - T_current2(1:3,1:3));
        S2 = Rdot2 * T_current2(1:3,1:3)';
        xdot2 = W * [(1 / deltaT) * deltaX2; S2(3,2); S2(1,3); S2(2,1)];

        % Compute Jacobians and DLS for manipulability
        J1 = robot1.model.jacob0(qMatrix1(i,:));
        J2 = robot2.model.jacob0(qMatrix2(i,:));

        lambda1 = (sqrt(abs(det(J1 * J1'))) < epsilon) * ((1 - sqrt(abs(det(J1 * J1'))) / epsilon) * 5E-2);
        lambda2 = (sqrt(abs(det(J2 * J2'))) < epsilon) * ((1 - sqrt(abs(det(J2 * J2'))) / epsilon) * 5E-2);

        invJ1 = inv(J1' * J1 + lambda1 * eye(robot1.model.n)) * J1';
        invJ2 = inv(J2' * J2 + lambda2 * eye(robot2.model.n)) * J2';

        % Calculate joint velocities for both robots
        qdot1 = (invJ1 * xdot1)';
        qdot2 = (invJ2 * xdot2)';

        % Update joint positions for Robot 1 with joint limits enforced
        qMatrix1(i+1,:) = min(max(qMatrix1(i,:) + deltaT * qdot1, robot1.model.qlim(:,1)'), robot1.model.qlim(:,2)');

        % Update joint positions for Robot 2 with joint limits enforced
        qMatrix2(i+1,:) = min(max(qMatrix2(i,:) + deltaT * qdot2, robot2.model.qlim(:,1)'), robot2.model.qlim(:,2)');

        % Animate both robots every 10 steps for efficiency
        if mod(i, 1) == 0
            robot1.model.animate(qMatrix1(i+1,:));
            robot2.model.animate(qMatrix2(i+1,:));
            drawnow;
        end

        % Control animation speed
        pause(0.01);
    end
end

