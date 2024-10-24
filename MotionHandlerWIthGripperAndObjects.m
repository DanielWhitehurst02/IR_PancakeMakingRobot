classdef MotionHandlerWIthGripperAndObjects
    properties
        collisionHandler  % CollisionEllipsoid instance for collision detection
        robot  % Robot model (e.g., UR3)
        RMRC  % RMRC motion planner
        obstaclePoints  % Points representing obstacles
        running  % Boolean indicating if the motion is running
        currentGoal  % Current target transformation
        prevGoal  % Previous goal for resuming
        prevtime  % Previous time for resuming
        prevdeltaT  % Previous deltaT for resuming
        leftFinger  % Left gripper instance
        rightFinger % Right gripper instance
        grippersPlotted  % Boolean flag to check if grippers are plotted
        gripperRotation  % Rotation matrix for gripper attachment
        gripperTranslation  % Translation vector for gripper attachment
        app % The emergency stop flag passed from the app
    end

    methods
        % Constructor
        function self = MotionHandlerWIthGripperAndObjects(robot, centerPoints, radii, obstaclePoints, leftFinger, rightFinger,app, varargin)
            self.robot = robot;
            self.obstaclePoints = obstaclePoints;
            self.collisionHandler = CollisionEllipsoid(robot, centerPoints, radii);  % Initialize collision handler
            self.RMRC = RMRC(robot);
            self.running = true;
            self.grippersPlotted = false;  % Initially, grippers are not plotted
            self.app = app; % Initialize eStop


            % Initialize the grippers and attach them to the robot
            self.leftFinger = leftFinger;
            self.rightFinger = rightFinger;

            % Set default gripper rotation and translation if not provided
            if length(varargin) >= 1 && ~isempty(varargin{1})
                self.gripperRotation = varargin{1};  % Rotation matrix passed as argument
            else
                self.gripperRotation = trotx(pi/2);  % Default rotation
            end
            
            if length(varargin) >= 2 && ~isempty(varargin{2})
                self.gripperTranslation = varargin{2};  % Translation vector passed as argument
            else
                self.gripperTranslation = transl(0, 0, 0);  % Default no translation
            end

            self.attachGrippersToEndEffector();
        end

        % Attach the grippers to the robot's end-effector
        function attachGrippersToEndEffector(self)
            disp('Attaching grippers to the robot...');
            endEffectorTr = self.robot.model.fkine(self.robot.model.getpos);

            % Apply the user-defined or default rotation and translation
            self.leftFinger.model.base = endEffectorTr.T * self.gripperRotation * self.gripperTranslation;
            self.rightFinger.model.base = endEffectorTr.T * self.gripperRotation * self.gripperTranslation;

            % Plot the grippers only if they haven't been plotted yet
            if ~self.grippersPlotted
                self.leftFinger.PlotAndColourRobot();
                self.rightFinger.PlotAndColourRobot();
                self.grippersPlotted = true;  % Set the flag to true after plotting
            end

            disp('Grippers successfully attached.');
        end

        % Update gripper positions without re-plotting
        function updateGrippersPosition(self)
            endEffectorTr = self.robot.model.fkine(self.robot.model.getpos);

            % Update the left and right gripper positions with the stored rotation and translation
            self.leftFinger.model.base = endEffectorTr.T * self.gripperRotation * self.gripperTranslation;
            self.rightFinger.model.base = endEffectorTr.T * self.gripperRotation * self.gripperTranslation;

            % Animate the grippers to reflect their new positions
            self.leftFinger.model.animate(self.leftFinger.model.getpos);
            self.rightFinger.model.animate(self.rightFinger.model.getpos);
        end
    
        % Open or Close the Grippers
        function OpenOrCloseGrippers(self, action, steps)
            % Define joint angles for gripper open and close positions
            openAnglesLeft = [0, 0, 0];  % Open angles for the left finger
            openAnglesRight = [0, 0, 0]; % Open angles for the right finger
            closedAnglesLeft = [deg2rad(5), deg2rad(5), deg2rad(5)];  % Close angles for the left finger
            closedAnglesRight = [-deg2rad(5), -deg2rad(5), -deg2rad(5)];  % Close angles for the right finger
        
            % Determine the trajectory based on action (open or close)
            if strcmp(action, 'open')
                qMatrixLeft = jtraj(closedAnglesLeft, openAnglesLeft, steps);   % Open trajectory for left finger
                qMatrixRight = jtraj(closedAnglesRight, openAnglesRight, steps); % Open trajectory for right finger
            elseif strcmp(action, 'close')
                qMatrixLeft = jtraj(openAnglesLeft, closedAnglesLeft, steps);   % Close trajectory for left finger
                qMatrixRight = jtraj(openAnglesRight, closedAnglesRight, steps); % Close trajectory for right finger
            else
                error('Unknown action. Use "open" or "close"');
            end
        
            % Animate gripper opening/closing
            for i = 1:steps
                self.leftFinger.model.animate(qMatrixLeft(i, :));  % Animate the left finger
                self.rightFinger.model.animate(qMatrixRight(i, :));  % Animate the right finger
                pause(0.01);  % Small pause for smoother rendering
            end
        end


        % Update ellipsoid centers dynamically based on current robot configuration
        function updateEllipsoidCenters(self)
            % Compute forward kinematics and automatically update ellipsoid centers
            self.collisionHandler.drawEllipsoids();  % This will update ellipsoid positions based on fkine internally
        end

        %% Check for collisions and stop if detected
        %function checkForCollisionAndStop(self)
        %    self.updateEllipsoidCenters();  % Update ellipsoid positions based on the current robot state
        %    collision = self.collisionHandler.detectCollision(self.obstaclePoints);  % Check for collisions
        %    if collision
        %        disp('Collision detected! Stopping robot.');
        %        %self.eStop();  % Stop the robot if a collision is detected
        %    end
        %end

        % Check for collisions and pause if detected, resume automatically when resolved
        function checkForCollisionAndPause(self)
            self.updateEllipsoidCenters();  % Update ellipsoid positions based on the current robot state
            collision = self.collisionHandler.detectCollision(self.obstaclePoints);  % Check for collisions
            if collision
                disp('Collision detected! Pausing robot motion...');
                self.running = false;  % Stop the robot's motion
        
                % Pause the loop until the collision is cleared
                while collision  % Stay in this loop until the collision is cleared
                    pause(0.1);  % Add a small delay to avoid overloading the system
                    self.updateEllipsoidCenters();  % Keep updating the ellipsoid centers
                    collision = self.collisionHandler.detectCollision(self.obstaclePoints);  % Recheck for collisions
        
                    if ~collision  % If the collision is cleared
                        disp('Collision resolved. Resuming robot motion...');
                        self.running = true;  % Resume robot motion
                    end
                end
            end
        end

        %% Emergency stop function
        %function eStop(self)
        %    self.running = false;  % Stop the robot motion
        %    self.prevGoal = self.currentGoal;  % Save the current goal for resuming
        %end

        % Check for eStop and pause if necessary
        function checkForEStopAndPause(self)
            while self.app.eStop  % Dynamically check app.eStop value
                disp('eStop is active, pausing motion...');
                pause(0.1);  % Small pause while checking for eStop status
            end
        end

        % Resume the robot motion after stopping
        function resume(self)
            self.running = true;
            self.runRMRC(self.robot.model.fkine(self.robot.model.getpos), self.prevGoal, self.prevtime, self.prevdeltaT);
        end

        % Run RMRC with gripper synchronization
        function runRMRC(self, startTr, endTr, time, deltaT, varargin)
            self.running = true;
            self.currentGoal = endTr;
            self.prevdeltaT = deltaT;
            self.prevtime = time;
        
            [s, ~, steps] = self.RMRC.ResolvedMotionRateControlPath(startTr, endTr, time, deltaT);
        
            epsilon = 0.1;  % Manipulability threshold
            W = diag([1 1 1 0.1 0.1 0.1]);  % Weighting matrix for control
        
            % Check if mesh_h and vertices were provided
            if nargin > 5
                mesh_h = varargin{1};
                vertices = varargin{2};
            else
                mesh_h = [];
                vertices = [];
            end
        
            % Perform RMRC loop
            for i = 1:steps-1

                self.checkForEStopAndPause();  % This will pause the loop if eStop is active

                % Interpolate position and orientation
                pos_interp = transl((1-s(i))*startTr(1:3,4)' + s(i)*endTr(1:3,4)');
                q_interp = UnitQuaternion(startTr(1:3,1:3)).interp(UnitQuaternion(endTr(1:3,1:3)), s(i));  % SLERP for orientation
                R_interp = q_interp.R;
                R_homogeneous = [R_interp, [0; 0; 0]; 0 0 0 1];  % 4x4 matrix
        
                % Desired transformation matrix
                T_desired = pos_interp * R_homogeneous;
        
                % Get current transformation matrix
                T_current_struct = self.robot.model.fkine(self.RMRC.qMatrix(i,:));
                if isobject(T_current_struct)
                    T_current = T_current_struct.T;
                else
                    T_current = T_current_struct;
                end
        
                % Compute position and orientation errors
                deltaX = T_desired(1:3,4) - T_current(1:3,4);
                Rd = T_desired(1:3, 1:3);
                Ra = T_current(1:3, 1:3);
                Rdot = (1 / deltaT) * (Rd - Ra);
                S = Rdot * Ra';
                angular_velocity = [S(3, 2); S(1, 3); S(2, 1)];
                linear_velocity = (1 / deltaT) * deltaX;
                xdot = W * [linear_velocity; angular_velocity];  % Compute end-effector velocity
        
                % Compute the Jacobian
                J = self.robot.model.jacob0(self.RMRC.qMatrix(i,:));
        
                % Apply Damped Least Squares (DLS) if necessary
                self.RMRC.manipulability(i) = sqrt(abs(det(J * J')));
                if self.RMRC.manipulability(i) < epsilon
                    lambda = (1 - self.RMRC.manipulability(i) / epsilon) * 5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J' * J + lambda * eye(self.robot.model.n)) * J';  % DLS inverse
        
                % Calculate joint velocities
                qdot = (invJ * xdot)';
        
                % Handle joint limits
                for j = 1:self.robot.model.n
                    if self.RMRC.qMatrix(i,j) + deltaT * qdot(j) < self.robot.model.qlim(j,1)
                        qdot(j) = 0;  % Prevent exceeding lower limit
                    elseif self.RMRC.qMatrix(i,j) + deltaT * qdot(j) > self.robot.model.qlim(j,2)
                        qdot(j) = 0;  % Prevent exceeding upper limit
                    end
                end
        
                % Update the joint angles
                self.RMRC.qMatrix(i+1,:) = self.RMRC.qMatrix(i,:) + deltaT * qdot;
        
                % Check for collisions
                self.checkForCollisionAndPause();
        
                % Animate the robot at the new joint configuration
                self.robot.model.animate(self.RMRC.qMatrix(i+1,:));
                drawnow();  % Force the figure to update
        
                % Update the grippers' positions to follow the end-effector
                self.updateGrippersPosition();
        
                % Update the object (mesh) position if mesh_h and vertices are provided
                if ~isempty(mesh_h) && ~isempty(vertices)
                    tr = self.robot.model.fkine(self.robot.model.getpos());
                    transformedVertices = [vertices, ones(size(vertices, 1), 1)] * tr.T';
                    set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update mesh position
                end
        
                pause(0.01);  % Adjust pause for smooth animation
            end
        end


        % Run inverse kinematics-based motion with gripper synchronization
        function runIK(self, startTr, endTr, steps, varargin)
            q0 = zeros(1, self.robot.model.n);  % Assuming the robot has n degrees of freedom
            qStart = self.robot.model.ikcon(startTr, q0);  % Initial pose
            qEnd = self.robot.model.ikcon(endTr, qStart);  % End pose
        
            qMatrix = jtraj(qStart, qEnd, steps);  % Generate smooth trajectory
        
            % Check if mesh_h and vertices were provided
            if nargin > 4
                mesh_h = varargin{1};
                vertices = varargin{2};
            else
                mesh_h = [];
                vertices = [];
            end
        
            for i = 1:steps

                self.checkForEStopAndPause();  % This will pause the loop if eStop is active

                q_current = qMatrix(i, :);  % Current joint configuration
                self.robot.model.animate(q_current);  % Animate the robot
        
                self.checkForCollisionAndPause();

                % Update the object (mesh) position if mesh_h and vertices are provided
                if ~isempty(mesh_h) && ~isempty(vertices)
                    tr = self.robot.model.fkine(self.robot.model.getpos());
                    transformedVertices = [vertices, ones(size(vertices, 1), 1)] * tr.T';
                    set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update mesh position
                end
        
                % Update the grippers' positions to follow the end-effector
                self.updateGrippersPosition();
        
                pause(0.05);  % Adjust the pause duration as needed for smooth animation
            end
        end
    end
end
