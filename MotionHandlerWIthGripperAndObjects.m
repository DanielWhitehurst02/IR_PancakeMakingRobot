classdef MotionHandlerWIthGripperAndObjects
    properties
        collisionHandler  % CollisionEllipsoid instance for collision detection
        robot  % Robot model (e.g., UR3)
        RMRC  % RMRC motion planner
        obstaclePoints  % Points representing obstacles
        running  % Boolean indicating if the motion is running

        goalMat %Matrix of goals to travel to
        goalIndex %Current Goal being completed by controller
        steps %steps per goal
        qMatrixGoals %joint position matrix to get to all goals

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

        collisionSwitch
        redHandle
        cubePoints
        plotObject

        lightCurtain  % Instance of LightCurtain for breach detection
        handHandle    % Handle for the loaded object (e.g., hand object)
        lightCurtainStartOffset



        % dedication for pancake 
        pancakeMesh  % Handle for the pancake mesh
        pancakeVertices
        isPancakeAttached = false;  % Boolean to check if pancake is attached to end effector
        dropLocation  % Predefined drop location for the pancake
        dropCondition = false;  % Flag to trigger the drop action



         % 
         % serialUsed = true;
         % serialportname = "/dev/ttyUSB0";
         % serialBaud = 9600;
         % serialObj
    end

    methods
        % Constructor
        function self = MotionHandlerWIthGripperAndObjects(robot, centerPoints, radii, obstaclePoints, leftFinger, rightFinger,app, varargin)
            self.robot = robot;
            self.obstaclePoints = obstaclePoints;
            % self.collisionHandler = CollisionEllipsoid(robot, centerPoints, radii);  % Initialize collision handler
            self.collisionHandler = CollisionEllipsoidDynamic(robot, centerPoints, radii);
            self.collisionHandler.setObstaclePoints(obstaclePoints);
            self.RMRC = RMRC(robot);
            self.running = true;
            self.grippersPlotted = false;  % Initially, grippers are not plotted
            self.app = app; % Initialize eStop
            self.app.collision = false;

            self.collisionSwitch = true;
            % Initialize the grippers and attach them to the robot
            self.leftFinger = leftFinger;
            self.rightFinger = rightFinger;

            % 
            % self.serialObj = serialport(self.serialportname,self.serialBaud);
            % configureTerminator(self.serialObj,"CR/LF");
            % flush(self.serialObj);

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

        % Update goals and calculate qmatrix for goals
        function goalSaved = setGoals(self, goals, steps)
            self.goalMat = goals;
            self.steps = steps * size(goals,1);
            self.goalIndex = 1;

            goalSaved = self.goalMat;

            for j=1:(size(goals,1))
                % for i=1:steps
                if j == 1
                    startTr = self.robot.model.getpos;
                else
                    startTr = self.robot.model.ikcon(goals{j-1});

                end
                  
                [s, ~] = self.RMRC.ResolvedMotionRateControlPath(startTr, goals{1},steps);
                self.qMatrixGoals = s;

                % end
                
                
            end

        end


        % Check for collisions and pause if detected, resume automatically when resolved
        % Check for collisions and pause only if app.collision is active
        function checkForCollisionAndPause(self)
            % Exit immediately if collision checking is disabled
            if ~self.app.collision
                return;
            end
        
            % Perform collision checking
            self.updateEllipsoidCenters();  % Update ellipsoid centers based on the current state
            collision = self.collisionHandler.detectCollision();
        
            if collision
                disp('Collision detected! Pausing robot motion...');
                self.running = false;
        
                % Loop until collision is cleared or app.collision is disabled
                while collision && self.app.collision
                    pause(0.1);
                    self.updateEllipsoidCenters();  % Continuously update ellipsoid centers
                    collision = self.collisionHandler.detectCollision();  % Re-check for collision
        
                    % If collision is cleared or app.collision is set to false, resume motion
                    if ~collision
                        disp('Collision resolved. Resuming robot motion...');
                        self.running = true;
                    elseif ~self.app.collision
                        disp('Collision checking disabled. Exiting pause...');
                        delete(self.redHandle);
                        self.redHandle = []; 
                        break;  % Exit the loop immediately if app.collision is false
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
            
            % data = str2double(readline(self.serialObj));
            % disp(data)
            %     if data == 1
            %         self.app.eStop = true;
            %     end

            while self.app.eStop  % Dynamically check app.eStop value
                disp('eStop is active, pausing motion...');
                pause(0.1);  % Small pause while checking for eStop status
            end
        end
        
        % function checkSerial(self)
        %     data = readline(self.serialObj);
        %     disp(data)
        % 
        %     doubdata = str2num(data);
        %     if doubdata == 1
        %         disp('wegaming')
        %     end
        % end


        function checkForCollisionCall(self)
            if self.app.collision
                if self.collisionSwitch
                    % Create a new collision mesh and store the handle
                    [cubePoints, ~, ~, ~, redHandle] = CollisionMesh(self.app.cubeSize(1), self.app.cubeSize(2), ...
                                                                     self.app.cubeRot, self.app.cubeDens, self.app.cubeCent);
                    pause(0.1);
                    self.collisionHandler.setObstaclePoints(cubePoints);
                    self.redHandle = redHandle;  % Store handle
                    self.collisionSwitch = false;
                    self.cubePoints = cubePoints;
                    disp(self.cubePoints)
                end
            else
                % Delete redHandle if it exists and reset switch
                if ~isempty(self.redHandle) && isvalid(self.redHandle)
                    delete(self.redHandle);
                    self.redHandle = [];  % Clear handle after deletion
                end
                self.collisionSwitch = true;
            end
        end



        % Vanilla function to check for light curtain breach and handle stopping/resuming
        function checkForLightCurtainBreachAndPause(self, iteration)
            % Persistent variables to retain across calls
            persistent lightCurtain handVertices handHandle initialTransform
        
            % Initialize only on the first call
            if isempty(lightCurtain) || isempty(handVertices) || isempty(handHandle)
                % Create the light curtain as a single surface
                lightCurtain = LightCurtain(-1.6, 1.7, 0, 1.5, 0.2, 0.1, 'none');
        
                % Load the hand object from the .ply file and apply an initial transformation
                initialTransform = transl(0, 0, 1) * trotx(pi/2);  % Initial transformation matrix
                hold on;
        
                % Load the hand.ply file and plot it using the LightCurtain class method
                [handVertices, handHandle] = lightCurtain.loadObject('hand.ply', initialTransform);
            end
        
            % Apply a transformation based on the iteration parameter
            newTransform = transl(0, 0.1 * iteration, 0) * initialTransform;  % Translate based on iteration
            updatedVertices = lightCurtain.updateObjectPosition(handVertices, newTransform);
            set(handHandle, 'Vertices', updatedVertices);  % Update plot with new vertices
        
            % Check if the hand has breached the light curtain
            if lightCurtain.detectObject(updatedVertices)
                disp("Light curtain breached! Pausing motion...");
        
                % Pause the loop until breach is cleared or light curtain is turned off
                while lightCurtain.detectObject(updatedVertices)
                    % Break if light curtain toggle is off
                    if ~self.app.lightCurtain
                        disp("Light curtain detection disabled. Exiting pause...");
                        break;
                    end
                    pause(0.1);  % Short pause to allow for reset condition
                    % Continuously update vertices if required
                    set(handHandle, 'Vertices', updatedVertices);
                end
        
                disp("Breach cleared. Resuming motion...");
            end
        end
        



        %Get current joints
        function currentpos = getCurrentPos(self)
            currentpos = self.robot.model.getpos;
        end

        %Get current end effector transformation
        function currentendTr = getCurrentEndTr(self)
            currentendTr = self.robot.model.fkine(self.robot.model.getpos);
        end


        % Resume the robot motion after stopping
        function resume(self)
            self.running = true;
            self.runRMRC(self.robot.model.fkine(self.robot.model.getpos), self.prevGoal, self.prevtime, self.prevdeltaT);
        end



        function DrawAndDelete(self)
            % Persistent variables to manage plotting state across calls
            persistent collisionCubePoints collisionPlotHandle isPlotted
        
            % Initialize the flag if it's empty
            if isempty(isPlotted)
                isPlotted = false;
            end
        
            % Check if collision is active
            if self.app.collision
                % Plot only if not already plotted
                if ~isPlotted
                    [collisionCubePoints, ~, ~, ~, redHandle] = CollisionMesh(self.app.cubeSize(1), self.app.cubeSize(2), ...
                                                                      self.app.cubeRot, self.app.cubeDens, self.app.cubeCent);
                    collisionPlotHandle = plot3(collisionCubePoints(:,1), collisionCubePoints(:,2), collisionCubePoints(:,3), 'r*'); % Plot the collision points
                    hold on; % Retain plot for further updates
                    isPlotted = true; % Set the flag to indicate that the plot exists
                end
            else
                % Delete the plot only if it was previously plotted
                if isPlotted && isvalid(collisionPlotHandle)
                    delete(collisionPlotHandle); % Delete the plot
                    collisionPlotHandle = []; % Clear the persistent reference
                    collisionCubePoints = []; % Clear the cube points
                    isPlotted = false; % Reset the flag
                end
            end
        end

        
        function attachPancake(self)
           if self.isPancakeAttached
               % Get the full transformation matrix of the end effector
               endEffectorTr = self.robot.model.fkine(self.robot.model.getpos).T;
      
               % Define the offset as a translation matrix
               offsetTranslation = transl(0, 0.06, 0.06)  * trotx(-pi/2);
      
               % Combine the end effector transformation with the offset translation
               pancakeTransform = endEffectorTr * offsetTranslation;
      
               % Apply the combined transformation to the pancake vertices
               transformedVertices = [self.pancakeVertices, ones(size(self.pancakeVertices, 1), 1)] * pancakeTransform';
      
               % Update pancake mesh vertices
               set(self.pancakeMesh, 'Vertices', transformedVertices(:, 1:3));  % Update pancake position
           end
       end





        function dropPancake(self)
            if self.dropCondition
                % Define transformation for drop location
                dropTransform = transl(self.dropLocation);
        
                % Update pancake vertices to the drop location
                transformedVertices = [self.pancakeVertices, ones(size(self.pancakeVertices, 1), 1)] * dropTransform';
                set(self.pancakeMesh, 'Vertices', transformedVertices(:, 1:3));  % Place pancake at drop location
        
                % Detach pancake and reset the drop condition
                self.isPancakeAttached = false;
                self.dropCondition = false;
            end
        end





        % Run RMRC with gripper synchronization
        function runRMRC(self, endTr, time, deltaT, varargin)
            self.running = true;
            self.currentGoal = endTr;
            self.prevdeltaT = deltaT;
            self.prevtime = time;
        
            steps = time/deltaT;

            self.lightCurtainStartOffset = NaN; % Initialize as NaN or empty

            startTr_struct = self.getCurrentEndTr();

            if isobject(startTr_struct)
                startTr = startTr_struct.T;
            else
                startTr = startTr_struct;
            end
            

            % Check if mesh_h and vertices were provided (first set of optional parameters)
            % Optional mesh for visualization
            if length(varargin) >= 2
                mesh_h = varargin{1};
                vertices = varargin{2};
            else
                mesh_h = [];
                vertices = [];
            end
            
            % Optional pancake mesh attachment
            if length(varargin) >= 3
                self.pancakeMesh = varargin{3};
                self.pancakeVertices = varargin{4};
                self.isPancakeAttached = true;
            else
                self.pancakeMesh = [];
                self.pancakeVertices = [];
                self.isPancakeAttached = false;
            end
            

            [s, ~] = self.RMRC.ResolvedMotionRateControlPath(self.getCurrentPos(), endTr, steps);
        
            epsilon = 0.1;  % Manipulability threshold
            W = diag([1 1 1 0.1 0.1 0.1]);  % Weighting matrix for control
        
            % Check if mesh_h and vertices were provided
            %if nargin > 5
            %    mesh_h = varargin{1};
            %    vertices = varargin{2};
            %else
            %    mesh_h = [];
            %    vertices = [];
            %end
        

            redHandle = 0
            baseIteration = 0;
            % Perform RMRC loop
            for i = 1:steps-1

                self.DrawAndDelete();

                % self.checkSerial();

                self.checkForEStopAndPause();  % This will pause the loop if eStop is active
                
                % Check for collisions and update/redraw if necessary
                self.checkForCollisionCall();
                self.checkForCollisionAndPause();  % Pause if collision is detected

                % Check for light curtain activation
                if self.app.lightCurtain
                    % Set the offset if it hasn't been set yet
                    if isnan(self.lightCurtainStartOffset)
                        self.lightCurtainStartOffset = i;
                    end
                
                    % Call light curtain function, passing the adjusted iteration
                    adjustedIteration = i - self.lightCurtainStartOffset;
                    self.checkForLightCurtainBreachAndPause(adjustedIteration);
                end


                
                % Attach pancake if currently attached
                if self.isPancakeAttached && ~isempty(self.pancakeMesh) && ~isempty(self.pancakeVertices)
                    self.attachPancake();
                end


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
                % self.checkForCollisionAndPause();
        
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

        function runtwoRMRC(self, robot2, endTr1,endTr2, time, deltaT, varargin)
            self.running = true;
            self.currentGoal = endTr1;
            self.prevdeltaT = deltaT;
            self.prevtime = time;

            steps = time/deltaT;

            thisRMRC = RMRC(robot2)

            [s1, ~] = self.RMRC.ResolvedMotionRateControlPath(self.getCurrentPos(), endTr1, steps);
            [s2, ~] = thisRMRC.ResolvedMotionRateControlPath(robot2.model.getpos, endTr2, steps);

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

            startTr_struct = self.getCurrentEndTr();

            if isobject(startTr_struct)
                startTr1 = startTr_struct.T;
            else
                startTr1 = startTr_struct;
            end


            startTr_struct2 = robot2.model.fkine(robot2.model.getpos);

            if isobject(startTr_struct2)
                startTr2 = startTr_struct2.T;
            else
                startTr2 = startTr_struct2;
            end
        
            % Perform RMRC loop
            for i = 1:steps-1

                self.checkForEStopAndPause();  % This will pause the loop if eStop is active

                % Interpolate position and orientation
                pos_interp1 = transl((1-s1(i))*startTr1(1:3,4)' + s1(i)*endTr1(1:3,4)');
                pos_interp2 = transl((1-s2(i))*startTr2(1:3,4)' + s2(i)*endTr1(1:3,4)');

                q_interp1 = UnitQuaternion(startTr1(1:3,1:3)).interp(UnitQuaternion(endTr1(1:3,1:3)), s1(i));  % SLERP for orientation
                q_interp2 = UnitQuaternion(startTr2(1:3,1:3)).interp(UnitQuaternion(endTr2(1:3,1:3)), s2(i));  % SLERP for orientation

                R_interp1 = q_interp1.R;
                R_interp2 = q_interp2.R;

                R_homogeneous1 = [R_interp1, [0; 0; 0]; 0 0 0 1];  % 4x4 matrix
                R_homogeneous2 = [R_interp2, [0; 0; 0]; 0 0 0 1];  % 4x4 matrix
        
                % Desired transformation matrix
                T_desired1 = pos_interp1 * R_homogeneous1;
                T_desired2 = pos_interp2 * R_homogeneous2;
        
                % Get current transformation matrix
                T_current_struct1 = self.robot.model.fkine(self.RMRC.qMatrix(i,:));
                T_current_struct2 = robot2.model.fkine(robot2.model.getpos);

                if isobject(T_current_struct1)
                    T_current1 = T_current_struct1.T;
                else
                    T_current1 = T_current_struct1;
                end
        
                if isobject(T_current_struct2)
                    T_current2 = T_current_struct2.T;
                else
                    T_current2 = T_current_struct2;
                end

                % Compute position and orientation errors
                deltaX1 = T_desired1(1:3,4) - T_current1(1:3,4);
                deltaX2 = T_desired2(1:3,4) - T_current2(1:3,4);

                Rd1 = T_desired1(1:3, 1:3);
                Rd2 = T_desired2(1:3, 1:3);

                Ra1 = T_current1(1:3, 1:3);
                Ra2 = T_current2(1:3, 1:3);

                Rdot1 = (1 / deltaT) * (Rd1 - Ra1);
                Rdot2 = (1 / deltaT) * (Rd2 - Ra2);

                S1 = Rdot1 * Ra1';
                S2 = Rdot2 * Ra2';

                angular_velocity1 = [S1(3, 2); S1(1, 3); S1(2, 1)];
                angular_velocity2 = [S2(3, 2); S2(1, 3); S2(2, 1)];

                linear_velocity1 = (1 / deltaT) * deltaX1;
                linear_velocity2 = (1 / deltaT) * deltaX2;
                
                xdot1 = W * [linear_velocity1; angular_velocity1];  % Compute end-effector velocity
                xdot2 = W * [linear_velocity2; angular_velocity2];  % Compute end-effector velocity
        
                % Compute the Jacobian
                J1 = self.robot.model.jacob0(self.RMRC.qMatrix(i,:));
                J2 = robot2.model.jacob0(robot2.model.getpos);
        
                % Apply Damped Least Squares (DLS) if necessary
                self.RMRC.manipulability(i) = sqrt(abs(det(J1 * J1')));
                if self.RMRC.manipulability(i) < epsilon
                    lambda1 = (1 - self.RMRC.manipulability(i) / epsilon) * 5E-2;
                else
                    lambda1 = 0;
                end

                thisRMRC.manipulability(i) = sqrt(abs(det(J2 * J2')));
                if thisRMRC.manipulability(i) < epsilon
                    lambda2 = (1 - thisRMRC.manipulability(i) / epsilon) * 5E-2;
                else
                    lambda2 = 0;
                end

                invJ1 = inv(J1' * J1 + lambda1 * eye(self.robot.model.n)) * J1';  % DLS inverse
                invJ2 = inv(J2' * J2 + lambda2 * eye(robot2.model.n)) * J2';  % DLS inverse
        
                % Calculate joint velocities
                qdot1 = (invJ1 * xdot1)';
                qdot2 = (invJ2 * xdot2)';
        
                % Handle joint limits
                for j = 1:self.robot.model.n
                    if self.RMRC.qMatrix(i,j) + deltaT * qdot1(j) < self.robot.model.qlim(j,1)
                        qdot1(j) = 0;  % Prevent exceeding lower limit
                    elseif self.RMRC.qMatrix(i,j) + deltaT * qdot1(j) > self.robot.model.qlim(j,2)
                        qdot1(j) = 0;  % Prevent exceeding upper limit
                    end
                end

                for j = 1:robot2.model.n
                    if thisRMRC.qMatrix(i,j) + deltaT * qdot2(j) < robot2.model.qlim(j,1)
                        qdot2(j) = 0;  % Prevent exceeding lower limit
                    elseif thisRMRC.qMatrix(i,j) + deltaT * qdot2(j) > robot2.model.qlim(j,2)
                        qdot2(j) = 0;  % Prevent exceeding upper limit
                    end
                end
        
                % Update the joint angles
                self.RMRC.qMatrix(i+1,:) = self.RMRC.qMatrix(i,:) + deltaT * qdot1;
                thisRMRC.qMatrix(i+1,:) = thisRMRC.qMatrix(i,:) + deltaT * qdot2;
                
                % Check for collisions
                self.checkForCollisionAndPause();
        
                % Animate the robot at the new joint configuration
                self.robot.model.animate(self.RMRC.qMatrix(i+1,:));
                robot2.model.animate(thisRMRC.qMatrix(i+1,:));
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
        
        % function [goalreached, endTr] = iterateRMRC(self, goalMat, j, time, deltaT, varargin)
        %     self.running = true;
        %     % self.currentGoal = endTr;
        %     self.prevdeltaT = deltaT;
        %     self.prevtime = time;
        %     goalreached = false;
        % 
        %     % endTr = goalMat;
        % 
        % 
        % 
        %     endTr = transl(0.2, -0.3, 0.5) * troty(-pi/2);
        % 
        %     startTr_struct = self.getCurrentEndTr();
        % 
        %     if isobject(startTr_struct)
        %         startTr = startTr_struct.T;
        %     else
        %         startTr = startTr_struct;
        %     end
        % 
        %     % steps = self.steps;
        %     steps = 50;
        %     % steps = time/deltaT;
        % 
        %     % if self.newgoal
        %     [s, ~] = self.RMRC.ResolvedMotionRateControlPath(self.getCurrentPos(), endTr, steps);
        %     %     self.newgoal = false;
        %     % end
        % 
        %     % s = self.qMatrixGoals;
        % 
        %     if j > steps
        %         i = steps;
        %         goalreached = true;
        %     else
        %         i = j;
        %     end
        % 
        %     epsilon = 0.1;  % Manipulability threshold
        %     W = diag([1 1 1 0.1 0.1 0.1]);  % Weighting matrix for control
        % 
        %     % Check if mesh_h and vertices were provided
        %     if nargin > 5
        %         mesh_h = varargin{1};
        %         vertices = varargin{2};
        %     else
        %         mesh_h = [];
        %         vertices = [];
        %     end
        % 
        %     self.checkForEStopAndPause();  % This will pause the loop if eStop is active
        % 
        %     % Interpolate position and orientation
        %     pos_interp = transl((1-s(i))*startTr(1:3,4)' + s(i)*endTr(1:3,4)');
        %     q_interp = UnitQuaternion(startTr(1:3,1:3)).interp(UnitQuaternion(endTr(1:3,1:3)), s(i));  % SLERP for orientation
        %     R_interp = q_interp.R;
        %     R_homogeneous = [R_interp, [0; 0; 0]; 0 0 0 1];  % 4x4 matrix
        % 
        %     % Desired transformation matrix
        %     T_desired = pos_interp * R_homogeneous;
        % 
        %     % Get current transformation matrix
        %     T_current_struct = self.robot.model.fkine(self.RMRC.qMatrix(i,:));
        %     if isobject(T_current_struct)
        %         T_current = T_current_struct.T;
        %     else
        %         T_current = T_current_struct;
        %     end
        % 
        %     % if T_current - self.currentGoal(self.goalIndex) < 0.05
        %     %     goalreached = true;
        %     % else
        %     %     goalreached = false;
        %     % end
        %     % 
        %     % if T_current - endTr < 0.05
        %     %     goalreached = true;
        %     %      curr = T_current;
        %     % else
        %     %     goalreached = false;
        %     %      curr = T_current;
        %     % end
        % 
        %     % Compute position and orientation errors
        %     deltaX = T_desired(1:3,4) - T_current(1:3,4);
        %     Rd = T_desired(1:3, 1:3);
        %     Ra = T_current(1:3, 1:3);
        %     Rdot = (1 / deltaT) * (Rd - Ra);
        %     S = Rdot * Ra';
        %     angular_velocity = [S(3, 2); S(1, 3); S(2, 1)];
        %     linear_velocity = (1 / deltaT) * deltaX;
        %     xdot = W * [linear_velocity; angular_velocity];  % Compute end-effector velocity
        % 
        %     % Compute the Jacobian
        %     J = self.robot.model.jacob0(self.RMRC.qMatrix(i,:));
        % 
        %     % Apply Damped Least Squares (DLS) if necessary
        %     self.RMRC.manipulability(i) = sqrt(abs(det(J * J')));
        %     if self.RMRC.manipulability(i) < epsilon
        %         lambda = (1 - self.RMRC.manipulability(i) / epsilon) * 5E-2;
        %     else
        %         lambda = 0;
        %     end
        %     invJ = inv(J' * J + lambda * eye(self.robot.model.n)) * J';  % DLS inverse
        % 
        %     % Calculate joint velocities
        %     qdot = (invJ * xdot)';
        % 
        %     % Handle joint limits
        %     for j = 1:self.robot.model.n
        %         if self.RMRC.qMatrix(i,j) + deltaT * qdot(j) < self.robot.model.qlim(j,1)
        %             qdot(j) = 0;  % Prevent exceeding lower limit
        %         elseif self.RMRC.qMatrix(i,j) + deltaT * qdot(j) > self.robot.model.qlim(j,2)
        %             qdot(j) = 0;  % Prevent exceeding upper limit
        %         end
        %     end
        % 
        %     % Update the joint angles
        %     self.RMRC.qMatrix(i+1,:) = self.RMRC.qMatrix(i,:) + deltaT * qdot;
        % 
        %     % Check for collisions
        %     self.checkForCollisionAndPause();
        % 
        %     % Animate the robot at the new joint configuration
        %     self.robot.model.animate(self.RMRC.qMatrix(i+1,:));
        %     drawnow();  % Force the figure to update
        % 
        %     % Update the grippers' positions to follow the end-effector
        %     self.updateGrippersPosition();
        % 
        %     % Update the object (mesh) position if mesh_h and vertices are provided
        %     if ~isempty(mesh_h) && ~isempty(vertices)
        %         tr = self.robot.model.fkine(self.robot.model.getpos());
        %         transformedVertices = [vertices, ones(size(vertices, 1), 1)] * tr.T';
        %         set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update mesh position
        %     end
        % 
        % end
        % 

        % Run inverse kinematics-based motion with gripper synchronization
        function runIK(self, startTr, endTr, steps, varargin)
            % Generate joint trajectory from start to end transformation
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
                % Check for emergency stop and pause if necessary
                self.checkForEStopAndPause();
        
                % Check for collisions if enabled in the app
                if self.app.collision
                    % Check and update collision settings
                    self.checkForCollisionCall();
                    self.checkForCollisionAndPause();  % Pause if a collision is detected
                end
        
                % Set current joint configuration and animate robot
                q_current = qMatrix(i, :);
                self.robot.model.animate(q_current);
        
                % Update object (mesh) position if mesh_h and vertices are provided
                if ~isempty(mesh_h) && ~isempty(vertices)
                    tr = self.robot.model.fkine(self.robot.model.getpos());
                    transformedVertices = [vertices, ones(size(vertices, 1), 1)] * tr.T';
                    set(mesh_h, 'Vertices', transformedVertices(:, 1:3)); % Update mesh position
                end
        
                % Update gripper positions to follow the end-effector
                self.updateGrippersPosition();
        
                % Small pause for smooth animation
                pause(0.05);
            end
        end

        % 
        % function complete = iterateGoals(self, i)
        %     self.goalIndex
        % 
        % 
        % 
        %     % complete = 
        % end
    end
end
