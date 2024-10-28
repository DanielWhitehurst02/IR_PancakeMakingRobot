classdef MotionHandler
    % MotionHandler handles robot motion control and collision detection
    properties
        robot  % Robot model (e.g., UR3)

        collisionHandler  % CollisionEllipsoid instance for collision detection
        RMRC  % RMRC motion planner
        
        obstaclePoints  % Points representing obstacles
        running  % Boolean indicating if the motion is running
        
        currentGoal  % Current target transformation
        prevGoal  % Previous goal for resuming
        prevtime  % Previous time for resuming
        prevdeltaT  % Previous deltaT for resuming
    end

    methods
        % Constructor
        function self = MotionHandler(robot, centerPoints, radii, obstaclePoints)
            self.robot = robot;
            self.obstaclePoints = obstaclePoints;
            self.collisionHandler = CollisionEllipsoid(robot, centerPoints, radii);  % Initialize collision handler
            self.RMRC = RMRC(robot);
            self.running = true;
        end
        
        % Update ellipsoid centers dynamically based on current robot configuration
        function updateEllipsoidCenters(self)
            % Compute forward kinematics and automatically update ellipsoid centers
            self.collisionHandler.drawEllipsoids();  % This will update ellipsoid positions based on fkine internally
        end


        % Check for collisions and stop if detected
        function checkForCollisionAndStop(self)
            self.updateEllipsoidCenters();  % Update ellipsoid positions based on the current robot state
            collision = self.collisionHandler.detectCollision(self.obstaclePoints);  % Check for collisions
            if collision
                disp('Collision detected! Stopping robot.');
                self.eStop();  % Stop the robot if a collision is detected
            end
        end

        % Emergency stop function
        function eStop(self)
            self.running = false;  % Stop the robot motion
            self.prevGoal = self.currentGoal;  % Save the current goal for resuming
        end

        % Resume the robot motion after stopping
        function resume(self)
            self.running = true;
            if self.prevGoal == null
                print("Error no previous goal to resume");
                return
            end

            self.runRMRC(self.getCurrentEndTr(), self.prevGoal, self.prevtime, self.prevdeltaT);
        end

        function currentpos = getCurrentPos(self)
            currentpos = self.robot.model.getpos;
        end

        function currentendTr = getCurrentEndTr(self)
            currentendTr = self.robot.model.fkine(self.robot.model.getpos);
        end

        % Run Resolved Motion Rate Control (RMRC)
        function runRMRC(self, endTr, time, deltaT)
            self.running = true;
            self.currentGoal = endTr;
            self.prevdeltaT = deltaT;
            self.prevtime = time;



            startTr_struct = self.getCurrentEndTr();
            
            if isobject(startTr_struct)
                startTr = startTr_struct.T;
            else
                startTr = startTr_struct;
            end

            [s, ~, steps] = self.RMRC.ResolvedMotionRateControlPath(self.getCurrentPos(), endTr, time, deltaT);

            epsilon = 0.1;  % Manipulability threshold
            W = diag([1 1 1 0.1 0.1 0.1]);  % Weighting matrix for control

            % Perform RMRC loop
            for i = 1:steps-1
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
                self.checkForCollisionAndStop();
                if ~self.running
                    break;  % Stop if a collision is detected
                end

                % Animate the robot at the new joint configuration
                self.robot.model.animate(self.RMRC.qMatrix(i+1,:));
                drawnow();  % Force the figure to update

                % Redraw the ellipsoids during the animation
                self.collisionHandler.drawEllipsoids();
                
                pause(0.01);  % Adjust pause for smooth animation
            end
        end

        % Run inverse kinematics-based motion
        function runIK(self, startTr, endTr, steps)
            % Initial joint configuration
            q0 = zeros(1, self.robot.model.n);  % Assuming the robot has n degrees of freedom

            % Solve for joint configurations at the start and end transformations
            qStart = self.robot.model.ikcon(startTr, q0);  % Initial pose
            qEnd = self.robot.model.ikcon(endTr, qStart);  % End pose, starting from qStart

            % Generate joint trajectory between the start and end configurations
            qMatrix = jtraj(qStart, qEnd, steps);  % Generate smooth trajectory

            % Animate the robot following the joint trajectory
            for i = 1:steps
                q_current = qMatrix(i, :);  % Current joint configuration
                self.robot.model.animate(q_current);  % Animate the robot

                % Check for collisions
                self.checkForCollisionAndStop();
                if ~self.running
                    break;  % Stop motion if a collision is detected
                end

                % Update the ellipsoid positions and draw them during animation
                self.collisionHandler.drawEllipsoids();

                pause(0.05);  % Adjust the pause duration as needed for smooth animation
            end
        end
    end
end
