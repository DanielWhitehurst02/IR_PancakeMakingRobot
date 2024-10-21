classdef MotionHandler
    %MotionHandler handles robot motion control and collision 
    %   Detailed explanation goes here

    properties
        collisionHandler
        robot
        RMRC
        obstaclePoints
        running
        obstaclesupdated
        currentGoal
        prevGoal
        prevtime
        prevdeltaT
    end

    methods

        %Constructor
        function self = MotionHandler(robot, centerPoints, radii, obstaclePoints)         
            self.robot = robot;
            self.obstaclePoints = obstaclePoints;
            self.collisionHandler = CollisionEllipsoid(robot, centerPoints, radii);
            self.RMRC = RMRC(robot);
            self.running = true;
            self.obstaclesupdated = false;
        end
        
        function updateObstacle(self, obstaclePoints)
            %updateObstacle updates point cloud of obstacles
            % updates point cloud of obstacles
            self.obstaclePoints = obstaclePoints;
            % self.obstaclesupdated = true;
        end

        %function to check path for collisions
        function isCollision = pathCheck(self, path)
            %pathCheck checks a path for collisions
            %   Detailed explanation goes here
            isCollision = false;
            for i=1:length(path)
                for j=1:size(self.obstaclePoints)
                    dist = sqrt((self.obstaclePoints(j,1)-path(1,i))^2+(self.obstaclePoints(j,2)-path(2,i))^2+(self.obstaclePoints(j,3)-path(3,i))^2);
                    if any(dist < 0.1)
                        isCollision = true;
                        break;
                    end
                end
            end
        end

        function [stopped,prevGoal] = eStop(self)
            %eStop Function to handle an emergency stop command
            %   Detailed explanation goes here
                self.running = false;
                self.prevGoal = self.currentGoal;
                
        end

        function resume(self)
            self.running = true;
            self.runRMRC(self.robot.model.fkine(self.robot.model.getpos),self.prevGoal,self.prevtime,self.prevdeltaT);
        end

        function runRMRC(self, startTr, endTr, time, deltaT)
            
            self.running = true;
            self.currentGoal = endTr;
            self.prevdeltaT = deltaT;
            self.prevtime = time;

            [s,x,steps] = self.RMRC.ResolvedMotionRateControlPath(startTr, endTr, time, deltaT);

            epsilon = 0.1;  % Manipulability threshold
            W = diag([1 1 1 0.1 0.1 0.1]);  % Weighting matrix

            collision = self.pathCheck(x);
            if collision
                print("collision on path")
            end


            % Convert the start and end orientations to quaternions for SLERP
            R_start = startTr(1:3, 1:3);
            R_end = endTr(1:3, 1:3);
            q_start = UnitQuaternion(R_start);
            q_end = UnitQuaternion(R_end);

                for i=1:steps-1
                    
                    % collision = self.pathCheck(x(:,i):x(:,steps-1));
    
                    % Interpolate position and orientation
                    pos_interp = transl((1-s(i))*startTr(1:3,4)' + s(i)*endTr(1:3,4)');
                    q_interp = q_start.interp(q_end, s(i));  % SLERP for orientation
                    R_interp = q_interp.R;
                    R_homogeneous = [R_interp, [0;0;0]; 0 0 0 1];  % 4x4 matrix
    
                    % Desired transformation matrix at step i
                    T_desired = pos_interp * R_homogeneous;
    
                    % Get the current transformation matrix
                    T_current_struct = self.robot.model.fkine(self.RMRC.qMatrix(i,:));
                    if isobject(T_current_struct)
                        T_current = T_current_struct.T;
                    else
                        T_current = T_current_struct;
                    end
                    
                    % Compute position and orientation errors
                    deltaX = T_desired(1:3, 4) - T_current(1:3, 4);
                    Rd = T_desired(1:3, 1:3);
                    Ra = T_current(1:3, 1:3);
                    Rdot = (1 / deltaT) * (Rd - Ra);
                    S = Rdot * Ra';
                    angular_velocity = [S(3, 2); S(1, 3); S(2, 1)];
    
                    % End-effector velocity
                    linear_velocity = (1 / deltaT) * deltaX;
                    xdot = W * [linear_velocity; angular_velocity];
    
                    % Compute the Jacobian
                    J = self.robot.model.jacob0(self.RMRC.qMatrix(i,:));
    
                    % Compute manipulability and apply Damped Least Squares (DLS) if necessary
                    self.RMRC.manipulability(i) = sqrt(abs(det(J * J')));
                    if self.RMRC.manipulability(i) < epsilon
                        lambda = (1 - self.RMRC.manipulability(i)/epsilon) * 5E-2;
                    else
                        lambda = 0;
                    end
                    
                    invJ = inv(J' * J + lambda * eye(self.robot.model.n)) * J';  % DLS inverse
                    
                    % Calculate joint velocities
                    qdot = (invJ * xdot)';  % Joint velocities
                    
                    % Joint limits handling
                    for j = 1:self.robot.model.n
                        if self.RMRC.qMatrix(i,j) + deltaT * qdot(j) < self.robot.model.qlim(j,1)
                            qdot(j) = 0;  % Prevent exceeding lower limit
                        elseif self.RMRC.qMatrix(i,j) + deltaT * qdot(j) > self.robot.model.qlim(j,2)
                            qdot(j) = 0;  % Prevent exceeding upper limit
                        end
                    end
                    
                    % Update the joint angles
                    self.RMRC.qMatrix(i+1,:) = self.RMRC.qMatrix(i,:) + deltaT * qdot;
                    
                    % Track errors for debugging or plotting
                    self.RMRC.positionError(:, i) = deltaX;
                    self.RMRC.angleError(:, i) = tr2rpy(Rd * Ra');
                    
                    %check for collisionsobstaclePoints
                    collision = self.collisionHandler.detectCollision(self.obstaclePoints);
    
                    if collision
                        print("collision on path")
                        self.eStop()
                        break
                    end
    
                    % Animate the robot at the new joint configuration
                    self.robot.model.animate(self.RMRC.qMatrix(i+1,:));
                    drawnow();
                    pause(0.01)
                    
                end
                
               
        end



        function runIK(self, startTr, endTr,  time, deltaT)
            % Function to animate a robot's motion between two poses using ikcon and jtraj
            % Inputs:
            % - robot: The robot model (e.g., UR3 robot)
            % - startTr: Start transformation matrix (4x4 matrix)
            % - endTr: End transformation matrix (4x4 matrix)
            % - steps: Number of steps for the animation

            steps = time / deltaT;
        
            % Initial joint configuration
            q0 = zeros(1, self.robot.model.n);  % Assuming the robot has n degrees of freedom
        
            % Solve for joint configurations at the start and end transformations
            qStart = self.robot.model.ikcon(startTr, q0);   % Initial pose
            qEnd = self.robot.model.ikcon(endTr, qStart);   % End pose, starting from qStart
             
            % Generate joint trajectory between the start and end configurations
            qMatrix = jtraj(qStart, qEnd, steps);  % Generate smooth trajectory
        
            % Animate the robot following the joint trajectory
            for i = 1:steps
                q_current = qMatrix(i, :);  % Current joint configuration
                self.robot.model.animate(q_current);  % Animate the robot

                collision = self.collisionHandler.detectCollision(self.obstaclePoints);
                
                if collision
                    self.eStop()
                    break
                end

                pause(0.05);  % Adjust the pause duration as needed for smooth animation
            end
        end


    end
end