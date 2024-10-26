classdef RMRC < handle
    properties
        robot; 
        qMatrix; 
        collisionChecker;
        positionError;   % Track position error
        angleError;      % Track orientation error
        manipulability;  % Track manipulability
    end
    
    methods 
        % Constructor
        function self = RMRC(robot)         
            self.robot = robot;
        end
        
        % Collision Checker Setup
        function setCollision(self, robot, centerPoints, radii)
            self.collisionChecker = CollisionChecker(robot, centerPoints, radii);
        end
        
        function ResolvedMotionRateControl(self, startTr, endTr, time, deltaT)
            % Initialize parameters
            steps = time / deltaT;
            epsilon = 0.1;  % Manipulability threshold
            W = diag([1 1 1 0.1 0.1 0.1]);  % Weighting matrix
        
            q0 = zeros(1, self.robot.model.n);  % Initial joint angles
            self.qMatrix = zeros(steps, self.robot.model.n);
            self.positionError = zeros(3, steps);  % Track position errors
            self.angleError = zeros(3, steps);     % Track orientation errors
            self.manipulability = zeros(steps, 1); % Track manipulability
        
            % Get the initial joint configuration using inverse kinematics
            self.qMatrix(1,:) = self.robot.model.ikcon(startTr, q0);
        
            % Convert the start and end orientations to quaternions for SLERP
            R_start = startTr(1:3, 1:3);  % Extract rotation matrix from start pose
            R_end = endTr(1:3, 1:3);      % Extract rotation matrix from end pose
            q_start = UnitQuaternion(R_start);  % Convert start rotation to quaternion
            q_end = UnitQuaternion(R_end);      % Convert end rotation to quaternion
        
            % Linear trapezoidal blending for position trajectory
            s = lspb(0, 1, steps);  % Generate scalar for trajectory
        
            % RMRC loop over time steps
            for i = 1:steps-1
                % 1. Interpolate position
                pos_interp = transl((1-s(i))*startTr(1:3,4)' + s(i)*endTr(1:3,4)');
        
                % 2. SLERP for smooth rotation interpolation
                q_interp = q_start.interp(q_end, s(i));  % Interpolate between start and end rotation
                R_interp = q_interp.R;  % Convert interpolated quaternion to rotation matrix
        
                % 3. Combine the interpolated position and rotation into one transformation matrix
                R_homogeneous = [R_interp, [0;0;0]; 0 0 0 1];
                T_desired = pos_interp * R_homogeneous;  % Desired transformation matrix
        
                % 4. Get the current transformation matrix
                T_current_struct = self.robot.model.fkine(self.qMatrix(i,:));
                if isobject(T_current_struct)
                    T_current = T_current_struct.T;
                else
                    T_current = T_current_struct;
                end
        
                % 5. Compute position and orientation errors
                deltaX = T_desired(1:3, 4) - T_current(1:3, 4);  % Position error
                Rd = T_desired(1:3, 1:3);                       % Desired rotation matrix
                Ra = T_current(1:3, 1:3);                       % Actual rotation matrix
                Rdot = (1 / deltaT) * (Rd - Ra);                % Rotation matrix error
                S = Rdot * Ra';                                 % Skew-symmetric matrix for angular velocity
                angular_velocity = [S(3, 2); S(1, 3); S(2, 1)]; % Angular velocity
        
                % 6. End-effector velocity
                linear_velocity = (1 / deltaT) * deltaX;
                xdot = W * [linear_velocity; angular_velocity];
        
                % 7. Compute the Jacobian
                J = self.robot.model.jacob0(self.qMatrix(i,:));
        
                % 8. Check manipulability and apply Damped Least Squares (DLS) if necessary
                self.manipulability(i) = sqrt(abs(det(J * J')));
                if self.manipulability(i) < epsilon
                    lambda = (1 - self.manipulability(i)/epsilon) * 5E-2;
                else
                    lambda = 0;
                end
        
                invJ = inv(J' * J + lambda * eye(self.robot.model.n)) * J';  % DLS inverse
        
                % 9. Calculate joint velocities
                qdot = (invJ * xdot)';  % Joint velocities
        
                % 10. Joint limits handling
                for j = 1:self.robot.model.n
                    if self.qMatrix(i,j) + deltaT * qdot(j) < self.robot.model.qlim(j,1)
                        qdot(j) = 0;  % Prevent exceeding lower limit
                    elseif self.qMatrix(i,j) + deltaT * qdot(j) > self.robot.model.qlim(j,2)
                        qdot(j) = 0;  % Prevent exceeding upper limit
                    end
                end
        
                % 11. Update the joint angles
                self.qMatrix(i+1,:) = self.qMatrix(i,:) + deltaT * qdot;
        
                % 12. Track errors for debugging or plotting
                self.positionError(:, i) = deltaX;
                self.angleError(:, i) = tr2rpy(Rd * Ra');
        
                % 13. Animate the robot at the new joint configuration
                self.robot.model.animate(self.qMatrix(i+1,:));
                drawnow();
            end
        end


                % Resolved Motion Rate Control Method
        function [s,x] = ResolvedMotionRateControlPath(self, pos, endTr, steps)

            % Initialize parameters
            % steps = time / deltaT;
            epsilon = 0.1;  % Manipulability threshold
            W = diag([1 1 1 0.1 0.1 0.1]);  % Weighting matrix

            q0 = zeros(1, self.robot.model.n);  % Initial joint angles
            self.qMatrix = zeros(steps, self.robot.model.n);
            self.positionError = zeros(3, steps);  % Track position errors
            self.angleError = zeros(3, steps);     % Track orientation errors
            self.manipulability = zeros(steps, 1); % Track manipulability
            
      

            % Get the initial joint configuration using inverse kinematics
            % self.qMatrix(1,:) = self.robot.model.ikcon(startTr, q0);

            self.qMatrix(1,:) = pos;

            startTr_struct = self.robot.model.fkine(pos);
            if isobject(startTr_struct)
                startTr = startTr_struct.T;  % Extract the .T property if it's an object
            else
                startTr = startTr_struct;    % Directly use if it's already a matrix
            end
            % 
            % 
            % self.qMatrix(1,:) = self.robot.model.ikcon(startTr, q0);

            % % Convert the start and end orientations to quaternions for SLERP
            % R_start = startTr(1:3, 1:3);
            % R_end = endTr(1:3, 1:3);
            % q_start = UnitQuaternion(R_start);
            % q_end = UnitQuaternion(R_end);
            
            % Linear trapezoidal blending for position trajectory
            s = lspb(0, 1, steps);  % Generate scalar for trajectory
            
            for i=1:steps
                pos_interp = transl((1-s(i))*startTr(1:3,4)' + s(i)*endTr(1:3,4)');
                x(1,i) = pos_interp(1,4);
                x(2,i) = pos_interp(2,4);
                x(3,i) = pos_interp(3,4);
            end

        end
        
        %getQmatrix outputs qmatrix
        function qMatrix = getQmatrix()
            qMatrix = self.qMatrix;
        end

    end
end
