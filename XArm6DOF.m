classdef XArm6DOF
    properties
        % Define the serial link for the xArm 6-DOF robot
        robot
    end
    
    methods
        % Constructor
        function obj = XArm6DOF()
            % Define the links using DH parameters for xArm 6-DOF
            link(1) = Link('d', 0.089159, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(2) = Link('d', 0, 'a', -0.425, 'alpha', 0, 'qlim', deg2rad([-180 180]), 'offset', -1.3849179);
            link(3) = Link('d', 0, 'a', -0.39225, 'alpha', 0, 'qlim', deg2rad([-180 180]), 'offset', 1.3849179);
            link(4) = Link('d', 0.10915, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d', 0.09465, 'a', 0, 'alpha', -pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(6) = Link('d', 0.0823, 'a', 0, 'alpha', 0, 'qlim', deg2rad([-360 360]), 'offset', 0);

            % Create the robot model
            obj.robot = SerialLink(link, 'name', 'xArm 6-DOF');
        end
        
        % Method to plot the robot
        function plotRobot(obj, jointAngles)
            if nargin < 2
                % If no joint angles are provided, plot in default zero configuration
                jointAngles = zeros(1,6); 
            end
            
            % Plot the robot with the specified joint angles
            obj.robot.plot(jointAngles);
        end
        
        % Method to teach the robot interactively
        function teachRobot(obj)
            % Use the teach method to allow GUI-based interaction with the robot
            obj.robot.teach();
        end
        
        % Method to display the robot's DH parameters
        function displayDHParameters(obj)
            disp('DH Parameters for xArm 6-DOF:');
            obj.robot.display();
        end
    end
end
