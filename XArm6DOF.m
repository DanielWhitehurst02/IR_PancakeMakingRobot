classdef XArm6DOF < RobotBaseClass
    %% XArm 6-DOF Robot Model
    % 
    % Class-based implementation of the XArm 6-DOF robot using embedded DH parameters.

    properties(Access = public)
        plyFileNameStem = 'XArm6DOF';
    end

    methods
        %% Constructor
        function self = XArm6DOF(baseTr, useTool, toolFilename)
            % Handle input arguments for base transformation and tool settings
            if nargin < 3
                if nargin == 2
                    error('If you set useTool, you must pass in the toolFilename as well');
                elseif nargin == 0 % No parameters passed
                    baseTr = transl(0, 0, 0);
                end
            else % All parameters passed in
                self.useTool = useTool;
                toolTrData = load([toolFilename, '.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename, '.ply'];
            end

            % Create the robot model using DH parameters
            self.CreateModel();
            self.model.base = self.model.base.T * baseTr; % Apply base transformation
            self.model.tool = self.toolTr; % Set tool transformation
            self.PlotAndColourRobot(); % Plot and color the robot

            drawnow
        end

        %% CreateModel
        function CreateModel(self)
            % Define the links using DH parameters for the xArm 6-DOF robot
            link(1) = Link('d', 0.089159, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0); % J1
            link(2) = Link('d', 0, 'a', -0.425, 'alpha', 0, 'qlim', deg2rad([-118 120]), 'offset', 0); % J2
            link(3) = Link('d', 0, 'a', -0.39225, 'alpha', 0, 'qlim', deg2rad([-225 11]), 'offset', 0); % J3
            link(4) = Link('d', 0.10915, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0); % J4
            link(5) = Link('d', 0.09465, 'a', 0, 'alpha', -pi/2, 'qlim', deg2rad([-97 180]), 'offset', 0); % J5
            link(6) = Link('d', 0.0823, 'a', 0, 'alpha', 0, 'qlim', deg2rad([-360 360]), 'offset', 0); % J6

            % Create the robot model
            self.model = SerialLink(link, 'name', self.name);
        end
    end
end
