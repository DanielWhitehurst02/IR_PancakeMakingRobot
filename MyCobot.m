classdef MyCobot < RobotBaseClass
    %% MyCobot Robot Model
    %
    % Class-based implementation of the MyCobot robot using embedded DH parameters.

    properties(Access = public)
        plyFileNameStem = 'MyCobot';
    end

    methods
        %% Constructor
        function self = MyCobot(baseTr, useTool, toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0, 0, 0);
                end
            else % All parameters passed in
                self.useTool = useTool;
                toolTrData = load([toolFilename, '.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename, '.ply'];
            end

            self.CreateModel();
            self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();

            drawnow
        end

        %% CreateModel
        function CreateModel(self)
            % Define the links directly using the DH parameters
            mm = 1e-3; % Conversion factor from meters to millimeters

            link(1) = Link('d', 173.9 * mm, 'a', 0, 'alpha', 0, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(2) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(3) = Link('d', 0, 'a', 135 * mm, 'alpha', 0, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d', 88.78 * mm, 'a', 120 * mm, 'alpha', 0, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d', 95 * mm, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);
            link(6) = Link('d', 65.5 * mm, 'a', 0, 'alpha', -pi/2, 'qlim', deg2rad([-360 360]), 'offset', 0);

            % Create the robot model
            self.model = SerialLink(link, 'name', self.name);
        end
    end
end
