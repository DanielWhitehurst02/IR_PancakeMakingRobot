classdef Panda < RobotBaseClass
    
    properties(Access = public)
        plyFileNameStem = 'Panda';
    end
    
    methods
        %% Constructor
        function self = Panda(baseTr, useTool, toolFilename)
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
            %self.model.plot(self.model.getpos())
            drawnow 
        end

        %% CreateModel
        function CreateModel(self)
            % Define the corrected links using DH parameters for the xArm 6-DOF robot


            link(1) = Link([pi   0.333     0       -pi/2   0]);
            link(2) = Link([0     0        0        pi/2   0]);
            link(3) = Link([0   0.316    0.0825     pi/2   0]);
            link(4) = Link([0     0     -0.0825    -pi/2   0]);
            link(5) = Link([0   0.384      0       -pi/2   0]);
            link(6) = Link([0     0      0.088      pi/2   0]);
            link(7) = Link([0  -0.0107     0        pi/2   0]);
    

            % Create the robot model
            self.model = SerialLink(link, 'name', self.name);
        end
    end
end
