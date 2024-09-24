classdef Finger < RobotBaseClass
    properties(Access = public)
        plyFileNameStem = 'Fingers';
    end 
    methods
        function self = Finger(baseTr)
            if nargin < 1
                baseTr = transl(0, 0, 0);
            end
            self.CreateModel();
            self.model.base = baseTr;
            %self.PlotAndColourRobot();
            drawnow;
        end
       
        function CreateModel(self)
            link(1) = Link('d', 0, 'a', 0, 'alpha', 0, 'qlim', [-pi pi]); % First Segment
            link(2) = Link('d', 0, 'a', 0, 'alpha', 0, 'qlim', [-pi pi]); % Second Segment
            link(3) = Link('d', 0, 'a', 0, 'alpha', 0, 'qlim', [-pi pi]); % Third Segment
            
            hold on;
            self.model = SerialLink(link,'name',self.name);
        end
        
        function AnimateOpenClose(self, steps, openFirst)
            % Method to animate the opening and closing of the finger
            % `openFirst` determines whether to start with opening or closing
            
            % Preserve the current base transformation
            currentBaseTr = self.model.base;
            
            % Define joint angles for open and closed positions
            openAngles = [0, 0, 0];        % Fully open position
            closedAngles = [deg2rad(5),deg2rad(5),deg2rad(6)]; % Fully closed position
            
            % Generate a trajectory based on whether opening or closing
            if openFirst
                % Trajectory from closed to open
                qMatrix = jtraj(closedAngles, openAngles, steps);
            else
                % Trajectory from open to closed
                qMatrix = jtraj(openAngles, closedAngles, steps);
            end
            
            % Animate the trajectory
            for i = 1:steps
                % Set the current joint angles
                self.model.animate(qMatrix(i, :));
                
                % Ensure the base remains fixed
                self.model.base = currentBaseTr;
                
                % Pause for animation effect
                pause(0.05);
            end
        end
    end 
end
