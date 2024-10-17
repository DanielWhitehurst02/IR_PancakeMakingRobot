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
            self.obstaclesupdated = true;
        end

        %function to check path for collisions
        function isCollision = pathCheck(path, obstaclePoints)
            %pathCheck checks a path for collisions
            %   Detailed explanation goes here
            isCollision = false;
            for i=1:length(path)
                for j=1:size(obstaclePoints)
                    dist = sqrt((obstaclePoints(j,1)-path(1,i))^2+(obstaclePoints(j,2)-path(2,i))^2+(obstaclePoints(j,3)-path(3,i))^2);
                    if any(dist < 0.1)
                        isCollision = true;
                        break;
                    end
                end
            end
        end

        function [stopped,currentgoal] = eStop(self, collision)
            %eStop Function to handle an emergency stop command
            %   Detailed explanation goes here
            if collision == true
                self.running = false;
            end
        end


        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end