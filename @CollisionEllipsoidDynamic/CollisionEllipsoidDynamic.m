classdef CollisionEllipsoidDynamic < handle
    properties
        endEffectorRadius   % Radius of end-effector ellipsoid
        robotModel          % Robot model (e.g., UR3)
        obstaclePoints      % Points representing dynamic obstacles (e.g., cube points)
    end
    
    methods
        % Constructor focusing on end effector only
        function obj = CollisionEllipsoidDynamic(robotModel, endEffectorRadius)
            obj.robotModel = robotModel;
            obj.endEffectorRadius = endEffectorRadius;
            obj.obstaclePoints = [];  % Initialize empty to set dynamically
        end
        
        % Set or update obstacle points dynamically (e.g., cube points)
        function setObstaclePoints(obj, newObstaclePoints)
            obj.obstaclePoints = newObstaclePoints;
        end
        
        % Check for collision with the end effector only
        function isCollision = detectCollision(obj)
            isCollision = false;
            q = obj.robotModel.model.getpos();  % Get current joint positions
            trEndEffector = obj.robotModel.model.fkine(q).T;  % Get end effector transformation matrix
            
            % Transform obstacle points to end-effector frame
            transformedPoints = (inv(trEndEffector) * [obj.obstaclePoints, ones(size(obj.obstaclePoints, 1), 1)]')';
            transformedPoints = transformedPoints(:, 1:3);  % Extract x, y, z
            
            % Compute distance to ellipsoid at the end effector
            distToEllipsoid = sum(transformedPoints.^2, 2) / obj.endEffectorRadius^2;
            if any(distToEllipsoid < 1)
                isCollision = true;
            end
        end
    end
end
