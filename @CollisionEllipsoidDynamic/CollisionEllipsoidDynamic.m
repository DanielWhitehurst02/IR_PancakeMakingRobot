classdef CollisionEllipsoidDynamic < handle
    properties
        ellipsoidCenters     % Centers of ellipsoids for robot links
        ellipsoidRadii       % Radii of ellipsoids
        robotModel           % Robot model (e.g., UR3)
        ellipsoidPlotHandles % Plot handles for ellipsoids
        obstaclePoints       % Points representing dynamic obstacles (e.g., cube points)
    end
    
    methods
        % Constructor
        function obj = CollisionEllipsoidDynamic(robotModel, centers, radii)
            obj.robotModel = robotModel;
            obj.ellipsoidCenters = centers;  % Initialize centers here
            obj.ellipsoidRadii = radii;
            obj.ellipsoidPlotHandles = [];
            obj.obstaclePoints = [];  % Initialize empty to set dynamically
        end
        
        % Set or update obstacle points dynamically (e.g., cube points)
        function setObstaclePoints(obj, newObstaclePoints)
            obj.obstaclePoints = newObstaclePoints;
        end

        % Draw or update ellipsoids based on current centers and transformations
        function drawEllipsoids(obj, plotFlag)
            if nargin < 2
                plotFlag = false;  % Default to not plotting
            end
            
            % Get current joint positions and transformations
            q = obj.robotModel.model.getpos();  
            tr = obj.computeTransforms(q);  

            for i = 1:length(obj.ellipsoidCenters)
                % Update ellipsoid center based on the current transformation
                transformedCenter = tr(:,:,i) * [obj.ellipsoidCenters(i, :), 1]';
                transformedCenter = transformedCenter(1:3)';  % Extract transformed x, y, z
                
                % Generate ellipsoid coordinates
                [X, Y, Z] = obj.generateEllipsoid(transformedCenter, obj.ellipsoidRadii(i, :));
                
                % Plot only if plotFlag is true
                if plotFlag
                    if length(obj.ellipsoidPlotHandles) < i || isempty(obj.ellipsoidPlotHandles{i}) || ~isvalid(obj.ellipsoidPlotHandles{i})
                        hold on;
                        obj.ellipsoidPlotHandles{i} = surf(X, Y, Z, 'FaceAlpha', 0.1, 'EdgeColor', 'none');
                    else
                        set(obj.ellipsoidPlotHandles{i}, 'XData', X, 'YData', Y, 'ZData', Z);
                    end
                end
            end
        end

        % Function to generate an ellipsoid at a specific center and with specific radii
        function [X, Y, Z] = generateEllipsoid(~, center, radii)
            [X, Y, Z] = ellipsoid(center(1), center(2), center(3), radii(1), radii(2), radii(3));
        end
        
        % Function to check for collisions with the latest obstacle points
        function isCollision = detectCollision(obj)
            isCollision = false;
            q = obj.robotModel.model.getpos();
            tr = obj.computeTransforms(q);

            % Check collision with dynamically updated obstacle points
            for i = 1:length(obj.ellipsoidCenters)
                % Transform obstacle points to the ellipsoid's local frame
                transformedPoints = (inv(tr(:, :, i)) * [obj.obstaclePoints, ones(size(obj.obstaclePoints, 1), 1)]')';
                transformedPoints = transformedPoints(:, 1:3);

                % Compute the distance to the ellipsoid
                distToEllipsoid = obj.computeAlgebraicDistance(transformedPoints, [0, 0, 0], obj.ellipsoidRadii(i, :));
                if any(distToEllipsoid < 1)
                    isCollision = true;
                    break;
                end
            end
        end

        % Function to compute forward kinematics of the robot's links
        function tr = computeTransforms(obj, q)
            tr = zeros(4, 4, obj.robotModel.model.n + 1);
            tr(:, :, 1) = obj.robotModel.model.base;
            for i = 1:obj.robotModel.model.n
                L = obj.robotModel.model.links(i);
                tr(:, :, i + 1) = tr(:, :, i) * trotz(q(i)) * transl(0, 0, L.d) * transl(L.a, 0, 0) * trotx(L.alpha);
            end
        end
        
        % Function to compute algebraic distance from a point to an ellipsoid
        function algebraicDist = computeAlgebraicDistance(~, points, center, radii)
            algebraicDist = ((points(:, 1) - center(1)) / radii(1)).^2 + ...
                            ((points(:, 2) - center(2)) / radii(2)).^2 + ...
                            ((points(:, 3) - center(3)) / radii(3)).^2;
        end
    end
end
