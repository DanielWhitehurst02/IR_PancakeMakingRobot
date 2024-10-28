classdef CollisionEllipsoid < handle
    properties
        ellipsoidCenters;  % Centers of ellipsoids for robot links
        ellipsoidRadii;    % Radii of ellipsoids
        robotModel;        % Robot model (e.g., UR3)
        ellipsoidPlotHandles; % Store plot handles for ellipsoids
    end
    
    methods
        % Constructor
        function obj = CollisionEllipsoid(robotModel, centers, radii)
            obj.robotModel = robotModel;
            obj.ellipsoidCenters = centers;
            obj.ellipsoidRadii = radii;
            obj.ellipsoidPlotHandles = [];
        end
        
        % Function to plot ellipsoids without changing zoom
        function drawEllipsoids(obj)
            [az, el] = view;  % Store current view
            originalLimits = axis;  % Store current axis limits
        
            q = obj.robotModel.model.getpos();  % Get joint positions
            tr = obj.computeTransforms(q);  % Compute link transformations
        
            % Set opacity directly based on visibility flag
            if obj.visible
                opacity = 0.1;  % Visible
            else
                opacity = 0;    % Invisible
            end
        
            for i = 1:size(obj.ellipsoidRadii, 1)
                transformedCenter = tr(:,:,i) * [0, 0, 0, 1]';
                transformedCenter = transformedCenter(1:3)';
        
                [X, Y, Z] = obj.generateEllipsoid(transformedCenter, obj.ellipsoidRadii(i, :));
        
                if length(obj.ellipsoidPlotHandles) < i || isempty(obj.ellipsoidPlotHandles{i}) || ~isvalid(obj.ellipsoidPlotHandles{i})
                    hold on;
                    obj.ellipsoidPlotHandles{i} = surf(X, Y, Z, 'FaceAlpha', opacity, 'EdgeColor', 'none');
                else
                    set(obj.ellipsoidPlotHandles{i}, 'XData', X, 'YData', Y, 'ZData', Z, 'FaceAlpha', opacity);
                end
            end
        
            view(az, el);  % Restore view
            axis(originalLimits);  % Restore axis limits
        end


        
        % Function to generate an ellipsoid at a specific center and with specific radii
        function [X, Y, Z] = generateEllipsoid(~, center, radii)
            [X, Y, Z] = ellipsoid(center(1), center(2), center(3), radii(1), radii(2), radii(3));
        end
        
        % Function to check for collisions with an obstacle's mesh points
        function isCollision = detectCollision(obj, obstaclePoints)
            isCollision = false;
            q = obj.robotModel.model.getpos();
            tr = obj.computeTransforms(q);
            
            % Iterate through each ellipsoid and check for collisions
            for i = 1:length(obj.ellipsoidCenters)
                transformedPoints = (inv(tr(:, :, i)) * [obstaclePoints, ones(size(obstaclePoints, 1), 1)]')';
                transformedPoints = transformedPoints(:, 1:3);
                
                distToEllipsoid = obj.computeAlgebraicDistance(transformedPoints, obj.ellipsoidCenters(i, :), obj.ellipsoidRadii(i, :));
                
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
