classdef LightCurtain
    properties
        xMin        % X-coordinate minimum
        xMax        % X-coordinate maximum
        yConstant   % Constant Y-coordinate for the light curtain
        zMin        % Z-coordinate minimum
        zMax        % Z-coordinate maximum
        alpha       % Transparency of the light curtain
        edgeColor   % Edge color of the light curtain
    end
    
    methods
        % Constructor for LightCurtain class (defines the surface bounds)
        function obj = LightCurtain(xMin, xMax, zMin, zMax, yConstant, alpha, edgeColor)
            obj.xMin = xMin;
            obj.xMax = xMax;
            obj.zMin = zMin;
            obj.zMax = zMax;
            obj.yConstant = yConstant;  % Set the constant Y-coordinate for the light curtain (fixed Y position)
            obj.alpha = alpha;
            obj.edgeColor = edgeColor;

            % Plot the single surface of the light curtain
            obj.plotSurface();
        end

        % Plot the single surface of the light curtain
        function plotSurface(obj)
            [x, z] = meshgrid(obj.xMin:0.01:obj.xMax, obj.zMin:0.01:obj.zMax);  % Create grid for X and Z
            y = obj.yConstant * ones(size(x));  % Use a constant Y value to create the surface
            surf(x, y, z, 'FaceAlpha', obj.alpha, 'EdgeColor', obj.edgeColor);  % Plot the surface with transparency

            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            view(3);  % Set a 3D view for better visualization
        end

        % Method to load and transform object vertices from a .ply file
        function [objectVertices, objectHandle] = loadObject(obj, plyFilePath, transformMatrix)
            % Load vertices from the ply file
            [f, v, data] = plyread(plyFilePath, 'tri');
            objectVertices = v;
            
            % Apply the transformation matrix to the object's vertices
            objectVertices = obj.applyTransformation(objectVertices, transformMatrix);

            % Plot the object
            objectHandle = trisurf(f, objectVertices(:,1), objectVertices(:,2), objectVertices(:,3), 'FaceAlpha', 1);
        end

        % Method to detect if an object crosses the light curtain
        function isBreached = detectObject(obj, objectVertices)
            % Input:
            % - objectVertices: Nx3 matrix of the object's vertices
            % Output:
            % - isBreached: Boolean indicating whether the object breached the light curtain
            
            % Check if any vertex has crossed the Y-plane of the light curtain
            yCrossing = objectVertices(:,2) >= obj.yConstant;  % Check Y-plane
            xWithinBounds = objectVertices(:,1) >= obj.xMin & objectVertices(:,1) <= obj.xMax;  % X bounds
            zWithinBounds = objectVertices(:,3) >= obj.zMin & objectVertices(:,3) <= obj.zMax;  % Z bounds
            
            % Breach occurs if any vertex satisfies all conditions
            isBreached = any(yCrossing & xWithinBounds & zWithinBounds);
            
            if isBreached
                fprintf("Object has breached the light curtain!\n");
            end
        end
        
        % Method to update object vertices during motion
        function updatedVertices = updateObjectPosition(obj, originalVertices, transformMatrix)
            % Apply the new transformation to the object's vertices
            updatedVertices = obj.applyTransformation(originalVertices, transformMatrix);
        end

        % Apply a transformation matrix to the object's vertices
        function transformedVertices = applyTransformation(obj, vertices, transformMatrix)
            % Apply the 4x4 transformation matrix to Nx3 vertices
            numVertices = size(vertices, 1);
            homogeneousVertices = [vertices, ones(numVertices, 1)] * transformMatrix';  % Apply transformation
            transformedVertices = homogeneousVertices(:, 1:3);  % Extract the updated vertices
        end
    end
end
