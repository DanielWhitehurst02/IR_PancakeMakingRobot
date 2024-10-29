function [cubePoints, vertex, face, faceNormals, redHandle] = CollisionMesh(width, height, rot, density, center)
    % Compute half width and half height for the cube
    hwidth = width / 2;
    hheight = height / 2;
    
    % Define one side of the cube
    [Y, Z] = meshgrid(-hwidth:density:hwidth, -hheight:density:hheight);
    sizeMat = size(Y);
    X = repmat(hheight, sizeMat(1), sizeMat(2));
    
    % Combine one surface as a point cloud
    cubePoints = [X(:), Y(:), Z(:)];

    % Make a cube by rotating the single side by 0, 90, 180, 270, and around y to make the top and bottom faces
    cubePoints = [cubePoints ...
                 ; cubePoints * rotz(pi/2) ...
                 ; cubePoints * rotz(pi) ...
                 ; cubePoints * rotz(3*pi/2) ...
                 ; cubePoints * roty(pi/2) ...
                 ; cubePoints * roty(-pi/2)];

    % Rotate cube based on input rotation
    cubePoints = cubePoints * rotz(rot(1)) * roty(rot(2)) * rotx(rot(3));
    
    % Transform points based on cube center
    cubePoints = cubePoints + repmat(center, size(cubePoints, 1), 1);
    
    % Plot the cube's point cloud (only red)
    % Plot the cube's point cloud (only red) if requested

    %redHandle = plot3(cubePoints(:,1), cubePoints(:,2), cubePoints(:,3), 'r*');
    redHandle = [];

    % Define plotOptions for the RectangularPrism
    plotOptions.plotVerts = false;  
    plotOptions.plotEdges = false;  
    plotOptions.plotFaces = false;  
    
    % Inflate the rectangular prism to encapsulate the cube
    inflateFactor = 1.05;  
    inflatedWidth = width * inflateFactor;
    inflatedHeight = height * inflateFactor;

    hInflatedWidth = inflatedWidth / 2;
    hInflatedHeight = inflatedHeight / 2;
    
    % Define lower and upper points for the inflated RectangularPrism
    lower = center - [hInflatedWidth, hInflatedHeight, hInflatedWidth];
    upper = center + [hInflatedWidth, hInflatedHeight, hInflatedWidth];
    
    % Call RectangularPrism and return the values
    [vertex, face, faceNormals] = RectangularPrism(lower, upper, plotOptions);
end
