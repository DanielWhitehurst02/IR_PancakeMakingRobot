function cubePoints = meshcube(width, height,rot,density,center)
    % One side of the cube
    
    hwidth = width/2;
    hheight = width/2;

    [Y,Z] = meshgrid(-hwidth:density:hwidth,-hheight:density:hheight);
    sizeMat = size(Y);
    X = repmat(hheight,sizeMat(1),sizeMat(2));
    % oneSideOfCube_h = surf(X,Y,Z);
    
    % Combine one surface as a point cloud
    cubePoints = [X(:),Y(:),Z(:)];
    
    % Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
    cubePoints = [ cubePoints * rotz(rot)...
                 ; cubePoints * rotz(rot+pi/2)...
                 ; cubePoints * rotz(rot+pi) ...
                 ; cubePoints * rotz(rot+3*pi/2) ...
                 ; cubePoints * roty(pi/2) * rotz(rot) ...
                 ; cubePoints * roty(pi/2) * rotz(rot)];         
    
    % Plot the cube's point cloud         
    % cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
    cubePoints = cubePoints + repmat(center,size(cubePoints,1),1);
    cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
    
    axis equal

end