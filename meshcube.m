function cubePoints = meshcube(width, height, rot, density, center)
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
    % cubePoints = [ cubePoints * rotz(rot(1)) * roty(rot(2)) * rotx(rot(3))...
    %              ; cubePoints * rotz(rot(1)+pi/2) * roty(rot(2))* rotx(rot(3))...
    %              ; cubePoints * rotz(rot(1)+pi) * roty(rot(2)) * rotx(rot(3))...
    %              ; cubePoints * rotz(rot(1)+3*pi/2) * roty(rot(2)) * rotx(rot(3))...
    %              ; cubePoints * roty(rot(2)+pi/2) * rotz(rot(1))*rotx(rot(3))...
    %              ; cubePoints * roty(rot(2)-pi/2) * rotz(rot(1))*rotx(rot(3))];         
    
    cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)]; 
    cubePoints = cubePoints * rotz(rot(1)) * roty(rot(2)) * rotx(rot(3));

    % Plot the cube's point cloud         
    % cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
    cubePoints = cubePoints + repmat(center,size(cubePoints,1),1);
    cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
    
    axis equal

end