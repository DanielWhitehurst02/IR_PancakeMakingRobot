function cubePoints = meshcube(width, height, rot, density, center)
    %Compute width and height
    hwidth = width/2;
    hheight = height/2;
    
    %Define one side of the cube
    [Y,Z] = meshgrid(-hwidth:density:hwidth,-hheight:density:hheight);
    [Ys,Zs] = meshgrid(-hheight:density:hheight,-hheight:density:hheight);
    sizeMat = size(Y);
    X = repmat(hheight,sizeMat(1),sizeMat(2));

    sizeMats = size(Ys);
    Xs = repmat(hheight,sizeMats(1),sizeMats(2));
    
    % Combine one surface as a point cloud
    cubePoints = [X(:),Y(:),Z(:)];
    cubePointsSide = [Xs(:),Ys(:),Zs(:)];

    % Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
    cubePoints = [ cubePoints ...
             ; cubePointsSide * rotz(pi/2) + repmat([0,-hwidth+hheight,0],size(cubePointsSide,1),1)...
             ; cubePoints * rotz(pi) ...
             ; cubePointsSide * rotz(3*pi/2) + repmat([0,hwidth-hheight,0],size(cubePointsSide,1),1)...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];

    %Rotate cube based on input
    cubePoints = cubePoints * rotz(rot(1)) * roty(rot(2)) * rotx(rot(3));
    %Transform points based on cube centre
    cubePoints = cubePoints + repmat(center,size(cubePoints,1),1);

    % cubePoints = cubePoints * transl(0.1,0.1,0.1);

    % Plot the cube's point cloud 
    cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
   

end