function test(app)
    %% Initialization of the Robot
    robot1 = UR3e(transl(0, 0, 0));  

    leftFinger = Finger();  
    rightFinger = Finger2();

    centerPoints = [0.0, 0.0, 0.05; 
                    0.0, -0.01, 0.01; 
                    0.125, 0.0, 0.125; 
                    0.105, 0.0, 0.05;  
                    0.0, 0.0, 0.01;  
                    0.0, 0.0, 0.06;   
                    0.0, 0.0, 0.0];   

    radii = [0.08, 0.09, 0.055;  
             0.075, 0.085, 0.075;
             0.175, 0.08, 0.085; 
             0.15, 0.06, 0.085; 
             0.04, 0.055, 0.065;
             0.04, 0.045, 0.125; 
             0.0, 0.0, 0.0]; 

    collisionEllipsoid = CollisionEllipsoidDynamic(robot1, radii);
    collisionEllipsoid.setObstaclePoints([]); 
    collisionEllipsoid.ellipsoidCenters = centerPoints;
    collisionEllipsoid.drawEllipsoids();

    hold on;
    
    spawnInterval = 2; 
    lastToggleTime = tic;  
    obstacleSpawned = false;  

    center = [-0.5, 0, 0.5];
    rot = [pi/3, pi/4, 2*pi/7];

    redHandle = [];  % Plot handle for red points

    while true
        if toc(lastToggleTime) > spawnInterval
            lastToggleTime = tic;  

            if obstacleSpawned
                % Despawn obstacle
                collisionEllipsoid.setObstaclePoints([]);
                if isgraphics(redHandle), delete(redHandle); end
                redHandle = [];
                obstacleSpawned = false;
            else
                % Spawn new obstacle
                [cubePoints, ~, ~, ~, redHandle] = CollisionMesh(0.5, 0.5, rot, 0.02, center);
                collisionEllipsoid.setObstaclePoints(cubePoints);
                obstacleSpawned = true;
            end
        end

        if obstacleSpawned && collisionEllipsoid.detectCollision()
            disp('Collision detected with dynamically spawned obstacle!');
        else
            disp('No collision')
        end
        
        pause(0.1);
    end
end
