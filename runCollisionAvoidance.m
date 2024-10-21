function runCollisionAvoidance(centerpnt, robot, q_start, q_end, vertex, faces, faceNormals, steps)
    % Function to perform collision detection and avoidance

    % Generate the joint trajectory using jtraj
    qMatrix = jtraj(q_start, q_end, steps);

    % First Run - Collision Detection
    [collisionIndices, collisionPoints] = detectCollisions(robot, qMatrix, faces, vertex, faceNormals);

    % Display collision results
    if ~isempty(collisionIndices)
        disp('Collisions occurred at the following indices:');
        disp(collisionIndices);
    else
        disp('No collisions detected.');
    end

    % Second Run - Primitive RRT-style Collision Avoidance
    qMatrixAvoidance = performCollisionAvoidance(robot, q_start, q_end, faces, vertex, faceNormals, steps);
    robot.model.animate(qMatrixAvoidance);

    % Helper Functions for Collision Detection and Avoidance
    function [collisionIndices, collisionPoints] = detectCollisions(robot, qMatrix, faces, vertex, faceNormals)
        collisionIndices = [];  % Initialize an array to store collision indices
        collisionPoints = [];   % Store the collision points

        for i = 1:5:size(qMatrix, 1)
            % Plot and animate the robot at the current configuration
            robot.model.plot(qMatrix(i, :));  
            robot.model.animate(qMatrix(i, :));  

            % Compute transformations for the robot links
            tr = GetLinkPoses(qMatrix(i, :), robot);

            % Collision Checking
            collisionDetected = false; % Initialize collision flag
            for j = 1:size(tr, 3) - 1 % Iterate through each link
                for faceIndex = 1:size(faces, 1) % Iterate through each triangular face of the obstacle
                    vertOnPlane = vertex(faces(faceIndex, 1)', :); % Get the vertices of the face
                    [intersectP, check] = LinePlaneIntersection(faceNormals(faceIndex, :), vertOnPlane, tr(1:3, 4, j)', tr(1:3, 4, j+1)'); 

                    % Check if there is an intersection and if it lies within the triangle
                    if check == 1 && IsIntersectionPointInsideTriangle(intersectP, vertex(faces(faceIndex, :)', :))
                        plot3(intersectP(1), intersectP(2), intersectP(3), 'g*'); % Plot intersection point
                        display('Collision detected!');
                        collisionDetected = true; % Set collision flag
                        collisionIndices = [collisionIndices; i];  % Store the index of the collision
                        collisionPoints = [collisionPoints; intersectP']; % Store collision point
                        break; % Exit the inner loop if a collision is found
                    end
                end
                if collisionDetected
                    break; % Exit the outer loop if a collision is found
                end
            end
            pause(0.05);  % Pause for a short duration to visualize the rotation
        end
    end

    function qMatrix = performCollisionAvoidance(robot, q_start, q_end, faces, vertex, faceNormals, steps)
        qWaypoints = [q_start; q_end];
        isCollision = true;
        checkedTillWaypoint = 1;
        qMatrix = [];
        failedAttempts = 0;
        maxFailedAttempts = 5;  % Limit the number of random configurations before backtracking

        while (isCollision)
            startWaypoint = checkedTillWaypoint;
            for i = startWaypoint:size(qWaypoints, 1) - 1
                qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:), deg2rad(5));
                if ~IsCollision(robot, qMatrixJoin, faces, vertex, faceNormals)
                    qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
                    robot.model.animate(qMatrixJoin);
                    isCollision = false;
                    checkedTillWaypoint = i + 1;
                    failedAttempts = 0;  % Reset failed attempts

                    % Try and join to the final goal (q_end)
                    qMatrixJoin = InterpolateWaypointRadians([qMatrix(end, :); q_end], deg2rad(5));
                    if ~IsCollision(robot, qMatrixJoin, faces, vertex, faceNormals)
                        qMatrix = [qMatrix; qMatrixJoin];
                        % Reached goal without collision, break out
                        break;
                    end
                else
                    % Randomly pick a pose that is not in collision
                    qRand = (2 * rand(1, 6) - 1) * pi;  % Random joint angles within limits
                    while IsCollision(robot, qRand, faces, vertex, faceNormals)
                        qRand = (2 * rand(1, 6) - 1) * pi;  % Keep generating until no collision
                    end
                    qWaypoints = [qWaypoints(1:i, :); qRand; qWaypoints(i+1:end, :)];
                    failedAttempts = failedAttempts + 1;

                    % If too many failed attempts, backtrack
                    if failedAttempts >= maxFailedAttempts
                        display('Too many failed attempts, returning to start...');
                        % Animate the robot back to the starting position smoothly
                        qReturn = jtraj(qMatrix(end, :), q_start, steps);  % Trajectory from current to start
                        for j = 1:size(qReturn, 1)
                            robot.model.animate(qReturn(j, :));  % Animate the robot back to the start position
                            pause(0.05);  % Add pause to visualize the movement
                        end
                        checkedTillWaypoint = 1;  % Reset to start
                        qMatrix = [];  % Clear the existing path
                        failedAttempts = 0;
                        break;
                    end
                    isCollision = true;
                    break;
                end
            end
        end
    end

    function result = IsIntersectionPointInsideTriangle(intersectP, triangleVerts)
        % Check if the intersection point lies inside a triangle
        u = triangleVerts(2,:) - triangleVerts(1,:);
        v = triangleVerts(3,:) - triangleVerts(1,:);

        uu = dot(u,u);
        uv = dot(u,v);
        vv = dot(v,v);

        w = intersectP - triangleVerts(1,:);
        wu = dot(w,u);
        wv = dot(w,v);

        D = uv * uv - uu * vv;

        s = (uv * wv - vv * wu) / D;
        if (s < 0.0 || s > 1.0)
            result = 0;
            return;
        end

        t = (uv * wu - uu * wv) / D;
        if (t < 0.0 || (s + t) > 1.0)
            result = 0;
            return;
        end

        result = 1;
    end

    function result = IsCollision(robot, qMatrix, faces, vertex, faceNormals, returnOnceFound)
        % Check if any part of the robot is in collision with obstacles
        if nargin < 6
            returnOnceFound = true;
        end
        result = false;

        for qIndex = 1:size(qMatrix,1)
            tr = GetLinkPoses(qMatrix(qIndex,:), robot);
            for i = 1:size(tr,3)-1    
                for faceIndex = 1:size(faces,1)
                    vertOnPlane = vertex(faces(faceIndex,1)',:);
                    [intersectP, check] = LinePlaneIntersection(faceNormals(faceIndex,:), vertOnPlane, tr(1:3,4,i)', tr(1:3,4,i+1)'); 
                    if check == 1 && IsIntersectionPointInsideTriangle(intersectP, vertex(faces(faceIndex,:)',:))
                        plot3(intersectP(1), intersectP(2), intersectP(3), 'g*');
                        display('Intersection');
                        result = true;
                        if returnOnceFound
                            return
                        end
                    end
                end    
            end
        end
    end

    function [transforms] = GetLinkPoses(q, robot)
        % Get the transformation matrices for each link of the robot given the joint angles q.
        links = robot.model.links;
        transforms = zeros(4, 4, length(links) + 1);
        transforms(:,:,1) = robot.model.base;

        for i = 1:length(links)
            L = links(1,i);
            current_transform = transforms(:,:, i);
            current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
            transforms(:,:,i + 1) = current_transform;
        end
    end

    function qMatrix = InterpolateWaypointRadians(waypointRadians, maxStepRadians)
        % Interpolate waypoints in radians for smooth transitions
        if nargin < 2
            maxStepRadians = deg2rad(1);
        end

        qMatrix = [];
        for i = 1: size(waypointRadians,1)-1
            qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
        end
    end

    function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
        % Fine interpolation between two joint configurations
        if nargin < 3
            maxStepRadians = deg2rad(1);
        end

        steps = 2;
        while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
            steps = steps + 1;
        end
        qMatrix = jtraj(q1,q2,steps);
    end
end
