% Load the pancake mesh and set the initial position
pancake_mesh = PlaceObject('pancake2.ply');
initialPosition = transl(0, 0.3, 0.05); % Initial landing position
axis([-2 2 -2 2 -1 2]);

% Set animation parameters
liftHeight = 0.2;        % Maximum height of the lift
liftSteps = 50;          % Number of steps to lift
liftSpeed = 0.02;        % Speed of lifting
pauseDuration = 0.5;     % Pause duration at the top
flipSteps = 30;          % Number of steps for flip and descent
flipSpeed = 0.03;        % Speed of descent

% Get pancake vertices for transformation control
pancake_vertices = get(pancake_mesh, 'Vertices');

% Call the function
pancake_vertices = animatePancakeFlip(pancake_mesh, pancake_vertices, initialPosition, liftHeight, liftSteps, liftSpeed, pauseDuration, flipSteps, flipSpeed);

function transformedVertices = animatePancakeFlip(pancake_mesh, pancake_vertices, initialPosition, liftHeight, liftSteps, liftSpeed, pauseDuration, flipSteps, flipSpeed)
    % Set initial position of the pancake
    transformedVertices = [pancake_vertices, ones(size(pancake_vertices, 1), 1)] * initialPosition';
    set(pancake_mesh, 'Vertices', transformedVertices(:, 1:3)); % Set initial position

    % Parameters for the flipping motion
    flipRotation = pi;  % Rotate pancake 180 degrees during flip

    %% Animation for Lifting the Pancake
    for i = 1:liftSteps
        % Calculate height offset for linear upward motion
        heightOffset = liftHeight * (i / liftSteps);

        % Transformation matrix for lifting motion
        pancakeLiftTransformation = transl(0, 0, heightOffset);

        % Apply transformation to pancake vertices
        transformedVertices = [pancake_vertices, ones(size(pancake_vertices, 1), 1)] * pancakeLiftTransformation' * initialPosition';
        set(pancake_mesh, 'Vertices', transformedVertices(:, 1:3));  % Update pancake position

        drawnow;  % Refresh the plot
        pause(liftSpeed);  % Control speed for the lift phase
    end

    % Pause at the top before flipping
    pause(pauseDuration);

    %% Animation for Flipping and Descending the Pancake
    for i = 1:flipSteps
        % Calculate descent offset to return to the initial position
        descentOffset = liftHeight * (1 - (i / flipSteps));  % Gradual return to original height

        % Calculate rotation angle for gradual 180-degree flip
        rotationAngle = flipRotation * (i / flipSteps);

        % Transformation matrix for flipping and descending motion
        pancakeFlipTransformation = transl(0, 0, descentOffset) * trotx(rotationAngle);

        % Apply flip and descent transformation based on the initial position
        transformedVertices = [pancake_vertices, ones(size(pancake_vertices, 1), 1)] * pancakeFlipTransformation' * initialPosition';
        set(pancake_mesh, 'Vertices', transformedVertices(:, 1:3));  % Update pancake position

        drawnow;  % Refresh the plot
        pause(flipSpeed);  % Control speed for the flip and descent phase
    end

    disp('Pancake flipping animation complete.');
end




%% Next animation

% Load the pancake mesh and set the initial position
pancake_mesh = PlaceObject('pancake2.ply');
initialPosition = transl(0, 0.3, 0); % Initial position in x, y, z
axis([-2 2 -2 2 -1 2]);

% Set animation parameters
moveDistance = 0.5;    % Distance to move along the x-axis
moveSteps = 50;        % Number of steps for the movement
moveSpeed = 0.02;      % Speed of the movement (seconds per step)

% Get pancake vertices for transformation control
pancake_vertices = get(pancake_mesh, 'Vertices');

% Call the function to move the pancake along the x-axis
transformedVertices = animatePancakeMoveX(pancake_mesh, pancake_vertices, initialPosition, moveDistance, moveSteps, moveSpeed);

function transformedVertices = animatePancakeMoveX(pancake_mesh, pancake_vertices, initialPosition, moveDistance, moveSteps, moveSpeed)
    % Expand pancake_vertices to homogeneous coordinates
    pancake_vertices_homogeneous = [pancake_vertices, ones(size(pancake_vertices, 1), 1)];

    %% Animation for Moving Along the X-axis
    for i = 1:moveSteps
        % Calculate x-offset for movement along the x-axis
        xOffset = moveDistance * (i / moveSteps);

        % Transformation matrix for x-axis motion
        pancakeMoveTransformation = transl(xOffset, 0, 0);

        % Apply transformation to pancake vertices based on initial position
        transformedVertices = pancake_vertices_homogeneous * (initialPosition * pancakeMoveTransformation)';

        % Update pancake mesh vertices
        set(pancake_mesh, 'Vertices', transformedVertices(:, 1:3));  % Update pancake position

        drawnow;  % Refresh the plot
        pause(moveSpeed);  % Control speed for the x-axis movement
    end

    disp('Pancake x-axis movement animation complete.');
end
