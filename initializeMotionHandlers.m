function [motionHandler1, motionHandler2] = initializeMotionHandlers()

    %% Setting up workspace
    workspace = [-2, 2.5, -2, 2.5, 0, 2];
    hold on;
    axis equal;
    axis(workspace);
    surf([-2.5, -2.5; 3.5, 3.5], [-2.5, 2.5; -2.5, 2.5], [0.01, 0.01; 0.01, 0.01], ...
        'CData', imread('florring.jpg'), 'FaceColor', 'texturemap');

    % Place objects in the environment
    PlaceObject('BlueSyrupBottle.ply', [1.5, 0, 0.55]);
    PlaceObject('PinkSyrupBottle.ply', [1.5, 0.1, 0.55]);
    PlaceObject('BrownSyrupBottle.ply', [1.5, -0.1, 0.55]);
    PlaceObject('DoughBottle.ply', [0.2, -0.1, 0.5]);
    PlaceObject('Stove.ply', [-0.2, 0.5, 0.5]);
    PlaceObject('ConveyorBelt.ply', [0.7, -0.5, 0]);
    PlaceObject('Plate.ply', [0.03, -0.5, 0.6]);
    PlaceObject('Table.ply', [-1.7, 0.5, -0.15]);

    %% UR3 motion Handler
    robot1 = UR3e(transl(-0.3, 0.2, 0.5));
    centerPoints_UR3 = [
        0.0, 0.0, 0.05;
        0.0, -0.01, 0.01;
        0.125, 0.0, 0.125;
        0.105, 0.0, 0.05;
        0.0, 0.0, 0.01;
        0.0, 0.0, 0.06;
        0.0, 0.0, 0.0;
    ];
    radii_UR3 = [
        0.08, 0.09, 0.055;
        0.075, 0.085, 0.075;
        0.175, 0.08, 0.085;
        0.15, 0.06, 0.085;
        0.04, 0.055, 0.065;
        0.04, 0.045, 0.125;
        0.0, 0.0, 0.0;
    ];
    obstaclePoints1 = [0.1, 1, 0.1];
    motionHandler1 = MotionHandler(robot1, centerPoints_UR3, radii_UR3, obstaclePoints1);

    %% Panda Motion Handler
    robot2 = Panda(transl(0.5, 0.5, 0.5));

    robot2.CreateModel()
    centerPoints_panda = [
        -0.025, 0.0, 0.06;
        0.0, 0.0975, -0.035;
        0.0, 0.03, 0.075;
        -0.075, -0.085, 0.0;
        0.05, 0.0, 0.035;
        0.0, 0.125, 0.025;
        -0.05, 0.025, 0.0;
    ];
    radii_Panda = [
        0.135, 0.125, 0.075;
        0.1, 0.175, 0.1;
        0.125, 0.12, 0.125;
        0.075, 0.075, 0.095;
        0.125, 0.125, 0.095;
        0.085, 0.15, 0.105;
        0.110, 0.1, 0.085;
    ];

    obstaclePoints2 = [0.05, 0, 0.5];

    motionHandler2 = MotionHandler(robot2, centerPoints_panda, radii_Panda, obstaclePoints2);
end
