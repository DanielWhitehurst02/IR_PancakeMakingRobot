clc;
clear;
clear all;

% Initial joint configuration of UR10
q0 = [pi/2 -pi/3 -pi/3 -pi/6 0 0];

% Initialize the custom UR10 model
robot = UR10();
robot.model.animate(q0);

% Camera parameters
focalLength = 0.08;
pixelSize = 10e-5;
resolution = [1024, 1024];
cam = CentralCamera('focal', focalLength, 'pixel', pixelSize, 'resolution', resolution, ...
    'centre', resolution / 2, 'name', 'UR10Camera');

% Position camera on the end-effector 
Tc0 = robot.model.fkine(robot.model.getpos());  % Get the end-effector pose (SE3 object)
cam.T = Tc0.T;  % Attach camera to the end-effector, convert SE3 to matrix

% Define the target points in 3D (3xN matrix)
P1 = [1.8, -0.25, 1.25]; 
P2 = [1.8, 0.25, 1.25]; 
P3 = [1.8, 0.25, 0.75]; 
P4 = [1.8, -0.25, 0.75]; 
P = [P1; P2; P3; P4]';  % Concatenate points as a 3x4 matrix

% Desired pixel locations in the image (u*, v*)
p1Star = [662 362];
p2Star = [362 362];
p3Star = [362 662];
p4Star = [662 662];
pStar = [p1Star; p2Star; p3Star; p4Star]';  % Concatenate points

% Plot 3D points in the camera view
figure;
axis([-2 2 -2 2 0 2])
plot_sphere(P, 0.05, 'b');  % Represent points as small spheres

% Initialize the IBVS Loop
% Parameters for visual servoing
lambda = 0.1;  % Gain of control (adjust if convergence is too slow/fast)
depth = 1.8;   % Approximate distance to the target
fps = 25;      % Frames per second (control loop speed)
maxSteps = 200;  % Maximum number of iterations
timestep = 1/fps;

% Initialize error trajectory for plotting
errorHistory = [];

for k = 1:maxSteps
    % 1. Project points into the image (current view)
    uv = cam.plot(P, 'o');  % Plot 3D points as 2D image points in the camera view
    
    % 2. Calculate error between current and desired image points
    errorFeatures = pStar - uv;
    errorFeatures = errorFeatures(:);  % Reshape into a column vector

    % Store error for plotting
    errorHistory = [errorHistory, norm(errorFeatures)];
    
    % 3. Image Jacobian (compute based on current 2D points)
    J = cam.visjac_p(uv, depth);
    
    % 4. Camera velocity
    v = lambda * pinv(J) * errorFeatures;
    
    % 5. Robot Jacobian for the current pose
    J_robot = robot.model.jacob0(q0);  
    
    % 6. Compute joint velocities
    qdot = pinv(J_robot) * v;
    
    % 7. Limit joint velocities to 180 degrees/sec (in radians)
    qdot = min(max(qdot, -pi), pi);

    % 8. Update joint angles
    q0 = q0 + qdot' * timestep;
    
    % 9. Update end-effector pose
    Tc0 = robot.model.fkine(q0);  % Tc0 is an SE3 object
    
    % 10. Update camera position
    cam.T = Tc0.T;  % Convert SE3 to matrix
    
    % 11. Plot the camera and robot in 3D and update 2D view
    robot.model.animate(q0);
    cam.plot_camera();  % Update camera view
    drawnow;

    % 12. Check for convergence (when the error is small)
    if norm(errorFeatures) < 1e-2
        disp('Target reached!');
        break;
    end
end

% 13. Plot the error history (feature error vs. time steps)
figure;
plot(errorHistory);
xlabel('Time steps');
ylabel('Feature error');
title('Feature Error vs Time');
grid on;

% End of IBVS loop
