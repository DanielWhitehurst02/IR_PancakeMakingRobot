clc;
clear;
close all;

%% Step 1: Load and Process the Checkerboard Image
% Load a checkerboard image (replace 'image_1.jpg' with your actual image file)
I1 = imread('image_1.jpg');  

% Convert to grayscale if necessary
if size(I1, 3) == 3
    I1 = rgb2gray(I1);
end

%% Step 2: Feature Detection using Harris Corner Detector
% Use Harris corner detector to find checkerboard corners
corners1 = detectHarrisFeatures(I1, 'MinQuality', 0.1);

%% Step 3: Display the Detected Corners for Verification
figure;
imshow(I1);  % Display the image before plotting corners
hold on;
plot(corners1.selectStrongest(4));  % Plot detected corners
title('Detected Corners in Checkerboard Image');
hold off;

% Extract the strongest 4 corner points (detected points for the checkerboard)
detectedPoints1 = corners1.selectStrongest(4).Location;  % For checkerboard image

%% Step 4: Define Desired Checkerboard Position in 3D (Real-World Coordinates)
% These are the real-world 3D coordinates of the checkerboard in the robot's workspace
checkerboard3D = [
    0.5, 0.0, 0.1;  % Top-left corner in 3D space
    0.7, 0.0, 0.1;  % Top-right corner in 3D space
    0.5, 0.2, 0.1;  % Bottom-left corner in 3D space
    0.7, 0.2, 0.1   % Bottom-right corner in 3D space
];

%% Step 5: Camera Parameters (Simulated)
% Simulate camera intrinsic parameters (adjust these based on your setup)
focal_length = [985, 978];  % Focal length [fx, fy]
principal_point = [932, 542];  % Principal point at center of the image
Z = 1.2;  % Depth (distance to the checkerboard in meters)
lambda = 0.18;  % Control gain for visual servoing

%% Step 6: Define Desired 2D Positions of the Checkerboard in Camera Image Plane
% These are the 2D image coordinates that correspond to the detected points
desired_corners = [
    20, 20;
    1550, 20;
    20, 750;
    1550, 800
];

%% Step 7: Compute Reprojection Error for the Detected Points
% Reprojection error for the checkerboard corners in image 1
e1 = reshape((desired_corners - detectedPoints1)', [], 1);  % Error for image 1

%% Step 8: Interaction Matrix Calculation for Detected Corners
Lx1 = [];

% Construct the interaction matrix for each detected corner in the image
for i = 1:size(detectedPoints1, 1)
    x = detectedPoints1(i, 1);  % u-coordinate in image
    y = detectedPoints1(i, 2);  % v-coordinate in image
    
    % Interaction matrix for the i-th corner in image 1
    Lxi1 = zeros(2, 6);
    Lxi1(1,1) = -1/Z;
    Lxi1(1,2) = 0;
    Lxi1(1,3) = x/Z;
    Lxi1(1,4) = x * y;
    Lxi1(1,5) = -(1 + x^2);
    Lxi1(1,6) = y;
    
    Lxi1(2,1) = 0;
    Lxi1(2,2) = -1/Z;
    Lxi1(2,3) = y/Z;
    Lxi1(2,4) = 1 + y^2;
    Lxi1(2,5) = -x * y;
    Lxi1(2,6) = -x;
    
    Lx1 = [Lx1; Lxi1];
end

%% Step 9: Compute the Camera Velocity Using the Control Law
% Calculate the pseudoinverse of the interaction matrix
Lx1_pseudo_inverse = pinv(Lx1);  % Moore-Penrose pseudoinverse of the interaction matrix
Vc1 = -lambda * Lx1_pseudo_inverse * e1;  % Camera velocity for image 1

%% Step 10: Inverse Kinematics to Move UR3 Robot to the Checkerboard Position
% Load the UR3 robot using Peter Corke's Robotics Toolbox
robot = UR3();  % Assuming robot is created using Peter Corke's Robotics Toolbox

% Define the target pose for the end-effector as a transformation matrix
targetPose = transl(mean(checkerboard3D, 1));  % Translate to the average checkerboard position

% Perform inverse kinematics using `robot.model.ikine`
initialGuess = robot.model.getpos();  % Get the robot's current joint configuration (if applicable)
configSol = robot.model.ikine(targetPose, 'q0', initialGuess);  % Solve IK (single output)

%% Step 11: Visualize the UR3 Robot Moving to the Checkerboard Position
figure;
robot.model.plot(configSol);  % Plot the robot in the new configuration
title('UR3 Robot Moving to Checkerboard Position');
