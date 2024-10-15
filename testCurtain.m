% Clear and close figures
close all;
clc;

% Create the light curtain as a single surface
lightCurtain = LightCurtain(-1.6, 1.7, 0, 1.5, 0.2, 0.1, 'none');

% Load the hand object from the .ply file and apply an initial transformation
initialTransform = transl(0, 0, 1) * trotx(pi/2);  % Define an initial transformation matrix
hold on;
% Load the hand.ply file and plot it using the LightCurtain class method
[handVertices, handHandle] = lightCurtain.loadObject('hand.ply', initialTransform);

% Simulate object movement over time (updating the hand position)
for i = 1:20
    % Apply a new transformation to simulate hand movement
    newTransform = transl(0, 0.1 * i, 0) * initialTransform;  % Translate the hand upwards over time
    
    % Update the vertices of the hand to reflect the movement
    updatedVertices = lightCurtain.updateObjectPosition(handVertices, newTransform);
    
    % Update the plot with the new vertices
    set(handHandle, 'Vertices', updatedVertices);
    
    % Check if the hand has breached the light curtain
    lightCurtain.detectObject(updatedVertices);
    
    % Pause for visualization purposes
    pause(0.1);
end
