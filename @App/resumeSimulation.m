function resumeSimulation()
    global robot;
    disp('Resuming the robot simulation...');
    
    % Resume the timer or loop for robot movement
    % Reinitialize the timer or loop for resuming the robot's movement along the trajectory
    
    % For example, if you stopped at a certain joint angle or position,
    % resume the motion from that position.
    if ~isempty(timerfindall)
        start(timerfindall);  % Restart the timer
    else
        % If no timer exists, restart the simulation logic
        startSimulation();  % You can call start if a restart is desired
    end
end
