function stopSimulation()
    global robot;  % Access the robot object from global scope or pass it explicitly
    disp('Stopping the robot simulation...');
    
    % If using a timer or loop, stop or pause it here
    % For example, if a timer is running the simulation, stop it
    if isvalid(timerfindall)
        stop(timerfindall);
        delete(timerfindall);  % Clean up the timer
    end
    
    % If the robot is moving along a trajectory, stop or freeze its current position
    % Add your logic to freeze or hold the robot in its current state.
end
