function resetSimulation()
    global robot;
    disp('Resetting the robot simulation...');
    
    % Stop any running timers or loops
    if isvalid(timerfindall)
        stop(timerfindall);
        delete(timerfindall);
    end
    
    % Reset the robot to the initial joint configuration
    initialJointPosition = [0, -pi/2, pi/2, 0, pi/2, 0];  % Modify based on your robot
    show(robot, initialJointPosition);  % Visualize the reset state
    
    % Reset any other simulation parameters
end
