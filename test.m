% Create an instance of the XArm6DOF class
xarm = XArm6DOF();

% Plot the robot in the default zero configuration
xarm.plotRobot();

% Enter teach mode where you can interact with the robot using sliders
xarm.teachRobot();
