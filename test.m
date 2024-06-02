% Define DH parameters for each link (example parameters, replace with actual values)
L1 = Link('d', 0.146, 'a', 0, 'alpha', pi/2, 'offset', 0);
L2 = Link('d', 0, 'a', 0.2, 'alpha', 0, 'offset', pi/2);
L3 = Link('d', 0, 'a', 0.3, 'alpha', 0, 'offset', 0);
L4 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0);
L5 = Link('d', 0, 'a', 0, 'alpha', 0, 'offset', 0);

% Create the robot model
RHINO = SerialLink([L1 L2 L3 L4 L5], 'name', 'RHINO');

% Verify the home transformation matrix
home_xyz = [-0.146, 0, 0.409];
home_rpy = [0, -90, -180];
home_tr = transl(home_xyz) * rpy2tr(home_rpy, 'deg');
disp('Home transformation matrix:');
disp(home_tr);

% Set the robot to the home position using inverse kinematics
try
    home_q = RHINO.ikcon(home_tr);
    disp('Inverse kinematics result for home_tr:');
    disp(home_q);
    RHINO.plot(home_q);
catch ME
    disp('Error computing inverse kinematics:');
    disp(ME.message);
end

% Test with a simple transformation matrix
simple_tr = transl(0.2, 0.1, 0.3) * rpy2tr(0, -90, -180, 'deg');
try
    simple_q = RHINO.ikcon(simple_tr);
    disp('Inverse kinematics result for simple_tr:');
    disp(simple_q);
    RHINO.plot(simple_q);
catch ME
    disp('Error computing inverse kinematics:');
    disp(ME.message);
end
