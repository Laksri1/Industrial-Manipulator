% Define DH parameters for each link (example parameters, replace with actual values)
L1 = Link('d', 0.248, 'a', 0, 'alpha', -pi/2, 'offset', 0);
L2 = Link('d', 0, 'a', 0.229, 'alpha', 0, 'offset', 0);
L3 = Link('d', 0, 'a', 0.229, 'alpha', 0, 'offset', 0);
L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0);
L5 = Link('d', 0.073, 'a', 0, 'alpha', 0, 'offset', 0);

% Create the robot model
RHINO = SerialLink([L1 L2 L3 L4 L5], 'name', 'RHINO');

% Verify the home transformation matrix
home_xyz = [-0.146, 0, 0.409];
home_rpy = [0, -90, -180];
home_tr = transl(home_xyz) * rpy2tr(home_rpy, 'deg');
disp('Home transformation matrix:');
disp(home_tr);

% Set the robot to the home position using inverse kinematics
home_q = RHINO.ikcon(home_tr);
RHINO.plot(home_q);

% Registration number example
EN = 23; % Replace with your actual registration number

% Calculate Ay and Az
Ay = 0.30;
Az = 0.010 + EN/420 * 0.40;

% Define positions and RPY for A, B, C, D
A_xyz = [-0.17, Ay, Az];
A_rpy = [-180, 0, 60];
B_xyz = [0.181, 0.313, 0.345];
B_rpy = [-125, 26, 106];
C_xyz = [0.420, 0, 0.540];
C_rpy = [0, 70, 0];
D_xyz = [0.237, -0.338, 0.100];
D_rpy = [180, 0, -125];

% Convert to transformation matrices
A_tr = transl(A_xyz) * rpy2tr(A_rpy, 'deg');
B_tr = transl(B_xyz) * rpy2tr(B_rpy, 'deg');
C_tr = transl(C_xyz) * rpy2tr(C_rpy, 'deg');
D_tr = transl(D_xyz) * rpy2tr(D_rpy, 'deg');

% Compute joint angles for each position using inverse kinematics
A_q = RHINO.ikcon(A_tr);
B_q = RHINO.ikcon(B_tr);
C_q = RHINO.ikcon(C_tr);
D_q = RHINO.ikcon(D_tr);

% Generate trajectories
t = linspace(0, 0.04, 2); % Adjust time vector as needed
traj_H_A = jtraj(home_q, A_q, t);
traj_A_B = jtraj(A_q, B_q, t);
traj_B_C = jtraj(B_q, C_q, t);
traj_C_D = jtraj(C_q, D_q, t);
traj_D_H = jtraj(D_q, home_q, t);

% Create the directory if it doesn't exist
frameDIR = './frameDIR/';
if ~exist(frameDIR, 'dir')
   mkdir(frameDIR)
end

% Plot and save frames for each trajectory
plot_and_save(RHINO, traj_H_A, 'H_A_', t);
plot_and_save(RHINO, traj_A_B, 'A_B_', t);
plot_and_save(RHINO, traj_B_C, 'B_C_', t);
plot_and_save(RHINO, traj_C_D, 'C_D_', t);
plot_and_save(RHINO, traj_D_H, 'D_H_', t);

% Debugging plot_and_save function
function plot_and_save(robot, trajectory, traj_name, t)
    frameDIR = './frameDIR/';
    num_points = size(trajectory, 1);
    EEp = zeros(num_points, 3);
    tolerance = 1e-10; % Tolerance for numerical precision issues

    for i = 1:num_points
        % Compute the forward kinematics
        EET = robot.fkine(trajectory(i,:));

        % Print diagnostic information
        fprintf('Point %d:\n', i);
        disp('Trajectory point:');
        disp(trajectory(i,:));
        disp('Transformation matrix EET:');
        disp(EET);

        % Check if the transformation matrix is approximately 4x4
        if size(EET, 1) ~= 4 || size(EET, 2) ~= 4 || ...
                any(abs(EET(4,:) - [0 0 0 1]) > tolerance)
            warning('Transformation matrix EET is not exactly 4x4. Check the matrix elements for errors.');
        end

        % Extract the end-effector position if the matrix is valid
        if size(EET, 1) == 4 && size(EET, 2) == 4 && ...
                all(abs(EET(4,:) - [0 0 0 1]) <= tolerance)
            EEp(i,:) = EET(1:3, 4)';
        else
            error('Transformation matrix EET does not have valid dimensions.');
        end

        % Plot end-effector position
        plot3(EEp(:,1), EEp(:,2), EEp(:,3), 'b');
        hold on;
        plot3(EEp(i,1), EEp(i,2), EEp(i,3), 'r.');

        % Plot the robot
        robot.plot(trajectory(i,:));

        % Save the frame
        saveas(gcf, [frameDIR traj_name num2str(i) '.png']);
        hold off;
    end
end

