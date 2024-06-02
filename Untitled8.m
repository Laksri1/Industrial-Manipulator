% Define the DH parameters for each link
Link1 = Link('d', 0.24765, 'a', 0, 'alpha', pi/2);
Link2 = Link('d', 0, 'a', 0.2286, 'alpha', 0);
Link3 = Link('d', 0, 'a', 0.2286, 'alpha', 0);
Link4 = Link('d', 0, 'a', 0, 'alpha', -pi/2);
Link5 = Link('d', 0.073025, 'a', 0, 'alpha', 0);

% Assemble the robot manipulator
Robot = SerialLink([Link1 Link2 Link3 Link4 Link5], 'name', 'RHINO');

% Set positions and orientations for key points
start_pos = [-0.146, 0, 0.409];
start_orient_rpy = [0, -90, -180];

reg_number = 023; % My reg no 
if mod(reg_number, 2) == 0
    A_y = 0.20;
else
    A_y = 0.30;
end
A_z = 0.010 + reg_number / 420 * 0.40;

A_pos = [-0.17, A_y, A_z];
A_orient_rpy = [-180, 0, 60];
B_pos = [0.181, 0.313, 0.345];
B_orient_rpy = [-125, 26, 106];
C_pos = [0.420, 0.000, 0.540];
C_orient_rpy = [0, 70, 0];
D_pos = [0.237, -0.338, 0.100];
D_orient_rpy = [180, 0, -125];

% Convert RPY angles to radians and construct transformation matrices
start_orient_rpy_rad = deg2rad(start_orient_rpy);
A_orient_rpy_rad = deg2rad(A_orient_rpy);
B_orient_rpy_rad = deg2rad(B_orient_rpy);
C_orient_rpy_rad = deg2rad(C_orient_rpy);
D_orient_rpy_rad = deg2rad(D_orient_rpy);

Home_T = transl(start_pos) * rpy2tr(start_orient_rpy_rad);
A_T = transl(A_pos) * rpy2tr(A_orient_rpy_rad);
B_T = transl(B_pos) * rpy2tr(B_orient_rpy_rad);
C_T = transl(C_pos) * rpy2tr(C_orient_rpy_rad);
D_T = transl(D_pos) * rpy2tr(D_orient_rpy_rad);

% Compute joint angles via inverse kinematics with constraints (ikcon)
q_home = Robot.ikcon(Home_T);
q_A = Robot.ikcon(A_T);
q_B = Robot.ikcon(B_T);
q_C = Robot.ikcon(C_T);
q_D = Robot.ikcon(D_T);


% Create joint space trajectories
time = 0:0.04:2;
traj_home_A = jtraj(q_home, q_A, time);
traj_A_B = jtraj(q_A, q_B, time);
traj_B_C = jtraj(q_B, q_C, time);
traj_C_D = jtraj(q_C, q_D, time);
traj_D_home = jtraj(q_D, q_home, time);
% Visualize all trajectory segments
segments = {traj_home_A, traj_A_B, traj_B_C, traj_C_D, traj_D_home};
segment_names = {'Home_to_A', 'A_to_B', 'B_to_C', 'C_to_D', 'D_to_Home'};
save_directory = './frameDIR/';

figure;
hold on;

for idx = 1:length(segments)
    plot_trajectory(Robot, segments{idx}, segment_names{idx}, save_directory);
end

hold off;

% Function to plot and save end-effector trajectory
function plot_trajectory(Robot, traj, segment_name, save_dir)
    EE_positions = zeros(size(traj, 1), 3); % End-effector positions array

    for i = 1:size(traj, 1)
        EET = double(Robot.fkine(traj(i, :)));
        EE_positions(i, :) = EET(1:3, 4)';
        plot3(EE_positions(1:i, 1), EE_positions(1:i, 2), EE_positions(1:i, 3), 'b');
        plot2(EE_positions(i, :), 'r.');
        Robot.plot(traj(i, :));
        frame_filename = fullfile(save_dir, [segment_name, '_frame_', num2str(i), '.png']);
        saveas(gcf, frame_filename);
        grid on;
        xlim([-0.5, 0.5]);
        ylim([-0.5, 0.5]);
        zlim([-0.5, 0.6]);
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title(['End-Effector Trajectory: ', segment_name]);
        pause(0.05);
    end
end

