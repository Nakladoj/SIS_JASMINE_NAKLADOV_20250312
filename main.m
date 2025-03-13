%% Jasmine Nakladov | AE 544 Analytical Dynamics | Programming Project 01
% Instructions: Write numerical programs to integrate an attitude kinematics or dynamics equation where 
% the simulation hits the singularity or ambiguity of each scenario. Do this by manipulating the angualr 
% velocity vector. Use one angular motion trace to compare a pair of representation methods and then 
% analyze your results by converting one representation to another.
% Explore whether different numerical integrators will have an impact.
% Make a 3D animation to help analyze the situation.

clear; clc; close all

%% Singularity for Asymmetric Euler Angle Sets

% The 3-2-1 Asymmetric Euler Angle Rotation will be used for this example. For this rotation, geometric
% singularity occurrs at a pitch (theta) angle of +/- 90 degrees. The two remaining angles are not uniquely defined. 

omega_body = [0.1 0.2 0.1]'; % Angular velocity, arbitrary conditions, rad/s

theta0 = [0 0 0]'; % Initial condition
tspan = [0 100]; % Timespan

% ode45 is used first
[t, theta] = ode45(@(t, theta) rates(theta, omega_body), tspan, theta0); % ode45 integrator

theta = rad2deg(theta); % Converting from rad to deg

% Plotting Euler angles over time
figure
subplot(3,1,1)
plot(t, theta(:,1), 'LineWidth', 1.3)
xlabel('Time (s)')
ylabel('Yaw (deg)')
grid on

subplot(3,1,2)
plot(t, theta(:,2), 'LineWidth', 1.3)
xlabel('Time (s)')
ylabel('Pitch (deg)')
grid on

subplot(3,1,3)
plot(t, theta(:,3), 'LineWidth', 1.3)
xlabel('Time (s)')
ylabel('Roll (deg)')
grid on

sgtitle('3-2-1 Euler Angles Over Time Using ode45')

%% 3D Animation for Euler Parameters
% Creating a cube to act as the investigated 3D object
cube_vertices = [-1 -1 -1; 1 -1 -1; 1  1 -1; -1  1 -1; -1 -1  1; 1 -1  1; 1  1  1; -1  1  1];
cube_faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

% Setting up animation figure and axes
figure
axis equal
axis([-3 3 -3 3 -3 3])
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
hold on
view(3)

% Create hgtransform object to rotate the cube
h = hgtransform;

% Plotting cube by connecting vertices, assigning hgtransform
hCube = patch('Faces', cube_faces, 'Vertices', cube_vertices,'FaceColor', 'c', 'FaceAlpha', 0.3, 'Parent', h); 

% Plotting body axis and its labels
hX = quiver3(0, 0, 0, 2, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
hY = quiver3(0, 0, 0, 0, 2, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
hZ = quiver3(0, 0, 0, 0, 0, 2, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);

labelX = text(2, 0, 0, 'X', 'Color', 'r');
labelY = text(0, 2, 0, 'Y', 'Color', 'g');
labelZ = text(0, 0, 2, 'Z', 'Color', 'b');

% GIF setup
gif_filename = 'EulerAnglesAnimation.gif';

% Animation loop
for i = 1:1:length(t)
    theta_3 = deg2rad(theta(i,1));
    theta_2 = deg2rad(theta(i,2));
    theta_1 = deg2rad(theta(i,3));
    
    % Rotation matrices
    Mz = [cos(theta_3) -sin(theta_3) 0 0; sin(theta_3)  cos(theta_3) 0 0; 0 0 1 0; 0 0 0 1];
    My = [ cos(theta_2) 0 sin(theta_2) 0; 0 1 0 0; -sin(theta_2) 0 cos(theta_2) 0; 0 0 0 1];
    Mx = [1 0 0 0; 0 cos(theta_1) -sin(theta_1) 0; 0 sin(theta_1) cos(theta_1) 0; 0 0 0 1];
    
    R = Mz * My * Mx;
    
    % Applying transform to cube
    set(h, 'Matrix', R);
    
    % Updating body-fixed axis and labels
    newX = (R(1:3,1)) * 2;
    newY = (R(1:3,2)) * 2;
    newZ = (R(1:3,3)) * 2;
    
    set(hX, 'UData', newX(1), 'VData', newX(2), 'WData', newX(3));
    set(hY, 'UData', newY(1), 'VData', newY(2), 'WData', newY(3));
    set(hZ, 'UData', newZ(1), 'VData', newZ(2), 'WData', newZ(3));
    
    set(labelX, 'Position', newX', 'String', 'X');
    set(labelY, 'Position', newY', 'String', 'Y');
    set(labelZ, 'Position', newZ', 'String', 'Z');
    
    title(sprintf('3D Animation for Euler Angles at t = %.2f s', t(i)));

    % Creating GIF
    drawnow
    
    if i == 1
        gif(gif_filename, 'DelayTime', 0.01, 'LoopCount', Inf) % First frame
    else
        gif
    end

    pause(0.01) % Animation speed
end

%% Repeating the process with ode15s
[t, theta] = ode15s(@(t, theta) rates(theta, omega_body), tspan, theta0); % ode15s integrator

theta_15s = rad2deg(theta); % Converting from rad to deg

% Plotting Euler angles over time
figure
subplot(3,1,1)
plot(t, theta_15s(:,1), 'LineWidth', 1.3)
xlabel('Time (s)')
ylabel('Yaw (deg)')
grid on

subplot(3,1,2)
plot(t, theta_15s(:,2), 'LineWidth', 1.3)
xlabel('Time (s)')
ylabel('Pitch (deg)')
grid on

subplot(3,1,3)
plot(t, theta_15s(:,3), 'LineWidth', 1.3)
xlabel('Time (s)')
ylabel('Roll (deg)')
grid on

sgtitle('3-2-1 Euler Angles Over Time Using ode15s')

%% Ambiguity of Euler Parameters

% Quaternions take care of the singularities present in the Euler angles. In other words, there are no 
% specific singularities for Euler parameters. However, there is still an ambiguity present in the fact 
% that quaternions can be positive or negative and yet still represent the same rotaion.

omega_body = [0.1 0.2 0.1]'; % Angular velocity, arbitrary conditions, rad/s. Same as previous angular velocity.

b_initial = [0 0 0 1]'; % Initial condition quaternions (norm must = 1)
tspan = [0 100]; % Timespan

% ode45 is used first
[t, b] = ode45(@(t, b) b_mat(b, omega_body), tspan, b_initial); % ode45 integrator
b_norm = sqrt(sum(b.^2, 2)); 
b = b ./ b_norm; % Normalizing

% Plotting
figure
subplot(4,1,1)
plot(t, b(:,4), 'LineWidth', 1.3)
hold on
plot(t, -b(:,4), 'LineWidth', 1.3)
hold off
ylabel('b_0')
grid on

subplot(4,1,2)
plot(t, b(:,1), 'LineWidth', 1.3)
hold on
plot(t, -b(:,1), 'LineWidth', 1.3)
hold off
ylabel('b_1')
grid on

subplot(4,1,3)
plot(t, b(:,2), 'LineWidth', 1.3)
hold on
plot(t, -b(:,2), 'LineWidth', 1.3)
hold off
ylabel('b_2')
grid on

subplot(4,1,4)
plot(t, b(:,3), 'LineWidth', 1.3)
hold on
plot(t, -b(:,3), 'LineWidth', 1.3)
hold off
ylabel('b_3')
xlabel('Time (s)')
legend('(+) Quaternions', '(-) Quaternions')
grid on

sgtitle('Euler Parameters (Quaternions) Over Time Using ode45')

%% Animation
% Setting up figire and axes
figure
axis equal
axis([-3 3 -3 3 -3 3])
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
hold on
view(3)

% Create hgtransform object to rotate the cube
h = hgtransform;

% Creating cube
hCube = patch('Faces', cube_faces, 'Vertices', cube_vertices, 'FaceColor', 'cyan', 'FaceAlpha', 0.3, 'Parent', h);

% Body-fixed axis and labels
hX = quiver3(0, 0, 0, 2, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
hY = quiver3(0, 0, 0, 0, 2, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
hZ = quiver3(0, 0, 0, 0, 0, 2, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);

labelX = text(2, 0, 0, 'X', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
labelY = text(0, 2, 0, 'Y', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
labelZ = text(0, 0, 2, 'Z', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');

% GIF setup
gif_filename = 'EulerParametersAnimation.gif';

% Animation loop
for i = 1:1:length(t)  % Adjust step size for smoothness
    
    % Extracting quaternion
    b1 = b(i,1);
    b2 = b(i,2);
    b3 = b(i,3);
    b0 = b(i,4);
    
    % Rotation matrix from quaternion, equations 3.93 a, b, c
    R = [ b0^2 + b1^2 - b2^2 - b3^2 2*(b1*b2 + b0*b3) 2*(b1*b3 - b0*b2); 2*(b1*b2 - b0*b3) b0^2 - b1^2 + b2^2 - b3^2,  2*(b2*b3 + b0*b1); 2*(b1*b3 + b0*b2) 2*(b2*b3 - b0*b1) b0^2 - b1^2 - b2^2 + b3^2 ];

    % Converting R to homogeneous transform matrix due to nature of hgtransform requiring a 3 x 3 matrix
    R_hom = eye(4);
    R_hom(1:3,1:3) = R;
    
    % Apply rotation to the hgtransform
    set(h, 'Matrix', R_hom);

    % Update body axis and its labels
    newX = R(:,1) * 2;
    newY = R(:,2) * 2;
    newZ = R(:,3) * 2;
    
    set(hX, 'UData', newX(1), 'VData', newX(2), 'WData', newX(3));
    set(hY, 'UData', newY(1), 'VData', newY(2), 'WData', newY(3));
    set(hZ, 'UData', newZ(1), 'VData', newZ(2), 'WData', newZ(3));

    set(labelX, 'Position', newX', 'String', 'X');
    set(labelY, 'Position', newY', 'String', 'Y');
    set(labelZ, 'Position', newZ', 'String', 'Z');
    
    title(sprintf('Quaternion Animation at t = %.2f s', t(i)));
    
    pause(0.005) % Animation speed

    % Creating GIF
    drawnow
   
    if i == 1
        gif(gif_filename, 'DelayTime', 0.01, 'LoopCount', Inf) % First frame
    else
        gif
    end
    
end

%%
% ode15s is used next
[t, b] = ode15s(@(t, b) b_mat(b, omega_body), tspan, b_initial); % ode15s integrator
b_norm = sqrt(sum(b.^2, 2)); % Ensuring quaternions are normalized
b = b ./ b_norm;

% Plotting
figure
subplot(4,1,1)
plot(t, b(:,4), 'LineWidth', 1.3)
hold on
plot(t, -b(:,4), 'LineWidth', 1.3)
hold off
ylabel('b_0')
grid on

subplot(4,1,2)
plot(t, b(:,1), 'LineWidth', 1.3)
hold on
plot(t, -b(:,1), 'LineWidth', 1.3)
hold off
ylabel('b_1')
grid on

subplot(4,1,3)
plot(t, b(:,2), 'LineWidth', 1.3)
hold on
plot(t, -b(:,2), 'LineWidth', 1.3)
hold off
ylabel('b_2')
grid on

subplot(4,1,4)
plot(t, b(:,3), 'LineWidth', 1.3)
hold on
plot(t, -b(:,3), 'LineWidth', 1.3)
hold off
ylabel('b_3')
xlabel('Time (s)')
legend('(+) Quaternions', '(-) Quaternions')
grid on

sgtitle('Euler Parameters (Quaternions) Over Time Using ode15s')

%%
% Converting Quaternions to Euler Angles
eul_pos = rad2deg(quat2eul(b));
eul_neg = rad2deg(quat2eul(-b));

figure
subplot(3,1,1)
plot(t, eul_pos(:,1), 'LineWidth', 1.3)
hold on
plot(t, eul_neg(:,1), 'r--', 'LineWidth', 1.3)
ylabel('Yaw (deg)')
legend('(+) Quaternions', '(-) Quaternions')
grid on

subplot(3,1,2)
plot(t, eul_pos(:,2), 'LineWidth', 1.3)
hold on
plot(t, eul_neg(:,2), 'r--', 'LineWidth', 1.3)
ylabel('Pitch (deg)')
legend('(+) Quaternions', '(-) Quaternions')
grid on

subplot(3,1,3)
plot(t, eul_pos(:,3), 'LineWidth', 1.3)
hold on
plot(t, eul_neg(:,3), 'r--', 'LineWidth', 1.3)
ylabel('Roll (deg)')
xlabel('Time (s)')
legend('(+) Quaternions', '(-) Quaternions')
grid on

sgtitle('Euler Angles from Positive and Negative Quaternions')

%% Classical Rodrigues Parameters
omega_body = [0.1 0.2 0.1]'; % Angular velocity, arbitrary conditions, rad/s. Same as previous angular velocity.

q_initial = [0 0 0]'; 
tspan = [0 100]; % Timespan

% ode45 is used first
[t, q] = ode15s(@(t, q) q_mat(q, omega_body), tspan, q_initial); % ode15s integrator

% Plotting
figure
subplot(3,1,1)
plot(t, q(:,1), 'LineWidth', 1.3)
ylabel('q_1')
grid on

subplot(3,1,2)
plot(t, q(:,2), 'LineWidth', 1.3)
ylabel('q_2')
grid on

subplot(3,1,3)
plot(t, q(:,3), 'LineWidth', 1.3)
ylabel('q_3')
grid on
sgtitle('Classical Rodriques Parameters Over Time Using ode45')

% ode45 is used first
[t, q] = ode45(@(t, q) q_mat(q, omega_body), tspan, q_initial); % ode45 integrator

% Plotting
figure
subplot(3,1,1)
plot(t, q(:,1), 'LineWidth', 1.3)
ylabel('q_1')
grid on

subplot(3,1,2)
plot(t, q(:,2), 'LineWidth', 1.3)
ylabel('q_2')
grid on

subplot(3,1,3)
plot(t, q(:,3), 'LineWidth', 1.3)
ylabel('q_3')
grid on
sgtitle('Classical Rodriques Parameters Over Time Using ode15s')

% Calculating Principal Angle to see when 180 degrees is hit 
q_norm = vecnorm(q, 2, 2); 
p_angle = rad2deg(2 * atan(q_norm));   

% Plotting the Principal Angle
figure
plot(t, p_angle, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('\phi (deg)')
title('Principal Rotation Angle Over Time')
grid on

%% Functions
% Creating function to calculate angular rates = [B(theta)]*omega. From the textbook, Appendix B.2.
function angular_rates = rates(theta, omega_body)
    theta1 = theta(1); theta2 = theta(2); theta3 = theta(3); % Extracting thetas from ode45
    
    % Defining B(theta) matrix
    B_theta = 1/cos(theta2) * [0 sin(theta3) cos(theta2); 0 cos(theta2) * cos(theta3) -cos(theta2) * sin(theta3); cos(theta2) sin(theta2) * sin(theta3) sin(theta2) * cos(theta3)];
    
    % Angular rates equation
    angular_rates = B_theta * omega_body;
end

% Creating function to calculate Euler parameter rates = 1/2*[B(Beta)]*omega. From the textbook, page 10, equation 3.105 and 3.106.
function b_rates = b_mat(b, omega_body)
    b = b / norm(b); % Ensuring quaternions are normlized
    
    B0 = b(4); B1 = b(1); B2 = b(2); B3 = b(3); % Extracting values

    % Defining B(Beta) matrix
    B_dot = 1/2 * [B0 -B1 -B2 -B3; B1 B0 -B3 B2; B2 B3 B0 -B1; B3 -B2 B1 B0];
    omega_b = [0 omega_body(1) omega_body(2) omega_body(3)]';

    % Quaternion rates equation
    b_rates = B_dot * omega_b;
end

% Creating functino to calculate Classical Rodriques Parameters
function q_rates = q_mat(q, omega_body)
    
    q1 = q(1); q2 = q(2); q3 = q(3); % Extracting values

    % Defining q matrix
    q_dot = 1/2 * [(1+q1^2) (q1*q2 - q3) (q1*q3 + q2); (q2*q1 + q3) (1 + q2^2) (q2*q3 - q1); (q3*q1 - q2) (q3*q2 + q1) (1 + q3^2)];
    omega_q = [omega_body(1) omega_body(2) omega_body(3)]';

    % Quaternion rates equation
    q_rates = q_dot * omega_q;
end

