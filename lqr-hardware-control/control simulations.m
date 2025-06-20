function V = acceleration(t, y)

    %X0 = [113.9146  -10.0000   57.5062  -14.5685];
    X1 = [ 261.5626  -31.6228  131.0769  -40.2996];
    V = -X1 * y; % Linear feedback controller
end

function dydt = odefun(t, y, mp, d, Ip, Bp, l, g)
    A = Ip + mp * d^2;
    dydt = zeros(4,1);
    theta = y(1);
    %phi = y(2);
    theta_dot = y(3);
    phi_dot = y(4);
    dydt(1) = theta_dot;
    dydt(2) = phi_dot;
    dydt(3) = (mp * l * d * cos(theta)*acceleration(t, y) + mp * d^2 * sin(theta) * cos(theta) * phi_dot^2 + mp * d * g * sin(theta) - Bp * theta_dot) / A;
    dydt(4) = acceleration(t, y);
end

% Main script
clear; clc; close all;

% Parameters
mp = 1;    % Mass of pendulum
d = 0.5;   % Distance to COM
Ip = 1;    % Pendulum MOI
Bp = 0;    % Damping
l = 1;     % Arm length
g = 9.81;  % Gravity

% Initial conditions and time span
y0 = [pi/8, 0, 0, 0];
t1 = [0 20];

%% Solve the system
[t, y] = ode45(@(t, y) odefun(t, y, mp, d, Ip, Bp, l, g), t1, y0);

%% Plot angular positions
figure;
subplot(2,1,1);
plot(t, y(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t, y(:,2), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angles (rad)');
legend('\theta (pendulum)', '\phi (arm)');
title('Angular Positions');
grid on;

%% Plot control input
u = zeros(length(t), 1);
for i = 1:length(t)
    u(i) = acceleration(t(i), y(i,:)');
end
subplot(2,1,2);
plot(t, u, 'k--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Input (acceleration)');
title('Control Input Over Time');
grid on;


%plot% 

L1 = 1;  % Length of arm
L2 = 0.5; % Length of pendulum

figure;
hold on;
grid on;
axis equal;
xlim([-L1 - L2, L1 + L2]);
ylim([-L1 - L2, L1 + L2]);
zlim([-L2, L1 + L2]);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Motion of the System');
view(3); % Ensure 3D perspective

% Initialize plot elements
line1 = plot3([0 0], [0 0], [0 0], 'bo-', 'LineWidth', 2); % Origin to A
line2 = plot3([0 0], [0 0], [0 0], 'ro-', 'LineWidth', 2); % A to B

for k = 1:length(t)
    % Compute point A
    Ax = L1 * cos(y(k, 2));
    Ay = L1 * sin(y(k, 2));
    Az = 0;

    % Compute point B
    Bx = Ax - L2 * sin(y(k, 1)) * sin(y(k, 2));
    By = Ay + L2 * sin(y(k, 1)) * cos(y(k, 2));
    Bz = (-1)*(Az - L2 * cos(y(k, 1)));

    % Update the lines dynamically
    set(line1, 'XData', [0 Ax], 'YData', [0 Ay], 'ZData', [0 Az]); % Origin to A
    set(line2, 'XData', [Ax Bx], 'YData', [Ay By], 'ZData', [Az Bz]); % A to B

    drawnow; % Update the figure
    pause(0.05); % Slow down animation
end

hold off;