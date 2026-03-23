% HW03_Martin_plotting.m

clear; clc; close all;
addpath("ME-0460_Kinematics\Functions")

%% Numeric dimensions (inches)
d0 = 3.1;
d1 = 1.5;
d2 = 2.4;
d3 = 3.3;

rb = [d3; 0; 0; 1];

%% Sine-wave joint trajectory
f   = 0.2;                                      
t   = linspace(0, 10, 1000);                    

th  = (3*pi/4) * (1 - cos(2*pi*f*t));           
thd = (3*pi/4) * (2*pi*f) * sin(2*pi*f*t);      

%% Compute position and velocity of point r at each timestep
n     = length(t);
y_pos = zeros(1, n);
z_pos = zeros(1, n);
y_vel = zeros(1, n);
z_vel = zeros(1, n);

for i = 1:n
    % Forward kinematics: position of end-effector in frame A
    G  = gabth_PCM(th(i), d0, d1, d2);
    ra = G * rb;

    % Spatial velocity of frame B (6x1): Vsab = [v^s; omega^s]
    Vs = Vsab_PCM(th(i), thd(i), d0, d1, d2);
    vs = Vs(1:3);
    ws = Vs(4:6);

    % Velocity of point r on the body (spatial frame):
    vel_r = vs + cross(ws, ra(1:3));

    y_pos(i) = ra(2);
    z_pos(i) = ra(3);
    y_vel(i) = vel_r(2);
    z_vel(i) = vel_r(3);
end

%% 2 Plots (R_a vs Time and v_a vs Time)
figure;
set(gcf, "units", "Inches", "Position", [0.5 0.5 8.5 11])
subplot(2,1,1);
hold on;
grid on;

plot(t, y_pos, 'b', 'LineWidth', 2);
plot(t, z_pos, 'r', 'LineWidth', 2);

xlabel('Time (s)');
ylabel('Inches');
title('R_a vs Time');
legend('r — Position vs. Time');
legend('y_{pos} (in)', 'z_{pos} (in)', ...
       'Location', 'best')
subplot(2,1,2);
hold on;
grid on;

plot(t, y_vel, 'c', 'LineWidth', 2);
plot(t, z_vel, 'm', 'LineWidth', 2);

xlabel('Time (s)');
ylabel('inches  or  inches/sec');
title('Velocity vs. Time');
legend('y_{vel} (in/s)', 'z_{vel} (in/s)', ...
       'Location', 'best');

hold off;
