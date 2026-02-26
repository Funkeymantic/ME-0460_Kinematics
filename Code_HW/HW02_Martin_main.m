% HW02 Main

clear; clc; close all;

% Problem 4: Triangle A rotating about axis through point q
pa1 = [1; 0; 1];
pa2 = [1; 0; 2];
pa3 = [2; 0; 1];

w_a = [1; 0; 1];
w_a = w_a / norm(w_a);
q_a = [0; 1; 2];

xi_a = twistr_PCM(w_a, q_a);

theta_vals = 0:pi/20:3*pi/2;

figure;
hold on;

for i = 1:length(theta_vals)
    theta = theta_vals(i);
    T_a = expt_PCM(xi_a, theta);  % FIXED: Changed from expr_PCM to expt_PCM
    
    pa1_rot = T_a * [pa1; 1];
    pa2_rot = T_a * [pa2; 1];
    pa3_rot = T_a * [pa3; 1];
    
    plot3([pa1_rot(1), pa2_rot(1)], [pa1_rot(2), pa2_rot(2)], [pa1_rot(3), pa2_rot(3)], 'b-');
    plot3([pa2_rot(1), pa3_rot(1)], [pa2_rot(2), pa3_rot(2)], [pa2_rot(3), pa3_rot(3)], 'b-');
    plot3([pa3_rot(1), pa1_rot(1)], [pa3_rot(2), pa1_rot(2)], [pa3_rot(3), pa1_rot(3)], 'b-');
end

% Problem 5: Triangle B in body frame rotating about axis
pb1 = [0; 1; 1];
pb2 = [0; 1; 0];
pb3 = [0; 0; 1];

w_b = [1; 0; 0];
q_b = [1; 1; 1];

xi_b = twistr_PCM(w_b, q_b);

p_ab = [2; 0; 2];
R_ab_init = [0 -1 0;
             1 0 0;
             0 0 1];

g_ab_0 = [R_ab_init, p_ab;
          0 0 0 1];

for i = 1:length(theta_vals)
    theta = theta_vals(i);
    T_b = expt_PCM(xi_b, theta);  % FIXED: Changed from expt_PGK to expt_PCM
    
    g_ab_theta = T_b * g_ab_0;
    
    pb1_world = g_ab_theta * [pb1; 1];
    pb2_world = g_ab_theta * [pb2; 1];
    pb3_world = g_ab_theta * [pb3; 1];
    
    plot3([pb1_world(1), pb2_world(1)], [pb1_world(2), pb2_world(2)], [pb1_world(3), pb2_world(3)], 'r-');
    plot3([pb2_world(1), pb3_world(1)], [pb2_world(2), pb3_world(2)], [pb2_world(3), pb3_world(3)], 'r-');
    plot3([pb3_world(1), pb1_world(1)], [pb3_world(2), pb1_world(2)], [pb3_world(3), pb1_world(3)], 'r-');
end

xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;
view(3);
hold off;

g_ab_0