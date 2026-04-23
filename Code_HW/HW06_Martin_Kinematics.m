% HW06_Martin_Kinematics.m
% ME4640 S26 - Code HW #6
% Paul Martin (PCM)

clear; clc;
addpath('Functions');

d1 = 40; d2 = 50; d3 = 60; d6 = 30;
syms th1 th2 th3 th4 th5 th6 real

% Twist vectors xi = [v; omega]
xi1 = [0;        0;      0;  0; 0; 1];
xi2 = [0;       d1;      0;  1; 0; 0];
xi3 = [0;        0;      1;  0; 0; 0];
xi4 = [d2;       0;      0;  0; 0; 1];
xi5 = [0;    d1+d3;    -d2;  1; 0; 0];
xi6 = [-(d1+d3); 0;      0;  0; 1; 0];

% Home transforms g_si(0)
gs10 = [eye(3), [0;  0;    d1]; zeros(1,3), 1];
gs20 = [eye(3), [0;  d2;   d1]; zeros(1,3), 1];
gs30 = [eye(3), [0;  d2; d1+d3]; zeros(1,3), 1];
gs40 = gs30;
gs50 = gs30;
gst0 = [0, 1, 0,     0;
        0, 0, 1, d2+d6;
        1, 0, 0, d1+d3;
        0, 0, 0,     1];

% Exponential maps
E1 = expt_PCM(xi1, th1);
E2 = expt_PCM(xi2, th2);
E3 = sym(eye(4)); E3(3,4) = th3;  % prismatic -- expt_PCM divides by |omega|=0
E4 = expt_PCM(xi4, th4);
E5 = expt_PCM(xi5, th5);
E6 = expt_PCM(xi6, th6);

% Partial products
G1     = E1;
G12    = G1    * E2;
G123   = G12   * E3;
G1234  = G123  * E4;
G12345 = G1234 * E5;

% Spatial Jacobian: J_i = Ad(G_{i-1}) * xi_i
J1 = sym(xi1);
J2 = simplify(Ad_PCM(G1)     * xi2);
J3 = simplify(Ad_PCM(G12)    * xi3);
J4 = simplify(Ad_PCM(G123)   * xi4);
J5 = simplify(Ad_PCM(G1234)  * xi5);
J6 = simplify(Ad_PCM(G12345) * xi6);

JstthPCM = [J1, J2, J3, J4, J5, J6];
disp(JstthPCM)

matlabFunction(JstthPCM, 'File', 'Functions/JstthPCM', ...
    'Vars', {[th1,th2,th3,th4,th5,th6]}, 'Outputs', {'J'});

% FK for each link
gs1th = G1     * gs10;
gs2th = G12    * gs20;
gs3th = G123   * gs30;
gs4th = G1234  * gs40;
gs5th = G12345 * gs50;
gstth = G1 * E2 * E3 * E4 * E5 * E6 * gst0;

% Home position check
hc = double(subs(gstth, {th1,th2,th3,th4,th5,th6}, {0,0,0,0,0,0}));
fprintf('p_home = [%.2f; %.2f; %.2f]  (expected [0; 80; 100])\n', hc(1,4), hc(2,4), hc(3,4));
if any(isnan(hc(:))) || abs(hc(1,4))>0.01 || abs(hc(2,4)-80)>0.01 || abs(hc(3,4)-100)>0.01
    error('Home position mismatch.');
end

matlabFunction(gs1th, 'File', 'Functions/gs1PCM', ...
    'Vars', {[th1,th2,th3,th4,th5,th6]}, 'Outputs', {'T'});
matlabFunction(gs2th, 'File', 'Functions/gs2PCM', ...
    'Vars', {[th1,th2,th3,th4,th5,th6]}, 'Outputs', {'T'});
matlabFunction(gs3th, 'File', 'Functions/gs3PCM', ...
    'Vars', {[th1,th2,th3,th4,th5,th6]}, 'Outputs', {'T'});
matlabFunction(gs4th, 'File', 'Functions/gs4PCM', ...
    'Vars', {[th1,th2,th3,th4,th5,th6]}, 'Outputs', {'T'});
matlabFunction(gs5th, 'File', 'Functions/gs5PCM', ...
    'Vars', {[th1,th2,th3,th4,th5,th6]}, 'Outputs', {'T'});
matlabFunction(gstth, 'File', 'Functions/gstthPCM', ...
    'Vars', {[th1,th2,th3,th4,th5,th6]}, 'Outputs', {'T'});

function A = Ad_PCM(g)
    R  = g(1:3,1:3);
    p  = g(1:3,4);
    ph = [0,-p(3),p(2); p(3),0,-p(1); -p(2),p(1),0];
    A  = [R, ph*R; zeros(3,3), R];
end