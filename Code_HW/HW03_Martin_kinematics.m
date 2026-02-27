% HW03_Martin_kinematics.m
% Generates gabth_PCM.m and Vsab_PCM.m via symbolic derivation
% using matlabFunction for the 1-DOF manipulator from Figure 1.
%
% Dimensions:
%   d0 = 3.1 in   (base x-offset to joint axis)
%   d1 = 1.5 in   (height of joint axis)
%   d2 = 2.4 in   (arm length, joint to frame B)
%   d3 = 3.3 in   (frame B to end-effector, along B's x-axis)

clear; clc;
addpath('../Functions')

%% Symbolic variables
syms th thd d0 d1 d2 real

%% Robot geometry (matches Written HW 3 setup)
w     = [1; 0; 0];           % rotation axis (x-axis of frame A)
q     = [d0; d1; 0];         % point on the joint axis

xi    = twistr_PCM(w, q);    % unit revolute twist xi = [v; w], v = -w x q

Rab_0 = [0 1 0;              % home orientation of frame B in frame A
          0 0 1;              %   B_x = A_z, B_y = A_x, B_z = A_y
          1 0 0];
Pab_0 = [d0; d1+d2; 0];     % home position of frame B origin in frame A

gab_0 = hgt_PCM(Rab_0, Pab_0);   % 4x4 home HGT

%% 1) Forward kinematics: gab(th)
%   gab(th) = e^(xi_hat * th) * gab(0)   [spatial frame formulation]
gab_th = simplify(expt_PCM(xi, th) * gab_0);

matlabFunction(gab_th, 'File', 'gabth_PCM', 'Vars', {th, d0, d1, d2});
fprintf('Generated: gabth_PCM.m\n')

%% 2) Spatial velocity of frame B: Vsab(th, thd)
%   V_hat^s = g_dot * g^{-1},   g_dot = d(gab)/dth * thd
%   Vsab    = veeTwist(V_hat^s)   [6x1: first 3 = v^s, last 3 = omega^s]
gab_th_d = diff(gab_th, th) * thd;          % time derivative of gab
Vsab_hat = simplify(gab_th_d * invHGT_PCM(gab_th));  % 4x4 twist matrix
Vsab     = simplify(veeTwist_PCM(Vsab_hat));          % 6x1 spatial velocity

matlabFunction(Vsab, 'File', 'Vsab_PCM', 'Vars', {th, thd, d0, d1, d2});
fprintf('Generated: Vsab_PCM.m\n')
