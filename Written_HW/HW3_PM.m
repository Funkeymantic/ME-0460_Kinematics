% HW 3 Demo
clear all; close all; clc;

addpath("ME-0460_Kinematics\Functions")

syms d0 d1 d2 d3 real
syms th(t)
assume(th(t), 'real')

rb = [d3 0 0 1];
w = [1 0 0]';
q = [d0 d1 0]';
v = -hat_PCM(w)*q;

xi = twistr_PCM(w,q); % Twist Function

Rab_0 = [0 1 0; 0 0 1; 1 0 0];
Pab_0 = [d0, d1+d2 0]';
gab_0 = [Rab_0, Pab_0; 0 0 0 1];

expt_HW3 = pretty(expt_PCM(xi, th(t)));
latex(expt_HW3);

gab_th = simplify(expt_HW3*gab_0);

gab_th_d = diff(gab_th,t);
% latex(gab_th_d);
R = gab_th(1:3,1:3);
P = gab_th(1:3,4);
gab_th_inv = [R', -R'*P; 0 0 0 1];
gab_th_inv = simplify(gab_th_inv);

Vs_ab = unwedge_PCM(simplify(gab_th_d*gab_th_inv));

%% Code HW3
syms theta real

gab_th = subs(gab_th, th(t), theta);
myFunHandle = matlabFunction(gab_th, "File", "HW3_p7_gab_th","Vars", [theta, d0, d1, d2]);

myFunHandle(d0, d1, d2, th(t));
% tic
% myFunHandle(3.1, 2, 4, 0)
% toc