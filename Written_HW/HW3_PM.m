% HW 3 Demo
clear all; close all; clc;

addpath("ME-0460_Kinematics\Functions")

syms d0 d1 d2 d3 real
syms th(t)
assume(th(t), 'real')

rb = [d3 0 0 1]';
w = [1 0 0]';
q = [d0 d1 0]';
v = -hat_PCM(w)*q;

xi = twistr_PCM(w,q); % Twist Function

Rab_0 = [0 1 0; 0 0 1; 1 0 0];
Pab_0 = [d0, d1+d2 0]';
gab_0 = [Rab_0, Pab_0; 0 0 0 1];

expt_HW3 = expt_PCM(xi, th(t));
% pretty(expt_HW3);
% latex(expt_HW3);

gab_th = simplify(expt_HW3*gab_0);

ra_th = simplify(gab_th*rb);
Vra_th = diff(ra_th,th(t));

gab_th_d = diff(gab_th,t);
% latex(gab_th_d);
R = gab_th(1:3,1:3);
P = gab_th(1:3,4);
gab_th_inv = [R', -R'*P; 0 0 0 1];
gab_th_inv = simplify(gab_th_inv);

Vs_ab = unwedge_PCM(simplify(gab_th_d*gab_th_inv));


Vb_ab = simplify(adjoint_PCM(invHGT_PCM(hgt_PCM(R, P)))*Vs_ab);
%% Code HW3
syms theta real

gab_th = subs(gab_th, th(t), theta);
myFunHandle = matlabFunction(gab_th, "File", "HW3_p7_gab_th","Vars", [theta, d0, d1, d2]);

myFunHandle(d0, d1, d2, th(t));
% tic
% myFunHandle(3.1, 2, 4, 0)
% toc

disp("1)")
disp(rb)

disp("2)")
disp(xi)

disp("3)")
disp(gab_0)

disp("4)")
disp(expt_HW3)

disp("5)")
disp(gab_th)

disp("6)")
disp(ra_th)

disp("7)")
disp(Vs_ab)

disp("8)")
disp(Vra_th)

disp("9)")
disp(Vb_ab)
