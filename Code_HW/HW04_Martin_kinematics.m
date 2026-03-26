% HW04_Martin_kinematics.m  —  ME4640 Code HW #4
%
% 1-DOF robot (Figure 2):
%   Revolute joint, omega = z-hat
%   d0=3, d1=1, d2=d3=2
%   Home position of B-origin in A-frame: [0, d1+d3, d0+d2] = [0, 3, 5]
%   R_ab(0) = I  (B axes aligned with A at home)
%
% Convention (MLS): twist = [v; omega],  wrench = [f; m]

clear; clc;
addpath("Functions\")

%% ── Symbolic setup ─────────────────────────────────────────────────────────
syms th fx fy fz real

d0 = 3; d1 = 1; d2 = 2; d3 = 2;

c = cos(th); s = sin(th);

%% ── 1. g_ab(th) ─────────────────────────────────────────────────────────────
R_sym = [c, -s, 0;
         s,  c, 0;
         0,  0, 1];

p0    = [0; d1+d3; d0+d2];          % home position [0;3;5]
p_sym = simplify(R_sym * p0);       % [-3s; 3c; 5]

g_sym = [R_sym, p_sym; 0, 0, 0, 1];

disp('g_ab(th)')
disp('-------------------------------------------------------------------')
disp(g_sym)

disp('g_ab(30 deg)')
disp('-------------------------------------------------------------------')
disp(gabthPCM(pi/6))

%% ── 2. Ad_{g_ab}(th) ────────────────────────────────────────────────────────
pH_sym = skew_sym(p_sym);

Ad_sym = [R_sym,      simplify(pH_sym * R_sym);
          zeros(3,3), R_sym               ];

disp('Ad_{g_ab}(th)')
disp('-------------------------------------------------------------------')
disp(Ad_sym)

disp('Ad_{g_ab}(30 deg)')
disp('-------------------------------------------------------------------')
disp(AdgPCM(pi/6))

%% ── 3. F^b  (body wrench) ───────────────────────────────────────────────────
fa_sym = [fx; fy; fz];
fb_sym = simplify(R_sym' * fa_sym);   % f^b = R^T * f_a (change of basis)
Fb_sym = [fb_sym; 0; 0; 0];

disp('F^b(th,[fx,fy,fz])  — body wrench [f^b; m^b]')
disp('-------------------------------------------------------------------')
disp(Fb_sym)

disp('F^b(30 deg, [7;6;9])')
disp('-------------------------------------------------------------------')
disp(FbPCM(pi/6, [7;6;9]))

%% ── 4. F^a  (spatial wrench) ────────────────────────────────────────────────
ma_sym = simplify(cross(p_sym, fa_sym));   % m^a = p x f_a
Fa_sym = [fa_sym; ma_sym];

disp('F^a(th,[fx,fy,fz])  — spatial wrench [f^a; m^a]')
disp('-------------------------------------------------------------------')
disp(Fa_sym)

disp('F^a(30 deg, [7;6;9])')
disp('-------------------------------------------------------------------')
disp(FaPCM(pi/6, [7;6;9]))

%% ── Helper ───────────────────────────────────────────────────────────────────
function S = skew_sym(v)
    S = [  0,   -v(3),  v(2);
           v(3),  0,   -v(1);
          -v(2),  v(1),  0  ];
end
