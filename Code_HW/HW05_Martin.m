% HW05_MacMurdo.m
% ME4640 - Homework 5: Forward Kinematics of the Stanford Manipulator
% Author: Paul Martin (PCM)

clear; clc;

syms th1 th2 th3 th4 th5 th6 real

%% -------------------------------------------------------------------------
%  Robot parameters (cm)
% -------------------------------------------------------------------------
d1 = 40;
d2 = 50;
d3 = 60;
d6 = 30;

Wz  = [0 -1  0;  1  0  0;  0  0  0];   % skew([0;0;1])
Wz2 = [-1  0  0;  0 -1  0;  0  0  0];  % Wz^2

Wx  = [0  0  0;  0  0 -1;  0  1  0];   % skew([1;0;0])
Wx2 = [0  0  0;  0 -1  0;  0  0 -1];   % Wx^2

Wy  = [0  0  1;  0  0  0; -1  0  0];   % skew([0;1;0])
Wy2 = [-1  0  0;  0  0  0;  0  0 -1];  % Wy^2


% Joint 1 -- revolute, Rz(th1), p = 0 (axis through origin, v = 0)
R1 = eye(3) + sin(th1)*Wz + (1-cos(th1))*Wz2;
E1 = [R1, [0;0;0]; 0 0 0 1];

% Joint 2 -- revolute, Rx(th2), p = (I-R2)*[0;0;d1]
R2 = eye(3) + sin(th2)*Wx + (1-cos(th2))*Wx2;
p2 = (eye(3)-R2) * [0; 0; d1];
E2 = [R2, p2; 0 0 0 1];

% Joint 3 -- prismatic, translation along [0;0;1]
E3 = [eye(3), [0; 0; th3]; 0 0 0 1];

% Joint 4 -- revolute, Rz(th4), p = (I-R4)*[0;d2;0]
R4 = eye(3) + sin(th4)*Wz + (1-cos(th4))*Wz2;
p4 = (eye(3)-R4) * [0; d2; 0];
E4 = [R4, p4; 0 0 0 1];

% Joint 5 -- revolute, Rx(th5), p = (I-R5)*[0;d2;d1+d3]
R5 = eye(3) + sin(th5)*Wx + (1-cos(th5))*Wx2;
p5 = (eye(3)-R5) * [0; d2; d1+d3];
E5 = [R5, p5; 0 0 0 1];

% Joint 6 -- revolute, Ry(th6), p = (I-R6)*[0;0;d1+d3]
R6 = eye(3) + sin(th6)*Wy + (1-cos(th6))*Wy2;
p6 = (eye(3)-R6) * [0; 0; d1+d3];
E6 = [R6, p6; 0 0 0 1];

gst0 = [0, 1, 0,      0;
        0, 0, 1,  d2+d6;
        1, 0, 0,  d1+d3;
        0, 0, 0,      1];

gstth = E1 * E2 * E3 * E4 * E5 * E6 * gst0;

gstth = simplify(gstth);

disp('gstth =')
disp(gstth)

%% -------------------------------------------------------------------------
%  Generate gstthPCM.m
%  Input: 1x6 vector [th1 th2 th3 th4 th5 th6]
% -------------------------------------------------------------------------
matlabFunction(gstth, ...
    'File',    'Functions/gstthPCM', ...
    'Vars',    {[th1, th2, th3, th4, th5, th6]}, ...
    'Outputs', {'T'});

fprintf('\ngstthPCM.m created successfully.\n')