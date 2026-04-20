% RobotC_Martin_Kinematics.m
% ME4640 S26 - Robot Homework C: Forward Kinematics

clear; clc;
addpath('Functions');

%% --- Link dimensions (mm, measured on physical robot) ---
x_base = 150;   
h_B    =  55;   
L_BC   =  75;   
L_CD   =  80;   
L_GRIP =  62;   

%% --- Symbolic variables (real -> simplify stays in sin/cos form) ---
syms thA thB thC thD real
syms th1 th2 th3 real

%% --- Twist vectors (L-shaped home configuration) ---
xi_A = [0;       -x_base;         0;   0; 0; 1];
xi_B = [0;           h_B;         0;   1; 0; 0];
xi_C = [0;      h_B+L_BC;         0;   1; 0; 0];   % C directly above B
xi_D = [0;      h_B+L_BC;      -L_CD;  1; 0; 0];   % D horizontal from C

%% --- Zero-configuration transform gst(0) ---
gst0 = [1, 0, 0,         x_base;
        0, 1, 0,  L_CD+L_GRIP;
        0, 0, 1,    h_B+L_BC;
        0, 0, 0,           1];

fprintf('gst(0) =\n'); disp(gst0)
fprintf('Tip at home: [%d; %d; %d] mm\n\n', x_base, L_CD+L_GRIP, h_B+L_BC)

%% --- Serial-chain FK (theta_A through theta_D) ---
gstABCD = expt_PCM(xi_A,thA) * expt_PCM(xi_B,thB) * ...
          expt_PCM(xi_C,thC) * expt_PCM(xi_D,thD) * gst0;
gstABCD = simplify(gstABCD);

fprintf('--- gstABCD ---\n')
disp(gstABCD)

%% --- Substitute servo angles ---
gstth = subs(gstABCD, [thA, thB, thC, thD], [th1, th2, -th3, th3-th2]);
gstth = simplify(gstth);

fprintf('--- gstth (theta_1, theta_2, theta_3) ---\n')
disp(gstth)

%% --- Home check ---
home_check = double(subs(gstth, {th1,th2,th3}, {0,0,0}));
fprintf('--- Home check (expect [I | [%d;%d;%d]]) ---\n', x_base, L_CD+L_GRIP, h_B+L_BC)
disp(home_check)

%% --- Generate gstthPCM_ME.m ---
matlabFunction(gstth, ...
    'File',    'Functions/gstthPCM_ME', ...
    'Vars',    {[th1, th2, th3]}, ...
    'Outputs', {'T'});
fprintf('gstthPCM_ME.m created.\n\n')

%% --- Cross-check: POE vs closed-form at validation points ---

fprintf('--- Cross-check: POE vs closed-form ---\n')
fprintf('%-6s %6s %6s %6s  |  %8s %8s %8s\n','Point','th1','th2','th3','x(mm)','y(mm)','z(mm)')
fprintf('%s\n', repmat('-',1,58))

pts = {{'HOME',0,0,0},{'A',30,30,0},{'B',-45,45,10},{'C',-30,20,20}};
for k = 1:numel(pts)
    p   = pts{k};
    nm  = p{1}; t1v=p{2}; t2v=p{3}; t3v=p{4};
    T   = gstthPCM_ME([t1v*pi/180, t2v*pi/180, t3v*pi/180]);
    z_cf = h_B    + L_BC*cosd(t2v) + L_CD*sind(t2v-t3v);
    r_cf = -L_BC*sind(t2v) + L_CD*cosd(t2v-t3v) + L_GRIP;
    x_cf = x_base - r_cf*sind(t1v);
    y_cf = r_cf*cosd(t1v);
    ok   = abs(T(1,4)-x_cf)<0.1 & abs(T(2,4)-y_cf)<0.1 & abs(T(3,4)-z_cf)<0.1;
    fprintf('%-6s %6.1f %6.1f %6.1f  |  %8.2f %8.2f %8.2f   match=%d\n', ...
            nm, t1v, t2v, t3v, T(1,4), T(2,4), T(3,4), ok)
end