%% ME Arm FK and Animation Example
% This script implements FK and validates via animation for the ME ARM V3 robot

%% Setup
clc
close all
%clear all % comment out for symbolic toolkit bug
addpath("functions")

%% define FK
% define symbolic variables
syms th1 th2 th3 th4 thA thB thC thD d0 d1 d2 d3 d4 real

% define rotation axes
wA = [0 0 1]';
wB = [-1 0 0]';
wC = wB;
wD = wB;

% define points on rotation axes
qA = [d0 0 0]';
qB = [d0 0 d1]';
qC = [d0 0 d1+d2]';
qD = [d0 d3 d1+d2]';

% define twists
z0 = twistr(wA,qA);
z1 = twistr(wB,qB);
z2 = twistr(wC,qC);
z3 = twistr(wD,qD);

% define home configuration HTs
ps0_0 = [d0, 0, 0]';
ps1_0 = [d0, 0, d1+d2/2]';
ps2_0 = [d0, d3/2 , d1+d2]';
pst_0 = [d0, d3+d4, d1+d2]';

gs0_0 = [eye(3),ps0_0; 0 0 0 1];
gs1_0 = [eye(3),ps1_0; 0 0 0 1];
gs2_0 = [eye(3),ps2_0; 0 0 0 1];
gst_0 = [eye(3),pst_0; 0 0 0 1];

% define expt terms
expt0 = expt(z0,thA);
expt1 = expt(z1,thB);
expt2 = expt(z2,thC);
expt3 = expt(z3,thD);

% define gst_th
gs0_th = expt0*gs0_0;
gs1_th = expt0*expt1*gs1_0;
gs2_th = expt0*expt1*expt2*gs2_0;
gst_th = expt0*expt1*expt2*expt3*gst_0;

gs0_th = simplify(gs0_th);
gs1_th = simplify(gs1_th);
gs2_th = simplify(gs2_th);
gst_th = simplify(gst_th);

% substitute known lengths
nd0 = 140; % mm
nd1 = 51.75; % mm
nd2 = 80; % mm
nd3 = 80; % mm
nd4 = 60; % mm

gs0_th = subs(gs0_th,[d0,d1,d2,d3,d4],[nd0,nd1,nd2,nd3,nd4]./1000);
gs1_th = subs(gs1_th,[d0,d1,d2,d3,d4],[nd0,nd1,nd2,nd3,nd4]./1000);
gs2_th = subs(gs2_th,[d0,d1,d2,d3,d4],[nd0,nd1,nd2,nd3,nd4]./1000);
gst_th = subs(gst_th,[d0,d1,d2,d3,d4],[nd0,nd1,nd2,nd3,nd4]./1000);

% apply kinematic constraint
% th4 = -th2;
gs0_th = subs(gs0_th,[thD],[-thB - thC]);
gs1_th = subs(gs1_th,[thD],[-thB - thC]);
gs2_th = subs(gs2_th,[thD],[-thB - thC]);
gst_th = subs(gst_th,[thD],[-thB - thC]);

thABCD = [thA,thB,thC,thD];

% save as matlab functions
matlabFunction(gs0_th,"File","me_arm_gs0","Vars",{thABCD});
matlabFunction(gs1_th,"File","me_arm_gs1","Vars",{thABCD});
matlabFunction(gs2_th,"File","me_arm_gs2","Vars",{thABCD});
matlabFunction(gst_th,"File","me_arm_gst","Vars",{thABCD});

%% Create motor angle based FK functions
th_1234 = [-th1,th2,-th3,th4];
gs0_th = subs(gs0_th,[thA,thB,thC,thD],th_1234);
gs1_th = subs(gs1_th,[thA,thB,thC,thD],th_1234);
gs2_th = subs(gs2_th,[thA,thB,thC,thD],th_1234);
gst_th = subs(gst_th,[thA,thB,thC,thD],th_1234);

% save as matlab functions
th = [th1,th2,th3,th4];
matlabFunction(gs0_th,"File","me_arm_gs0_mot","Vars",{th});
matlabFunction(gs1_th,"File","me_arm_gs1_mot","Vars",{th});
matlabFunction(gs2_th,"File","me_arm_gs2_mot","Vars",{th});
matlabFunction(gst_th,"File","me_arm_gst_mot","Vars",{th});


%% Create 3D animation of the ME arm robot
% define Link Prisms for visualization

d0 = 140/1000; % m
d1 = 51.75/1000; % m
d2 = 80/1000; % m
d3 = 80/1000; % m
d4 = 60/1000; % m

% base (wrt base center)
wxB = 55/1000; % m
wyB = 60/1000; % m
lyfB = 18/1000;
lybB = wyB - lyfB;
hzB = 65/1000; % m
[F0, V0, C0] = rectverts( ...
    [-0.5*wxB, 0.5*wxB,...
     -lybB, lyfB, ...
      0,       hzB],[1 0 0]); % base

% shoulder (wrt center)
wxS = 23/1000; % m
wyS = 6/1000; % m
padZ = 6/1000; % m

[F1, V1, C1] = rectverts( ...
    [-0.5*wxS, 0.5*wxS, ...
     -0.5*wyS, 0.5*wyS, ...
     -padZ - d2/2, padZ+d2/2],[0 1 0]); % shoulder

% elbow (wrt center)
wxE = 29/1000; % m
wzE = 6/1000; % m
padY = 4/1000; % m
[F2, V2, C2] = rectverts( ...
    [-0.5*wxE, 0.5*wxE, ...
     -padY-d3/2,padY+d3/2, ...
     -0.5*wzE, 0.5*wzE],[0 0 1]); % elbow

% hand (wrt tool frame)
lhX = 40/1000; % m
lhY = d4;
lhZup = 23/1000; % m
lhZdown = 15/1000; % m
padYh = 4/1000; % m
[F3, V3, C3] = rectverts( ...
    [-0.5*lhX, 0.5*lhX, ...
     -lhY - padYh, 0, ...
     -lhZdown, lhZup],[1 0 1]);

% plot the prisms before FK mapping
% this step is helpful for confirming the dimensions of the prisms are
% correct and their local reference frame is located as desired
close all
fig = figure("Color",[1 1 1]);
grid on; hold on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z')   % add labels
light                                   % add a default light
daspect([1 1 1])                        % Setting the aspect ratio
view(3)                                 % Isometric view
%axis([-0.5 0.5 -0.5 0.5 -0 .5])         % set axis limits
set(gca,'FontSize',12)                  % set font size

% create patch objects
p0 = patch('faces', F0, 'vertices' ,V0);
p1 = patch('faces', F1, 'vertices' ,V1);
p2 = patch('faces', F2, 'vertices' ,V2);
p3 = patch('faces', F3, 'vertices' ,V3);

% Set the face color flat
set([p0 p1 p2 p3], 'facec', 'flat');

% Set the color (from file)
set(p0, 'FaceVertexCData', C0);
set(p1, 'FaceVertexCData', C1);
set(p2, 'FaceVertexCData', C2);
set(p3, 'FaceVertexCData', C3);

% Use for transparency (1 equals solid)
set([p0 p1 p2 p3], 'facealpha',.5)

% Set the edge color (use 'none' for STL files)
set([p0 p1 p2 p3], 'EdgeColor','k');    % use none for STL files

%% plot the robot in the home configuration using FK functions

% convert patch verts to homogenous format
HV0 = [V0';ones(1,length(V0))];
HV1 = [V1';ones(1,length(V1))];
HV2 = [V2';ones(1,length(V2))];
HV3 = [V3';ones(1,length(V3))];

% home configuration
th = deg2rad([0 0 0]);

% plotting
close all

% figure setup
s = get(0,'ScreenSize');
h = 0.7*s(4);     % 70% of screen height
dy = (s(4)-h)/2;    % vertical offset
dx = (s(3)-h)/2;    % horizontal offset
figure('Position',[dx dy h h],'color','w') %centered
grid on; hold on; axis equal
xlabel('X'); ylabel('Y'); zlabel('Z')
light                               % add a default light
daspect([1 1 1])                    % Setting the aspect ratio
view(3)                             % Isometric view
%axis([-0.6 0.6 -0.6 0.6 -0 .8])               % set axis limits
set(gca,'FontSize',12)

% rotate the points using homogeneous transformations
PV0 = me_arm_gs0(th)*HV0;
PV1 = me_arm_gs1(th)*HV1;
PV2 = me_arm_gs2(th)*HV2;
PV3 = me_arm_gst(th)*HV3;

% restore format of patch verts to non homogeneous format
PV0 = PV0(1:3,:)';
PV1 = PV1(1:3,:)';
PV2 = PV2(1:3,:)';
PV3 = PV3(1:3,:)';

% plot xyz of tool frame origin
%plot3(pts(1,1:end),pts(2,1:end),pts(3,1:end),'r-')

% create patch objects
p0 = patch('faces', F0, 'vertices' ,PV0);
p1 = patch('faces', F1, 'vertices' ,PV1);
p2 = patch('faces', F2, 'vertices' ,PV2);
p3 = patch('faces', F3, 'vertices' ,PV3);

% Set the face color flat
set([p0 p1 p2 p3], 'facec', 'flat');

% Set the color (from file)
set(p0, 'FaceVertexCData', C0);
set(p1, 'FaceVertexCData', C1);
set(p2, 'FaceVertexCData', C2);
set(p3, 'FaceVertexCData', C3);

% Use for transparency (1 equals solid)
set([p0 p1 p2 p3], 'facealpha',.5)

% Set the edge color (use 'none' for STL files)
set([p0 p1 p2 p3], 'EdgeColor','k');    % use none for STL files

%% Run and record an animation of the kinematics using video writer
close all
record_video = false;   % set true to save animation as MP4

% generate TH
framerate = 30;
tf = 4;
t = 0:1/framerate:tf; % minimum time steps for FPS frame rate
f1 = .5;
f2 = .75;

TH = [];
for n=1:length(t)
    th1 = pi/4*sin(2*pi*f1*t(n));
    th2 = pi/3*sin(2*pi*f2*t(n));
    th3 = pi/4*sin(2*pi*f1*t(n))-.1;
    TH = [TH; th1 th2 th3];
end 
label = "FK_Test";

% run animation
animate_me_arm_fk(TH, label, record_video, ['me_arm_' label]);

