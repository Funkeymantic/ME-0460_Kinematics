%% ME 4640 S26 - Robot HW D: ME Arm IK Trajectory (3D Trefoil)
%  Author: Paul C. Martin (PCM)
%
%  Workflow:
%    1. Build a (2,3) torus-knot trefoil in a local frame centred on
%       the origin (build_trefoil_PCM).  Topologically this is a single
%       closed loop -- a "henagon" in the loose sense -- but it threads
%       through all three coordinate directions, giving six distinct
%       arc segments (one per zero-crossing of the z coordinate),
%       which easily satisfies the four-segment minimum.
%    2. Translate the trefoil into the robot's reachable workspace
%       with a single 4x4 homogeneous transform.
%    3. (Parametric sampling is already dense -- no linear interpolation
%       step is needed; the interp helper remains available below.)
%    4. Run ME_Arm_IK_PCM on every sample to obtain the kinematic
%       joint angles [th1 th2 th3].
%    5. Validate by running FK (gstthPCM_ME) and checking that the
%       recovered tip position matches the commanded one.
%    6. Animate the simulated arm tracing the full path (local function
%       animate_arm_PCM below).  Optionally write an MP4.
%    7. Export trajectory.h with the motor-angle mapping required by
%       HWD_IK_Trajectory.ino  (via ME_Arm_IK_Export_PCM).
% ============================================================

clear; clc; close all;

% Anchor paths to THIS script's folder so the script runs regardless of pwd
% and we do not accidentally shadow our functions with the starter versions.
this_dir  = fileparts(mfilename('fullpath'));     % .../Robot_HW/HW_D
addpath(this_dir);                                % IK, build_trefoil, exporter
addpath(fullfile(this_dir, '..', '..', 'Functions'));   % gstthPCM_ME etc.

%% --- Trajectory tuning knobs ---------------------------------------------
R_MAJ   = 35;                                     % mm, torus major radius
R_MIN   = 15;                                     % mm, torus minor radius
N_SAMP  = 360;                                    % parametric samples along knot

% Plane to draw the local frame in, and world-frame centre (mm).
% 'xy' means identity rotation (local u,v,w -> world X,Y,Z).
PLANE_STR  = 'xy';
CENTER_XYZ = [150, 120, 130];                     % world coords of knot centre

% Animation / export controls
RECORD_VIDEO   = true;                            % save MP4 alongside figure
VIDEO_FILENAME = 'RobotD_Martin_Simulation';      % no extension
ANIM_FPS       = 15;                              % -> 24 s for 360 frames
EXPORT_HEADER  = true;                            % write trajectory.h
HEADER_FILE    = 'trajectory.h';

%% --- Step 1 -- build the trefoil in its local frame ---------------------
[waypts_local, label] = build_trefoil_PCM(R_MAJ, R_MIN, N_SAMP);

figure('Name','Step 1 - trefoil in local frame','Color','w');
plot3(waypts_local(1,:), waypts_local(2,:), waypts_local(3,:), ...
      '-', 'LineWidth', 1.8); hold on; grid on; axis equal
plot3(0, 0, 0, 'k+', 'MarkerSize', 14, 'LineWidth', 2);
xlabel('u (mm)'); ylabel('v (mm)'); zlabel('w (mm)');
title(sprintf('Local frame "%s"  (R_{maj}=%g, R_{min}=%g mm)', ...
              strrep(label,'_',' '), R_MAJ, R_MIN));
view(30, 25);

%% --- Step 2 -- map the trefoil into the world frame ---------------------
T_world      = build_T_world_PCM(CENTER_XYZ, PLANE_STR);
pts_h        = T_world * [waypts_local; ones(1, size(waypts_local,2))];
waypts_world = pts_h(1:3, :);

figure('Name','Step 2 - trefoil in world frame','Color','w');
plot3(waypts_world(1,:), waypts_world(2,:), waypts_world(3,:), ...
      '-', 'LineWidth', 1.8); hold on; grid on; axis equal
plot3(CENTER_XYZ(1), CENTER_XYZ(2), CENTER_XYZ(3), 'k+', ...
      'MarkerSize', 14, 'LineWidth', 2);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title(sprintf('World frame "%s"  (centre=[%g %g %g] mm)', ...
              strrep(label,'_',' '), CENTER_XYZ));
view(120, 25);

%% --- Step 3 -- parametric samples already dense; no interp needed -------
px = waypts_world(1, :);
py = waypts_world(2, :);
pz = waypts_world(3, :);
N  = numel(px);
fprintf('Trefoil path: %d parametric samples.\n', N);

%% --- Step 4 -- run IK for every point ------------------------------------
TH = zeros(N, 3);
for k = 1:N
    TH(k,:) = ME_Arm_IK_PCM(px(k), py(k), pz(k));
end
if any(isnan(TH(:)))
    error(['%d waypoints fell outside the reachable workspace. ', ...
           'Shrink R_MAJ/R_MIN or move CENTER_XYZ closer to home.'], ...
           nnz(any(isnan(TH),2)));
end

%% --- Step 5 -- validate IK by running FK and comparing to the command ---
tip_fk = zeros(3, N);
for k = 1:N
    T = gstthPCM_ME(TH(k,:));                     % Paul's HW_C FK
    tip_fk(:,k) = T(1:3, 4);
end
err = sqrt(sum((tip_fk - [px;py;pz]).^2, 1));
fprintf('IK-FK round-trip: max |error| = %.3e mm,  mean = %.3e mm.\n', ...
        max(err), mean(err));
if max(err) > 1e-6
    warning('IK round-trip error exceeds 1e-6 mm; check link dimensions.');
end

figure('Name','Step 4 - joint trajectories','Color','w');
subplot(3,1,1); plot(rad2deg(TH(:,1)),'LineWidth',1.5); grid on
ylabel('\theta_1 (deg)'); title('Base');
subplot(3,1,2); plot(rad2deg(TH(:,2)),'LineWidth',1.5); grid on
ylabel('\theta_2 (deg)'); title('Shoulder');
subplot(3,1,3); plot(rad2deg(TH(:,3)),'LineWidth',1.5); grid on
ylabel('\theta_3 (deg)'); title('Elbow');
xlabel('Waypoint index')
sgtitle('Kinematic joint angles (pre-motor-mapping)');

%% --- Step 6 -- animate the arm tracing the path -------------------------
animate_arm_PCM(TH, [px;py;pz], label, ...
                RECORD_VIDEO, VIDEO_FILENAME, ANIM_FPS);

%% --- Step 7 -- export trajectory.h for the Arduino ----------------------
if EXPORT_HEADER
    ME_Arm_IK_Export_PCM(TH, HEADER_FILE);
end


% =========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

function T = build_T_world_PCM(center_xyz, plane_str)
% build_T_world_PCM  4x4 transform mapping the local shape frame into
% the world frame.  Unlike the starter's plane_rot, this maps +y_local to
% +z_world for 'xz', so planar shapes drawn in local XY appear right-side-up.
    switch lower(plane_str)
        case 'xy'
            R = eye(3);
        case 'xz'
            R = [1 0 0; 0 0 -1; 0 1 0];           % +y_local -> +z_world
        case 'yz'
            R = [0 0 1; 1 0 0; 0 1 0];            % +x_local -> +y_world,
                                                  % +y_local -> +z_world
        otherwise
            error('build_T_world_PCM: unknown plane "%s".', plane_str);
    end
    T         = eye(4);
    T(1:3,1:3)= R;
    T(1:3,4)  = center_xyz(:);
end


function [px, py, pz] = interp_waypoints_PCM(waypts, steps_per_seg)
% Linear interpolation between successive waypoints (same contract as the
% starter's interp_waypoints.m).
    px = [];  py = [];  pz = [];
    for k = 1:size(waypts,2)-1
        px = [px, linspace(waypts(1,k), waypts(1,k+1), steps_per_seg)]; %#ok<AGROW>
        py = [py, linspace(waypts(2,k), waypts(2,k+1), steps_per_seg)]; %#ok<AGROW>
        pz = [pz, linspace(waypts(3,k), waypts(3,k+1), steps_per_seg)]; %#ok<AGROW>
    end
end


function animate_arm_PCM(TH, ref_mm, label, record_video, filename, fps)
% animate_arm_PCM  Minimal 3-link arm visualisation using Paul's FK
% (gstthPCM_ME).  Draws the shoulder, elbow, wrist, and tip as line
% segments, with a blue dashed reference path and a red "actually
% visited" trail for quick visual validation.
%
%   TH       - N x 3 kinematic joint angles (rad)
%   ref_mm   - 3 x N desired tip path (mm), drawn as blue dashed
%   label    - figure/title string
%   record_video, filename, fps - video writer options

    % Link dimensions (mm) -- keep in sync with ME_Arm_IK_PCM.m
    x_base = 150;  h_B = 55;  L_BC = 75;  L_CD = 80;  L_GRIP = 62;

    N = size(TH, 1);
    fig = figure('Name', sprintf('ME Arm IK - %s', label), ...
                 'Position', [100 100 900 800], 'Color', 'w');
    grid on; hold on; axis equal
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)')
    view(120, 20); set(gca, 'FontSize', 12)
    title(sprintf('ME Arm tracing "%s"', strrep(label,'_',' ')))

    % Conservative axis limits
    axmin = min([ref_mm(1,:) 0 x_base]) - 50;
    axmax = max([ref_mm(1,:) x_base]) + 50;
    aymin = min([ref_mm(2,:) 0]) - 50;
    aymax = max([ref_mm(2,:) 0]) + 50;
    azmin = 0;
    azmax = max([ref_mm(3,:) h_B+L_BC+L_CD]) + 20;
    axis([axmin axmax aymin aymax azmin azmax])

    % Video writer setup
    vw = [];
    if record_video
        vw = VideoWriter(filename, 'MPEG-4');
        vw.FrameRate = fps;
        open(vw);
    end

    trail = zeros(3, N);
    for n = 1:N
        th1 = TH(n,1); th2 = TH(n,2); th3 = TH(n,3);

        % Joint positions from Paul's FK (see RobotC_Martin_Kinematics.m)
        r_elbow = -L_BC * sin(th2);
        r_wrist = r_elbow + L_CD * cos(th2 - th3);

        shoulder = [x_base; 0; h_B];
        elbow    = [x_base - r_elbow*sin(th1);
                           r_elbow*cos(th1);
                    h_B + L_BC*cos(th2)];
        wrist    = [x_base - r_wrist*sin(th1);
                           r_wrist*cos(th1);
                    h_B + L_BC*cos(th2) + L_CD*sin(th2 - th3)];
        T        = gstthPCM_ME([th1 th2 th3]);
        tip      = T(1:3,4);
        trail(:,n) = tip;

        cla

        % Ground disc for visual reference
        th_d = linspace(0, 2*pi, 40);
        fill3(x_base + 30*cos(th_d), 30*sin(th_d), zeros(size(th_d)), ...
              [0.85 0.85 0.85], 'EdgeColor', [0.5 0.5 0.5]);

        % Reference path (blue dashed) and trail (red solid)
        plot3(ref_mm(1,:), ref_mm(2,:), ref_mm(3,:), 'b--', 'LineWidth', 1.4);
        plot3(trail(1,1:n), trail(2,1:n), trail(3,1:n), 'r-', 'LineWidth', 2);

        % Arm links
        plot3([x_base x_base], [0 0], [0 h_B], 'k-', 'LineWidth', 4);            % base post
        plot3([shoulder(1) elbow(1)], [shoulder(2) elbow(2)], ...
              [shoulder(3) elbow(3)], 'Color', [0.1 0.5 0.9], 'LineWidth', 5);   % upper arm
        plot3([elbow(1) wrist(1)], [elbow(2) wrist(2)], ...
              [elbow(3) wrist(3)], 'Color', [0.1 0.7 0.3], 'LineWidth', 5);      % forearm
        plot3([wrist(1) tip(1)], [wrist(2) tip(2)], [wrist(3) tip(3)], ...
              'Color', [0.9 0.3 0.1], 'LineWidth', 4);                           % gripper

        % Joints
        plot3([shoulder(1) elbow(1) wrist(1) tip(1)], ...
              [shoulder(2) elbow(2) wrist(2) tip(2)], ...
              [shoulder(3) elbow(3) wrist(3) tip(3)], ...
              'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'y');

        title(sprintf('ME Arm tracing "%s"  (frame %d / %d)', ...
                      strrep(label,'_',' '), n, N))
        drawnow

        if record_video
            writeVideo(vw, getframe(fig));
        end
    end

    if record_video
        close(vw);
        fprintf('Saved simulation video: %s.mp4\n', filename);
    end
end
