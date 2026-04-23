%% ME Arm IK Test
% Demonstrates inverse kinematics by commanding the arm to trace a shape
% in 3D space.
%
% The workflow has five explicit steps so the coordinate-frame story is
% visible at every stage:
%
%   1. Build the shape in its own LOCAL frame  (flat in the XY plane)
%   2. Visualise the local frame shape
%   3. Build T_world (a 4x4 homogeneous transform) and multiply
%      every point into world coordinates
%   4. Visualise the result in the world frame
%   5. Solve IK for every waypoint, then animate the arm
%
% To switch trajectories, comment/uncomment ONE line in "Choose trajectory".
% To move or reorient the shape, edit center_xyz and plane_str there too.

%% Setup
clc; close all;
addpath("functions")

%% ── Choose trajectory ────────────────────────────────────────────────────
%  Uncomment ONE shape.  All other parameters live here too

[waypts_local, label] = build_star([35, 15], 4);   % [r_outer r_inner], n_tips
%[waypts_local, label] = build_moon(35, 28, 20);  % R_outer, r_inner, dx_inner_offset

center_xyz = [140, 150, 130];   % mm — world-frame location of the shape centre
plane_str  = 'xz';              % plane the shape lies in: 'xy'  'xz'  'yz'
% ─────────────────────────────────────────────────────────────────────────

%% Step 1 — Visualise shape in its local frame
% The shape is defined flat in the XY plane, centred at the origin.
% "First tip" (or top of moon) always points along +Y_local.
plot_local_frame(waypts_local, label, plane_str);


%% Step 2 — Build the world transform and apply it
% build_T_world combines a pure rotation (from plane_str) with a
% translation (from center_xyz) into one 4x4 matrix.
T_world = build_T_world(center_xyz, plane_str);

% move every local point into world coordinates with transform.
n_local   = size(waypts_local, 2);
pts_h     = T_world * [waypts_local; ones(1, n_local)];   % homogeneous multiply
waypts_world = pts_h(1:3, :);                             % strip homogeneous row


%% Step 3 — Visualise shape in the world frame
plot_world_frame(waypts_world, center_xyz, plane_str, label);


%% Step 4 — Interpolate waypoints and solve IK
steps_per_seg = 20;
[px, py, pz] = interp_waypoints(waypts_world, steps_per_seg);

n_frames = length(px);
TH = zeros(n_frames, 3);
for k = 1:n_frames
    [th1, th2, th3] = MEarmIK(px(k), py(k), pz(k));
    TH(k, :) = [th1, th2, th3];
end


%% Step 5 - Animate
record_video = false;
animate_me_arm(TH, [px; py; pz], label, record_video, ['me_arm_' label]);

%% Step 6 - Export
ME_Arm_IK_Export(TH)


%% ═══════════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS
% ═══════════════════════════════════════════════════════════════════════════

% ---------------------------------------------------------------------------
%  Primitive 4x4 rotation matrices
%  Each rotates by angle a (radians) about the named world axis.
% ---------------------------------------------------------------------------
function T = rot_x(a)
    T = [1,      0,       0,  0;
         0, cos(a), -sin(a),  0;
         0, sin(a),  cos(a),  0;
         0,      0,       0,  1];
end

function T = rot_y(a)
    T = [ cos(a), 0, sin(a), 0;
               0, 1,      0, 0;
         -sin(a), 0, cos(a), 0;
               0, 0,      0, 1];
end

function T = rot_z(a)
    T = [cos(a), -sin(a), 0, 0;
         sin(a),  cos(a), 0, 0;
              0,       0, 1, 0;
              0,       0, 0, 1];
end

% ---------------------------------------------------------------------------
%  plane_rot  –  4x4 rotation that maps the canonical local frame
%                (shape flat in XY, first tip along +Y_local, normal = +Z_local)
%                into the requested world plane.
%
%  'xy'  ->  no rotation needed   (Y_local -> +Y_world, normal -> +Z_world)
%  'xz'  ->  rot_x(-90°)          (Y_local -> +Z_world, normal -> -Y_world)
%  'yz'  ->  rot_y(90°)*rot_z(90°)(X_local -> +Y_world, Y_local -> +Z_world)
% ---------------------------------------------------------------------------
function T = plane_rot(plane_str)
    switch lower(plane_str)
        case 'xy'
            T = eye(4);
        case 'xz'
            T = rot_x(-pi/2);
        case 'yz'
            T = rot_y(pi/2) * rot_z(pi/2);
        otherwise
            error('plane_rot: unknown plane "%s". Use ''xy'', ''xz'', or ''yz''.', plane_str);
    end
end

% ---------------------------------------------------------------------------
%  build_T_world  –  assemble the full 4x4 world transform
%                    rotation from plane_str, translation from center_xyz
% ---------------------------------------------------------------------------
function T = build_T_world(center_xyz, plane_str)
    T        = plane_rot(plane_str);   % start with pure rotation
    T(1:3,4) = center_xyz(:);          % insert translation (mm)
end

% ---------------------------------------------------------------------------
%  plot_local_frame  –  Figure 1: shape before any world transform
%
%  Shows the canonical shape flat in the XY plane.  The local X and Y
%  axis arrows are drawn so students can see what "local frame" means,
%  and a caption tells them which world plane it will map into.
% ---------------------------------------------------------------------------
function plot_local_frame(pts, label, plane_str)
    figure('Name', sprintf('Step 1 — "%s" in local frame', label), 'color', 'w');
    hold on;  grid on;  axis equal;

    % shape
    plot(pts(1,:), pts(2,:), 'b-o', 'LineWidth', 2, 'MarkerSize', 4);

    % origin marker
    plot(0, 0, 'k+', 'MarkerSize', 14, 'LineWidth', 2);

    % local axis arrows
    sc = max(abs(pts(:))) * 0.45;
    quiver3(0, 0, 0, sc, 0,  0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(0, 0, 0, 0, sc,  0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(0, 0, 0, 0,  0, sc, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    text( sc*1.08, 0, 0, 'x_{local}', 'FontSize', 11, 'Color', 'r');
    text( 0, sc*1.08, 0, 'y_{local}', 'FontSize', 11, 'Color', [0 0.6 0]);
    text( 0, 0, sc*1.08, 'z_{local}', 'FontSize', 11, 'Color', [0 0 0.6]);

    xlabel('u  (mm)');  ylabel('v  (mm)');
    title(sprintf('"%s" — local frame  (will map into the %s plane)', ...
        label, upper(plane_str)), 'FontSize', 13);
    set(gca, 'FontSize', 12);
end

% ---------------------------------------------------------------------------
%  plot_world_frame  –  Figure 2: shape after T_world is applied
%
%  Shows the world frame axes and the transformed shape in 3-D so students
%  can directly compare with Figure 1.
% ---------------------------------------------------------------------------
function plot_world_frame(pts_mm, center_xyz, plane_str, label)
    figure('Name', sprintf('Step 2 — "%s" in world frame (%s)', label, plane_str), ...
        'color', 'w');
    hold on;  grid on;  axis equal;

    % world frame axes (drawn from origin)
    sc = 60;   % mm
    quiver3(0,0,0, sc,0,0, 'r', 'LineWidth',2,'MaxHeadSize',0.5,'AutoScale','off');
    quiver3(0,0,0, 0,sc,0, 'g', 'LineWidth',2,'MaxHeadSize',0.5,'AutoScale','off');
    quiver3(0,0,0, 0,0,sc, 'b', 'LineWidth',2,'MaxHeadSize',0.5,'AutoScale','off');
    text(sc*1.1, 0,     0,     'X_W','FontSize',11,'Color','r');
    text(0,      sc*1.1,0,     'Y_W','FontSize',11,'Color',[0 0.6 0]);
    text(0,      0,     sc*1.1,'Z_W','FontSize',11,'Color','b');

    % shape
    plot3(pts_mm(1,:), pts_mm(2,:), pts_mm(3,:), 'b-o', ...
        'LineWidth', 2, 'MarkerSize', 4);

    % centre marker
    plot3(center_xyz(1), center_xyz(2), center_xyz(3), 'k+', ...
        'MarkerSize', 14, 'LineWidth', 2);

    xlabel('X  (mm)');  ylabel('Y  (mm)');  zlabel('Z  (mm)');
    title(sprintf('"%s" — after T_{world}  (plane = %s,  centre = [%g %g %g] mm)', ...
        label, upper(plane_str), center_xyz(1), center_xyz(2), center_xyz(3)), ...
        'FontSize', 12);
    view(120, 25);
    set(gca, 'FontSize', 12);
end
