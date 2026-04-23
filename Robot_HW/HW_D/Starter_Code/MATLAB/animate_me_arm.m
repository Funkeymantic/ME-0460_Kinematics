function animate_me_arm(TH, path_ref_mm, label, record_video, filename)
% animate_me_arm  Animate the ME Arm V3 through a pre-computed joint trajectory.
%
%   animate_me_arm(TH, path_ref_mm, label, record_video, filename)
%
%   Inputs
%   ------
%   TH           – N×3 matrix of joint angles [theta, phi, psi] (radians)
%                  as returned by MEarmIK.  Row k is the pose at frame k.
%
%   path_ref_mm  – 3×N matrix of the desired tool-tip path in mm (world frame).
%                  Plotted as a blue dashed reference line.
%
%   label        – short string used in the figure title (e.g. 'star', 'moon')
%
%   record_video – logical — set true to save an MP4 alongside the animation
%
%   filename     – base filename for the video file (no extension needed)
%
%   Notes
%   -----
%   * Robot geometry (link lengths, patch shapes, colours) for the ME Arm V3
%     is hardcoded here so this function has no external dependencies beyond
%     the me_arm_gs* FK functions and rectverts.
%   * The red trail shows where the tool tip has actually been according to FK,
%     so any IK error is immediately visible as a gap between the trail and
%     the blue reference path.

% --- ME Arm V3 link dimensions (metres) ---
d2 = 80/1000;
d3 = 80/1000;
d4 = 60/1000;

% --- Build patch geometry for each link ---
% Base
wxB = 55/1000;  wyB = 60/1000;  lyfB = 18/1000;
lybB = wyB - lyfB;  hzB = 65/1000;
[F0,V0,C0] = rectverts([-0.5*wxB, 0.5*wxB, -lybB, lyfB, 0, hzB], [1 0 0]);

% Shoulder
wxS = 23/1000;  padZ = 6/1000;
[F1,V1,C1] = rectverts([-0.5*wxS, 0.5*wxS, -0.5*(6/1000), 0.5*(6/1000), ...
    -padZ-d2/2, padZ+d2/2], [0 1 0]);

% Elbow
wxE = 29/1000;  wzE = 6/1000;  padY = 4/1000;  %#ok<NASGU>
[F2,V2,C2] = rectverts([-0.5*wxE, 0.5*wxE, -padY-d3/2, padY+d3/2, ...
    -0.5*wzE, 0.5*wzE], [0 0 1]);

% Hand / tool
lhX = 40/1000;  lhZup = 23/1000;  lhZdown = 15/1000;  padYh = 4/1000;
[F3,V3,C3] = rectverts([-0.5*lhX, 0.5*lhX, -d4-padYh, 0, -lhZdown, lhZup], [1 0 1]);

% Homogeneous vertex matrices (4 × nVerts, ready to left-multiply by a 4×4 FK)
HV0 = [V0'; ones(1, size(V0,1))];
HV1 = [V1'; ones(1, size(V1,1))];
HV2 = [V2'; ones(1, size(V2,1))];
HV3 = [V3'; ones(1, size(V3,1))];

% Reference path in metres for plotting
ref_m = path_ref_mm / 1000;

% --- Figure setup ---
s  = get(0, 'ScreenSize');
hw = 0.7 * s(4);
fig = figure('Name', sprintf('ME Arm IK — %s', label), ...
    'Position', [(s(3)-hw)/2, (s(4)-hw)/2, hw, hw], 'color', 'w');
grid on;  hold on;  axis equal
xlabel('X (m)');  ylabel('Y (m)');  zlabel('Z (m)')
light;  daspect([1 1 1]);  view(120, 25)
axis([-0.05, 0.25, -0.1, 0.25, 0, 0.3])
set(gca, 'FontSize', 12)
title(sprintf('ME Arm IK Test — drawing a %s', label))

% static reference path drawn once (redrawn after every cla below)
plot3(ref_m(1,:), ref_m(2,:), ref_m(3,:), 'b--', 'LineWidth', 1.5);

% --- Optional video writer ---
vw = [];
if record_video
    vw = VideoWriter(filename, 'MPEG-4');
    vw.FrameRate = 30;
    set(gca, 'nextplot', 'replacechildren');
    set(gcf, 'Renderer', 'zbuffer');
    open(vw);
end

% --- Main animation loop ---
n_frames = size(TH, 1);
pts = zeros(4, n_frames);

for n = 1:n_frames
    th = TH(n, :);

    % Apply FK: transform patch vertices into world frame
    PV0 = me_arm_gs0(th) * HV0;
    PV1 = me_arm_gs1(th) * HV1;
    PV2 = me_arm_gs2(th) * HV2;
    PV3 = me_arm_gst(th) * HV3;

    % Record actual tool-tip position for the trail
    pts(:,n) = me_arm_gst(th) * [0; 0; 0; 1];

    % Strip homogeneous row, transpose to N×3 for patch()
    PV0 = PV0(1:3,:)';
    PV1 = PV1(1:3,:)';
    PV2 = PV2(1:3,:)';
    PV3 = PV3(1:3,:)';

    cla

    % Reference path (must redraw after cla)
    plot3(ref_m(1,:), ref_m(2,:), ref_m(3,:), 'b--', 'LineWidth', 1.5);

    % FK tool-tip trail  (red = where the arm has actually been)
    plot3(pts(1,1:n), pts(2,1:n), pts(3,1:n), 'r-', 'LineWidth', 2)

    % Robot links
    patch('Faces',F0,'Vertices',PV0,'FaceColor','flat','FaceVertexCData',C0,'FaceAlpha',0.6,'EdgeColor','k');
    patch('Faces',F1,'Vertices',PV1,'FaceColor','flat','FaceVertexCData',C1,'FaceAlpha',0.6,'EdgeColor','k');
    patch('Faces',F2,'Vertices',PV2,'FaceColor','flat','FaceVertexCData',C2,'FaceAlpha',0.6,'EdgeColor','k');
    patch('Faces',F3,'Vertices',PV3,'FaceColor','flat','FaceVertexCData',C3,'FaceAlpha',0.6,'EdgeColor','k');

    drawnow

    if record_video
        writeVideo(vw, getframe(fig));
    end
end

if record_video
    close(vw);
end

end
