% HW06_Martin_Animation.m
% ME4640 S26 - Code HW #6
% Paul Martin (PCM)

clear; clc;
addpath('Functions');

d3   = 60;
fps  = 24;
t_end = 4;
t_vec = linspace(0, t_end, fps*t_end + 1);
N = length(t_vec);

% Trajectory
th1_f = @(t)  3*pi/4 + (pi/4)*sin(pi*t/2);
th2_f = @(t)  (pi/2)*sin(pi*t/2);
th3_f = @(t)  (d3/2)*sin(pi*t);
th4_f = @(t)  (pi/2)*sin(pi*t/2);
th5_f = @(t)  pi/2 + (pi/2)*sin(pi*t);
th6_f = @(t)  (pi/2)*sin(pi*t/2);

% Derivatives
dth1_f = @(t)  (pi^2/8)*cos(pi*t/2);
dth2_f = @(t)  (pi^2/4)*cos(pi*t/2);
dth3_f = @(t)  (d3/2)*pi*cos(pi*t);
dth4_f = @(t)  (pi^2/4)*cos(pi*t/2);
dth5_f = @(t)  (pi^2/2)*cos(pi*t);
dth6_f = @(t)  (pi^2/4)*cos(pi*t/2);

out_dir = 'Code_HW';
if ~exist(out_dir,'dir'), mkdir(out_dir); end

% FK check
T_check = real(gstthPCM([3*pi/4, 0, 0, 0, pi/2, 0]));
p_check = T_check(1:3,4);
fprintf('t=0 tool position: x=%.2f  y=%.2f  z=%.2f cm\n', p_check(1), p_check(2), p_check(3));
if norm(p_check) < 5
    error('gstthPCM returned near-zero -- re-run HW06_Martin_Kinematics.m first.');
end

% Precompute
p0_all   = zeros(3,N);
p1_all   = zeros(3,N);
p2_all   = zeros(3,N);
p3_all   = zeros(3,N);
pst_all  = zeros(3,N);
R5_all   = zeros(3,3,N);
Rst_all  = zeros(3,3,N);
pos_tool = zeros(3,N);
vel_tool = zeros(3,N);

for k = 1:N
    t   = t_vec(k);
    th  = [th1_f(t), th2_f(t), th3_f(t), th4_f(t), th5_f(t), th6_f(t)];
    dth = [dth1_f(t); dth2_f(t); dth3_f(t); dth4_f(t); dth5_f(t); dth6_f(t)];

    T1 = real(gs1PCM(th));
    T2 = real(gs2PCM(th));
    T3 = real(gs3PCM(th));
    T5 = real(gs5PCM(th));
    Ts = real(gstthPCM(th));

    p1_all(:,k)   = T1(1:3,4);
    p2_all(:,k)   = T2(1:3,4);
    p3_all(:,k)   = T3(1:3,4);
    pst_all(:,k)  = Ts(1:3,4);
    R5_all(:,:,k) = T5(1:3,1:3);
    Rst_all(:,:,k)= Ts(1:3,1:3);
    pos_tool(:,k) = Ts(1:3,4);

    J = real(JstthPCM(th));
    vel_tool(:,k) = J(1:3,:) * dth;
end

% Animation
fig_anim = figure('Name','Stanford Manipulator','Color','k','Position',[50 50 960 720]);
vid = VideoWriter(fullfile(out_dir,'HW06_Martin_Video'),'MPEG-4');
vid.FrameRate = fps; vid.Quality = 92;
open(vid);

W = 8;
for k = 1:N
    p1  = p1_all(:,k);
    p2  = p2_all(:,k);
    p3  = p3_all(:,k);
    pst = pst_all(:,k);
    R5  = R5_all(:,:,k);
    Rst = Rst_all(:,:,k);

    clf(fig_anim);
    ax = axes('Parent',fig_anim,'Color',[0.08 0.08 0.12], ...
              'GridColor',[0.3 0.3 0.3],'XColor','w','YColor','w','ZColor','w');
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    view(ax, 40, 20);
    xlim(ax,[-200  100]); ylim(ax,[-200  100]); zlim(ax,[-10  200]);
    xlabel(ax,'X (cm)','Color','w'); ylabel(ax,'Y (cm)','Color','w'); zlabel(ax,'Z (cm)','Color','w');
    title(ax, sprintf('Stanford Manipulator  |  t = %.2f s', t_vec(k)), ...
          'Color','w','FontSize',13,'FontWeight','normal');

    drawBasePlate(ax, 80);
    drawLink3D(ax, [0;0;0], p1, W, W, [0.60 0.60 0.65]);
    drawLink3D(ax, p1, p2, W, W, [0.25 0.45 0.85]);
    drawLink3D(ax, p2, p3, W, W, [0.20 0.70 0.30]);
    drawLink3D(ax, p3, pst, W*0.5, W*0.5, [0.95 0.80 0.15]);

    aL = 22;
    plotFrameAxis(ax, p3,  R5(:,1),  aL*0.8, [1.0 0.3 0.3]);
    plotFrameAxis(ax, p3,  R5(:,2),  aL*0.8, [0.3 1.0 0.3]);
    plotFrameAxis(ax, p3,  R5(:,3),  aL*0.8, [0.3 0.3 1.0]);
    plotFrameAxis(ax, pst, Rst(:,1), aL,     [1.0 0.2 0.2]);
    plotFrameAxis(ax, pst, Rst(:,2), aL,     [0.2 1.0 0.2]);
    plotFrameAxis(ax, pst, Rst(:,3), aL,     [0.2 0.2 1.0]);

    drawSphere3D(ax, [0;0;0], 6, [0.7 0.7 0.7]);
    drawSphere3D(ax, p1, 6, [0.4 0.6 1.0]);
    drawSphere3D(ax, p2, 6, [0.3 0.85 0.4]);
    drawSphere3D(ax, p3, 8, [1.0 0.45 0.2]);
    drawSphere3D(ax, pst, 5, [1.0 0.9 0.2]);

    if k > 1
        plot3(ax, pos_tool(1,1:k), pos_tool(2,1:k), pos_tool(3,1:k), 'c-', 'LineWidth', 1.2);
    end

    drawnow limitrate;
    writeVideo(vid, getframe(fig_anim));
end
close(vid); close(fig_anim);

% Position and velocity plots
fig_pv = figure('Name','Tool Position & Velocity','Units','normalized','Position',[0.08 0.05 0.84 0.88]);

subplot(2,1,1);
plot(t_vec, pos_tool(1,:), 'r-', 'LineWidth', 2); hold on;
plot(t_vec, pos_tool(2,:), 'g-', 'LineWidth', 2);
plot(t_vec, pos_tool(3,:), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Position (cm)');
title('Tool Frame Origin Position vs Time');
legend('x','y','z','Location','best');
xlim([0 t_end]); grid on;

subplot(2,1,2);
plot(t_vec, vel_tool(1,:), 'r-', 'LineWidth', 2); hold on;
plot(t_vec, vel_tool(2,:), 'g-', 'LineWidth', 2);
plot(t_vec, vel_tool(3,:), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Velocity (cm/s)');
title('Tool Frame Origin Velocity vs Time');
legend('$\dot{x}$','$\dot{y}$','$\dot{z}$','Interpreter','latex','Location','best');
xlim([0 t_end]); grid on;

set(fig_pv,'PaperUnits','inches','PaperSize',[8.5 11],'PaperPositionMode','auto');
print(fig_pv, fullfile(out_dir,'HW06_Martin_posvelplot'), '-dpdf', '-bestfit');

%% Local functions

function drawLink3D(ax, p1, p2, W, H, color)
    L = norm(p2-p1);
    if L < 1e-6, return; end
    z_hat = (p2-p1)/L;
    if abs(dot(z_hat,[1;0;0])) < 0.9
        x_hat = cross(z_hat,[1;0;0]);
    else
        x_hat = cross(z_hat,[0;1;0]);
    end
    x_hat = x_hat/norm(x_hat);
    y_hat = cross(z_hat,x_hat);
    R = [x_hat, y_hat, z_hat];
    v = [-W/2,-H/2,0; W/2,-H/2,0; W/2,H/2,0; -W/2,H/2,0;
         -W/2,-H/2,L; W/2,-H/2,L; W/2,H/2,L; -W/2,H/2,L]';
    v_world = R*v + p1*ones(1,8);
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Vertices',v_world','Faces',faces,'FaceColor',color, ...
          'EdgeColor','k','FaceAlpha',0.78,'LineWidth',0.5,'Parent',ax);
end

function drawSphere3D(ax, center, r, color)
    [sx,sy,sz] = sphere(14);
    surf(ax, r*sx+center(1), r*sy+center(2), r*sz+center(3), ...
         'FaceColor',color,'EdgeColor','none','FaceAlpha',0.9);
end

function plotFrameAxis(ax, origin, dir, L, color)
    d = dir/norm(dir);
    p_end = origin + L*d;
    plot3(ax,[origin(1),p_end(1)],[origin(2),p_end(2)],[origin(3),p_end(3)], ...
          '-','Color',color,'LineWidth',2.2);
    drawSphere3D(ax, p_end, L*0.07, color);
end

function drawBasePlate(ax, r)
    th = linspace(0,2*pi,60);
    fill3(ax, r*cos(th), r*sin(th), zeros(1,60), [0.25 0.25 0.30], ...
          'EdgeColor','none','FaceAlpha',0.6);
    for ang = 0:30:150
        dx = r*cosd(ang); dy = r*sind(ang);
        plot3(ax,[-dx dx],[-dy dy],[0 0],'-','Color',[0.4 0.4 0.45],'LineWidth',0.5);
    end
end