function [th_a, th_b, th_c] = MEarmIK(x, y, z)
% MEarmIK  Inverse kinematics for the ME Arm V3 robot.
%
%   [th_a, th_b, th_c] = MEarmIK(x, y, z)
%
%   Inputs:  x, y, z  - desired tool tip position in WORLD frame (mm)
%                        World frame origin is at the base foot, with the
%                        shoulder pivot offset d0 = 140 mm along world X.
%
%   Outputs: theta - base rotation angle (rad)
%            phi   - shoulder angle mapped to motor frame (rad), elbow-up
%            psi   - shoulder angle mapped to motor frame (rad), elbow-down

% --- Robot constants (all mm) ---
L1  = 80;    % upper arm link length
L2  = 80;    % forearm link length

d0  = 140;   % base pivot offset from world origin along X (mm)
d4  = 60;    % tool tip to shoulder pivot, radial offset (mm)
d5  = 0;     % vertical offset from commanded z to FK tool frame (mm)
             % (set to 0 because me_arm_gst already places the tool frame at the
             %  correct tip position; gz=9 is a physical robot correction, not needed here)
d1  = 52.2;  % shoulder pivot height above ground plane (mm)

% --- Shift input from world frame into IK frame (origin at shoulder pivot XY) ---
x_ik = x - d0;   % subtract base X offset so IK origin is at pivot axis
y_ik = y;
z_ik = z;

% --- Solve base angle ---
% Arm home direction is +Y from the pivot, so angle is measured from +Y (not +X).
th_a = atan2(x_ik, y_ik);

% --- Solve planar 2-link IK in the arm's reach/elevation plane ---
% Radial reach in XY plane, subtract tool radial offset
r = sqrt(x_ik^2 + y_ik^2) - d4;

% Height relative to shoulder pivot, subtract tool vertical offset
h = z_ik - d5 - d1;

% Distance from shoulder pivot to wrist pivot
d = sqrt(r^2 + h^2);

% Reachability check
reach_ratio = d / (2*L1);  % valid only when L1 == L2
if abs(reach_ratio) > 1
    warning('MEarmIK: target out of workspace (reach_ratio = %.3f). Clamping.', reach_ratio);
    reach_ratio = sign(reach_ratio) * min(abs(reach_ratio), 1);
end

alpha = atan2(h, r);           % angle of target vector from horizontal
beta  = acos(reach_ratio);     % half-angle from law of cosines (L1 == L2 case)

% Two solutions:
phi = alpha + beta;

% Map geometric shoulder angle to motor angle convention
% In FK, th2=0 is arm vertical; th2=pi/2 is arm horizontal.
% phi_geom is angle of upper arm from horizontal, so th2 = pi/2 - phi_geom.
%th_b = -(phi - pi/2);   % (shoulder, elbow-up solution)
th_b = pi/2 - phi;   % (shoulder, elbow-up solution)

% Elbow joint angle
gamma = acos((d^2 - 2*L1^2)/(2*L1^2));
th_c = gamma- pi/2;

end
