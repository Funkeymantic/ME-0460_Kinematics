% ---------------------------------------------------------------------------
%  build_moon  –  crescent moon in the local XY plane, centred at origin
%
%  R     = outer (convex back) radius, mm
%  r_in  = inner (concave front) radius, mm
%  dx_in = inner circle centre offset in +X_local, mm
%          (controls how thin/thick the crescent is)
%
%  Returns pts (3xN, mm) with Z = 0, and a short label string.
% ---------------------------------------------------------------------------
function [pts, label] = build_moon(R, r_in, dx_in)

    % intersection of the two circles gives the crescent tips
    dx_tip = (R^2 - r_in^2 + dx_in^2) / (2*dx_in);
    dy_tip = sqrt(R^2 - dx_tip^2);

    phi_top = atan2( dy_tip, dx_tip);       % tip angles on outer circle
    phi_bot = atan2(-dy_tip, dx_tip);
    psi_top = atan2( dy_tip, dx_tip-dx_in); % tip angles on inner circle
    psi_bot = atan2(-dy_tip, dx_tip-dx_in);

    % outer arc: top tip → (CCW) → bottom tip  (convex back)
    n_outer = 60;
    ao = linspace(phi_top, phi_bot + 2*pi, n_outer);
    xo = R * cos(ao);   yo = R * sin(ao);

    % inner arc: bottom tip → (CW through 180°) → top tip  (concave front)
    n_inner = 40;
    ai = linspace(psi_bot, psi_top - 2*pi, n_inner);
    xi = dx_in + r_in*cos(ai);   yi = r_in*sin(ai);

    x = [xo, xi, xo(1)];
    y = [yo, yi, yo(1)];
    z = zeros(size(x));

    pts   = [x; y; z];
    label = 'moon';
end
