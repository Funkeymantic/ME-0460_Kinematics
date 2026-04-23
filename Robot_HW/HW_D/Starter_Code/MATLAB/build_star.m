% ---------------------------------------------------------------------------
%  build_star  –  5-pointed star in the local XY plane, centred at origin
%
%  radii  = [r_outer, r_inner]  (mm)
%  n_tips = number of star points  (5 for a standard star)
%
%  Returns pts (3xN, mm) with Z = 0, and a short label string.
% ---------------------------------------------------------------------------
function [pts, label] = build_star(radii, n_tips)
    r_outer = radii(1);
    r_inner = radii(2);

    ang_outer = pi/2 + (0:n_tips-1) * (2*pi/n_tips);   % first tip at +Y
    ang_inner = ang_outer + pi/n_tips;

    angs = reshape([ang_outer; ang_inner], 1, []);
    r    = reshape([repmat(r_outer,1,n_tips); repmat(r_inner,1,n_tips)], 1, []);

    x = r .* cos(angs);
    y = r .* sin(angs);
    z = zeros(size(x));

    % close the path
    pts   = [x, x(1); y, y(1); z, z(1)];
    label = 'star';
end