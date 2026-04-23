function [pts, label] = build_trefoil_PCM(R_maj, R_min, N)
% build_trefoil_PCM  Sample a (2,3) torus-knot trefoil as a dense
% sequence of 3D Cartesian waypoints in the local frame, centred on
% the origin.
%
%   [pts, label] = build_trefoil_PCM(R_maj, R_min, N)
%
%   Parametric form (t in [0, 2*pi]):
%       u(t) = (R_maj + R_min*cos(3t)) * cos(2t)
%       v(t) = (R_maj + R_min*cos(3t)) * sin(2t)
%       w(t) =  R_min*sin(3t)
%
%   The (2,3) torus knot is topologically a single closed loop (the
%   "henagon" / "monogon" in loose usage -- one continuous self-
%   connected curve) but threads through all three coordinate
%   directions.  The six zeros of w(t) over [0, 2*pi] split the path
%   into six distinct arc sections, well above the assignment's
%   four-segment minimum.
%
%   Inputs (all optional):
%       R_maj - major torus radius (default 35 mm)
%       R_min - minor torus radius (default 15 mm)
%       N     - number of waypoint samples along the knot (default 360)
%
%   Outputs:
%       pts   - 3xN waypoint matrix (mm)
%       label - short string identifier used in figure titles
%
%   Bounding box:
%       u,v in [-(R_maj+R_min), +(R_maj+R_min)]
%       w   in [-R_min, +R_min]
%   Defaults give a 100 x 100 x 30 mm envelope, comfortably inside
%   the ME arm's reachable workspace when centred at (150, 120, 130) mm.
%
%   Author: Paul C. Martin (PCM)   ME 4640 S26  HW D

    if nargin < 1 || isempty(R_maj), R_maj = 35; end
    if nargin < 2 || isempty(R_min), R_min = 15; end
    if nargin < 3 || isempty(N),     N     = 360; end

    t    = linspace(0, 2*pi, N);
    cos3 = cos(3*t);  sin3 = sin(3*t);
    cos2 = cos(2*t);  sin2 = sin(2*t);

    u = (R_maj + R_min .* cos3) .* cos2;
    v = (R_maj + R_min .* cos3) .* sin2;
    w =  R_min .* sin3;

    pts   = [u; v; w];
    label = 'trefoil_3D';
end
