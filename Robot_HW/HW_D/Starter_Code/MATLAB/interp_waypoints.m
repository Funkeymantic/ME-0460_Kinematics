% ---------------------------------------------------------------------------
%  interp_waypoints  –  linearly interpolate between successive waypoints
% ---------------------------------------------------------------------------
function [px, py, pz] = interp_waypoints(waypts, steps_per_seg)
    px = [];  py = [];  pz = [];
    for k = 1:size(waypts,2)-1
        px = [px, linspace(waypts(1,k), waypts(1,k+1), steps_per_seg)]; %#ok<AGROW>
        py = [py, linspace(waypts(2,k), waypts(2,k+1), steps_per_seg)]; %#ok<AGROW>
        pz = [pz, linspace(waypts(3,k), waypts(3,k+1), steps_per_seg)]; %#ok<AGROW>
    end
end