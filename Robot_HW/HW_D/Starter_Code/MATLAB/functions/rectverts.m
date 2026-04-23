function [ F, V, C ] = rectverts( rect, c )
% function [ F, V, C ] = rectverts( rect, color )
% rect = [xmin xmax ymin ymax zmin zmax] for the desired rectangular prism
% c1 = face1 color using [r g b] = [red green blue], values 0 to 1
% c2,c3,c4,c5,c6 same as c1
% face1 is x-z plane at ymin
% face2 is y-z plane at xmax
% face3 is x-z plane at ymax
% face4 is y-z plane at xmin
% face5 is x-y plane at zmin
% face5 is x-y plane at zmax
if length(c) == 3
    c1 = c;
    c2 = c;
    c3 = c;
    c4 = c;
    c5 = c;
    c6 = c;
elseif length(c2)~=6
    '1x3 row vector or 6x3 matrix expected for c'
end

V = [ rect(1) rect(3) rect(5);
    rect(2) rect(3) rect(5);
    rect(2) rect(4) rect(5);
    rect(1) rect(4) rect(5);
    rect(1) rect(3) rect(6);
    rect(2) rect(3) rect(6);
    rect(2) rect(4) rect(6);
    rect(1) rect(4) rect(6)];

F = [1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8;
    1 2 3 4;
    5 6 7 8];

C = [c1;c2;c3;c4;c5;c6];
