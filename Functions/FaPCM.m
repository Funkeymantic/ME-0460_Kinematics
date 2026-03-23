function Fa = FaPCM(th, fa)
% FaPCM  Spatial wrench F^a caused by pure force fa applied at B-origin.
%   Fa = FaPCM(th, fa)
%   th : joint angle in radians (scalar)
%   fa : 3x1 force vector expressed in A-frame
%
% Force at point p_ab produces moment about A-origin:
%   m^a = p_ab x f_a
%   F^a = [f_a; m^a]   (6x1, MLS convention [f; m])

fa = fa(:);
g  = gabthPCM(th);
p  = g(1:3, 4);

ma = cross(p, fa);
Fa = [fa; ma];
end
