function Fb = FbPCM(th, fa)
% FbPCM  Body wrench F^b caused by pure force fa applied at B-origin.
%   Fb = FbPCM(th, fa)
%   th : joint angle in radians (scalar)
%   fa : 3x1 force vector expressed in A-frame
%
% Since force is applied at B-origin (no moment arm in B), m^b = 0.
% Force change of basis:  f^b = R_ab^T * f_a   (per hint)
%   F^b = [f^b; m^b] = [R^T*fa; 0_3]   (6x1, MLS convention [f; m])

fa = fa(:);
R  = gabthPCM(th);
R  = R(1:3, 1:3);

fb = R' * fa;
Fb = [fb; 0; 0; 0];
end
