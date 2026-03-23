function g = gabthPCM(th)
% gabthPCM  Homogeneous transformation g_ab(theta) for 1-DOF robot.
%   g = gabthPCM(th)  returns 4x4 g_ab at joint angle th (radians).
%
% Robot geometry (Figure 2):
%   d0=3, d1=1, d2=d3=2
%   Revolute joint, omega = z-hat; home position of B-origin: [0, d1+d3, d0+d2] = [0,3,5]

d0 = 3; d1 = 1; d2 = 2; d3 = 2;

c = cos(th);
s = sin(th);

% p_ab(th) = Rz(th) * [0; d1+d3; d0+d2]
px = -(d1+d3)*s;
py =  (d1+d3)*c;
pz =  (d0+d2);

g = [ c, -s, 0, px;
      s,  c, 0, py;
      0,  0, 1, pz;
      0,  0, 0,  1 ];
end
