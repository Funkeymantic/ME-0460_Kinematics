function Ad = AdgPCM(th)
% AdgPCM  6x6 Adjoint of g_ab(theta).
%   Ad = AdgPCM(th)  returns 6x6 Ad_{g_ab} at joint angle th (radians).
%
% MLS convention  (twist = [v; omega], wrench = [f; m])
%   Ad_g = [ R,   skew(p)*R ]
%          [ 0,           R ]

g  = gabthPCM(th);
R  = g(1:3, 1:3);
p  = g(1:3, 4);
pH = skew(p);

Ad = [ R,       pH*R ;
       zeros(3), R   ];
end

function S = skew(v)
S = [  0,   -v(3),  v(2);
       v(3),  0,   -v(1);
      -v(2),  v(1),  0  ];
end
