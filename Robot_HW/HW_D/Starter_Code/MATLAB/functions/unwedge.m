function [ xi ] = unwedge( xi_wedge )

s = size(xi_wedge);

if s(2) == 4 && s(1) == 4;
    xi = [xi_wedge(1:3,4); unhat(xi_wedge(1:3,1:3))];
else
    '4x4 matrix expected'
end

end
