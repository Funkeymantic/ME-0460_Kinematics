function [ xi_wedge ] = wedge( xi )

s = size(xi);

if s(2) == 1 && s(1) == 6;
    xi_wedge = [hat(xi(4:6)),xi(1:3);0 0 0 0];
else
    '6x1 column vector expected'
end

end
