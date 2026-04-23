function [ g_inv ] = ginv( g )

s = size(g);

if s(2) == 4 && s(1) == 4
    R = g(1:3,1:3);
    p = g(1:3,4);
    g_inv = [R' -R'*p; 0 0 0 1];
else
    '4x4 matrix expected'
end

end
