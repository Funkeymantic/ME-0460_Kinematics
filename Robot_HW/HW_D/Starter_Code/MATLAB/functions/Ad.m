function [ adg ] = Ad( g )

s = size(g);

if s(1)==4 && s(2)==4
    R = g(1:3,1:3);
    p = g(1:3,4);
    adg = [R,hat(p)*R;zeros(3,3),R];
else
    '4x4 matrix expected'
end

end
