function [ w ] = unhat( what )

s = size(what);

if s(2) == 3 && s(1) == 3
        w(1) = -what(2,3);
        w(2) =  what(1,3);
        w(3) = -what(1,2);
        w=w';
else
    '3x3 matrix expected'
end

end
