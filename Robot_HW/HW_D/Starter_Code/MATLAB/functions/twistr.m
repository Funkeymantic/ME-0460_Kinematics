function [ z ] = twistr( w,q)

s1 = size(w);
s2 = size(q);

if s1(2) == 1 && s1(1) == 3 && s2(2) == 1 && s2(1) == 3
    z = [-hat(w)*q;w];
else
    '3x1 column vectors expected'
end

end
