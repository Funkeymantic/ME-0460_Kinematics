function [ xiout ] = bracket( xi1,xi2 )

s1= size(xi1);
s2= size(xi2);

if s1(2) == 1 && s1(1) == 6;
    if s2(2) == 1 && s2(1) == 6;
        xiout = unwedge(wedge(xi1)*wedge(xi2)-wedge(xi2)*wedge(xi1));
    else
        '6x1 column vectors expected'
    end
else
    '6x1 column vectors expected'
end

end
