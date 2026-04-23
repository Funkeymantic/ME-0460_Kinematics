function [ R ] = expr( w,th )

s1 = size(w);
s2 = size(th);

if s1(2) == 1 && s1(1) == 3
    if s2(1) == 1 && s2(2) == 1
        what=hat(w);
        R = eye(3)+what*sin(th)+what^2*(1-cos(th));
    else
        'scalar expected for theta'
    end
else
    '3x1 column vectors expected for w'
end

end
