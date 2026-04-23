function [ T ] = expt( t,th )

v = t(1:3);
w = t(4:6);

if w(1)==0 && w(2)==0 && w(3)==0
    T = [eye(3)   th*v;
        0 0 0        1];
else
    R = expr(w,th);
    T = [   R    (eye(3)-R)*(hat(w)*v);
        0 0 0                  1];
end

end
