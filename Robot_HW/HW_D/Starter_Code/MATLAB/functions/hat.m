function [ w_hat ] = hat( w )

s = size(w);

if s(2) == 1 && s(1) == 3
    w_hat = [ 0, -w(3), w(2);
             w(3), 0, -w(1);
            -w(2) w(1) 0];
else
    '3x1 column vector expected'
end

end
