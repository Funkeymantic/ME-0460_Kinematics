function w_hat = hat_PCM(w)
    % Returns the skew-symmetric matrix representation of a 3D vector
    % w: 3x1 vector
    % Returns: w_hat (3x3 skew-symmetric matrix)

    w = w(:);
    w_hat = [  0    -w(3)  w(2);
              w(3)    0   -w(1);
             -w(2)  w(1)    0  ];
end