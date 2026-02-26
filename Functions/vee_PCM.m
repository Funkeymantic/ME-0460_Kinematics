function w = vee_PCM(W)
    % Inverse of the hat operator: extracts 3x1 vector from 3x3 skew-symmetric matrix
    % W: 3x3 skew-symmetric matrix (w_hat)
    % Returns: w (3x1 vector)
    %
    % If W = hat_PCM(w), then w = vee_PCM(W)
    % W = [  0   -w3   w2 ]
    %     [  w3   0   -w1 ]
    %     [ -w2   w1   0  ]

    w = [W(3,2); W(1,3); W(2,1)];
end
