function xi = veeTwist_PCM(Xi)
    % Inverse of the wedge operator: extracts 6x1 twist from 4x4 twist matrix
    % Xi: 4x4 twist matrix (xi_hat)
    % Returns: xi = [v; w] (6x1 twist vector)
    %
    % If Xi = wedge_PCM(xi), then xi = veeTwist_PCM(Xi)
    % Xi = [ w_hat   v ]
    %      [  0  0  0  0 ]

    v = Xi(1:3, 4);
    w = vee_PCM(Xi(1:3, 1:3));
    xi = [v; w];
end
