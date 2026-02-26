function g_inv = invHGT_PCM(g)
    % Computes the inverse of a 4x4 HGT matrix analytically
    % g: 4x4 HGT matrix [R, p; 0, 1]
    % Returns: g_inv (4x4 inverse HGT matrix)
    %
    % For g = [R, p; 0, 1]:
    %   g_inv = [R^T,  -R^T*p]
    %           [ 0,      1  ]

    R = g(1:3, 1:3);
    p = g(1:3, 4);

    g_inv = [R',    -R'*p;
             0, 0, 0,  1 ];
end
