function Ad = adjoint_PCM(g)
    % Computes the 6x6 Adjoint transformation matrix for a HGT
    % g: 4x4 HGT matrix [R, p; 0, 1]
    % Returns: Ad (6x6 Adjoint matrix)
    %
    % Ad_g = [ R,      p_hat * R ]
    %        [ 0,          R     ]
    %
    % Used to transform twists between frames:
    %   V^a = Ad_{g_ab} * V^b
    %
    % where g_ab is the HGT from frame b to frame a,
    % and V = [v; w] (linear velocity first, angular second)

    R     = g(1:3, 1:3);
    p     = g(1:3, 4);
    p_hat = hat_PCM(p);

    Ad = [R,       p_hat * R;
          zeros(3),    R    ];
end
