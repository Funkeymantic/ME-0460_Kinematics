function Vs = spatialVel_PCM(g, g_dot)
    % Computes the spatial velocity of a rigid body
    % g:     4x4 HGT matrix g(t)
    % g_dot: 4x4 time derivative of g(t)
    % Returns: Vs (6x1 spatial velocity vector [v^s; w^s])
    %
    % The spatial velocity is defined as:
    %   V_hat^s = g_dot * g^{-1}     (4x4 matrix form)
    %   V^s     = vee( V_hat^s )     (6x1 vector form)
    %
    % V^s is expressed in the fixed/world (spatial) frame

    V_hat = g_dot * invHGT_PCM(g);
    Vs    = veeTwist_PCM(V_hat);
end
