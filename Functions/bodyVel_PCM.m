function Vb = bodyVel_PCM(g, g_dot)
    % Computes the body velocity of a rigid body
    % g:     4x4 HGT matrix g(t)
    % g_dot: 4x4 time derivative of g(t)
    % Returns: Vb (6x1 body velocity vector [v^b; w^b])
    %
    % The body velocity is defined as:
    %   V_hat^b = g^{-1} * g_dot     (4x4 matrix form)
    %   V^b     = vee( V_hat^b )     (6x1 vector form)
    %
    % V^b is expressed in the moving body frame
    %
    % Relationship to spatial velocity:
    %   V^s = Ad_g * V^b

    V_hat = invHGT_PCM(g) * g_dot;
    Vb    = veeTwist_PCM(V_hat);
end
