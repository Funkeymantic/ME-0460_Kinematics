function F_b = wrenchTransform_PCM(g_ab, F_a)
    % Transforms a wrench from frame a to frame b
    % g_ab:  4x4 HGT matrix from frame b to frame a (i.e., g_ab maps b coords to a coords)
    % F_a:   6x1 wrench expressed in frame a: F_a = [f_a; tau_a]
    % Returns: F_b (6x1 wrench expressed in frame b)
    %
    % Wrenches transform as the dual of twists (power duality):
    %   V^a = Ad_{g_ab} * V^b      (twist transformation)
    %   F^b = Ad_{g_ab}^T * F^a    (wrench transformation)
    %
    % This preserves power: F^a . V^a = F^b . V^b

    Ad   = adjoint_PCM(g_ab);
    F_b  = Ad' * F_a;
end
